from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_angle_limits, apply_meas_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker
from common.op_params import opParams
import math

SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert

# EPS faults if you request steer while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before steer can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# EPS ignores commands above this angle and causes PCS faults
MAX_STEER_ANGLE = 94.9461  # deg
MAX_ANGLE_LATERAL_ACCEL = 3.5  # m/s^2


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.op_params = opParams()
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0
    self.VM = VM

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # gas and brake
    if self.CP.enableGasInterceptor and CC.longActive:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if self.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

    apply_angle = 0
    apply_steer = 0

    if self.CP.steerControlType != SteerControlType.angle:
      # - steer torque
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)
    else:
      # - steer angle
      # angle command is in terms of the torque sensor angle (may or may not have an offset)
      apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
      torque_sensor_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

      # limit max angle error between cmd and actual to reduce EPS integral windup
      # TODO: can we configure this with a signal?
      apply_angle = clip(apply_angle,
                         -abs(torque_sensor_angle) - self.params.ANGLE_DELTA_MAX,
                         abs(torque_sensor_angle) + self.params.ANGLE_DELTA_MAX)

      # Clip max angle to acceptable lateral accel limits
      # v_ego = max(CS.out.vEgo, 5.)
      # max_steer_angle = abs(MAX_ANGLE_LATERAL_ACCEL / (self.VM.calc_curvature(math.radians(1), v_ego, 0) * v_ego ** 2))  # TODO: roll
      # max_steer_angle = min(max_steer_angle, MAX_STEER_ANGLE)
      max_steer_angle = MAX_STEER_ANGLE
      apply_angle = clip(apply_angle, -max_steer_angle, max_steer_angle)

      # Angular rate limit based on speed
      apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgo, self.params)

      # Clip to max angle limits
      # apply_angle = clip(apply_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

      if not lat_active:
        apply_angle = clip(torque_sensor_angle, -max_steer_angle, max_steer_angle)

    # Count up to MAX_STEER_RATE_FRAMES, at which point we need to cut steer request bit to avoid a steering fault
    if lat_active and abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE:
      self.steer_rate_counter += 1
    else:
      self.steer_rate_counter = 0

    apply_steer_req = 1
    if not lat_active:
      apply_steer = 0
      apply_steer_req = 0
    elif self.steer_rate_counter > MAX_STEER_RATE_FRAMES:
      apply_steer_req = 0
      self.steer_rate_counter = 0

    # Never actuate with LKA on cars that only support LTA
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      apply_steer = 0
      apply_steer_req = 0

    # TODO: probably can delete this. CS.pcm_acc_status uses a different signal
    # than CS.cruiseState.enabled. confirm they're not meaningfully different
    if not CC.enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    can_sends = []

    # *** control msgs ***
    # print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages

    angle_control = self.CP.steerControlType == SteerControlType.angle
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req and not angle_control))
    if TSS2_CAR:
      limit_torque = CS.out.steeringPressed
      can_sends.append(create_lta_steer_command(self.packer, apply_angle, apply_steer_req and angle_control, limit_torque, self.op_params))

    self.last_angle = apply_angle
    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    # we can spam can to cancel the system even if we are using lat only control
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type))

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd

    if self.CP.carFingerprint != CAR.PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      fcw_alert = hud_control.visualAlert == VisualAlert.fcw
      steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                           hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                           hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and self.CP.enableDsu:
        can_sends.append(create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.steeringAngleDeg = apply_angle
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
