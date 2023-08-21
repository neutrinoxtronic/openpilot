#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8768843555348429618);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3012897669997168453);
void car_H_mod_fun(double *state, double *out_2657927576300526539);
void car_f_fun(double *state, double dt, double *out_8865250004206688022);
void car_F_fun(double *state, double dt, double *out_3915349042177754336);
void car_h_25(double *state, double *unused, double *out_5676062757260047138);
void car_H_25(double *state, double *unused, double *out_3133507402404727067);
void car_h_24(double *state, double *unused, double *out_4160542853645524808);
void car_H_24(double *state, double *unused, double *out_89190192400378875);
void car_h_30(double *state, double *unused, double *out_5803366151768471984);
void car_H_30(double *state, double *unused, double *out_3004168455261486997);
void car_h_26(double *state, double *unused, double *out_7153620018278968026);
void car_H_26(double *state, double *unused, double *out_607995916469329157);
void car_h_27(double *state, double *unused, double *out_7769008048788658833);
void car_H_27(double *state, double *unused, double *out_829405143461062086);
void car_h_29(double *state, double *unused, double *out_4821282058433173872);
void car_H_29(double *state, double *unused, double *out_3514399799575879181);
void car_h_28(double *state, double *unused, double *out_7640949973476560098);
void car_H_28(double *state, double *unused, double *out_1079672688156837304);
void car_h_31(double *state, double *unused, double *out_1814379280831545486);
void car_H_31(double *state, double *unused, double *out_3164153364281687495);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}