#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7183379710779838187);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3641883451336189102);
void gnss_H_mod_fun(double *state, double *out_4041252958982134742);
void gnss_f_fun(double *state, double dt, double *out_6498801594821524435);
void gnss_F_fun(double *state, double dt, double *out_8116234435880951018);
void gnss_h_6(double *state, double *sat_pos, double *out_8388678651318873895);
void gnss_H_6(double *state, double *sat_pos, double *out_6899375805488604336);
void gnss_h_20(double *state, double *sat_pos, double *out_7101097884500362100);
void gnss_H_20(double *state, double *sat_pos, double *out_5849766626813778537);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4355116750214246697);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3178723702682916025);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4355116750214246697);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3178723702682916025);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}