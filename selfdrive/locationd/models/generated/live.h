#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8588883011379206042);
void live_err_fun(double *nom_x, double *delta_x, double *out_1089152390741304887);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_889248675038484345);
void live_H_mod_fun(double *state, double *out_518752642075915095);
void live_f_fun(double *state, double dt, double *out_1911395663799077180);
void live_F_fun(double *state, double dt, double *out_7729970800791593087);
void live_h_4(double *state, double *unused, double *out_6434574856810337111);
void live_H_4(double *state, double *unused, double *out_6169578775284415192);
void live_h_9(double *state, double *unused, double *out_7135467533032239650);
void live_H_9(double *state, double *unused, double *out_8119997562070358941);
void live_h_10(double *state, double *unused, double *out_6922592501021669737);
void live_H_10(double *state, double *unused, double *out_127422989705360210);
void live_h_12(double *state, double *unused, double *out_5566068380469629794);
void live_H_12(double *state, double *unused, double *out_5548479750236821525);
void live_h_35(double *state, double *unused, double *out_8638633768235939949);
void live_H_35(double *state, double *unused, double *out_2802916717911807816);
void live_h_32(double *state, double *unused, double *out_8943295691282783094);
void live_H_32(double *state, double *unused, double *out_4868496817229023687);
void live_h_13(double *state, double *unused, double *out_7198274286225375730);
void live_H_13(double *state, double *unused, double *out_7168384760668621129);
void live_h_14(double *state, double *unused, double *out_7135467533032239650);
void live_H_14(double *state, double *unused, double *out_8119997562070358941);
void live_h_33(double *state, double *unused, double *out_319395186784536697);
void live_H_33(double *state, double *unused, double *out_347640286727049788);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}