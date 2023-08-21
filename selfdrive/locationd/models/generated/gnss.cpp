#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7183379710779838187) {
   out_7183379710779838187[0] = delta_x[0] + nom_x[0];
   out_7183379710779838187[1] = delta_x[1] + nom_x[1];
   out_7183379710779838187[2] = delta_x[2] + nom_x[2];
   out_7183379710779838187[3] = delta_x[3] + nom_x[3];
   out_7183379710779838187[4] = delta_x[4] + nom_x[4];
   out_7183379710779838187[5] = delta_x[5] + nom_x[5];
   out_7183379710779838187[6] = delta_x[6] + nom_x[6];
   out_7183379710779838187[7] = delta_x[7] + nom_x[7];
   out_7183379710779838187[8] = delta_x[8] + nom_x[8];
   out_7183379710779838187[9] = delta_x[9] + nom_x[9];
   out_7183379710779838187[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3641883451336189102) {
   out_3641883451336189102[0] = -nom_x[0] + true_x[0];
   out_3641883451336189102[1] = -nom_x[1] + true_x[1];
   out_3641883451336189102[2] = -nom_x[2] + true_x[2];
   out_3641883451336189102[3] = -nom_x[3] + true_x[3];
   out_3641883451336189102[4] = -nom_x[4] + true_x[4];
   out_3641883451336189102[5] = -nom_x[5] + true_x[5];
   out_3641883451336189102[6] = -nom_x[6] + true_x[6];
   out_3641883451336189102[7] = -nom_x[7] + true_x[7];
   out_3641883451336189102[8] = -nom_x[8] + true_x[8];
   out_3641883451336189102[9] = -nom_x[9] + true_x[9];
   out_3641883451336189102[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4041252958982134742) {
   out_4041252958982134742[0] = 1.0;
   out_4041252958982134742[1] = 0;
   out_4041252958982134742[2] = 0;
   out_4041252958982134742[3] = 0;
   out_4041252958982134742[4] = 0;
   out_4041252958982134742[5] = 0;
   out_4041252958982134742[6] = 0;
   out_4041252958982134742[7] = 0;
   out_4041252958982134742[8] = 0;
   out_4041252958982134742[9] = 0;
   out_4041252958982134742[10] = 0;
   out_4041252958982134742[11] = 0;
   out_4041252958982134742[12] = 1.0;
   out_4041252958982134742[13] = 0;
   out_4041252958982134742[14] = 0;
   out_4041252958982134742[15] = 0;
   out_4041252958982134742[16] = 0;
   out_4041252958982134742[17] = 0;
   out_4041252958982134742[18] = 0;
   out_4041252958982134742[19] = 0;
   out_4041252958982134742[20] = 0;
   out_4041252958982134742[21] = 0;
   out_4041252958982134742[22] = 0;
   out_4041252958982134742[23] = 0;
   out_4041252958982134742[24] = 1.0;
   out_4041252958982134742[25] = 0;
   out_4041252958982134742[26] = 0;
   out_4041252958982134742[27] = 0;
   out_4041252958982134742[28] = 0;
   out_4041252958982134742[29] = 0;
   out_4041252958982134742[30] = 0;
   out_4041252958982134742[31] = 0;
   out_4041252958982134742[32] = 0;
   out_4041252958982134742[33] = 0;
   out_4041252958982134742[34] = 0;
   out_4041252958982134742[35] = 0;
   out_4041252958982134742[36] = 1.0;
   out_4041252958982134742[37] = 0;
   out_4041252958982134742[38] = 0;
   out_4041252958982134742[39] = 0;
   out_4041252958982134742[40] = 0;
   out_4041252958982134742[41] = 0;
   out_4041252958982134742[42] = 0;
   out_4041252958982134742[43] = 0;
   out_4041252958982134742[44] = 0;
   out_4041252958982134742[45] = 0;
   out_4041252958982134742[46] = 0;
   out_4041252958982134742[47] = 0;
   out_4041252958982134742[48] = 1.0;
   out_4041252958982134742[49] = 0;
   out_4041252958982134742[50] = 0;
   out_4041252958982134742[51] = 0;
   out_4041252958982134742[52] = 0;
   out_4041252958982134742[53] = 0;
   out_4041252958982134742[54] = 0;
   out_4041252958982134742[55] = 0;
   out_4041252958982134742[56] = 0;
   out_4041252958982134742[57] = 0;
   out_4041252958982134742[58] = 0;
   out_4041252958982134742[59] = 0;
   out_4041252958982134742[60] = 1.0;
   out_4041252958982134742[61] = 0;
   out_4041252958982134742[62] = 0;
   out_4041252958982134742[63] = 0;
   out_4041252958982134742[64] = 0;
   out_4041252958982134742[65] = 0;
   out_4041252958982134742[66] = 0;
   out_4041252958982134742[67] = 0;
   out_4041252958982134742[68] = 0;
   out_4041252958982134742[69] = 0;
   out_4041252958982134742[70] = 0;
   out_4041252958982134742[71] = 0;
   out_4041252958982134742[72] = 1.0;
   out_4041252958982134742[73] = 0;
   out_4041252958982134742[74] = 0;
   out_4041252958982134742[75] = 0;
   out_4041252958982134742[76] = 0;
   out_4041252958982134742[77] = 0;
   out_4041252958982134742[78] = 0;
   out_4041252958982134742[79] = 0;
   out_4041252958982134742[80] = 0;
   out_4041252958982134742[81] = 0;
   out_4041252958982134742[82] = 0;
   out_4041252958982134742[83] = 0;
   out_4041252958982134742[84] = 1.0;
   out_4041252958982134742[85] = 0;
   out_4041252958982134742[86] = 0;
   out_4041252958982134742[87] = 0;
   out_4041252958982134742[88] = 0;
   out_4041252958982134742[89] = 0;
   out_4041252958982134742[90] = 0;
   out_4041252958982134742[91] = 0;
   out_4041252958982134742[92] = 0;
   out_4041252958982134742[93] = 0;
   out_4041252958982134742[94] = 0;
   out_4041252958982134742[95] = 0;
   out_4041252958982134742[96] = 1.0;
   out_4041252958982134742[97] = 0;
   out_4041252958982134742[98] = 0;
   out_4041252958982134742[99] = 0;
   out_4041252958982134742[100] = 0;
   out_4041252958982134742[101] = 0;
   out_4041252958982134742[102] = 0;
   out_4041252958982134742[103] = 0;
   out_4041252958982134742[104] = 0;
   out_4041252958982134742[105] = 0;
   out_4041252958982134742[106] = 0;
   out_4041252958982134742[107] = 0;
   out_4041252958982134742[108] = 1.0;
   out_4041252958982134742[109] = 0;
   out_4041252958982134742[110] = 0;
   out_4041252958982134742[111] = 0;
   out_4041252958982134742[112] = 0;
   out_4041252958982134742[113] = 0;
   out_4041252958982134742[114] = 0;
   out_4041252958982134742[115] = 0;
   out_4041252958982134742[116] = 0;
   out_4041252958982134742[117] = 0;
   out_4041252958982134742[118] = 0;
   out_4041252958982134742[119] = 0;
   out_4041252958982134742[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6498801594821524435) {
   out_6498801594821524435[0] = dt*state[3] + state[0];
   out_6498801594821524435[1] = dt*state[4] + state[1];
   out_6498801594821524435[2] = dt*state[5] + state[2];
   out_6498801594821524435[3] = state[3];
   out_6498801594821524435[4] = state[4];
   out_6498801594821524435[5] = state[5];
   out_6498801594821524435[6] = dt*state[7] + state[6];
   out_6498801594821524435[7] = dt*state[8] + state[7];
   out_6498801594821524435[8] = state[8];
   out_6498801594821524435[9] = state[9];
   out_6498801594821524435[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8116234435880951018) {
   out_8116234435880951018[0] = 1;
   out_8116234435880951018[1] = 0;
   out_8116234435880951018[2] = 0;
   out_8116234435880951018[3] = dt;
   out_8116234435880951018[4] = 0;
   out_8116234435880951018[5] = 0;
   out_8116234435880951018[6] = 0;
   out_8116234435880951018[7] = 0;
   out_8116234435880951018[8] = 0;
   out_8116234435880951018[9] = 0;
   out_8116234435880951018[10] = 0;
   out_8116234435880951018[11] = 0;
   out_8116234435880951018[12] = 1;
   out_8116234435880951018[13] = 0;
   out_8116234435880951018[14] = 0;
   out_8116234435880951018[15] = dt;
   out_8116234435880951018[16] = 0;
   out_8116234435880951018[17] = 0;
   out_8116234435880951018[18] = 0;
   out_8116234435880951018[19] = 0;
   out_8116234435880951018[20] = 0;
   out_8116234435880951018[21] = 0;
   out_8116234435880951018[22] = 0;
   out_8116234435880951018[23] = 0;
   out_8116234435880951018[24] = 1;
   out_8116234435880951018[25] = 0;
   out_8116234435880951018[26] = 0;
   out_8116234435880951018[27] = dt;
   out_8116234435880951018[28] = 0;
   out_8116234435880951018[29] = 0;
   out_8116234435880951018[30] = 0;
   out_8116234435880951018[31] = 0;
   out_8116234435880951018[32] = 0;
   out_8116234435880951018[33] = 0;
   out_8116234435880951018[34] = 0;
   out_8116234435880951018[35] = 0;
   out_8116234435880951018[36] = 1;
   out_8116234435880951018[37] = 0;
   out_8116234435880951018[38] = 0;
   out_8116234435880951018[39] = 0;
   out_8116234435880951018[40] = 0;
   out_8116234435880951018[41] = 0;
   out_8116234435880951018[42] = 0;
   out_8116234435880951018[43] = 0;
   out_8116234435880951018[44] = 0;
   out_8116234435880951018[45] = 0;
   out_8116234435880951018[46] = 0;
   out_8116234435880951018[47] = 0;
   out_8116234435880951018[48] = 1;
   out_8116234435880951018[49] = 0;
   out_8116234435880951018[50] = 0;
   out_8116234435880951018[51] = 0;
   out_8116234435880951018[52] = 0;
   out_8116234435880951018[53] = 0;
   out_8116234435880951018[54] = 0;
   out_8116234435880951018[55] = 0;
   out_8116234435880951018[56] = 0;
   out_8116234435880951018[57] = 0;
   out_8116234435880951018[58] = 0;
   out_8116234435880951018[59] = 0;
   out_8116234435880951018[60] = 1;
   out_8116234435880951018[61] = 0;
   out_8116234435880951018[62] = 0;
   out_8116234435880951018[63] = 0;
   out_8116234435880951018[64] = 0;
   out_8116234435880951018[65] = 0;
   out_8116234435880951018[66] = 0;
   out_8116234435880951018[67] = 0;
   out_8116234435880951018[68] = 0;
   out_8116234435880951018[69] = 0;
   out_8116234435880951018[70] = 0;
   out_8116234435880951018[71] = 0;
   out_8116234435880951018[72] = 1;
   out_8116234435880951018[73] = dt;
   out_8116234435880951018[74] = 0;
   out_8116234435880951018[75] = 0;
   out_8116234435880951018[76] = 0;
   out_8116234435880951018[77] = 0;
   out_8116234435880951018[78] = 0;
   out_8116234435880951018[79] = 0;
   out_8116234435880951018[80] = 0;
   out_8116234435880951018[81] = 0;
   out_8116234435880951018[82] = 0;
   out_8116234435880951018[83] = 0;
   out_8116234435880951018[84] = 1;
   out_8116234435880951018[85] = dt;
   out_8116234435880951018[86] = 0;
   out_8116234435880951018[87] = 0;
   out_8116234435880951018[88] = 0;
   out_8116234435880951018[89] = 0;
   out_8116234435880951018[90] = 0;
   out_8116234435880951018[91] = 0;
   out_8116234435880951018[92] = 0;
   out_8116234435880951018[93] = 0;
   out_8116234435880951018[94] = 0;
   out_8116234435880951018[95] = 0;
   out_8116234435880951018[96] = 1;
   out_8116234435880951018[97] = 0;
   out_8116234435880951018[98] = 0;
   out_8116234435880951018[99] = 0;
   out_8116234435880951018[100] = 0;
   out_8116234435880951018[101] = 0;
   out_8116234435880951018[102] = 0;
   out_8116234435880951018[103] = 0;
   out_8116234435880951018[104] = 0;
   out_8116234435880951018[105] = 0;
   out_8116234435880951018[106] = 0;
   out_8116234435880951018[107] = 0;
   out_8116234435880951018[108] = 1;
   out_8116234435880951018[109] = 0;
   out_8116234435880951018[110] = 0;
   out_8116234435880951018[111] = 0;
   out_8116234435880951018[112] = 0;
   out_8116234435880951018[113] = 0;
   out_8116234435880951018[114] = 0;
   out_8116234435880951018[115] = 0;
   out_8116234435880951018[116] = 0;
   out_8116234435880951018[117] = 0;
   out_8116234435880951018[118] = 0;
   out_8116234435880951018[119] = 0;
   out_8116234435880951018[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8388678651318873895) {
   out_8388678651318873895[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6899375805488604336) {
   out_6899375805488604336[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6899375805488604336[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6899375805488604336[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6899375805488604336[3] = 0;
   out_6899375805488604336[4] = 0;
   out_6899375805488604336[5] = 0;
   out_6899375805488604336[6] = 1;
   out_6899375805488604336[7] = 0;
   out_6899375805488604336[8] = 0;
   out_6899375805488604336[9] = 0;
   out_6899375805488604336[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7101097884500362100) {
   out_7101097884500362100[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5849766626813778537) {
   out_5849766626813778537[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5849766626813778537[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5849766626813778537[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5849766626813778537[3] = 0;
   out_5849766626813778537[4] = 0;
   out_5849766626813778537[5] = 0;
   out_5849766626813778537[6] = 1;
   out_5849766626813778537[7] = 0;
   out_5849766626813778537[8] = 0;
   out_5849766626813778537[9] = 1;
   out_5849766626813778537[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4355116750214246697) {
   out_4355116750214246697[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3178723702682916025) {
   out_3178723702682916025[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[6] = 0;
   out_3178723702682916025[7] = 1;
   out_3178723702682916025[8] = 0;
   out_3178723702682916025[9] = 0;
   out_3178723702682916025[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4355116750214246697) {
   out_4355116750214246697[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3178723702682916025) {
   out_3178723702682916025[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3178723702682916025[6] = 0;
   out_3178723702682916025[7] = 1;
   out_3178723702682916025[8] = 0;
   out_3178723702682916025[9] = 0;
   out_3178723702682916025[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7183379710779838187) {
  err_fun(nom_x, delta_x, out_7183379710779838187);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3641883451336189102) {
  inv_err_fun(nom_x, true_x, out_3641883451336189102);
}
void gnss_H_mod_fun(double *state, double *out_4041252958982134742) {
  H_mod_fun(state, out_4041252958982134742);
}
void gnss_f_fun(double *state, double dt, double *out_6498801594821524435) {
  f_fun(state,  dt, out_6498801594821524435);
}
void gnss_F_fun(double *state, double dt, double *out_8116234435880951018) {
  F_fun(state,  dt, out_8116234435880951018);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8388678651318873895) {
  h_6(state, sat_pos, out_8388678651318873895);
}
void gnss_H_6(double *state, double *sat_pos, double *out_6899375805488604336) {
  H_6(state, sat_pos, out_6899375805488604336);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7101097884500362100) {
  h_20(state, sat_pos, out_7101097884500362100);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5849766626813778537) {
  H_20(state, sat_pos, out_5849766626813778537);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4355116750214246697) {
  h_7(state, sat_pos_vel, out_4355116750214246697);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3178723702682916025) {
  H_7(state, sat_pos_vel, out_3178723702682916025);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4355116750214246697) {
  h_21(state, sat_pos_vel, out_4355116750214246697);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3178723702682916025) {
  H_21(state, sat_pos_vel, out_3178723702682916025);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
