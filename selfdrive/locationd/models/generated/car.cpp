#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8768843555348429618) {
   out_8768843555348429618[0] = delta_x[0] + nom_x[0];
   out_8768843555348429618[1] = delta_x[1] + nom_x[1];
   out_8768843555348429618[2] = delta_x[2] + nom_x[2];
   out_8768843555348429618[3] = delta_x[3] + nom_x[3];
   out_8768843555348429618[4] = delta_x[4] + nom_x[4];
   out_8768843555348429618[5] = delta_x[5] + nom_x[5];
   out_8768843555348429618[6] = delta_x[6] + nom_x[6];
   out_8768843555348429618[7] = delta_x[7] + nom_x[7];
   out_8768843555348429618[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3012897669997168453) {
   out_3012897669997168453[0] = -nom_x[0] + true_x[0];
   out_3012897669997168453[1] = -nom_x[1] + true_x[1];
   out_3012897669997168453[2] = -nom_x[2] + true_x[2];
   out_3012897669997168453[3] = -nom_x[3] + true_x[3];
   out_3012897669997168453[4] = -nom_x[4] + true_x[4];
   out_3012897669997168453[5] = -nom_x[5] + true_x[5];
   out_3012897669997168453[6] = -nom_x[6] + true_x[6];
   out_3012897669997168453[7] = -nom_x[7] + true_x[7];
   out_3012897669997168453[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2657927576300526539) {
   out_2657927576300526539[0] = 1.0;
   out_2657927576300526539[1] = 0;
   out_2657927576300526539[2] = 0;
   out_2657927576300526539[3] = 0;
   out_2657927576300526539[4] = 0;
   out_2657927576300526539[5] = 0;
   out_2657927576300526539[6] = 0;
   out_2657927576300526539[7] = 0;
   out_2657927576300526539[8] = 0;
   out_2657927576300526539[9] = 0;
   out_2657927576300526539[10] = 1.0;
   out_2657927576300526539[11] = 0;
   out_2657927576300526539[12] = 0;
   out_2657927576300526539[13] = 0;
   out_2657927576300526539[14] = 0;
   out_2657927576300526539[15] = 0;
   out_2657927576300526539[16] = 0;
   out_2657927576300526539[17] = 0;
   out_2657927576300526539[18] = 0;
   out_2657927576300526539[19] = 0;
   out_2657927576300526539[20] = 1.0;
   out_2657927576300526539[21] = 0;
   out_2657927576300526539[22] = 0;
   out_2657927576300526539[23] = 0;
   out_2657927576300526539[24] = 0;
   out_2657927576300526539[25] = 0;
   out_2657927576300526539[26] = 0;
   out_2657927576300526539[27] = 0;
   out_2657927576300526539[28] = 0;
   out_2657927576300526539[29] = 0;
   out_2657927576300526539[30] = 1.0;
   out_2657927576300526539[31] = 0;
   out_2657927576300526539[32] = 0;
   out_2657927576300526539[33] = 0;
   out_2657927576300526539[34] = 0;
   out_2657927576300526539[35] = 0;
   out_2657927576300526539[36] = 0;
   out_2657927576300526539[37] = 0;
   out_2657927576300526539[38] = 0;
   out_2657927576300526539[39] = 0;
   out_2657927576300526539[40] = 1.0;
   out_2657927576300526539[41] = 0;
   out_2657927576300526539[42] = 0;
   out_2657927576300526539[43] = 0;
   out_2657927576300526539[44] = 0;
   out_2657927576300526539[45] = 0;
   out_2657927576300526539[46] = 0;
   out_2657927576300526539[47] = 0;
   out_2657927576300526539[48] = 0;
   out_2657927576300526539[49] = 0;
   out_2657927576300526539[50] = 1.0;
   out_2657927576300526539[51] = 0;
   out_2657927576300526539[52] = 0;
   out_2657927576300526539[53] = 0;
   out_2657927576300526539[54] = 0;
   out_2657927576300526539[55] = 0;
   out_2657927576300526539[56] = 0;
   out_2657927576300526539[57] = 0;
   out_2657927576300526539[58] = 0;
   out_2657927576300526539[59] = 0;
   out_2657927576300526539[60] = 1.0;
   out_2657927576300526539[61] = 0;
   out_2657927576300526539[62] = 0;
   out_2657927576300526539[63] = 0;
   out_2657927576300526539[64] = 0;
   out_2657927576300526539[65] = 0;
   out_2657927576300526539[66] = 0;
   out_2657927576300526539[67] = 0;
   out_2657927576300526539[68] = 0;
   out_2657927576300526539[69] = 0;
   out_2657927576300526539[70] = 1.0;
   out_2657927576300526539[71] = 0;
   out_2657927576300526539[72] = 0;
   out_2657927576300526539[73] = 0;
   out_2657927576300526539[74] = 0;
   out_2657927576300526539[75] = 0;
   out_2657927576300526539[76] = 0;
   out_2657927576300526539[77] = 0;
   out_2657927576300526539[78] = 0;
   out_2657927576300526539[79] = 0;
   out_2657927576300526539[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8865250004206688022) {
   out_8865250004206688022[0] = state[0];
   out_8865250004206688022[1] = state[1];
   out_8865250004206688022[2] = state[2];
   out_8865250004206688022[3] = state[3];
   out_8865250004206688022[4] = state[4];
   out_8865250004206688022[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8865250004206688022[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8865250004206688022[7] = state[7];
   out_8865250004206688022[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3915349042177754336) {
   out_3915349042177754336[0] = 1;
   out_3915349042177754336[1] = 0;
   out_3915349042177754336[2] = 0;
   out_3915349042177754336[3] = 0;
   out_3915349042177754336[4] = 0;
   out_3915349042177754336[5] = 0;
   out_3915349042177754336[6] = 0;
   out_3915349042177754336[7] = 0;
   out_3915349042177754336[8] = 0;
   out_3915349042177754336[9] = 0;
   out_3915349042177754336[10] = 1;
   out_3915349042177754336[11] = 0;
   out_3915349042177754336[12] = 0;
   out_3915349042177754336[13] = 0;
   out_3915349042177754336[14] = 0;
   out_3915349042177754336[15] = 0;
   out_3915349042177754336[16] = 0;
   out_3915349042177754336[17] = 0;
   out_3915349042177754336[18] = 0;
   out_3915349042177754336[19] = 0;
   out_3915349042177754336[20] = 1;
   out_3915349042177754336[21] = 0;
   out_3915349042177754336[22] = 0;
   out_3915349042177754336[23] = 0;
   out_3915349042177754336[24] = 0;
   out_3915349042177754336[25] = 0;
   out_3915349042177754336[26] = 0;
   out_3915349042177754336[27] = 0;
   out_3915349042177754336[28] = 0;
   out_3915349042177754336[29] = 0;
   out_3915349042177754336[30] = 1;
   out_3915349042177754336[31] = 0;
   out_3915349042177754336[32] = 0;
   out_3915349042177754336[33] = 0;
   out_3915349042177754336[34] = 0;
   out_3915349042177754336[35] = 0;
   out_3915349042177754336[36] = 0;
   out_3915349042177754336[37] = 0;
   out_3915349042177754336[38] = 0;
   out_3915349042177754336[39] = 0;
   out_3915349042177754336[40] = 1;
   out_3915349042177754336[41] = 0;
   out_3915349042177754336[42] = 0;
   out_3915349042177754336[43] = 0;
   out_3915349042177754336[44] = 0;
   out_3915349042177754336[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3915349042177754336[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3915349042177754336[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3915349042177754336[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3915349042177754336[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3915349042177754336[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3915349042177754336[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3915349042177754336[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3915349042177754336[53] = -9.8000000000000007*dt;
   out_3915349042177754336[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3915349042177754336[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3915349042177754336[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3915349042177754336[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3915349042177754336[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3915349042177754336[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3915349042177754336[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3915349042177754336[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3915349042177754336[62] = 0;
   out_3915349042177754336[63] = 0;
   out_3915349042177754336[64] = 0;
   out_3915349042177754336[65] = 0;
   out_3915349042177754336[66] = 0;
   out_3915349042177754336[67] = 0;
   out_3915349042177754336[68] = 0;
   out_3915349042177754336[69] = 0;
   out_3915349042177754336[70] = 1;
   out_3915349042177754336[71] = 0;
   out_3915349042177754336[72] = 0;
   out_3915349042177754336[73] = 0;
   out_3915349042177754336[74] = 0;
   out_3915349042177754336[75] = 0;
   out_3915349042177754336[76] = 0;
   out_3915349042177754336[77] = 0;
   out_3915349042177754336[78] = 0;
   out_3915349042177754336[79] = 0;
   out_3915349042177754336[80] = 1;
}
void h_25(double *state, double *unused, double *out_5676062757260047138) {
   out_5676062757260047138[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3133507402404727067) {
   out_3133507402404727067[0] = 0;
   out_3133507402404727067[1] = 0;
   out_3133507402404727067[2] = 0;
   out_3133507402404727067[3] = 0;
   out_3133507402404727067[4] = 0;
   out_3133507402404727067[5] = 0;
   out_3133507402404727067[6] = 1;
   out_3133507402404727067[7] = 0;
   out_3133507402404727067[8] = 0;
}
void h_24(double *state, double *unused, double *out_4160542853645524808) {
   out_4160542853645524808[0] = state[4];
   out_4160542853645524808[1] = state[5];
}
void H_24(double *state, double *unused, double *out_89190192400378875) {
   out_89190192400378875[0] = 0;
   out_89190192400378875[1] = 0;
   out_89190192400378875[2] = 0;
   out_89190192400378875[3] = 0;
   out_89190192400378875[4] = 1;
   out_89190192400378875[5] = 0;
   out_89190192400378875[6] = 0;
   out_89190192400378875[7] = 0;
   out_89190192400378875[8] = 0;
   out_89190192400378875[9] = 0;
   out_89190192400378875[10] = 0;
   out_89190192400378875[11] = 0;
   out_89190192400378875[12] = 0;
   out_89190192400378875[13] = 0;
   out_89190192400378875[14] = 1;
   out_89190192400378875[15] = 0;
   out_89190192400378875[16] = 0;
   out_89190192400378875[17] = 0;
}
void h_30(double *state, double *unused, double *out_5803366151768471984) {
   out_5803366151768471984[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3004168455261486997) {
   out_3004168455261486997[0] = 0;
   out_3004168455261486997[1] = 0;
   out_3004168455261486997[2] = 0;
   out_3004168455261486997[3] = 0;
   out_3004168455261486997[4] = 1;
   out_3004168455261486997[5] = 0;
   out_3004168455261486997[6] = 0;
   out_3004168455261486997[7] = 0;
   out_3004168455261486997[8] = 0;
}
void h_26(double *state, double *unused, double *out_7153620018278968026) {
   out_7153620018278968026[0] = state[7];
}
void H_26(double *state, double *unused, double *out_607995916469329157) {
   out_607995916469329157[0] = 0;
   out_607995916469329157[1] = 0;
   out_607995916469329157[2] = 0;
   out_607995916469329157[3] = 0;
   out_607995916469329157[4] = 0;
   out_607995916469329157[5] = 0;
   out_607995916469329157[6] = 0;
   out_607995916469329157[7] = 1;
   out_607995916469329157[8] = 0;
}
void h_27(double *state, double *unused, double *out_7769008048788658833) {
   out_7769008048788658833[0] = state[3];
}
void H_27(double *state, double *unused, double *out_829405143461062086) {
   out_829405143461062086[0] = 0;
   out_829405143461062086[1] = 0;
   out_829405143461062086[2] = 0;
   out_829405143461062086[3] = 1;
   out_829405143461062086[4] = 0;
   out_829405143461062086[5] = 0;
   out_829405143461062086[6] = 0;
   out_829405143461062086[7] = 0;
   out_829405143461062086[8] = 0;
}
void h_29(double *state, double *unused, double *out_4821282058433173872) {
   out_4821282058433173872[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3514399799575879181) {
   out_3514399799575879181[0] = 0;
   out_3514399799575879181[1] = 1;
   out_3514399799575879181[2] = 0;
   out_3514399799575879181[3] = 0;
   out_3514399799575879181[4] = 0;
   out_3514399799575879181[5] = 0;
   out_3514399799575879181[6] = 0;
   out_3514399799575879181[7] = 0;
   out_3514399799575879181[8] = 0;
}
void h_28(double *state, double *unused, double *out_7640949973476560098) {
   out_7640949973476560098[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1079672688156837304) {
   out_1079672688156837304[0] = 1;
   out_1079672688156837304[1] = 0;
   out_1079672688156837304[2] = 0;
   out_1079672688156837304[3] = 0;
   out_1079672688156837304[4] = 0;
   out_1079672688156837304[5] = 0;
   out_1079672688156837304[6] = 0;
   out_1079672688156837304[7] = 0;
   out_1079672688156837304[8] = 0;
}
void h_31(double *state, double *unused, double *out_1814379280831545486) {
   out_1814379280831545486[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3164153364281687495) {
   out_3164153364281687495[0] = 0;
   out_3164153364281687495[1] = 0;
   out_3164153364281687495[2] = 0;
   out_3164153364281687495[3] = 0;
   out_3164153364281687495[4] = 0;
   out_3164153364281687495[5] = 0;
   out_3164153364281687495[6] = 0;
   out_3164153364281687495[7] = 0;
   out_3164153364281687495[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8768843555348429618) {
  err_fun(nom_x, delta_x, out_8768843555348429618);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3012897669997168453) {
  inv_err_fun(nom_x, true_x, out_3012897669997168453);
}
void car_H_mod_fun(double *state, double *out_2657927576300526539) {
  H_mod_fun(state, out_2657927576300526539);
}
void car_f_fun(double *state, double dt, double *out_8865250004206688022) {
  f_fun(state,  dt, out_8865250004206688022);
}
void car_F_fun(double *state, double dt, double *out_3915349042177754336) {
  F_fun(state,  dt, out_3915349042177754336);
}
void car_h_25(double *state, double *unused, double *out_5676062757260047138) {
  h_25(state, unused, out_5676062757260047138);
}
void car_H_25(double *state, double *unused, double *out_3133507402404727067) {
  H_25(state, unused, out_3133507402404727067);
}
void car_h_24(double *state, double *unused, double *out_4160542853645524808) {
  h_24(state, unused, out_4160542853645524808);
}
void car_H_24(double *state, double *unused, double *out_89190192400378875) {
  H_24(state, unused, out_89190192400378875);
}
void car_h_30(double *state, double *unused, double *out_5803366151768471984) {
  h_30(state, unused, out_5803366151768471984);
}
void car_H_30(double *state, double *unused, double *out_3004168455261486997) {
  H_30(state, unused, out_3004168455261486997);
}
void car_h_26(double *state, double *unused, double *out_7153620018278968026) {
  h_26(state, unused, out_7153620018278968026);
}
void car_H_26(double *state, double *unused, double *out_607995916469329157) {
  H_26(state, unused, out_607995916469329157);
}
void car_h_27(double *state, double *unused, double *out_7769008048788658833) {
  h_27(state, unused, out_7769008048788658833);
}
void car_H_27(double *state, double *unused, double *out_829405143461062086) {
  H_27(state, unused, out_829405143461062086);
}
void car_h_29(double *state, double *unused, double *out_4821282058433173872) {
  h_29(state, unused, out_4821282058433173872);
}
void car_H_29(double *state, double *unused, double *out_3514399799575879181) {
  H_29(state, unused, out_3514399799575879181);
}
void car_h_28(double *state, double *unused, double *out_7640949973476560098) {
  h_28(state, unused, out_7640949973476560098);
}
void car_H_28(double *state, double *unused, double *out_1079672688156837304) {
  H_28(state, unused, out_1079672688156837304);
}
void car_h_31(double *state, double *unused, double *out_1814379280831545486) {
  h_31(state, unused, out_1814379280831545486);
}
void car_H_31(double *state, double *unused, double *out_3164153364281687495) {
  H_31(state, unused, out_3164153364281687495);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
