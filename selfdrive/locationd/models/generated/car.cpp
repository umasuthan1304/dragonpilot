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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6013312621321988663) {
   out_6013312621321988663[0] = delta_x[0] + nom_x[0];
   out_6013312621321988663[1] = delta_x[1] + nom_x[1];
   out_6013312621321988663[2] = delta_x[2] + nom_x[2];
   out_6013312621321988663[3] = delta_x[3] + nom_x[3];
   out_6013312621321988663[4] = delta_x[4] + nom_x[4];
   out_6013312621321988663[5] = delta_x[5] + nom_x[5];
   out_6013312621321988663[6] = delta_x[6] + nom_x[6];
   out_6013312621321988663[7] = delta_x[7] + nom_x[7];
   out_6013312621321988663[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_859444703069037077) {
   out_859444703069037077[0] = -nom_x[0] + true_x[0];
   out_859444703069037077[1] = -nom_x[1] + true_x[1];
   out_859444703069037077[2] = -nom_x[2] + true_x[2];
   out_859444703069037077[3] = -nom_x[3] + true_x[3];
   out_859444703069037077[4] = -nom_x[4] + true_x[4];
   out_859444703069037077[5] = -nom_x[5] + true_x[5];
   out_859444703069037077[6] = -nom_x[6] + true_x[6];
   out_859444703069037077[7] = -nom_x[7] + true_x[7];
   out_859444703069037077[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6012836658678105946) {
   out_6012836658678105946[0] = 1.0;
   out_6012836658678105946[1] = 0;
   out_6012836658678105946[2] = 0;
   out_6012836658678105946[3] = 0;
   out_6012836658678105946[4] = 0;
   out_6012836658678105946[5] = 0;
   out_6012836658678105946[6] = 0;
   out_6012836658678105946[7] = 0;
   out_6012836658678105946[8] = 0;
   out_6012836658678105946[9] = 0;
   out_6012836658678105946[10] = 1.0;
   out_6012836658678105946[11] = 0;
   out_6012836658678105946[12] = 0;
   out_6012836658678105946[13] = 0;
   out_6012836658678105946[14] = 0;
   out_6012836658678105946[15] = 0;
   out_6012836658678105946[16] = 0;
   out_6012836658678105946[17] = 0;
   out_6012836658678105946[18] = 0;
   out_6012836658678105946[19] = 0;
   out_6012836658678105946[20] = 1.0;
   out_6012836658678105946[21] = 0;
   out_6012836658678105946[22] = 0;
   out_6012836658678105946[23] = 0;
   out_6012836658678105946[24] = 0;
   out_6012836658678105946[25] = 0;
   out_6012836658678105946[26] = 0;
   out_6012836658678105946[27] = 0;
   out_6012836658678105946[28] = 0;
   out_6012836658678105946[29] = 0;
   out_6012836658678105946[30] = 1.0;
   out_6012836658678105946[31] = 0;
   out_6012836658678105946[32] = 0;
   out_6012836658678105946[33] = 0;
   out_6012836658678105946[34] = 0;
   out_6012836658678105946[35] = 0;
   out_6012836658678105946[36] = 0;
   out_6012836658678105946[37] = 0;
   out_6012836658678105946[38] = 0;
   out_6012836658678105946[39] = 0;
   out_6012836658678105946[40] = 1.0;
   out_6012836658678105946[41] = 0;
   out_6012836658678105946[42] = 0;
   out_6012836658678105946[43] = 0;
   out_6012836658678105946[44] = 0;
   out_6012836658678105946[45] = 0;
   out_6012836658678105946[46] = 0;
   out_6012836658678105946[47] = 0;
   out_6012836658678105946[48] = 0;
   out_6012836658678105946[49] = 0;
   out_6012836658678105946[50] = 1.0;
   out_6012836658678105946[51] = 0;
   out_6012836658678105946[52] = 0;
   out_6012836658678105946[53] = 0;
   out_6012836658678105946[54] = 0;
   out_6012836658678105946[55] = 0;
   out_6012836658678105946[56] = 0;
   out_6012836658678105946[57] = 0;
   out_6012836658678105946[58] = 0;
   out_6012836658678105946[59] = 0;
   out_6012836658678105946[60] = 1.0;
   out_6012836658678105946[61] = 0;
   out_6012836658678105946[62] = 0;
   out_6012836658678105946[63] = 0;
   out_6012836658678105946[64] = 0;
   out_6012836658678105946[65] = 0;
   out_6012836658678105946[66] = 0;
   out_6012836658678105946[67] = 0;
   out_6012836658678105946[68] = 0;
   out_6012836658678105946[69] = 0;
   out_6012836658678105946[70] = 1.0;
   out_6012836658678105946[71] = 0;
   out_6012836658678105946[72] = 0;
   out_6012836658678105946[73] = 0;
   out_6012836658678105946[74] = 0;
   out_6012836658678105946[75] = 0;
   out_6012836658678105946[76] = 0;
   out_6012836658678105946[77] = 0;
   out_6012836658678105946[78] = 0;
   out_6012836658678105946[79] = 0;
   out_6012836658678105946[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1242451129020597584) {
   out_1242451129020597584[0] = state[0];
   out_1242451129020597584[1] = state[1];
   out_1242451129020597584[2] = state[2];
   out_1242451129020597584[3] = state[3];
   out_1242451129020597584[4] = state[4];
   out_1242451129020597584[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1242451129020597584[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1242451129020597584[7] = state[7];
   out_1242451129020597584[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1946512517403504011) {
   out_1946512517403504011[0] = 1;
   out_1946512517403504011[1] = 0;
   out_1946512517403504011[2] = 0;
   out_1946512517403504011[3] = 0;
   out_1946512517403504011[4] = 0;
   out_1946512517403504011[5] = 0;
   out_1946512517403504011[6] = 0;
   out_1946512517403504011[7] = 0;
   out_1946512517403504011[8] = 0;
   out_1946512517403504011[9] = 0;
   out_1946512517403504011[10] = 1;
   out_1946512517403504011[11] = 0;
   out_1946512517403504011[12] = 0;
   out_1946512517403504011[13] = 0;
   out_1946512517403504011[14] = 0;
   out_1946512517403504011[15] = 0;
   out_1946512517403504011[16] = 0;
   out_1946512517403504011[17] = 0;
   out_1946512517403504011[18] = 0;
   out_1946512517403504011[19] = 0;
   out_1946512517403504011[20] = 1;
   out_1946512517403504011[21] = 0;
   out_1946512517403504011[22] = 0;
   out_1946512517403504011[23] = 0;
   out_1946512517403504011[24] = 0;
   out_1946512517403504011[25] = 0;
   out_1946512517403504011[26] = 0;
   out_1946512517403504011[27] = 0;
   out_1946512517403504011[28] = 0;
   out_1946512517403504011[29] = 0;
   out_1946512517403504011[30] = 1;
   out_1946512517403504011[31] = 0;
   out_1946512517403504011[32] = 0;
   out_1946512517403504011[33] = 0;
   out_1946512517403504011[34] = 0;
   out_1946512517403504011[35] = 0;
   out_1946512517403504011[36] = 0;
   out_1946512517403504011[37] = 0;
   out_1946512517403504011[38] = 0;
   out_1946512517403504011[39] = 0;
   out_1946512517403504011[40] = 1;
   out_1946512517403504011[41] = 0;
   out_1946512517403504011[42] = 0;
   out_1946512517403504011[43] = 0;
   out_1946512517403504011[44] = 0;
   out_1946512517403504011[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1946512517403504011[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1946512517403504011[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1946512517403504011[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1946512517403504011[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1946512517403504011[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1946512517403504011[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1946512517403504011[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1946512517403504011[53] = -9.8000000000000007*dt;
   out_1946512517403504011[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1946512517403504011[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1946512517403504011[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1946512517403504011[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1946512517403504011[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1946512517403504011[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1946512517403504011[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1946512517403504011[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1946512517403504011[62] = 0;
   out_1946512517403504011[63] = 0;
   out_1946512517403504011[64] = 0;
   out_1946512517403504011[65] = 0;
   out_1946512517403504011[66] = 0;
   out_1946512517403504011[67] = 0;
   out_1946512517403504011[68] = 0;
   out_1946512517403504011[69] = 0;
   out_1946512517403504011[70] = 1;
   out_1946512517403504011[71] = 0;
   out_1946512517403504011[72] = 0;
   out_1946512517403504011[73] = 0;
   out_1946512517403504011[74] = 0;
   out_1946512517403504011[75] = 0;
   out_1946512517403504011[76] = 0;
   out_1946512517403504011[77] = 0;
   out_1946512517403504011[78] = 0;
   out_1946512517403504011[79] = 0;
   out_1946512517403504011[80] = 1;
}
void h_25(double *state, double *unused, double *out_689772175321388301) {
   out_689772175321388301[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1514765360324712778) {
   out_1514765360324712778[0] = 0;
   out_1514765360324712778[1] = 0;
   out_1514765360324712778[2] = 0;
   out_1514765360324712778[3] = 0;
   out_1514765360324712778[4] = 0;
   out_1514765360324712778[5] = 0;
   out_1514765360324712778[6] = 1;
   out_1514765360324712778[7] = 0;
   out_1514765360324712778[8] = 0;
}
void h_24(double *state, double *unused, double *out_532768920875316661) {
   out_532768920875316661[0] = state[4];
   out_532768920875316661[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1044307878281374054) {
   out_1044307878281374054[0] = 0;
   out_1044307878281374054[1] = 0;
   out_1044307878281374054[2] = 0;
   out_1044307878281374054[3] = 0;
   out_1044307878281374054[4] = 1;
   out_1044307878281374054[5] = 0;
   out_1044307878281374054[6] = 0;
   out_1044307878281374054[7] = 0;
   out_1044307878281374054[8] = 0;
   out_1044307878281374054[9] = 0;
   out_1044307878281374054[10] = 0;
   out_1044307878281374054[11] = 0;
   out_1044307878281374054[12] = 0;
   out_1044307878281374054[13] = 0;
   out_1044307878281374054[14] = 1;
   out_1044307878281374054[15] = 0;
   out_1044307878281374054[16] = 0;
   out_1044307878281374054[17] = 0;
}
void h_30(double *state, double *unused, double *out_760859217307494866) {
   out_760859217307494866[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1385426413181472708) {
   out_1385426413181472708[0] = 0;
   out_1385426413181472708[1] = 0;
   out_1385426413181472708[2] = 0;
   out_1385426413181472708[3] = 0;
   out_1385426413181472708[4] = 1;
   out_1385426413181472708[5] = 0;
   out_1385426413181472708[6] = 0;
   out_1385426413181472708[7] = 0;
   out_1385426413181472708[8] = 0;
}
void h_26(double *state, double *unused, double *out_6450050268946726645) {
   out_6450050268946726645[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2171619424435024682) {
   out_2171619424435024682[0] = 0;
   out_2171619424435024682[1] = 0;
   out_2171619424435024682[2] = 0;
   out_2171619424435024682[3] = 0;
   out_2171619424435024682[4] = 0;
   out_2171619424435024682[5] = 0;
   out_2171619424435024682[6] = 0;
   out_2171619424435024682[7] = 1;
   out_2171619424435024682[8] = 0;
}
void h_27(double *state, double *unused, double *out_5566052546637658953) {
   out_5566052546637658953[0] = state[3];
}
void H_27(double *state, double *unused, double *out_789336898618952203) {
   out_789336898618952203[0] = 0;
   out_789336898618952203[1] = 0;
   out_789336898618952203[2] = 0;
   out_789336898618952203[3] = 1;
   out_789336898618952203[4] = 0;
   out_789336898618952203[5] = 0;
   out_789336898618952203[6] = 0;
   out_789336898618952203[7] = 0;
   out_789336898618952203[8] = 0;
}
void h_29(double *state, double *unused, double *out_1928603018383636603) {
   out_1928603018383636603[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1895657757495864892) {
   out_1895657757495864892[0] = 0;
   out_1895657757495864892[1] = 1;
   out_1895657757495864892[2] = 0;
   out_1895657757495864892[3] = 0;
   out_1895657757495864892[4] = 0;
   out_1895657757495864892[5] = 0;
   out_1895657757495864892[6] = 0;
   out_1895657757495864892[7] = 0;
   out_1895657757495864892[8] = 0;
}
void h_28(double *state, double *unused, double *out_3290523729024606819) {
   out_3290523729024606819[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3186741259573665682) {
   out_3186741259573665682[0] = 1;
   out_3186741259573665682[1] = 0;
   out_3186741259573665682[2] = 0;
   out_3186741259573665682[3] = 0;
   out_3186741259573665682[4] = 0;
   out_3186741259573665682[5] = 0;
   out_3186741259573665682[6] = 0;
   out_3186741259573665682[7] = 0;
   out_3186741259573665682[8] = 0;
}
void h_31(double *state, double *unused, double *out_9117094421669621431) {
   out_9117094421669621431[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1545411322201673206) {
   out_1545411322201673206[0] = 0;
   out_1545411322201673206[1] = 0;
   out_1545411322201673206[2] = 0;
   out_1545411322201673206[3] = 0;
   out_1545411322201673206[4] = 0;
   out_1545411322201673206[5] = 0;
   out_1545411322201673206[6] = 0;
   out_1545411322201673206[7] = 0;
   out_1545411322201673206[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6013312621321988663) {
  err_fun(nom_x, delta_x, out_6013312621321988663);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_859444703069037077) {
  inv_err_fun(nom_x, true_x, out_859444703069037077);
}
void car_H_mod_fun(double *state, double *out_6012836658678105946) {
  H_mod_fun(state, out_6012836658678105946);
}
void car_f_fun(double *state, double dt, double *out_1242451129020597584) {
  f_fun(state,  dt, out_1242451129020597584);
}
void car_F_fun(double *state, double dt, double *out_1946512517403504011) {
  F_fun(state,  dt, out_1946512517403504011);
}
void car_h_25(double *state, double *unused, double *out_689772175321388301) {
  h_25(state, unused, out_689772175321388301);
}
void car_H_25(double *state, double *unused, double *out_1514765360324712778) {
  H_25(state, unused, out_1514765360324712778);
}
void car_h_24(double *state, double *unused, double *out_532768920875316661) {
  h_24(state, unused, out_532768920875316661);
}
void car_H_24(double *state, double *unused, double *out_1044307878281374054) {
  H_24(state, unused, out_1044307878281374054);
}
void car_h_30(double *state, double *unused, double *out_760859217307494866) {
  h_30(state, unused, out_760859217307494866);
}
void car_H_30(double *state, double *unused, double *out_1385426413181472708) {
  H_30(state, unused, out_1385426413181472708);
}
void car_h_26(double *state, double *unused, double *out_6450050268946726645) {
  h_26(state, unused, out_6450050268946726645);
}
void car_H_26(double *state, double *unused, double *out_2171619424435024682) {
  H_26(state, unused, out_2171619424435024682);
}
void car_h_27(double *state, double *unused, double *out_5566052546637658953) {
  h_27(state, unused, out_5566052546637658953);
}
void car_H_27(double *state, double *unused, double *out_789336898618952203) {
  H_27(state, unused, out_789336898618952203);
}
void car_h_29(double *state, double *unused, double *out_1928603018383636603) {
  h_29(state, unused, out_1928603018383636603);
}
void car_H_29(double *state, double *unused, double *out_1895657757495864892) {
  H_29(state, unused, out_1895657757495864892);
}
void car_h_28(double *state, double *unused, double *out_3290523729024606819) {
  h_28(state, unused, out_3290523729024606819);
}
void car_H_28(double *state, double *unused, double *out_3186741259573665682) {
  H_28(state, unused, out_3186741259573665682);
}
void car_h_31(double *state, double *unused, double *out_9117094421669621431) {
  h_31(state, unused, out_9117094421669621431);
}
void car_H_31(double *state, double *unused, double *out_1545411322201673206) {
  H_31(state, unused, out_1545411322201673206);
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
