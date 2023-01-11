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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_252452724878274323) {
   out_252452724878274323[0] = delta_x[0] + nom_x[0];
   out_252452724878274323[1] = delta_x[1] + nom_x[1];
   out_252452724878274323[2] = delta_x[2] + nom_x[2];
   out_252452724878274323[3] = delta_x[3] + nom_x[3];
   out_252452724878274323[4] = delta_x[4] + nom_x[4];
   out_252452724878274323[5] = delta_x[5] + nom_x[5];
   out_252452724878274323[6] = delta_x[6] + nom_x[6];
   out_252452724878274323[7] = delta_x[7] + nom_x[7];
   out_252452724878274323[8] = delta_x[8] + nom_x[8];
   out_252452724878274323[9] = delta_x[9] + nom_x[9];
   out_252452724878274323[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3611133554066969385) {
   out_3611133554066969385[0] = -nom_x[0] + true_x[0];
   out_3611133554066969385[1] = -nom_x[1] + true_x[1];
   out_3611133554066969385[2] = -nom_x[2] + true_x[2];
   out_3611133554066969385[3] = -nom_x[3] + true_x[3];
   out_3611133554066969385[4] = -nom_x[4] + true_x[4];
   out_3611133554066969385[5] = -nom_x[5] + true_x[5];
   out_3611133554066969385[6] = -nom_x[6] + true_x[6];
   out_3611133554066969385[7] = -nom_x[7] + true_x[7];
   out_3611133554066969385[8] = -nom_x[8] + true_x[8];
   out_3611133554066969385[9] = -nom_x[9] + true_x[9];
   out_3611133554066969385[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3143155953764565366) {
   out_3143155953764565366[0] = 1.0;
   out_3143155953764565366[1] = 0;
   out_3143155953764565366[2] = 0;
   out_3143155953764565366[3] = 0;
   out_3143155953764565366[4] = 0;
   out_3143155953764565366[5] = 0;
   out_3143155953764565366[6] = 0;
   out_3143155953764565366[7] = 0;
   out_3143155953764565366[8] = 0;
   out_3143155953764565366[9] = 0;
   out_3143155953764565366[10] = 0;
   out_3143155953764565366[11] = 0;
   out_3143155953764565366[12] = 1.0;
   out_3143155953764565366[13] = 0;
   out_3143155953764565366[14] = 0;
   out_3143155953764565366[15] = 0;
   out_3143155953764565366[16] = 0;
   out_3143155953764565366[17] = 0;
   out_3143155953764565366[18] = 0;
   out_3143155953764565366[19] = 0;
   out_3143155953764565366[20] = 0;
   out_3143155953764565366[21] = 0;
   out_3143155953764565366[22] = 0;
   out_3143155953764565366[23] = 0;
   out_3143155953764565366[24] = 1.0;
   out_3143155953764565366[25] = 0;
   out_3143155953764565366[26] = 0;
   out_3143155953764565366[27] = 0;
   out_3143155953764565366[28] = 0;
   out_3143155953764565366[29] = 0;
   out_3143155953764565366[30] = 0;
   out_3143155953764565366[31] = 0;
   out_3143155953764565366[32] = 0;
   out_3143155953764565366[33] = 0;
   out_3143155953764565366[34] = 0;
   out_3143155953764565366[35] = 0;
   out_3143155953764565366[36] = 1.0;
   out_3143155953764565366[37] = 0;
   out_3143155953764565366[38] = 0;
   out_3143155953764565366[39] = 0;
   out_3143155953764565366[40] = 0;
   out_3143155953764565366[41] = 0;
   out_3143155953764565366[42] = 0;
   out_3143155953764565366[43] = 0;
   out_3143155953764565366[44] = 0;
   out_3143155953764565366[45] = 0;
   out_3143155953764565366[46] = 0;
   out_3143155953764565366[47] = 0;
   out_3143155953764565366[48] = 1.0;
   out_3143155953764565366[49] = 0;
   out_3143155953764565366[50] = 0;
   out_3143155953764565366[51] = 0;
   out_3143155953764565366[52] = 0;
   out_3143155953764565366[53] = 0;
   out_3143155953764565366[54] = 0;
   out_3143155953764565366[55] = 0;
   out_3143155953764565366[56] = 0;
   out_3143155953764565366[57] = 0;
   out_3143155953764565366[58] = 0;
   out_3143155953764565366[59] = 0;
   out_3143155953764565366[60] = 1.0;
   out_3143155953764565366[61] = 0;
   out_3143155953764565366[62] = 0;
   out_3143155953764565366[63] = 0;
   out_3143155953764565366[64] = 0;
   out_3143155953764565366[65] = 0;
   out_3143155953764565366[66] = 0;
   out_3143155953764565366[67] = 0;
   out_3143155953764565366[68] = 0;
   out_3143155953764565366[69] = 0;
   out_3143155953764565366[70] = 0;
   out_3143155953764565366[71] = 0;
   out_3143155953764565366[72] = 1.0;
   out_3143155953764565366[73] = 0;
   out_3143155953764565366[74] = 0;
   out_3143155953764565366[75] = 0;
   out_3143155953764565366[76] = 0;
   out_3143155953764565366[77] = 0;
   out_3143155953764565366[78] = 0;
   out_3143155953764565366[79] = 0;
   out_3143155953764565366[80] = 0;
   out_3143155953764565366[81] = 0;
   out_3143155953764565366[82] = 0;
   out_3143155953764565366[83] = 0;
   out_3143155953764565366[84] = 1.0;
   out_3143155953764565366[85] = 0;
   out_3143155953764565366[86] = 0;
   out_3143155953764565366[87] = 0;
   out_3143155953764565366[88] = 0;
   out_3143155953764565366[89] = 0;
   out_3143155953764565366[90] = 0;
   out_3143155953764565366[91] = 0;
   out_3143155953764565366[92] = 0;
   out_3143155953764565366[93] = 0;
   out_3143155953764565366[94] = 0;
   out_3143155953764565366[95] = 0;
   out_3143155953764565366[96] = 1.0;
   out_3143155953764565366[97] = 0;
   out_3143155953764565366[98] = 0;
   out_3143155953764565366[99] = 0;
   out_3143155953764565366[100] = 0;
   out_3143155953764565366[101] = 0;
   out_3143155953764565366[102] = 0;
   out_3143155953764565366[103] = 0;
   out_3143155953764565366[104] = 0;
   out_3143155953764565366[105] = 0;
   out_3143155953764565366[106] = 0;
   out_3143155953764565366[107] = 0;
   out_3143155953764565366[108] = 1.0;
   out_3143155953764565366[109] = 0;
   out_3143155953764565366[110] = 0;
   out_3143155953764565366[111] = 0;
   out_3143155953764565366[112] = 0;
   out_3143155953764565366[113] = 0;
   out_3143155953764565366[114] = 0;
   out_3143155953764565366[115] = 0;
   out_3143155953764565366[116] = 0;
   out_3143155953764565366[117] = 0;
   out_3143155953764565366[118] = 0;
   out_3143155953764565366[119] = 0;
   out_3143155953764565366[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6578905184550417904) {
   out_6578905184550417904[0] = dt*state[3] + state[0];
   out_6578905184550417904[1] = dt*state[4] + state[1];
   out_6578905184550417904[2] = dt*state[5] + state[2];
   out_6578905184550417904[3] = state[3];
   out_6578905184550417904[4] = state[4];
   out_6578905184550417904[5] = state[5];
   out_6578905184550417904[6] = dt*state[7] + state[6];
   out_6578905184550417904[7] = dt*state[8] + state[7];
   out_6578905184550417904[8] = state[8];
   out_6578905184550417904[9] = state[9];
   out_6578905184550417904[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3464436217934003020) {
   out_3464436217934003020[0] = 1;
   out_3464436217934003020[1] = 0;
   out_3464436217934003020[2] = 0;
   out_3464436217934003020[3] = dt;
   out_3464436217934003020[4] = 0;
   out_3464436217934003020[5] = 0;
   out_3464436217934003020[6] = 0;
   out_3464436217934003020[7] = 0;
   out_3464436217934003020[8] = 0;
   out_3464436217934003020[9] = 0;
   out_3464436217934003020[10] = 0;
   out_3464436217934003020[11] = 0;
   out_3464436217934003020[12] = 1;
   out_3464436217934003020[13] = 0;
   out_3464436217934003020[14] = 0;
   out_3464436217934003020[15] = dt;
   out_3464436217934003020[16] = 0;
   out_3464436217934003020[17] = 0;
   out_3464436217934003020[18] = 0;
   out_3464436217934003020[19] = 0;
   out_3464436217934003020[20] = 0;
   out_3464436217934003020[21] = 0;
   out_3464436217934003020[22] = 0;
   out_3464436217934003020[23] = 0;
   out_3464436217934003020[24] = 1;
   out_3464436217934003020[25] = 0;
   out_3464436217934003020[26] = 0;
   out_3464436217934003020[27] = dt;
   out_3464436217934003020[28] = 0;
   out_3464436217934003020[29] = 0;
   out_3464436217934003020[30] = 0;
   out_3464436217934003020[31] = 0;
   out_3464436217934003020[32] = 0;
   out_3464436217934003020[33] = 0;
   out_3464436217934003020[34] = 0;
   out_3464436217934003020[35] = 0;
   out_3464436217934003020[36] = 1;
   out_3464436217934003020[37] = 0;
   out_3464436217934003020[38] = 0;
   out_3464436217934003020[39] = 0;
   out_3464436217934003020[40] = 0;
   out_3464436217934003020[41] = 0;
   out_3464436217934003020[42] = 0;
   out_3464436217934003020[43] = 0;
   out_3464436217934003020[44] = 0;
   out_3464436217934003020[45] = 0;
   out_3464436217934003020[46] = 0;
   out_3464436217934003020[47] = 0;
   out_3464436217934003020[48] = 1;
   out_3464436217934003020[49] = 0;
   out_3464436217934003020[50] = 0;
   out_3464436217934003020[51] = 0;
   out_3464436217934003020[52] = 0;
   out_3464436217934003020[53] = 0;
   out_3464436217934003020[54] = 0;
   out_3464436217934003020[55] = 0;
   out_3464436217934003020[56] = 0;
   out_3464436217934003020[57] = 0;
   out_3464436217934003020[58] = 0;
   out_3464436217934003020[59] = 0;
   out_3464436217934003020[60] = 1;
   out_3464436217934003020[61] = 0;
   out_3464436217934003020[62] = 0;
   out_3464436217934003020[63] = 0;
   out_3464436217934003020[64] = 0;
   out_3464436217934003020[65] = 0;
   out_3464436217934003020[66] = 0;
   out_3464436217934003020[67] = 0;
   out_3464436217934003020[68] = 0;
   out_3464436217934003020[69] = 0;
   out_3464436217934003020[70] = 0;
   out_3464436217934003020[71] = 0;
   out_3464436217934003020[72] = 1;
   out_3464436217934003020[73] = dt;
   out_3464436217934003020[74] = 0;
   out_3464436217934003020[75] = 0;
   out_3464436217934003020[76] = 0;
   out_3464436217934003020[77] = 0;
   out_3464436217934003020[78] = 0;
   out_3464436217934003020[79] = 0;
   out_3464436217934003020[80] = 0;
   out_3464436217934003020[81] = 0;
   out_3464436217934003020[82] = 0;
   out_3464436217934003020[83] = 0;
   out_3464436217934003020[84] = 1;
   out_3464436217934003020[85] = dt;
   out_3464436217934003020[86] = 0;
   out_3464436217934003020[87] = 0;
   out_3464436217934003020[88] = 0;
   out_3464436217934003020[89] = 0;
   out_3464436217934003020[90] = 0;
   out_3464436217934003020[91] = 0;
   out_3464436217934003020[92] = 0;
   out_3464436217934003020[93] = 0;
   out_3464436217934003020[94] = 0;
   out_3464436217934003020[95] = 0;
   out_3464436217934003020[96] = 1;
   out_3464436217934003020[97] = 0;
   out_3464436217934003020[98] = 0;
   out_3464436217934003020[99] = 0;
   out_3464436217934003020[100] = 0;
   out_3464436217934003020[101] = 0;
   out_3464436217934003020[102] = 0;
   out_3464436217934003020[103] = 0;
   out_3464436217934003020[104] = 0;
   out_3464436217934003020[105] = 0;
   out_3464436217934003020[106] = 0;
   out_3464436217934003020[107] = 0;
   out_3464436217934003020[108] = 1;
   out_3464436217934003020[109] = 0;
   out_3464436217934003020[110] = 0;
   out_3464436217934003020[111] = 0;
   out_3464436217934003020[112] = 0;
   out_3464436217934003020[113] = 0;
   out_3464436217934003020[114] = 0;
   out_3464436217934003020[115] = 0;
   out_3464436217934003020[116] = 0;
   out_3464436217934003020[117] = 0;
   out_3464436217934003020[118] = 0;
   out_3464436217934003020[119] = 0;
   out_3464436217934003020[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_617243780525906715) {
   out_617243780525906715[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4939543408551761490) {
   out_4939543408551761490[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4939543408551761490[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4939543408551761490[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4939543408551761490[3] = 0;
   out_4939543408551761490[4] = 0;
   out_4939543408551761490[5] = 0;
   out_4939543408551761490[6] = 1;
   out_4939543408551761490[7] = 0;
   out_4939543408551761490[8] = 0;
   out_4939543408551761490[9] = 0;
   out_4939543408551761490[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5529500322506579321) {
   out_5529500322506579321[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2747612584649990050) {
   out_2747612584649990050[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2747612584649990050[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2747612584649990050[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2747612584649990050[3] = 0;
   out_2747612584649990050[4] = 0;
   out_2747612584649990050[5] = 0;
   out_2747612584649990050[6] = 1;
   out_2747612584649990050[7] = 0;
   out_2747612584649990050[8] = 0;
   out_2747612584649990050[9] = 1;
   out_2747612584649990050[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1090084589167975596) {
   out_1090084589167975596[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_514157753381773474) {
   out_514157753381773474[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[6] = 0;
   out_514157753381773474[7] = 1;
   out_514157753381773474[8] = 0;
   out_514157753381773474[9] = 0;
   out_514157753381773474[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1090084589167975596) {
   out_1090084589167975596[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_514157753381773474) {
   out_514157753381773474[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_514157753381773474[6] = 0;
   out_514157753381773474[7] = 1;
   out_514157753381773474[8] = 0;
   out_514157753381773474[9] = 0;
   out_514157753381773474[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_252452724878274323) {
  err_fun(nom_x, delta_x, out_252452724878274323);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3611133554066969385) {
  inv_err_fun(nom_x, true_x, out_3611133554066969385);
}
void gnss_H_mod_fun(double *state, double *out_3143155953764565366) {
  H_mod_fun(state, out_3143155953764565366);
}
void gnss_f_fun(double *state, double dt, double *out_6578905184550417904) {
  f_fun(state,  dt, out_6578905184550417904);
}
void gnss_F_fun(double *state, double dt, double *out_3464436217934003020) {
  F_fun(state,  dt, out_3464436217934003020);
}
void gnss_h_6(double *state, double *sat_pos, double *out_617243780525906715) {
  h_6(state, sat_pos, out_617243780525906715);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4939543408551761490) {
  H_6(state, sat_pos, out_4939543408551761490);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5529500322506579321) {
  h_20(state, sat_pos, out_5529500322506579321);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2747612584649990050) {
  H_20(state, sat_pos, out_2747612584649990050);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1090084589167975596) {
  h_7(state, sat_pos_vel, out_1090084589167975596);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_514157753381773474) {
  H_7(state, sat_pos_vel, out_514157753381773474);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1090084589167975596) {
  h_21(state, sat_pos_vel, out_1090084589167975596);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_514157753381773474) {
  H_21(state, sat_pos_vel, out_514157753381773474);
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
