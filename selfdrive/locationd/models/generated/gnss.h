#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_252452724878274323);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3611133554066969385);
void gnss_H_mod_fun(double *state, double *out_3143155953764565366);
void gnss_f_fun(double *state, double dt, double *out_6578905184550417904);
void gnss_F_fun(double *state, double dt, double *out_3464436217934003020);
void gnss_h_6(double *state, double *sat_pos, double *out_617243780525906715);
void gnss_H_6(double *state, double *sat_pos, double *out_4939543408551761490);
void gnss_h_20(double *state, double *sat_pos, double *out_5529500322506579321);
void gnss_H_20(double *state, double *sat_pos, double *out_2747612584649990050);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1090084589167975596);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_514157753381773474);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1090084589167975596);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_514157753381773474);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}