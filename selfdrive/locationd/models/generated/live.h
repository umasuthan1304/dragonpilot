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
void live_H(double *in_vec, double *out_2498972411965863268);
void live_err_fun(double *nom_x, double *delta_x, double *out_4385107662726466387);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2203140634195456099);
void live_H_mod_fun(double *state, double *out_1740095069296428);
void live_f_fun(double *state, double dt, double *out_6355359268090963344);
void live_F_fun(double *state, double dt, double *out_3682195849465915566);
void live_h_4(double *state, double *unused, double *out_5896279927389784864);
void live_H_4(double *state, double *unused, double *out_8419195983841068719);
void live_h_9(double *state, double *unused, double *out_8607283333105355994);
void live_H_9(double *state, double *unused, double *out_4262028247486291236);
void live_h_10(double *state, double *unused, double *out_7959890215227314543);
void live_H_10(double *state, double *unused, double *out_4038247209695052382);
void live_h_12(double *state, double *unused, double *out_7889566143973264349);
void live_H_12(double *state, double *unused, double *out_9040295008888662386);
void live_h_35(double *state, double *unused, double *out_6791993495871529456);
void live_H_35(double *state, double *unused, double *out_6660886032495875521);
void live_h_32(double *state, double *unused, double *out_1504692811306052386);
void live_H_32(double *state, double *unused, double *out_8899376618365221599);
void live_h_13(double *state, double *unused, double *out_3693507123269137062);
void live_H_13(double *state, double *unused, double *out_3503664213132522511);
void live_h_14(double *state, double *unused, double *out_8607283333105355994);
void live_H_14(double *state, double *unused, double *out_4262028247486291236);
void live_h_33(double *state, double *unused, double *out_1580845305884005075);
void live_H_33(double *state, double *unused, double *out_3510329027857017917);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}