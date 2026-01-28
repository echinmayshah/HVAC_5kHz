/*
 * var.h
 *
 *  Created on: Jan 23, 2026
 *      Author: Chinmay.Shah
 */

#ifndef VAR_H_
#define VAR_H_


#include "Controller.h"

extern float arr_speedRef[8];		//speedRef
extern float arr_speedFdb[8];		//speedFdb

extern float arr_w_cmd_int[8];
extern float arr_w_final_filt[8];

//extern float arr_acc_cmd_int[8];// = {0.0};	//?
//extern float arr_integ_ff[8];// = {0.0};		//?
//extern float arr_prop_ff[8];// 			= {0.0};
//extern float arr_output_raw[8];// 		= {0.0};

//extern float qty_prop_ff;// = 0.0f;
//extern float qty_output_raw;// = 0.0f;
extern bool get;
extern float g_cmd;
extern float g_fb;
extern float g_output_raw;
extern float arr_g_cmd[8];
extern float arr_g_fb[8];
extern float arr_g_output_raw[8];

extern float g_kp;
extern float g_ff;
extern float g_integ;
extern float arr_g_kp[8];
extern float arr_g_ff[8];
extern float arr_g_integ[8];

extern float g_ff_inertia;
extern float g_ff_friction;
extern float g_ff_viscous;
extern float g_ff_total;

extern float arr_g_ff_inertia[8];
extern float arr_g_ff_friction[8];
extern float arr_g_ff_viscous[8];
extern float arr_g_ff_total[8];

//extern float arr_pi_output[8];
//extern float arr_i_cmd_spd[8];
//extern float i_qd_r_ref_d[8];	//0

extern uint8_t i_0;
extern uint16_t tick_0;


#endif /* VAR_H_ */
