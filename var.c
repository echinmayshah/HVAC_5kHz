/*
 * var.c
 *
 *  Created on: Jan 23, 2026
 *      Author: Chinmay.Shah
 */




#include "var.h"

float arr_speedRef[8] = {0.0};		//speedRef
float arr_speedFdb[8] = {0.0};		//speedFdb

float arr_w_cmd_int[8] 			= {0.0};
float arr_w_final_filt[8] 		= {0.0};

//float arr_acc_cmd_int[8] 		= {0.0};	//?
//float arr_integ_ff[8] 		= {0.0};	//?
//float arr_prop_ff[8] 			= {0.0};	
//float arr_output_raw[8] 		= {0.0};

//float qty_prop_ff = 0.0f;
//float qty_output_raw = 0.0f;
bool get = 0U;
float g_cmd = 0.0f;
float g_fb = 0.0f;
float g_output_raw = 0.0f;
float arr_g_cmd[8] = {0.0};
float arr_g_fb[8] = {0.0};
float arr_g_output_raw[8] = {0.0};

float g_kp = 0.0;
float g_ff = 0.0;
float g_integ = 0.0;
float arr_g_kp[8] = {0.0};
float arr_g_ff[8] = {0.0};
float arr_g_integ[8] = {0.0};

float g_ff_inertia;
float g_ff_friction;
float g_ff_viscous;
float g_ff_total;

float arr_g_ff_inertia[8];
float arr_g_ff_friction[8];
float arr_g_ff_viscous[8];
float arr_g_ff_total[8];

//float arr_pi_output[8] 	= {0.0};
//float arr_i_cmd_spd[8] 	= {0.0};
//float i_qd_r_ref_d[8] 	= {0.0};	//0

uint8_t i_0 = 0U;
uint16_t tick_0 = 0U;