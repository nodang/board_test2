/*
 * struct.h
 *
 *  Created on: Mar 10, 2022
 *      Author: kimjs
 */

#ifndef INC_STRUCT_H_
#define INC_STRUCT_H_


#ifdef __STRUCT__

#undef __STRUCT__
#define __STRUCT_EXT__

#else
#define __STRUCT_EXT__	extern
#endif



typedef volatile struct bit_field_flag
{

	uint16_t motor_start:1;
	uint16_t motor_ISR_flag:1;
	uint16_t move_state:1;
	uint16_t start_flag:1;
	uint16_t cross_flag:1;
	uint16_t speed_up_flag:1;
	uint16_t debug_flag:1;
	uint16_t fast_flag:1;
	uint16_t lineout_flag:1;
	uint16_t err:1;
	uint16_t speed_up_start:1;
	uint16_t speed_up:1;
	uint16_t straight_run:1;


}bit_field_flag_t;

__STRUCT_EXT__ bit_field_flag_t	g_Flag;


typedef volatile struct motor_variable
{
	uint16_t	u16qep_sample;
	uint16_t	u16decel_flag;

	int16_t		int16qep_value;
	int32_t		int32accel;

	float		fp32next_acc;

	float		fp32distance_sum;
	float		fp32err_distance;
	float		fp32user_distacne;

	float		fp32user_vel;
	float		fp32next_vel;

	float		fp32current_vel[ 4 ];
	float		fp32cur_vel_avr;

	float		fp32err_vel[ 4 ];
	float		fp32err_vel_sum;

	float		fp32PID_output;
	float		fp32kp;
	float		fp32ki;
	float		fp32kd;
	float		fp32proportion_val;
	float		fp32differential_val;
	float		fp32integral_val;

	float		fp32decel_distance;
	float		fp32decel_vel;

	float		fp32gone_distance;
	float		fp32tick_distance;

}motor_vari;

__STRUCT_EXT__ motor_vari g_motor;
//__STRUCT_EXT__ motor_vari	g_motor;
//__STRUCT_EXT__ motor_vari	R_motor;
//__STRUCT_EXT__ motor_vari	L_motor;


#endif /* INC_STRUCT_H_ */
