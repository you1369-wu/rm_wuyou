#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

typedef struct
{
	float target_angle_set;	 // 绝对角度的目标值，rad
	float target_speed_set;	 // 绝对速度的目标值，rad/s
	float given_torque;		 // 电机给定的力矩，Nm
} GimbalData;

#endif	// CONTROL_TASK_HPP
