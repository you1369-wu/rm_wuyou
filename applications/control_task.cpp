#include "control_task.hpp"

#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "pid_train.hpp"
#include "remote_task.h"

extern sp::CAN can1;
extern sp::RM_Motor motor6020;

GimbalData rm_motor_data;
RemoteSwitchMode last_sw_l = REMOTE_SWITCH_DOWN;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
PID rm_motor_pid_angle(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, true, true);
PID rm_motor_pid_speed(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, true);

void run_disable_mode()
{
	// 失能模式: 所有电机发送零力矩, 保持无力状态
	rm_motor_data.given_torque = 0.0f;
	motor6020.cmd(0.0f);
	motor6020.write(can1.tx_data);
	can1.send(motor6020.tx_id);
}

extern "C" void control_task(void const * argument)
{
	(void)argument;

	osDelay(100);

	// control_task统一负责CAN启动和电机命令发送, 避免多个任务抢控制权
	can1.config();
	can1.start();
	osDelay(100);

	// 初始化
	rm_motor_data.target_angle_set = motor6020.angle;
	rm_motor_data.target_speed_set = 0.0f;
	rm_motor_data.given_torque = 0.0f;

	while (true) {
		RemoteTaskData remote_data = {};
		const bool remote_ready = remote_task_get_data(&remote_data);

		// 右拨杆下档、遥控器未连接或遥控器失联时, 进入失能模式
		if ((!remote_ready) || (!remote_data.is_alive) || (remote_data.sw_r == REMOTE_SWITCH_DOWN)) {
			run_disable_mode();
		}
		else if (remote_data.sw_r == REMOTE_SWITCH_MID) {
			// 单速度环pid
			rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, motor6020.speed);
			rm_motor_data.given_torque = rm_motor_pid_speed.out;

			// 双环pid
			// rm_motor_pid_angle.calc(rm_motor_data.target_angle_set, motor6020.angle);
			// rm_motor_data.target_speed_set = rm_motor_pid_angle.out;
			// rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, motor6020.speed);
			// rm_motor_data.given_torque = rm_motor_pid_speed.out;

			motor6020.cmd(rm_motor_data.given_torque);
			motor6020.write(can1.tx_data);
			can1.send(motor6020.tx_id);
		}
		else {
			// 右拨杆上档复位模式还未实现, 当前先发送零力矩保护电机
			run_disable_mode();
		}

		last_sw_l = remote_ready ? remote_data.sw_l : REMOTE_SWITCH_DOWN;

		osDelay(1);
	}
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
	auto stamp_ms = osKernelSysTick();

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
		if (hcan == &hcan1) {
			can1.recv();

			// 在CAN接收中断中更新电机反馈, control_task使用最新反馈计算PID
			if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
		}
	}
}
