#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

extern sp::RM_Motor motor6020;
extern GimbalData rm_motor_data;

sp::Plotter plotter(&huart1, false);

// clang-format off
extern "C" void plot_task(void const * argument)
{
	(void)argument;

	while (1) {
		// 角度环调试: 目标角度、实际角度、角度环输出速度、速度环输出力矩、电机反馈在线
		plotter.plot(
			rm_motor_data.target_angle_set,
			sp::limit_angle(motor6020.angle),
			rm_motor_data.target_speed_set,
			rm_motor_data.given_torque);

		osDelay(5);
	}
}
// clang-format on
