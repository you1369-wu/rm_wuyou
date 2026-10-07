#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

extern sp::RM_Motor motor_a;
extern GimbalData motor_a_data;

sp::Plotter plotter(&huart1, false);

// clang-format off
extern "C" void plot_task(void const * argument)
{
	(void)argument;

	while (1) {
		// A电机角度环调试: 目标角度、实际角度、目标速度、给定力矩
		plotter.plot(
			motor_a_data.target_angle_set,
			sp::limit_angle(motor_a.angle),
			motor_a_data.target_speed_set,
			motor_a_data.given_torque);

		osDelay(5);
	}
}
// clang-format on
