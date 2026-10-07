// CAN和电机对象定义
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);  // A电机, CAN ID 1
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);  // B电机, CAN ID 2

extern "C" void can_task(void const * argument)
{
  (void)argument;

  while (true) {
    // 当前CAN收发由control_task统一负责, 保留空闲任务入口便于CubeMX配置回退
    osDelay(1000);
  }
}
