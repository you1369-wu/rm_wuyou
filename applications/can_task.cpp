// CAN和电机对象定义
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

extern "C" void can_task(void const * argument)
{
  (void)argument;

  while (true) {
    // 当前CAN收发由control_task统一负责, 保留空闲任务入口便于CubeMX配置回退
    osDelay(1000);
  }
}
