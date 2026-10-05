//can与电机通信，并且发送和接受电机数据（测试三）
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"

sp::CAN can1(&hcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

void run_disable_mode()
{
  // 失能模式: 所有电机发送零力矩, 保持无力状态
  motor6020.cmd(0.0f);
  motor6020.write(can1.tx_data);
  can1.send(motor6020.tx_id);
}

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    RemoteTaskData remote_data = {};
    const bool remote_ready = remote_task_get_data(&remote_data);

    // 右拨杆下档或遥控器失联时, 进入失能模式
    if ((!remote_ready) || (!remote_data.is_alive) || (remote_data.sw_r == REMOTE_SWITCH_DOWN)) {
      run_disable_mode();
    }
    else {
      motor6020.cmd(0.1f);
      motor6020.write(can1.tx_data);
      can1.send(motor6020.tx_id);
    }

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
    }
  }
}
