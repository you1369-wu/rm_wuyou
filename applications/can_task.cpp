//can与电机通信，并且发送和接受电机数据（测试三）
#include "can_task.h"

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

namespace
{
constexpr uint32_t kCanTaskPeriodMs = 1U;

volatile float motor_a_torque_cmd = 0.0F;// 调整为0.1f, 电机会旋转, 注意安全
volatile float motor_b_torque_cmd = 0.0F;

void clear_tx_data(void)
{
  for (uint8_t i = 0U; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0U;
  }
}

void update_motor_command_frame(void)
{
  clear_tx_data();

  motor_a.cmd(motor_a_torque_cmd);
  motor_b.cmd(motor_b_torque_cmd);

  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);

  can1.send(motor_a.tx_id);
}

void fill_motor_data(const sp::RM_Motor & motor, CanMotorData * data, uint32_t now_ms)
{
  data->angle_rad = motor.angle;
  data->speed_rad_s = motor.speed;
  data->torque_nm = motor.torque;
  data->temperature_c = motor.temperature;
  data->is_open = motor.is_open();
  data->is_alive = motor.is_alive(now_ms);
}
}  // namespace

extern "C" void can_task(void)
{
  can1.config();
  can1.start();

  while (true) {
    update_motor_command_frame();
    osDelay(kCanTaskPeriodMs);
  }
}

extern "C" bool can_task_get_motor_a_data(CanMotorData * data)
{
  if (data == nullptr) {
    return false;
  }

  fill_motor_data(motor_a, data, osKernelSysTick());
  return true;
}

extern "C" bool can_task_get_motor_b_data(CanMotorData * data)
{
  if (data == nullptr) {
    return false;
  }

  fill_motor_data(motor_b, data, osKernelSysTick());
  return true;
}

extern "C" void can_task_set_motor_torque(float motor_a_torque_nm, float motor_b_torque_nm)
{
  motor_a_torque_cmd = motor_a_torque_nm;
  motor_b_torque_cmd = motor_b_torque_nm;
}

extern "C" void can_task_disable_motors(void)
{
  can_task_set_motor_torque(0.0F, 0.0F);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    can1.recv();

    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp_ms);
    }

    if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
