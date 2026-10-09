//负责can的收发和电机控制
#include "control_task.hpp"

#include "can.h"
#include "cmsis_os.h"
#include "imu_task.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "pid_train.hpp"
#include "remote_task.h"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::CAN can1;
extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

GimbalData motor_a_data;
GimbalData motor_b_data;

// 复位模式人工标定零点:
// 当C板与两台电机的R标机械对齐时, 记录当时的IMU yaw和两台电机角度。
constexpr float kResetImuYawZero = 0.0f;
constexpr float kResetMotorAAngleZero = 2.86164117f;
constexpr float kResetMotorBAngleZero = 2.10692263f;

// 联动模式参考零点:
// 进入中档时记录三者的当前角度, 以此作为本次联动的参考零点。
bool link_mode_initialized = false;
float link_imu_yaw_ref = 0.0f;
float link_motor_a_ref = 0.0f;
float link_motor_b_ref = 0.0f;
float link_motor_b_ratio = 0.5f;
sp::AngleUnwrapper imu_yaw_unwrapper;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
sp::PID motor_a_pid_angle(0.001f, 5.0f, 2.5f, 0.0f, 5.0f, 2.5f, 1.0f, true, true);
sp::PID motor_a_pid_speed(0.001f, 0.035f, 0.0f, 0.0f, 0.3f, 0.2f, 1.0f, false, true);
sp::PID motor_b_pid_angle(0.001f, 5.0f, 2.5f, 0.0f, 5.0f, 2.5f, 1.0f, true, true);
sp::PID motor_b_pid_speed(0.001f, 0.035f, 0.0f, 0.0f, 0.3f, 0.2f, 1.0f, false, true);

void run_disable_mode()
{
  // 失能模式: 所有电机发送零力矩, 保持无力状态
  motor_a_data.given_torque = 0.0f;
  motor_b_data.given_torque = 0.0f;
  motor_a_data.target_speed_set = 0.0f;
  motor_b_data.target_speed_set = 0.0f;
  motor_a_pid_angle.clear();
  motor_a_pid_speed.clear();
  motor_b_pid_angle.clear();
  motor_b_pid_speed.clear();
  motor_a.cmd(0.0f);
  motor_b.cmd(0.0f);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

void reset_link_mode_reference()
{
  // 离开中档联动模式后清除参考零点, 下次进入中档时重新以当前位置为零点。
  link_mode_initialized = false;
}

void run_motor_angle_control(float target_angle_a, float target_angle_b)
{
  // 两台电机各自运行角度/速度双环PID, 控制量写入同一帧后发送。
  motor_a_data.target_angle_set = target_angle_a;
  motor_a_pid_angle.calc(target_angle_a, motor_a.angle);
  motor_a_data.target_speed_set = motor_a_pid_angle.out;
  motor_a_pid_speed.calc(motor_a_data.target_speed_set, motor_a.speed);
  motor_a_data.given_torque = motor_a_pid_speed.out;

  motor_b_data.target_angle_set = target_angle_b;
  motor_b_pid_angle.calc(target_angle_b, motor_b.angle);
  motor_b_data.target_speed_set = motor_b_pid_angle.out;
  motor_b_pid_speed.calc(motor_b_data.target_speed_set, motor_b.speed);
  motor_b_data.given_torque = motor_b_pid_speed.out;

  motor_a.cmd(motor_a_data.given_torque);
  motor_b.cmd(motor_b_data.given_torque);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

void run_reset_mode(float imu_yaw)
{
  // 复位模式: 两台电机的R标分别对齐当前C板R标方向。
  const float yaw_delta = sp::limit_angle(imu_yaw - kResetImuYawZero);
  run_motor_angle_control(kResetMotorAAngleZero + yaw_delta, kResetMotorBAngleZero + yaw_delta);
}

void run_link_mode(float imu_yaw, RemoteSwitchMode sw_l)
{
  float ratio_b = 0.0f;
  switch (sw_l) {
    case REMOTE_SWITCH_DOWN:
      ratio_b = 0.5f;
      break;
    case REMOTE_SWITCH_MID:
      ratio_b = -1.0f;
      break;
    case REMOTE_SWITCH_UP:
      ratio_b = 3.0f;
      break;
    default:
      break;
  }

  // 首次进入联动模式时, 以当前姿态和两台电机位置作为参考点。
  if (!link_mode_initialized) {
    link_imu_yaw_ref = imu_yaw;
    link_motor_a_ref = motor_a.angle;
    link_motor_b_ref = motor_b.angle;
    link_motor_b_ratio = ratio_b;
    link_mode_initialized = true;
  }
  else if (ratio_b != link_motor_b_ratio) {
    // 换档时沿用旧比例计算当前目标, 再以此为新参考点, 防止目标角度跳变。
    const float yaw_delta = imu_yaw - link_imu_yaw_ref;
    link_motor_a_ref += yaw_delta;
    link_motor_b_ref += link_motor_b_ratio * yaw_delta;
    link_imu_yaw_ref = imu_yaw;
    link_motor_b_ratio = ratio_b;
  }

  const float yaw_delta = imu_yaw - link_imu_yaw_ref;
  run_motor_angle_control(link_motor_a_ref + yaw_delta, link_motor_b_ref + ratio_b * yaw_delta);
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
  motor_a_data.target_angle_set = motor_a.angle;
  motor_b_data.target_angle_set = motor_b.angle;

  while (true) {
    RemoteTaskData remote_data = {};
    const bool remote_ready = remote_task_get_data(&remote_data);
    ImuTaskData imu_data = {};
    const bool imu_ready = imu_task_get_data(&imu_data);
    const float imu_yaw = imu_ready ? imu_yaw_unwrapper.update(imu_data.euler.yaw_rad) : 0.0f;
    const uint32_t now_ms = osKernelSysTick();
    const bool motors_ready = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

    // 右拨杆下档、遥控器未连接或遥控器失联时, 进入失能模式
    if ((!remote_ready) || (!remote_data.is_alive) || (remote_data.sw_r == REMOTE_SWITCH_DOWN)) {
      reset_link_mode_reference();
      run_disable_mode();
    }
    else if (remote_data.sw_r == REMOTE_SWITCH_MID) {
      if (imu_ready && motors_ready) {
        run_link_mode(imu_yaw, remote_data.sw_l);
      }
      else {
        // 姿态或任一电机反馈缺失时, 不运行角度闭环。
        reset_link_mode_reference();
        run_disable_mode();
      }
    }
    else if (remote_data.sw_r == REMOTE_SWITCH_UP) {
      reset_link_mode_reference();
      if (imu_ready && motors_ready) {
        run_reset_mode(imu_yaw);
      }
      else {
        // 姿态或任一电机反馈缺失时, 不运行角度闭环。
        run_disable_mode();
      }
    }
    else {
      // 未知拨杆状态, 先发送零力矩保护电机
      run_disable_mode();
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

      // 在CAN接收中断中更新电机反馈, control_task使用最新反馈计算PID
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
