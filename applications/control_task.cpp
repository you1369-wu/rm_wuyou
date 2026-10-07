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
extern sp::RM_Motor motor6020;

GimbalData rm_motor_data;
RemoteSwitchMode last_sw_l = REMOTE_SWITCH_DOWN;

// 复位模式人工标定零点:
// 当C板R标与motor6020的R标机械对齐时, 记录当时的IMU yaw和电机角度并填到这里。
constexpr float kResetImuYawZero = 0.0f;
constexpr float kResetMotor6020AngleZero = 0.0f;

// 联动模式参考零点:
// 进入中档联动模式时, 记录当时的IMU yaw和motor6020角度, 后续按yaw变化量做1:1跟随。
bool link_mode_initialized = false;
float link_imu_yaw_ref = 0.0f;
float link_motor6020_ref = 0.0f;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
sp::PID rm_motor_pid_angle(0.001f, 5.0f, 2.5f, 0.0f, 5.0f, 2.5f, 1.0f, true, true);
sp::PID rm_motor_pid_speed(0.001f, 0.035f, 0.0f, 0.0f, 0.3f, 0.2f, 1.0f, false, true);

void run_disable_mode()
{
  // 失能模式: 所有电机发送零力矩, 保持无力状态
  rm_motor_data.given_torque = 0.0f;
  motor6020.cmd(0.0f);
  motor6020.write(can1.tx_data);
  can1.send(motor6020.tx_id);
}

void reset_link_mode_reference()
{
  // 离开中档联动模式后清除参考零点, 下次进入中档时重新以当前位置为零点。
  link_mode_initialized = false;
}

void run_motor6020_angle_control(float target_angle)
{
  // 电机角度双环控制: 角度环输出目标速度, 速度环输出给定力矩。
  // 这个函数只负责motor6020本体的闭环控制, 复位/联动模式只需要传入不同目标角度。
  rm_motor_data.target_angle_set = target_angle;

  //双环pid
  rm_motor_pid_angle.calc(rm_motor_data.target_angle_set, motor6020.angle);
  rm_motor_data.target_speed_set = rm_motor_pid_angle.out;

  rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, motor6020.speed);
  rm_motor_data.given_torque = rm_motor_pid_speed.out;

  motor6020.cmd(rm_motor_data.given_torque);
  motor6020.write(can1.tx_data);
  can1.send(motor6020.tx_id);
}

void run_reset_mode(float imu_yaw)
{
  // 复位模式: 让motor6020的R标跟随当前C板R标方向。
  // yaw_delta表示当前C板相对人工标定姿态转过的yaw角。
  const float yaw_delta = sp::limit_angle(imu_yaw - kResetImuYawZero);
  const float target_angle = kResetMotor6020AngleZero + yaw_delta;

  run_motor6020_angle_control(target_angle);
}

void run_link_mode(float imu_yaw, RemoteSwitchMode sw_l)
{
  (void)sw_l;

  // 2a: C板绕yaw轴转动时, motor6020按1:1比例跟随。
  // 第一次进入中档时记录当前姿态和电机角度, 避免切入联动模式时电机突然回旧零位。
  if (!link_mode_initialized) {
    link_imu_yaw_ref = imu_yaw;
    link_motor6020_ref = motor6020.angle;
    link_mode_initialized = true;
  }

  const float yaw_delta = sp::limit_angle(imu_yaw - link_imu_yaw_ref);
  const float target_angle = link_motor6020_ref + yaw_delta;

  run_motor6020_angle_control(target_angle);
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
  rm_motor_data.target_angle_set = motor6020.angle + 0.5f;
  rm_motor_data.target_speed_set = 15.0f;
  rm_motor_data.given_torque = 0.03f;

  while (true) {
    RemoteTaskData remote_data = {};
    const bool remote_ready = remote_task_get_data(&remote_data);
    ImuTaskData imu_data = {};
    const bool imu_ready = imu_task_get_data(&imu_data);

    // 右拨杆下档、遥控器未连接或遥控器失联时, 进入失能模式
    if ((!remote_ready) || (!remote_data.is_alive) || (remote_data.sw_r == REMOTE_SWITCH_DOWN)) {
      reset_link_mode_reference();
      run_disable_mode();
    }
    else if (remote_data.sw_r == REMOTE_SWITCH_MID) {
      if (imu_ready) {
        run_link_mode(imu_data.euler.yaw_rad, remote_data.sw_l);
      }
      else {
        // 没有IMU姿态时无法计算联动目标角度, 先失能保护电机
        reset_link_mode_reference();
        run_disable_mode();
      }
    }
    else if (remote_data.sw_r == REMOTE_SWITCH_UP) {
      reset_link_mode_reference();
      if (imu_ready) {
        run_reset_mode(imu_data.euler.yaw_rad);
      }
      else {
        // 没有IMU姿态时无法计算R标方向, 先失能保护电机
        run_disable_mode();
      }
    }
    else {
      // 未知拨杆状态, 先发送零力矩保护电机
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
