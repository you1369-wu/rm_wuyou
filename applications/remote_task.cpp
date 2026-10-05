// 遥控器控制
#include "remote_task.h"

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

namespace
{
constexpr uint32_t kRemoteTaskPeriodMs = 10U;

sp::DBus remote(&huart3);

RemoteTaskData latest_remote_data = {};
bool remote_data_ready = false;
uint32_t remote_rx_count = 0U;
uint32_t remote_frame_count = 0U;
uint32_t remote_error_count = 0U;
uint32_t remote_last_error = 0U;
uint32_t remote_last_rx_event = 0U;
uint32_t remote_last_rx_ms = 0U;
uint16_t remote_last_rx_size = 0U;

RemoteSwitchMode to_remote_switch(sp::DBusSwitchMode mode)
{
  switch (mode) {
    case sp::DBusSwitchMode::UP:
      return REMOTE_SWITCH_UP;
    case sp::DBusSwitchMode::MID:
      return REMOTE_SWITCH_MID;
    case sp::DBusSwitchMode::DOWN:
    default:
      return REMOTE_SWITCH_DOWN;
  }
}

void update_remote_data_snapshot(uint32_t now_ms)
{
  latest_remote_data.rx_count = remote_rx_count;
  latest_remote_data.frame_count = remote_frame_count;
  latest_remote_data.error_count = remote_error_count;
  latest_remote_data.last_error = remote_last_error;
  latest_remote_data.last_rx_event = remote_last_rx_event;
  latest_remote_data.last_rx_ms = remote_last_rx_ms;
  latest_remote_data.last_rx_size = remote_last_rx_size;
  latest_remote_data.is_open = remote.is_open();
  latest_remote_data.is_alive = remote.is_alive(now_ms);

  if (remote.is_open()) {
    latest_remote_data.ch_rh = remote.ch_rh;
    latest_remote_data.ch_rv = remote.ch_rv;
    latest_remote_data.ch_lh = remote.ch_lh;
    latest_remote_data.ch_lv = remote.ch_lv;
    latest_remote_data.ch_lu = remote.ch_lu;
    latest_remote_data.sw_r = to_remote_switch(remote.sw_r);
    latest_remote_data.sw_l = to_remote_switch(remote.sw_l);
    latest_remote_data.mouse.vx = remote.mouse.vx;
    latest_remote_data.mouse.vy = remote.mouse.vy;
    latest_remote_data.mouse.vs = remote.mouse.vs;
    latest_remote_data.mouse.left = remote.mouse.left;
    latest_remote_data.mouse.right = remote.mouse.right;
    latest_remote_data.keys.w = remote.keys.w;
    latest_remote_data.keys.s = remote.keys.s;
    latest_remote_data.keys.a = remote.keys.a;
    latest_remote_data.keys.d = remote.keys.d;
    latest_remote_data.keys.shift = remote.keys.shift;
    latest_remote_data.keys.ctrl = remote.keys.ctrl;
    latest_remote_data.keys.q = remote.keys.q;
    latest_remote_data.keys.e = remote.keys.e;
    latest_remote_data.keys.r = remote.keys.r;
    latest_remote_data.keys.f = remote.keys.f;
    latest_remote_data.keys.g = remote.keys.g;
    latest_remote_data.keys.z = remote.keys.z;
    latest_remote_data.keys.x = remote.keys.x;
    latest_remote_data.keys.c = remote.keys.c;
    latest_remote_data.keys.v = remote.keys.v;
    latest_remote_data.keys.b = remote.keys.b;
    latest_remote_data.keyboard_value = remote.keyboard_value;
  }

  remote_data_ready = (remote_rx_count > 0U);
}
}  // namespace

extern "C" void remote_task(void)
{
  remote.request();

  while (true) {
    update_remote_data_snapshot(osKernelSysTick());
    osDelay(kRemoteTaskPeriodMs);
  }
}

extern "C" bool remote_task_get_data(RemoteTaskData * data)
{
  if ((data == nullptr) || !remote_data_ready) {
    return false;
  }

  *data = latest_remote_data;
  return true;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  const uint32_t stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote_rx_count++;
    remote_last_rx_event = HAL_UARTEx_GetRxEventType(huart);
    remote_last_rx_ms = stamp_ms;
    remote_last_rx_size = Size;
    remote.update(Size, stamp_ms);
    if (Size == sp::DBUS_BUFF_SIZE) {
      remote_frame_count++;
    }
    update_remote_data_snapshot(stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote_error_count++;
    remote_last_error = huart->ErrorCode;
    remote.request();
  }
}
