// 遥控器控制
#include "remote_task.h"

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板
sp::DBus remote(&huart3);

// 将sp_middleware里的C++拨杆枚举转换成application层的C兼容枚举
RemoteSwitchMode convert_switch_mode(sp::DBusSwitchMode mode)
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

extern "C" void remote_task(void const * argument)
{
  (void)argument;

  remote.request();

  while (true) {
    // 使用调试(f5)查看remote内部变量的变化
    osDelay(10);
  }
}

// 读取遥控器当前快照, 供其他任务判断拨杆、摇杆和键盘状态
extern "C" bool remote_task_get_data(RemoteTaskData * data)
{
  if (data == nullptr) return false;

  const uint32_t now_ms = osKernelSysTick();
  if (!remote.is_open()) {
    // 尚未收到有效遥控器数据时, 返回安全默认值
    *data = {};
    data->sw_r = REMOTE_SWITCH_DOWN;
    data->sw_l = REMOTE_SWITCH_DOWN;
    return false;
  }

  // 复制遥控器当前快照, 供其他任务读取, 不直接暴露sp::DBus对象
  data->is_open = remote.is_open();
  data->is_alive = remote.is_alive(now_ms);
  data->sw_r = convert_switch_mode(remote.sw_r);
  data->sw_l = convert_switch_mode(remote.sw_l);
  data->ch_rh = remote.ch_rh;
  data->ch_rv = remote.ch_rv;
  data->ch_lh = remote.ch_lh;
  data->ch_lv = remote.ch_lv;
  data->ch_lu = remote.ch_lu;
  data->keyboard_value = remote.keyboard_value;

  return data->is_open;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}
