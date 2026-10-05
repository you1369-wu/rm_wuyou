#ifndef APPLICATIONS_REMOTE_TASK_H
#define APPLICATIONS_REMOTE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  REMOTE_SWITCH_DOWN,
  REMOTE_SWITCH_MID,
  REMOTE_SWITCH_UP
} RemoteSwitchMode;

typedef struct
{
  // 遥控器是否已经收到过有效数据
  bool is_open;
  // 遥控器数据是否还在持续更新
  bool is_alive;

  // 右三位开关和左三位开关
  RemoteSwitchMode sw_r;
  RemoteSwitchMode sw_l;

  // 摇杆和拨轮数据, 取值范围约为 [-1, 1]
  float ch_rh;
  float ch_rv;
  float ch_lh;
  float ch_lv;
  float ch_lu;

  // 键盘原始按键值
  uint16_t keyboard_value;
} RemoteTaskData;

void remote_task(void const * argument);
bool remote_task_get_data(RemoteTaskData * data);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_REMOTE_TASK_H
