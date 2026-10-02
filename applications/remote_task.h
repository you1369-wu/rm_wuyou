#ifndef APPLICATIONS_REMOTE_TASK_H
#define APPLICATIONS_REMOTE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  REMOTE_SWITCH_DOWN = 0,
  REMOTE_SWITCH_MID,
  REMOTE_SWITCH_UP,
} RemoteSwitchMode;

typedef struct
{
  float vx;
  float vy;
  float vs;
  bool left;
  bool right;
} RemoteMouseData;

typedef struct
{
  bool w;
  bool s;
  bool a;
  bool d;
  bool shift;
  bool ctrl;
  bool q;
  bool e;
  bool r;
  bool f;
  bool g;
  bool z;
  bool x;
  bool c;
  bool v;
  bool b;
} RemoteKeysData;

typedef struct
{
  float ch_rh;
  float ch_rv;
  float ch_lh;
  float ch_lv;
  float ch_lu;
  RemoteSwitchMode sw_r;
  RemoteSwitchMode sw_l;
  RemoteMouseData mouse;
  RemoteKeysData keys;
  uint16_t keyboard_value;
  bool is_open;
  bool is_alive;
} RemoteTaskData;

void remote_task(void);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_REMOTE_TASK_H
