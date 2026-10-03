#ifndef APPLICATIONS_CAN_TASK_H
#define APPLICATIONS_CAN_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float angle_rad;
  float speed_rad_s;
  float torque_nm;
  uint8_t temperature_c;
  bool is_open;
  bool is_alive;
} CanMotorData;

void can_task(void);

bool can_task_get_motor_a_data(CanMotorData * data);
bool can_task_get_motor_b_data(CanMotorData * data);

void can_task_set_motor_torque(float motor_a_torque_nm, float motor_b_torque_nm);
void can_task_disable_motors(void);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_CAN_TASK_H
