#ifndef APPLICATIONS_IMU_TASK_H
#define APPLICATIONS_IMU_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float x;
  float y;
  float z;
} ImuVector3Data;

typedef struct
{
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
} ImuEulerData;

typedef struct
{
  ImuVector3Data acc_mps2;
  ImuVector3Data gyro_rad_s;
  ImuEulerData euler;
  float temp_c;
} ImuTaskData;

void imu_task(void const * argument);
bool imu_task_get_data(ImuTaskData * data);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_IMU_TASK_H
