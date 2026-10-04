// 读 BMI088 + Mahony 解算
#include "imu_task.h"

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

namespace
{
constexpr float kTaskPeriodS = 0.001F;

const float kRotationBmi088ToBoard[3][3] = {
  {0.0F, -1.0F, 0.0F},
  {1.0F, 0.0F, 0.0F},
  {0.0F, 0.0F, 1.0F},
};

sp::BMI088 bmi088(
  &hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin,
  kRotationBmi088ToBoard);
sp::Mahony imu(kTaskPeriodS);

ImuTaskData latest_imu_data = {};
bool imu_data_ready = false;

void update_imu_data_snapshot()
{
  latest_imu_data.acc_mps2.x = bmi088.acc[0];
  latest_imu_data.acc_mps2.y = bmi088.acc[1];
  latest_imu_data.acc_mps2.z = bmi088.acc[2];
  latest_imu_data.gyro_rad_s.x = bmi088.gyro[0];
  latest_imu_data.gyro_rad_s.y = bmi088.gyro[1];
  latest_imu_data.gyro_rad_s.z = bmi088.gyro[2];
  latest_imu_data.euler.roll_rad = imu.roll;
  latest_imu_data.euler.pitch_rad = imu.pitch;
  latest_imu_data.euler.yaw_rad = imu.yaw;
  latest_imu_data.temp_c = bmi088.temp;
  imu_data_ready = true;
}
}  // namespace

extern "C" void imu_task(void)
{
  osDelay(100);
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);
    update_imu_data_snapshot();

    osDelay(1);
  }
}

extern "C" bool imu_task_get_data(ImuTaskData * data)
{
  if ((data == nullptr) || !imu_data_ready) {
    return false;
  }

  *data = latest_imu_data;
  return true;
}
