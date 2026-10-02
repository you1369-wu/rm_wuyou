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
}  // namespace

extern "C" void imu_task(void)
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    osDelay(1);
  }
}