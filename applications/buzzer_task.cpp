#include "cmsis_os.h"
#include "power_on_melody.hpp"

namespace
{
constexpr uint32_t kBuzzerIdlePeriodMs = 1000U;
}

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;

  app_play_power_on_melody();

  while (true) {
    osDelay(kBuzzerIdlePeriodMs);
  }
}
