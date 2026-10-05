#include "cmsis_os.h"

#include <cstdint>

#include "io/led/led.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

namespace
{
struct Rgb
{
  float r;
  float g;
  float b;
};

constexpr Rgb kFlowColors[] = {
  {1.00F, 0.00F, 0.00F},
  {0.85F, 0.45F, 0.00F},
  {0.00F, 1.00F, 0.00F},
  {0.00F, 0.65F, 0.85F},
  {0.00F, 0.00F, 1.00F},
  {0.70F, 0.00F, 0.85F},
};

constexpr float kBrightnessWave[] = {
  0.060F, 0.064F, 0.075F, 0.093F, 0.117F, 0.147F, 0.181F, 0.217F,
  0.255F, 0.293F, 0.329F, 0.363F, 0.393F, 0.417F, 0.435F, 0.446F,
  0.450F, 0.446F, 0.435F, 0.417F, 0.393F, 0.363F, 0.329F, 0.293F,
  0.255F, 0.217F, 0.181F, 0.147F, 0.117F, 0.093F, 0.075F, 0.064F,
};

class LedFlow
{
public:
  explicit LedFlow(sp::LED & led) : led_(led) {}

  void init()
  {
    led_.start();
    color_step_ = 0U;
    brightness_step_ = 0U;
    const uint32_t now = HAL_GetTick();
    last_brightness_update_ms_ = now;
    last_color_update_ms_ = now;
    apply();
  }

  void update()
  {
    const uint32_t now = HAL_GetTick();
    bool need_apply = false;

    if ((now - last_brightness_update_ms_) >= kBrightnessIntervalMs) {
      last_brightness_update_ms_ = now;
      brightness_step_ = static_cast<uint8_t>((brightness_step_ + 1U) % kBrightnessStepCount);
      need_apply = true;
    }

    if ((now - last_color_update_ms_) >= kColorIntervalMs) {
      last_color_update_ms_ = now;
      color_step_ = static_cast<uint8_t>((color_step_ + 1U) % kColorStepCount);
      need_apply = true;
    }

    if (need_apply) {
      apply();
    }
  }

private:
  static constexpr uint32_t kBrightnessIntervalMs = 40U;
  static constexpr uint32_t kColorIntervalMs = 2400U;
  static constexpr uint8_t kBrightnessStepCount =
    static_cast<uint8_t>(sizeof(kBrightnessWave) / sizeof(kBrightnessWave[0]));
  static constexpr uint8_t kColorStepCount =
    static_cast<uint8_t>(sizeof(kFlowColors) / sizeof(kFlowColors[0]));

  void apply()
  {
    const Rgb & color = kFlowColors[color_step_];
    const float brightness = kBrightnessWave[brightness_step_];
    led_.set(color.r * brightness, color.g * brightness, color.b * brightness);
  }

  sp::LED & led_;
  uint32_t last_brightness_update_ms_ = 0U;
  uint32_t last_color_update_ms_ = 0U;
  uint8_t color_step_ = 0U;
  uint8_t brightness_step_ = 0U;
};

sp::LED board_led(&htim5);
LedFlow led_flow(board_led);

constexpr uint32_t kLedTaskPeriodMs = 10U;
}  // namespace

extern "C" void led_task(void const * argument)
{
  (void)argument;

  led_flow.init();

  while (true) {
    led_flow.update();
    osDelay(kLedTaskPeriodMs);
  }
}
