#include "power_on_melody.hpp"

#include "cmsis_os.h"

namespace
{
constexpr float kTim4ClockHz = 84e6F;
constexpr float kBuzzerDuty = 0.08F;

constexpr app::PowerOnMelody::Note kPowerOnMelody[] = {
  {2000U, 120U, 80U, kBuzzerDuty},
  {2000U, 120U, 80U, kBuzzerDuty},
  {2000U, 120U, 0U, kBuzzerDuty},
};

constexpr std::size_t kPowerOnMelodyLength =
  sizeof(kPowerOnMelody) / sizeof(kPowerOnMelody[0]);
}  // namespace

namespace app
{

PowerOnMelody::PowerOnMelody(sp::Buzzer & buzzer) : buzzer_(buzzer) {}

void PowerOnMelody::play(const Note * notes, std::size_t note_count)
{
  for (std::size_t i = 0U; i < note_count; ++i) {
    play_note(notes[i]);
  }

  buzzer_.stop();
}

void PowerOnMelody::play_note(const Note & note)
{
  if ((note.frequency_hz == 0U) || (note.duration_ms == 0U)) {
    buzzer_.stop();
    delay_ms(note.duration_ms);
  }
  else {
    buzzer_.set(static_cast<float>(note.frequency_hz), note.duty);
    buzzer_.start();
    delay_ms(note.duration_ms);
    buzzer_.stop();
  }

  delay_ms(note.pause_ms);
}

void PowerOnMelody::delay_ms(uint16_t ms)
{
  if (ms == 0U) {
    return;
  }

  osDelay(ms);
}

}  // namespace app

extern "C" void app_play_power_on_melody(void)
{
  sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, kTim4ClockHz);
  app::PowerOnMelody melody(buzzer);
  melody.play(kPowerOnMelody, kPowerOnMelodyLength);
}
