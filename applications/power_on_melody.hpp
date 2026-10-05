#ifndef APPLICATIONS_POWER_ON_MELODY_HPP
#define APPLICATIONS_POWER_ON_MELODY_HPP

#include <cstddef>
#include <cstdint>

#include "io/buzzer/buzzer.hpp"

namespace app
{

class PowerOnMelody
{
public:
  struct Note
  {
    uint16_t frequency_hz;
    uint16_t duration_ms;
    uint16_t pause_ms;
    float duty;
  };

  explicit PowerOnMelody(sp::Buzzer & buzzer);

  void play(const Note * notes, std::size_t note_count);

private:
  void play_note(const Note & note);
  void delay_ms(uint16_t ms);

  sp::Buzzer & buzzer_;
};

}  // namespace app

extern "C" void app_play_power_on_melody(void);

#endif  // APPLICATIONS_POWER_ON_MELODY_HPP
