#pragma once

#include "junior-rocket-state.hpp"
#include "farduino_constants.h"
// Must come after farduino_constants.h !
#include "rtttl_songs.h"

#include <RF24.h>

using namespace far::junior;

class StateReactions : public StateObserver
{
public:
  StateReactions(RF24& radio_nrf24)
    : _radio_nrf24(radio_nrf24)
  {}

  bool safe_to_flush_sd_card() const
  {
    return _current_state == state::IDLE || _current_state == state::ESTABLISH_GROUND_PRESSURE;
  }

  void state_changed(timestamp_t timestamp, state state) override
  {
    _current_state = state;
    switch(state)
    {
    case state::ACCELERATING:
      break;
    case state::LAUNCHED:
      _radio_nrf24.setPALevel(RF24_PA_MAX);
      tone(TONE_PIN, 440, 500);
      delay(500);
      tone(TONE_PIN, 880, 500);
      delay(500);
      tone(TONE_PIN, 1760, 500);
      break;
    case state::FALLING_:
      tone(TONE_PIN, 1500, 100);
      delay(200);
      tone(TONE_PIN, 1500, 100);
      delay(200);
      tone(TONE_PIN, 1500, 100);
      break;
    case state::LANDED:
      digitalWrite(PYRO0, LOW);
      digitalWrite(PYRO1, LOW);
      digitalWrite(PYRO2, LOW);
      digitalWrite(PYRO3, LOW);
      play_rtttl(indiana_song);
      _radio_nrf24.setPALevel(RF24_PA_HIGH);
      break;
    default:
      break;
    }
  }

private:
  RF24& _radio_nrf24;
  state _current_state;
};
