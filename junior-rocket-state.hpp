// (c) Diez Roggisch, 2023
// SPDX-License-Identifier: MIT
#pragma once

#include <chrono>
#include <ostream>

namespace far::junior {

using timestamp_t = std::chrono::steady_clock::time_point;
using duration_t = std::chrono::steady_clock::duration;

}


namespace tfa {

std::ostream& operator<<(std::ostream&, const far::junior::duration_t&);

}

#include "timed-finite-automaton.hpp"
#include "statistics.hpp"
#include <cstdint>
#include <optional>

namespace far::junior {

using timestamp_t = std::chrono::steady_clock::time_point;
using duration_t = std::chrono::steady_clock::duration;
using namespace std::chrono_literals;


enum class state
{
  // Start state
  IDLE,
  // Compute the ground pressuer by averaging
  // pressure over a certain amount of time
  ESTABLISH_GROUND_PRESSURE,
  // After stable pressure is detected
  WAIT_FOR_LAUNCH,
  // We measured acceleration beyond the threshold,
  // but this needs to be stable for a while
  // to qualify as ACCELERATING
  ACCELERATION_DETECTED,
  // We reached the acceleration phase,
  ACCELERATING,
  // once pressure fell off enough, we declare us being
  // launched.
  LAUNCHED,
  // The motor cut out
  BURNOUT,
  // after a second, separate
  SEPARATION,
  // shortly after the separation, we are in coasting
  COASTING,
  // We detected a pressure drop, and thus reached
  // the peak.
  PEAK_REACHED,
  FALLING_,
  MEASURE_FALLING_PRESSURE1,
  MEASURE_FALLING_PRESSURE2,
  MEASURE_FALLING_PRESSURE3,
  DROUGE_OPENED,
  DROUGE_FAILED,
  LANDED,
};

struct timeouts {
  static constexpr duration_t ACCELERATION = 400ms;
  static constexpr duration_t SEPARATION_TIMEOUT = 1s;
  static constexpr duration_t MOTOR_BURNTIME = 2500ms;
  static constexpr duration_t FALLING_PRESSURE_TIMEOUT = 1s;
};

enum class pressure_drop {
  LINEAR,
  QUADRATIC,
};

// We seem to reach a shoulder of ~20m/s^2, so
// this looks safe
constexpr float LAUNCH_ACCELERATION_THRESHOLD = 15.0;
// We only get an absolute acceleration, so it is never
// below zero, instead we need to
constexpr float FREEFALL_ACCELERATION_THRESHOLD = 3.0;
// mbar difference between our ground pressure and
// the height we consider safely as "launched".
constexpr float LAUNCH_PRESSURE_DIFFERENTIAL = 5.0;
// The drop in pressure from the minimum we
// accept to say "we've peaked"
constexpr float PEAK_PRESSURE_MARGIN = .6;
// Apogee according to simulation
constexpr duration_t APOGEE_TIME = 6565ms;
// Together with APOGEE_TIME used to trigger
// chute ejection.
constexpr duration_t APOGEE_DETECTION_MARGIN = 5s;
constexpr float INITIAL_PRESSURE_VARIANCE = 1.0;
constexpr float PRESSURE_VARIANCE_THRESHOLD = 3.0;

enum class event {
  GROUND_PRESSURE_ESTABLISHED,
  // Happens when the difference between ground pressure
  // and measured pressure is bigger than LAUNCH_PRESSURE_DIFFERENTIAL
  // which means that we are higher in the air. The name is thus
  // inverted
  PRESSURE_BELOW_LAUNCH_THRESHOLD,
  PRESSURE_ABOVE_LAUNCH_THRESHOLD,
  PRESSURE_PEAK_REACHED,
  ACCELERATION_BELOW_THRESHOLD,
  ACCELERATION_ABOVE_THRESHOLD,
  ACCELERATION_AROUND_ZERO,
  EXPECTED_APOGEE_TIME_REACHED,
  PRESSURE_LINEAR,
  PRESSURE_QUADRATIC,
  RESTART_PRESSURE_MEASUREMENT,
};

#define M_UNUSED(variable) (void)variable;

struct StateObserver {
  virtual void data(timestamp_t timestamp, float pressure, float acceleration)
  {
    M_UNUSED(timestamp);
    M_UNUSED(pressure);
    M_UNUSED(acceleration);
  }

  virtual void state_changed(timestamp_t timestamp, state)
  {
    M_UNUSED(timestamp);
  }
  virtual void event_produced(timestamp_t timestamp, event)
  {
    M_UNUSED(timestamp);
  }

  virtual void elapsed(timestamp_t timestamp, duration_t elapsed)
  {
    M_UNUSED(timestamp);
    M_UNUSED(elapsed);
  }
};


class JuniorRocketState {
  using state_machine_t = tfa::TimedFiniteAutomaton<state, event, timestamp_t>;

public:

  JuniorRocketState(StateObserver&);
  JuniorRocketState(const JuniorRocketState&) = delete;
  JuniorRocketState& operator=(const JuniorRocketState&) = delete;
  JuniorRocketState(JuniorRocketState&&) = delete;

  void dot(std::ostream& os);
  void drive(timestamp_t, float, float);
  std::optional<duration_t> flighttime() const;
  std::optional<float> ground_pressure() const;

private:
  void process_pressure(float pressure);
  void produce_events(timestamp_t timestamp, float pressure, float acceleration);
  void handle_state_transition(state to, float pressure);
  void feed(timestamp_t timestamp, event);
  void assess_pressure_drop();

  state_machine_t _state_machine;

  std::optional<timestamp_t> _last_timestamp;
  std::optional<float> _ground_pressure;
  std::optional<timestamp_t> _liftoff_timestamp;

  StateObserver& _state_observer;

  std::optional<deets::statistics::ArrayStatistics<float, 2>> _ground_pressure_stats;
  std::optional<deets::statistics::ArrayStatistics<float, 10>> _peak_pressure_stats;
  float _pressure_measurements[3];
  std::optional<pressure_drop> _pressure_drop_assessment;
  std::optional<float> _peak_pressure;
};

#ifdef USE_IOSTREAM
// To allow graphviz output
std::ostream& operator<<(std::ostream&, const state&);
std::ostream& operator<<(std::ostream&, const event&);
#endif

} // namespace far::junior
