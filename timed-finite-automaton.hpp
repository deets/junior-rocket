// (c) Diez Roggisch, 2023
// SPDX-License-Identifier: MIT
#pragma once
#ifdef USE_IOSTREAM
#include <ostream>
#endif
#include <unordered_map>

namespace tfa {

template<typename State, typename Event, typename TimePoint>
class TimedFiniteAutomaton {
public:
  using Duration = decltype(TimePoint{} - TimePoint{});

  TimedFiniteAutomaton(State start_state)
    : _start_state{start_state}
    , _state{start_state}
    , _state_change{}
    , _now{}
  {}

  State state() { return _state; }

  void add_transition(State from, Event what, State to)
  {
    _event_transitions[from][what] = to;
  }

  void add_transition(State from, Duration after, State to)
  {
    _timeout_transitions[from] = std::make_pair(after, to);
  }

  bool elapsed(Duration duration)
  {
    _now += duration;
    const auto elapsed = _now - _state_change;

    if(_timeout_transitions.count(_state))
    {
      auto[timeout, to] = _timeout_transitions[_state];
      if(elapsed >= timeout)
      {
        _state = to;
        _state_change = _now;
        return true;
      }
    }
    return false;
  }

  bool feed(Event what)
  {
    if(_event_transitions.count(_state))
    {
      auto& candidate = _event_transitions[_state];
      if(candidate.count(what))
      {
        _state = candidate[what];
        _state_change = _now;
        return true;
      }
    }
    return false;
  }
#ifdef USE_IOSTREAM
  void dot(std::ostream& os, const char* time_signature) {
    os << "digraph timed_finite_automaton {\n";
    // Render our start state
    os << "node [shape = doublecircle];\n";
    // if start is current, mark it as such
    if(_start_state == _state) {
      os << "node [style = filled];\n";
      os << _start_state << ";\n";
    }
    else
    {
      // _start_state is not current, so spit it out
      os << _start_state << ";\n";
      // Explicitly render the active state
      os << "node [shape = circle, style = filled];\n";
      os << _state << ";\n";
    }
    // Reset node state for all other nodes
    os << "node [shape = circle, style = \"\"];\n";
    for(const auto& transition : _timeout_transitions)
    {
      const auto [from, edge] = transition;
      const auto [timeout, to] = edge;
      os << from << "->" << to << "[label = \"" << timeout;
      os << time_signature << "\"];\n";
    }
    for(const auto& transition : _event_transitions)
    {
      const auto& [from, edges] = transition;
      for(const auto& edge : edges) {
        const auto [event, to] = edge;
        os << from << "->" << to << "[label = \"" << event << "\"];\n";
      }
    }
    os << "}\n";
  }
#endif
private:
  State _start_state, _state;
  TimePoint _state_change;
  TimePoint _now;

  std::unordered_map<State, std::unordered_map<Event, State>> _event_transitions;
  std::unordered_map<State, std::pair<Duration, State>> _timeout_transitions;
};

} // namespace tfa
