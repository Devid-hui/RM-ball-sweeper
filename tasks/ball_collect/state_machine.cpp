#include "tasks/ball_collect/state_machine.hpp"

namespace ball_collect
{
namespace
{
StepResult step_search(const Inputs & input)
{
  if (input.target == TargetStatus::Locked || input.target == TargetStatus::Aligned) {
    return {State::Approach, {}};
  }
  if (input.route_finished) return {State::Idle, {}};
  return {State::Search, input.search_velocity};
}

StepResult step_approach(const Inputs & input)
{
  if (input.target == TargetStatus::Lost) return {State::Search, {}};
  if (input.target == TargetStatus::Aligned) return {State::Collect, {}};
  if (input.target != TargetStatus::Locked) return {State::Approach, {}};
  return {State::Approach, input.approach_velocity};
}

StepResult step_collect(const Inputs & input)
{
  if (input.collection == CollectionStatus::Failed) return {State::Fault, {}};
  if (input.collection == CollectionStatus::Complete) return {State::Search, {}};
  return {State::Collect, input.collect_velocity};
}
}  // namespace

StepResult step(State state, const Inputs & input)
{
  const bool ready = input.vision_fresh && input.chassis_fresh;
  if (input.emergency_stop) return {State::Fault, {}};
  if (state == State::Fault) {
    if (input.reset_requested && ready) return {State::Idle, {}};
    return {State::Fault, {}};
  }
  if (state == State::Idle) {
    if (input.start_requested && !input.stop_requested && ready) return {State::Search, {}};
    return {State::Idle, {}};
  }

  if (!ready) return {State::Fault, {}};
  if (input.stop_requested) return {State::Idle, {}};

  switch (state) {
    case State::Search: return step_search(input);
    case State::Approach: return step_approach(input);
    case State::Collect: return step_collect(input);
    case State::Idle:
    case State::Fault: break;
  }
  return {State::Fault, {}};
}

std::string_view state_name(State state)
{
  switch (state) {
    case State::Idle: return "Idle";
    case State::Search: return "Search";
    case State::Approach: return "Approach";
    case State::Collect: return "Collect";
    case State::Fault: return "Fault";
  }
  return "Unknown";
}
}  // namespace ball_collect
