#include "tasks/ball_collect/decision.hpp"

namespace ball_collect
{
Decision decide(const Inputs & input)
{
  if (input.emergency_stop) return {{}, "emergency stop", true};
  if (input.collection == CollectionFeedback::Failed) return {{}, "collection failed", true};
  if (!input.motion_authorized) return {{}, "motion disabled"};
  if (!input.vision_fresh || !input.chassis_fresh) return {{}, "feedback stale", true};
  if (input.collection == CollectionFeedback::Complete) return {{}, "collection complete"};
  if (input.collection == CollectionFeedback::Active) return {input.collect_velocity, "collect"};

  if (input.target == TargetObservation::TemporarilyLost) return {{}, "target temporarily lost"};
  if (input.target == TargetObservation::Aligned) return {input.collect_velocity, "collect"};
  if (input.target == TargetObservation::Visible) return {input.approach_velocity, "approach"};
  if (input.route_finished) return {{}, "route finished"};
  return {input.search_velocity, "search"};
}
}  // namespace ball_collect
