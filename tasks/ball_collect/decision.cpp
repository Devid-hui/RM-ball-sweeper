#include "tasks/ball_collect/decision.hpp"

#include <cmath>

namespace ball_collect
{
namespace
{
constexpr double k_ball_radius_m = 0.0085;

Decision continue_push(const Inputs & input)
{
  if (!input.wall_tag_fresh) {
    return {{}, "wall tag unavailable"};
  }
  if (input.dropoff_reached) return {{}, "dropoff reached"};
  return {input.push_velocity, "push to dropoff"};
}

Decision approach_ball(const Inputs & input, const BallPosition & ball)
{
  const double safe_half_width = input.mouth.width_m / 2 - k_ball_radius_m;
  if (!input.wall_tag_fresh && ball.x_m <= input.mouth.forward_m + input.mouth.visual_standoff_m + 0.01) {
    return {{}, "waiting for wall tag"};
  }
  if (input.wall_tag_fresh && input.push_path_aligned && ball.x_m <= input.mouth.forward_m + 0.05 &&
      std::abs(ball.y_m) <= safe_half_width) {
    return {{}, "ready to push", false, true};
  }
  return {input.approach_velocity, "approach ball"};
}
}  // namespace

Decision decide(const Inputs & input)
{
  if (input.emergency_stop) return {{}, "emergency stop", true};
  if (!input.motion_authorized) return {{}, "motion disabled"};
  if (!input.vision_fresh || !input.chassis_fresh) return {{}, "feedback stale", true};
  if (input.pushing_to_dropoff) return continue_push(input);
  if (input.target_temporarily_lost) return {{}, "target temporarily lost"};

  if (input.target && !input.wall_tag_fresh && input.target->x_m < input.mouth.forward_m) {
    return {{}, "ball inside fork"};
  }
  if (input.target && input.target->x_m >= input.mouth.forward_m) {
    return approach_ball(input, *input.target);
  }

  if (input.route_finished) return {{}, "route finished"};
  if (!input.wall_tag_fresh) return {{}, "wall tag unavailable"};
  return {input.search_velocity, "search"};
}
}  // namespace ball_collect
