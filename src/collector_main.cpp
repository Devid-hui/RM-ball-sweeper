#include <iostream>
#include <string_view>

#include "tasks/ball_collect/decision.hpp"

int main(int argc, char * argv[])
{
  if (argc != 2 || std::string_view(argv[1]) != "--demo") {
    std::cout << "Usage: ball_collector --demo\n";
    return argc == 1 ? 0 : 2;
  }

  using namespace ball_collect;
  Inputs input;
  input.vision_fresh = true;
  input.chassis_fresh = true;
  input.search_velocity = {0.2, 0.0, 0.0};
  input.approach_velocity = {0.1, 0.02, 0.0};
  input.push_velocity = {0.05, 0.0, 0.0};

  auto show_decision = [&](std::string_view event) {
    const Decision decision = decide(input);
    const auto & velocity = decision.velocity;
    if (decision.revoke_motion_authorization) input.motion_authorized = false;
    std::cout << event << " | " << decision.reason
              << " | vx=" << velocity.vx_m_s
              << " vy=" << velocity.vy_m_s
              << " wz=" << velocity.wz_rad_s
              << " begin_push=" << decision.begin_push << '\n';
    return decision;
  };

  show_decision("before authorization");
  input.motion_authorized = true;
  show_decision("start");
  input.target = BallPosition{0.30, 0.12};
  show_decision("ball outside fork");
  input.target_temporarily_lost = true;
  show_decision("target briefly out of view");
  input.target_temporarily_lost = false;
  input.target = BallPosition{0.30, 0.02};
  input.push_path_aligned = true;
  input.pushing_to_dropoff = show_decision("ball and path aligned").begin_push;
  input.target.reset();
  show_decision("ball below camera during push");
  input.near_wall = true;
  show_decision("wall tag lost");
  input.wall_tag_fresh = true;
  show_decision("wall tag restored");
  input.dropoff_reached = true;
  show_decision("dropoff reached");
  input.pushing_to_dropoff = false;
  input.near_wall = false;
  input.dropoff_reached = false;
  show_decision("navigation requests search again");

  input.vision_fresh = false;
  show_decision("vision stale");
  input.vision_fresh = true;
  show_decision("vision restored");
  input.motion_authorized = true;
  show_decision("operator authorized motion");
  input.emergency_stop = true;
  show_decision("emergency stop pressed");
  return 0;
}
