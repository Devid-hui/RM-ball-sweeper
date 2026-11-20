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
  input.collect_velocity = {0.05, 0.0, 0.0};

  auto show_decision = [&](std::string_view event) {
    const auto [velocity, reason, revoke_motion_authorization] = decide(input);
    if (revoke_motion_authorization) input.motion_authorized = false;
    std::cout << event << " | " << reason
              << " | vx=" << velocity.vx_m_s
              << " vy=" << velocity.vy_m_s
              << " wz=" << velocity.wz_rad_s << '\n';
  };

  show_decision("before authorization");
  input.motion_authorized = true;
  show_decision("start");
  input.target = TargetObservation::Visible;
  show_decision("target confirmed");
  input.target = TargetObservation::TemporarilyLost;
  show_decision("target briefly out of view");
  input.target = TargetObservation::Aligned;
  show_decision("target aligned");
  input.collection = CollectionFeedback::Active;
  input.target = TargetObservation::NoConfirmedTarget;
  show_decision("mechanism collecting");
  input.collection = CollectionFeedback::Complete;
  show_decision("mechanism finished");
  input.collection = CollectionFeedback::Inactive;
  show_decision("new search");

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
