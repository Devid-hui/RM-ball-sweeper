#include <iostream>
#include <string_view>

#include "tasks/ball_collect/state_machine.hpp"

int main(int argc, char * argv[])
{
  if (argc != 2 || std::string_view(argv[1]) != "--demo") {
    std::cout << "Usage: ball_collector --demo\n";
    return argc == 1 ? 0 : 2;
  }

  using namespace ball_collect;
  State state = State::Idle;
  Inputs input;
  input.vision_fresh = true;
  input.chassis_fresh = true;
  input.search_velocity = {0.2, 0.0, 0.0};
  input.approach_velocity = {0.1, 0.02, 0.0};
  input.collect_velocity = {0.05, 0.0, 0.0};

  auto tick = [&](std::string_view event) {
    const StepResult result = step(state, input);
    const auto & velocity = result.velocity;
    std::cout << event << " | " << state_name(state)
              << " -> " << state_name(result.next_state)
              << " | vx=" << velocity.vx_m_s
              << " vy=" << velocity.vy_m_s
              << " wz=" << velocity.wz_rad_s << '\n';
    state = result.next_state;
  };

  input.start_requested = true;
  tick("start");
  input.start_requested = false;
  tick("searching");

  input.target = TargetStatus::Locked;
  tick("target confirmed");
  tick("approaching");
  input.target = TargetStatus::TemporarilyLost;
  tick("target temporarily lost");
  input.target = TargetStatus::Aligned;
  tick("target aligned");

  tick("collecting");
  input.collection = CollectionStatus::Complete;
  input.target = TargetStatus::Unconfirmed;
  tick("collection complete");

  input.vision_fresh = false;
  tick("vision stale");
  input.vision_fresh = true;
  tick("vision restored");
  input.reset_requested = true;
  tick("manual reset");
  return 0;
}
