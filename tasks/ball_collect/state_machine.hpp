#pragma once

#include <string_view>

#include "io/chassis/velocity_command.hpp"

namespace ball_collect
{
enum class State { Idle, Search, Approach, Collect, Fault };
enum class TargetStatus { Unconfirmed, Locked, TemporarilyLost, Lost, Aligned };
enum class CollectionStatus { InProgress, Complete, Failed };

struct Inputs
{
  bool start_requested = false;
  bool stop_requested = false;
  bool reset_requested = false;
  bool emergency_stop = false;

  bool vision_fresh = false;
  bool chassis_fresh = false;
  bool route_finished = false;
  TargetStatus target = TargetStatus::Unconfirmed;
  CollectionStatus collection = CollectionStatus::InProgress;

  io::chassis::VelocityCommand search_velocity;
  io::chassis::VelocityCommand approach_velocity;
  io::chassis::VelocityCommand collect_velocity;
};

struct StepResult
{
  State next_state;
  io::chassis::VelocityCommand velocity;
};

StepResult step(State state, const Inputs & input);
std::string_view state_name(State state);
}  // namespace ball_collect
