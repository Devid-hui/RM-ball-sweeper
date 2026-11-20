#pragma once

#include <string_view>

#include "io/chassis/velocity_command.hpp"

namespace ball_collect
{
enum class TargetObservation { NoConfirmedTarget, Visible, TemporarilyLost, Aligned };
enum class CollectionFeedback { Inactive, Active, Complete, Failed };

struct Inputs
{
  // 上层提供持续的运动许可。故障后应撤销，待人工重新授权。
  bool motion_authorized = false;
  bool emergency_stop = false;
  bool vision_fresh = false;
  bool chassis_fresh = false;
  bool route_finished = false;
  TargetObservation target = TargetObservation::NoConfirmedTarget;
  CollectionFeedback collection = CollectionFeedback::Inactive;

  io::chassis::VelocityCommand search_velocity;
  io::chassis::VelocityCommand approach_velocity;
  io::chassis::VelocityCommand collect_velocity;
};

struct Decision
{
  io::chassis::VelocityCommand velocity;
  std::string_view reason;
  bool revoke_motion_authorization = false;
};

Decision decide(const Inputs & input);
}  // namespace ball_collect
