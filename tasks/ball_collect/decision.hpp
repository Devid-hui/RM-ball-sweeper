#pragma once

#include <optional>
#include <string_view>

#include "io/chassis/velocity_command.hpp"

namespace ball_collect
{
struct BallPosition
{
  double x_m;
  double y_m;
};

struct PushMouth
{
  double forward_m = 0.06;
  double width_m = 0.20;
};

struct Inputs
{
  // 上层持续提供运动许可，撤销后由操作员重新授权。
  bool motion_authorized = false;
  bool emergency_stop = false;
  bool vision_fresh = false;
  bool chassis_fresh = false;
  bool route_finished = false;

  std::optional<BallPosition> target;
  bool target_temporarily_lost = false;
  bool push_path_aligned = false;
  PushMouth mouth;

  // 由导航提供当前推球任务及墙边观测。
  bool pushing_to_dropoff = false;
  bool near_wall = false;
  bool wall_tag_fresh = false;
  bool dropoff_reached = false;

  io::chassis::VelocityCommand search_velocity;
  io::chassis::VelocityCommand approach_velocity;
  io::chassis::VelocityCommand push_velocity;
};

struct Decision
{
  io::chassis::VelocityCommand velocity;
  std::string_view reason;
  bool revoke_motion_authorization = false;
  bool begin_push = false;
};

Decision decide(const Inputs & input);
}  // namespace ball_collect
