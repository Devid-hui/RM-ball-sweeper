#pragma once

#include <optional>

#include "io/chassis/velocity_command.hpp"

namespace io::chassis
{
struct WheelAngularVelocity
{
  // rad/s, positive when the wheel rolls toward the front of the chassis.
  double front_left = 0.0;
  double front_right = 0.0;
  double rear_left = 0.0;
  double rear_right = 0.0;
};

class MecanumKinematics
{
public:
  // X roller layout, with the chassis origin at the center of the four wheels.
  // Wheelbase and track are full center-to-center distances.
  MecanumKinematics(double wheel_radius_m, double wheelbase_m, double track_width_m);

  WheelAngularVelocity inverse(
    const VelocityCommand & command, std::optional<double> max_wheel_rad_s = std::nullopt) const;
  VelocityCommand forward(const WheelAngularVelocity & wheels) const;

private:
  double wheel_radius_m_;
  double rotation_arm_m_;
};
}  // namespace io::chassis
