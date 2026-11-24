#include "io/chassis/mecanum_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace io::chassis
{
namespace
{
bool positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_wheels(const WheelAngularVelocity & wheels)
{
  return std::isfinite(wheels.front_left) && std::isfinite(wheels.front_right) &&
    std::isfinite(wheels.rear_left) && std::isfinite(wheels.rear_right);
}
}  // namespace

MecanumKinematics::MecanumKinematics(double wheel_radius_m, double wheelbase_m, double track_width_m)
: wheel_radius_m_(wheel_radius_m), rotation_arm_m_(0.5 * wheelbase_m + 0.5 * track_width_m)
{
  if (!positive_finite(wheel_radius_m_) || !positive_finite(wheelbase_m) ||
      !positive_finite(track_width_m) || !positive_finite(rotation_arm_m_)) {
    throw std::invalid_argument("Wheel radius, wheelbase and track width must be positive finite meters");
  }
}

WheelAngularVelocity MecanumKinematics::inverse(
  const VelocityCommand & command, std::optional<double> max_wheel_rad_s) const
{
  if (!std::isfinite(command.vx_m_s) || !std::isfinite(command.vy_m_s) ||
      !std::isfinite(command.wz_rad_s)) {
    throw std::invalid_argument("Chassis velocity must be finite");
  }
  if (max_wheel_rad_s && !positive_finite(*max_wheel_rad_s)) {
    throw std::invalid_argument("Max wheel speed must be positive finite rad/s");
  }

  const double forward_m_s = command.vx_m_s;
  const double left_m_s = command.vy_m_s;
  // Convert chassis yaw rate to the tangential speed contributed at each wheel.
  const double turn_m_s = rotation_arm_m_ * command.wz_rad_s;
  WheelAngularVelocity wheels{
    (forward_m_s - left_m_s - turn_m_s) / wheel_radius_m_,
    (forward_m_s + left_m_s + turn_m_s) / wheel_radius_m_,
    (forward_m_s + left_m_s - turn_m_s) / wheel_radius_m_,
    (forward_m_s - left_m_s + turn_m_s) / wheel_radius_m_,
  };
  if (!finite_wheels(wheels)) {
    throw std::overflow_error("Wheel speed exceeds the numeric range");
  }

  if (max_wheel_rad_s) {
    const double peak = std::max({
      std::abs(wheels.front_left), std::abs(wheels.front_right),
      std::abs(wheels.rear_left), std::abs(wheels.rear_right),
    });
    if (peak > *max_wheel_rad_s) {
      // Scale the four wheels together to preserve the requested motion ratios.
      wheels.front_left = wheels.front_left / peak * *max_wheel_rad_s;
      wheels.front_right = wheels.front_right / peak * *max_wheel_rad_s;
      wheels.rear_left = wheels.rear_left / peak * *max_wheel_rad_s;
      wheels.rear_right = wheels.rear_right / peak * *max_wheel_rad_s;
    }
  }
  return wheels;
}

VelocityCommand MecanumKinematics::forward(const WheelAngularVelocity & wheels) const
{
  if (!finite_wheels(wheels)) {
    throw std::invalid_argument("Wheel speeds must be finite");
  }

  const double fl = 0.25 * wheels.front_left;
  const double fr = 0.25 * wheels.front_right;
  const double rl = 0.25 * wheels.rear_left;
  const double rr = 0.25 * wheels.rear_right;
  const VelocityCommand command{
    (fl + fr + rl + rr) * wheel_radius_m_,
    (-fl + fr + rl - rr) * wheel_radius_m_,
    (-fl + fr - rl + rr) * wheel_radius_m_ / rotation_arm_m_,
  };
  if (!std::isfinite(command.vx_m_s) || !std::isfinite(command.vy_m_s) ||
      !std::isfinite(command.wz_rad_s)) {
    throw std::overflow_error("Chassis velocity exceeds the numeric range");
  }
  return command;
}
}  // namespace io::chassis
