#include "io/chassis/mecanum_kinematics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace io::chassis;

void expect_near(double actual, double expected, const std::string & label)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error(
      label + " expected " + std::to_string(expected) + ", got " + std::to_string(actual));
  }
}

void expect_wheels(
  const WheelAngularVelocity & actual, const WheelAngularVelocity & expected,
  const std::string & label)
{
  expect_near(actual.front_left, expected.front_left, label + " FL");
  expect_near(actual.front_right, expected.front_right, label + " FR");
  expect_near(actual.rear_left, expected.rear_left, label + " RL");
  expect_near(actual.rear_right, expected.rear_right, label + " RR");
}

void expect_velocity(
  const VelocityCommand & actual, const VelocityCommand & expected, const std::string & label)
{
  expect_near(actual.vx_m_s, expected.vx_m_s, label + " vx");
  expect_near(actual.vy_m_s, expected.vy_m_s, label + " vy");
  expect_near(actual.wz_rad_s, expected.wz_rad_s, label + " wz");
}

template<typename Function>
void expect_invalid(Function run, const std::string & label)
{
  try {
    run();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error(label + " was accepted");
}
}  // namespace

int main()
{
  try {
    const MecanumKinematics mecanum(0.05, 0.30, 0.20);
    expect_wheels(mecanum.inverse({}, 6.0), {}, "stop");
    expect_wheels(mecanum.inverse({0.20, 0.0, 0.0}, 6.0), {4.0, 4.0, 4.0, 4.0}, "forward");
    expect_wheels(mecanum.inverse({0.0, 0.20, 0.0}), {-4.0, 4.0, 4.0, -4.0}, "left");
    expect_wheels(mecanum.inverse({0.0, 0.0, 0.80}), {-4.0, 4.0, -4.0, 4.0}, "counterclockwise");

    const VelocityCommand mixed{0.30, 0.10, 0.80};
    const auto mixed_wheels = mecanum.inverse(mixed);
    expect_wheels(mixed_wheels, {0.0, 12.0, 4.0, 8.0}, "mixed");
    expect_velocity(mecanum.forward(mixed_wheels), mixed, "round trip");
    const auto limited_wheels = mecanum.inverse(mixed, 6.0);
    expect_wheels(limited_wheels, {0.0, 6.0, 2.0, 4.0}, "limited");
    expect_velocity(mecanum.forward(limited_wheels), {0.15, 0.05, 0.40}, "limited round trip");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    expect_invalid([] { MecanumKinematics(0.0, 0.3, 0.2); }, "zero radius");
    expect_invalid([] { MecanumKinematics(0.05, 0.3, -0.2); }, "negative track width");
    expect_invalid([&] { mecanum.inverse({0.0, nan, 0.0}); }, "non-finite chassis speed");
    expect_invalid([&] { mecanum.inverse({}, -1.0); }, "negative limit");
    expect_invalid([&] { mecanum.forward({0.0, nan, 0.0, 0.0}); }, "non-finite wheel speed");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
