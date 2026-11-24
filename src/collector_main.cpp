#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "io/chassis/mecanum_kinematics.hpp"
#include "tasks/ball_collect/decision.hpp"

namespace
{
using io::chassis::MecanumKinematics;
using io::chassis::WheelAngularVelocity;

double parse_number(const std::string & text)
{
  std::size_t consumed = 0;
  try {
    const double value = std::stod(text, &consumed);
    if (consumed == text.size()) return value;
  } catch (const std::logic_error &) {
    // Report the input token for both invalid numbers and range errors.
  }
  throw std::invalid_argument("Invalid number: " + text);
}

void show_wheels(const WheelAngularVelocity & wheels)
{
  std::cout << " | wheel_rad_s"
            << " FL=" << wheels.front_left
            << " FR=" << wheels.front_right
            << " RL=" << wheels.rear_left
            << " RR=" << wheels.rear_right;
}

void run_demo(
  const MecanumKinematics * mecanum = nullptr, std::optional<double> max_wheel_rad_s = std::nullopt)
{
  using namespace ball_collect;
  Inputs input;
  input.vision_fresh = true;
  input.chassis_fresh = true;
  input.search_velocity = {0.2, 0.0, 0.0};
  input.approach_velocity = {0.04, 0.02, 0.0};
  input.push_velocity = {0.05, 0.0, 0.0};

  auto show_decision = [&](std::string_view event) {
    const Decision decision = decide(input);
    const auto & velocity = decision.velocity;
    if (decision.revoke_motion_authorization) input.motion_authorized = false;
    std::cout << event << " | " << decision.reason
              << " | vx=" << velocity.vx_m_s
              << " vy=" << velocity.vy_m_s
              << " wz=" << velocity.wz_rad_s
              << " begin_push=" << decision.begin_push;
    if (mecanum) show_wheels(mecanum->inverse(velocity, max_wheel_rad_s));
    std::cout << '\n';
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
  input.target = BallPosition{0.10, 0.02};
  input.push_path_aligned = true;
  show_decision("ball near fork without wall tag");
  input.wall_tag_fresh = true;
  input.pushing_to_dropoff = show_decision("ball and path aligned").begin_push;
  input.target.reset();
  show_decision("ball below camera during push");
  input.near_wall = true;
  input.wall_tag_fresh = false;
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
}

void show_usage()
{
  std::cout
    << "Usage: ball_collector --demo [radius_m wheelbase_m track_m [max_wheel_rad_s]]\n"
    << "       ball_collector --mecanum radius_m wheelbase_m track_m"
    << " vx_m_s vy_m_s wz_rad_s [max_wheel_rad_s]\n";
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string_view mode = argc > 1 ? std::string_view(argv[1]) : std::string_view{};
  try {
    if (mode == "--demo" && argc == 2) {
      run_demo();
    } else if (mode == "--demo" && (argc == 5 || argc == 6)) {
      const MecanumKinematics mecanum(parse_number(argv[2]), parse_number(argv[3]), parse_number(argv[4]));
      const auto limit = argc == 6 ? std::optional<double>(parse_number(argv[5])) : std::nullopt;
      run_demo(&mecanum, limit);
    } else if (mode == "--mecanum" && (argc == 8 || argc == 9)) {
      const MecanumKinematics mecanum(parse_number(argv[2]), parse_number(argv[3]), parse_number(argv[4]));
      const io::chassis::VelocityCommand command{
        parse_number(argv[5]), parse_number(argv[6]), parse_number(argv[7]),
      };
      const auto limit = argc == 9 ? std::optional<double>(parse_number(argv[8])) : std::nullopt;
      const auto wheels = mecanum.inverse(command, limit);
      std::cout << "mecanum";
      show_wheels(wheels);
      std::cout << '\n';
    } else {
      show_usage();
      return argc == 1 || mode == "--help" ? 0 : 2;
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
