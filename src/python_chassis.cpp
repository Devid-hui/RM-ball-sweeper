#include <array>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "io/chassis/mecanum_kinematics.hpp"

PYBIND11_MODULE(ball_chassis, module)
{
  using io::chassis::MecanumKinematics;
  pybind11::class_<MecanumKinematics>(module, "Mecanum")
    .def(pybind11::init<double, double, double>())
    .def("inverse", [](
      const MecanumKinematics & model, const std::array<double, 3> & velocity, double limit) {
      const auto wheels = model.inverse({velocity[0], velocity[1], velocity[2]}, limit);
      return std::array<double, 4>{
        wheels.front_left, wheels.front_right, wheels.rear_left, wheels.rear_right};
    })
    .def("forward", [](
      const MecanumKinematics & model, const std::array<double, 4> & wheels) {
      const auto body = model.forward({wheels[0], wheels[1], wheels[2], wheels[3]});
      return std::array<double, 3>{body.vx_m_s, body.vy_m_s, body.wz_rad_s};
    });
}
