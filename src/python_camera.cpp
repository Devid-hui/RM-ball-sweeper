#include <chrono>
#include <filesystem>
#include <memory>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "io/hikrobot/hikrobot.hpp"
#include "tools/logger.hpp"

namespace py = pybind11;

namespace
{
py::object read_frame(io::HikRobot & camera)
{
  cv::Mat frame;
  std::chrono::steady_clock::time_point received;
  bool has_frame;
  {
    py::gil_scoped_release release;
    has_frame = camera.read_for(frame, received, std::chrono::milliseconds(100));
  }
  if (!has_frame) return py::none();

  // Keep the driver's frame alive as long as Python references the array.
  auto owner = std::make_unique<cv::Mat>(std::move(frame));
  auto & mat = *owner;
  py::capsule storage(owner.get(), [](void * p) { delete static_cast<cv::Mat *>(p); });
  owner.release();
  const py::ssize_t stride = static_cast<py::ssize_t>(mat.step[0]);
  py::array_t<unsigned char> image({mat.rows, mat.cols, 3},
                                   {stride, py::ssize_t(3), py::ssize_t(1)}, mat.data, storage);
  const auto received_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch());
  return py::make_tuple(image, received_ns.count());
}
}  // namespace

PYBIND11_MODULE(ball_camera, module)
{
  py::class_<io::HikRobot>(module, "HikCamera")
    .def(py::init([](std::string serial, double exposure_ms, double gain, double fps) {
      std::filesystem::create_directories("logs");
      tools::logger();
      return std::make_unique<io::HikRobot>(exposure_ms, gain, "collector", serial, fps);
    }), py::arg("serial"), py::arg("exposure_ms") = 8.0, py::arg("gain") = 8.0, py::arg("fps") = 15.0)
    .def("read", &read_frame);
}
