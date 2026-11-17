#include "io/camera.hpp"

namespace io
{
// Link the original driver's base class without building the RoboMaster camera factory.
cv::Mat CameraBase::read() { return {}; }
}  // namespace io
