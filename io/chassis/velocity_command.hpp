#pragma once

namespace io::chassis
{
struct VelocityCommand
{
  double vx_m_s = 0.0;      // Forward.
  double vy_m_s = 0.0;      // Left.
  double wz_rad_s = 0.0;    // Counterclockwise from above.
};
}  // namespace io::chassis
