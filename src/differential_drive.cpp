#include "robotics_control/differential_drive.hpp"

namespace robotics_control {

WheelSpeeds DifferentialDriveKinematics::inverse(BodyTwist twist) const noexcept {
    const double half_track = track_width_m_ * 0.5;
    return WheelSpeeds{twist.linear_mps - twist.angular_rps * half_track,
                       twist.linear_mps + twist.angular_rps * half_track};
}

BodyTwist DifferentialDriveKinematics::forward(WheelSpeeds wheels) const noexcept {
    return BodyTwist{0.5 * (wheels.left_mps + wheels.right_mps),
                     (wheels.right_mps - wheels.left_mps) / track_width_m_};
}

}  // namespace robotics_control
