#include "robotics_control/odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics_control {

DifferentialDriveOdometry::DifferentialDriveOdometry(double track_width_m, Pose2D initial)
    : track_width_m_(track_width_m), pose_(initial) {
    if (track_width_m <= 0.0) {
        throw std::invalid_argument("track width must be positive");
    }
}

Pose2D DifferentialDriveOdometry::update(double left_distance_m, double right_distance_m) {
    const double ds = 0.5 * (left_distance_m + right_distance_m);
    const double dtheta = (right_distance_m - left_distance_m) / track_width_m_;

    if (std::abs(dtheta) < 1e-9) {
        pose_.x += ds * std::cos(pose_.yaw);
        pose_.y += ds * std::sin(pose_.yaw);
    } else {
        const double mid_yaw = pose_.yaw + 0.5 * dtheta;
        const double chord = 2.0 * (ds / dtheta) * std::sin(0.5 * dtheta);
        pose_.x += chord * std::cos(mid_yaw);
        pose_.y += chord * std::sin(mid_yaw);
    }

    pose_.yaw = normalize_angle(pose_.yaw + dtheta);
    return pose_;
}

}  // namespace robotics_control
