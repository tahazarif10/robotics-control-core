#pragma once

#include "robotics_control/types.hpp"

namespace robotics_control {

class DifferentialDriveOdometry {
public:
    explicit DifferentialDriveOdometry(double track_width_m, Pose2D initial = {});

    [[nodiscard]] Pose2D update(double left_distance_m, double right_distance_m);
    [[nodiscard]] Pose2D pose() const noexcept { return pose_; }
    void reset(Pose2D pose = {}) noexcept { pose_ = pose; }

private:
    double track_width_m_;
    Pose2D pose_;
};

}  // namespace robotics_control
