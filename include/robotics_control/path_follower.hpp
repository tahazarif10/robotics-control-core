#pragma once

#include "robotics_control/pid.hpp"
#include "robotics_control/types.hpp"

#include <cstddef>
#include <vector>

namespace robotics_control {

struct PathFollowerConfig {
    double cruise_speed_mps{0.8};
    double max_angular_rps{1.8};
    double waypoint_tolerance_m{0.18};
    double goal_tolerance_m{0.12};
    double heading_slowdown_rad{0.9};
};

struct FollowCommand {
    BodyTwist twist;
    std::size_t target_index{};
    bool goal_reached{};
};

class PathFollower {
public:
    PathFollower(PathFollowerConfig config, PidController heading_pid);

    [[nodiscard]] FollowCommand update(const Pose2D& pose, const std::vector<Vec2>& path, double dt_s);
    void reset() noexcept;

private:
    PathFollowerConfig config_;
    PidController heading_pid_;
    std::size_t target_index_{};
};

}  // namespace robotics_control
