#include "robotics_control/path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robotics_control {

PathFollower::PathFollower(PathFollowerConfig config, PidController heading_pid)
    : config_(config), heading_pid_(std::move(heading_pid)) {
    if (config_.cruise_speed_mps <= 0.0 || config_.max_angular_rps <= 0.0 ||
        config_.waypoint_tolerance_m <= 0.0 || config_.goal_tolerance_m <= 0.0 ||
        config_.heading_slowdown_rad <= 0.0) {
        throw std::invalid_argument("invalid path follower configuration");
    }
}

FollowCommand PathFollower::update(const Pose2D& pose, const std::vector<Vec2>& path, double dt_s) {
    if (path.empty()) {
        return FollowCommand{{0.0, 0.0}, 0U, true};
    }

    target_index_ = std::min(target_index_, path.size() - 1U);
    while (target_index_ + 1U < path.size() && distance(pose, path[target_index_]) <= config_.waypoint_tolerance_m) {
        ++target_index_;
    }

    const Vec2& target = path[target_index_];
    const bool on_last = target_index_ + 1U == path.size();
    if (on_last && distance(pose, target) <= config_.goal_tolerance_m) {
        return FollowCommand{{0.0, 0.0}, target_index_, true};
    }

    const double desired_heading = std::atan2(target.y - pose.y, target.x - pose.x);
    const double heading_error = normalize_angle(desired_heading - pose.yaw);
    const double angular = std::clamp(heading_pid_.update(heading_error, dt_s),
                                      -config_.max_angular_rps,
                                      config_.max_angular_rps);

    const double heading_scale = std::clamp(1.0 - std::abs(heading_error) / config_.heading_slowdown_rad, 0.12, 1.0);
    const double goal_scale = on_last ? std::clamp(distance(pose, target) / 0.75, 0.15, 1.0) : 1.0;
    const double linear = config_.cruise_speed_mps * heading_scale * goal_scale;

    return FollowCommand{{linear, angular}, target_index_, false};
}

void PathFollower::reset() noexcept {
    target_index_ = 0U;
    heading_pid_.reset();
}

}  // namespace robotics_control
