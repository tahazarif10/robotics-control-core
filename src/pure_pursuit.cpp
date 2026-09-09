#include "robotics_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics_control {

PurePursuitController::PurePursuitController(PurePursuitConfig config) : config_(config) {
    if (config_.lookahead_m <= 0.0 || config_.max_linear_mps <= 0.0 ||
        config_.max_angular_rps <= 0.0 || config_.goal_tolerance_m <= 0.0 ||
        config_.curvature_slowdown < 0.0) {
        throw std::invalid_argument("invalid pure-pursuit configuration");
    }
}

PurePursuitCommand PurePursuitController::update(const Pose2D& pose, const std::vector<Vec2>& path) {
    if (path.empty()) {
        return PurePursuitCommand{{0.0, 0.0}, 0U, 0.0, true};
    }

    target_index_ = std::min(target_index_, path.size() - 1U);
    const Vec2& goal = path.back();
    if (distance(pose, goal) <= config_.goal_tolerance_m) {
        target_index_ = path.size() - 1U;
        return PurePursuitCommand{{0.0, 0.0}, target_index_, 0.0, true};
    }

    std::size_t nearest = target_index_;
    double nearest_distance = distance(pose, path[nearest]);
    for (std::size_t i = target_index_ + 1U; i < path.size(); ++i) {
        const double candidate_distance = distance(pose, path[i]);
        if (candidate_distance <= nearest_distance) {
            nearest = i;
            nearest_distance = candidate_distance;
        } else if (i > nearest + 2U) {
            break;
        }
    }
    target_index_ = nearest;

    std::size_t lookahead_index = nearest;
    while (lookahead_index + 1U < path.size() &&
           distance(pose, path[lookahead_index]) < config_.lookahead_m) {
        ++lookahead_index;
    }
    target_index_ = std::max(target_index_, lookahead_index);

    const Vec2& target = path[target_index_];
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double x_local = cosine * dx + sine * dy;
    const double y_local = -sine * dx + cosine * dy;
    const double lookahead_sq = std::max(dx * dx + dy * dy, 1e-9);
    const double curvature = 2.0 * y_local / lookahead_sq;

    const double curvature_scale = 1.0 / (1.0 + config_.curvature_slowdown * std::abs(curvature));
    const double facing_scale = x_local >= 0.0 ? 1.0 : 0.20;
    const double goal_scale = std::clamp(distance(pose, goal) / (2.0 * config_.lookahead_m), 0.20, 1.0);
    const double linear = config_.max_linear_mps * curvature_scale * facing_scale * goal_scale;
    const double angular = std::clamp(linear * curvature, -config_.max_angular_rps, config_.max_angular_rps);

    return PurePursuitCommand{{linear, angular}, target_index_, curvature, false};
}

}  // namespace robotics_control
