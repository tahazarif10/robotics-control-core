#include "robotics_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robotics_control {
namespace {

struct Projection {
    std::size_t segment_index{};
    double t{};
    Vec2 point{};
    double distance_m{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] Projection nearest_projection(const Pose2D& pose,
                                            const std::vector<Vec2>& path,
                                            std::size_t first_segment) {
    Projection best{};
    const std::size_t start = std::min(first_segment, path.size() - 2U);

    for (std::size_t i = start; i + 1U < path.size(); ++i) {
        const Vec2& a = path[i];
        const Vec2& b = path[i + 1U];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double length_sq = vx * vx + vy * vy;
        if (length_sq <= 1e-12) {
            continue;
        }

        const double wx = pose.x - a.x;
        const double wy = pose.y - a.y;
        const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
        const Vec2 point{a.x + t * vx, a.y + t * vy};
        const double candidate_distance = distance(pose, point);
        if (candidate_distance < best.distance_m) {
            best = Projection{i, t, point, candidate_distance};
        }
    }

    return best;
}

[[nodiscard]] std::pair<Vec2, std::size_t> point_ahead(const std::vector<Vec2>& path,
                                                       const Projection& projection,
                                                       double lookahead_m) {
    Vec2 cursor = projection.point;
    std::size_t segment = projection.segment_index;
    double remaining = lookahead_m;

    while (segment + 1U < path.size()) {
        const Vec2& endpoint = path[segment + 1U];
        const double segment_remaining = distance(cursor, endpoint);
        if (segment_remaining >= remaining && segment_remaining > 1e-12) {
            const double ratio = remaining / segment_remaining;
            return {{cursor.x + ratio * (endpoint.x - cursor.x),
                     cursor.y + ratio * (endpoint.y - cursor.y)},
                    segment + 1U};
        }

        remaining -= segment_remaining;
        ++segment;
        cursor = path[segment];
    }

    return {path.back(), path.size() - 1U};
}

}  // namespace

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
    if (path.size() == 1U || distance(pose, path.back()) <= config_.goal_tolerance_m) {
        target_index_ = path.size() - 1U;
        return PurePursuitCommand{{0.0, 0.0}, target_index_, 0.0, true};
    }

    const std::size_t first_segment = target_index_ > 0U ? target_index_ - 1U : 0U;
    const Projection projection = nearest_projection(pose, path, first_segment);
    const auto [target, lookahead_index] = point_ahead(path, projection, config_.lookahead_m);
    target_index_ = std::max(target_index_, lookahead_index);

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double x_local = cosine * dx + sine * dy;
    const double y_local = -sine * dx + cosine * dy;
    const double target_distance_sq = std::max(dx * dx + dy * dy, 1e-9);
    const double curvature = 2.0 * y_local / target_distance_sq;

    const double curvature_scale = 1.0 / (1.0 + config_.curvature_slowdown * std::abs(curvature));
    const double facing_scale = x_local >= 0.0 ? 1.0 : 0.20;
    const double goal_scale = std::clamp(distance(pose, path.back()) / (2.0 * config_.lookahead_m), 0.20, 1.0);
    const double linear = config_.max_linear_mps * curvature_scale * facing_scale * goal_scale;
    const double angular = std::clamp(linear * curvature, -config_.max_angular_rps, config_.max_angular_rps);

    return PurePursuitCommand{{linear, angular}, target_index_, curvature, false};
}

}  // namespace robotics_control
