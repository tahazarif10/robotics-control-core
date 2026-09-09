#include "robotics_control/a_star.hpp"
#include "robotics_control/differential_drive.hpp"
#include "robotics_control/map_inflation.hpp"
#include "robotics_control/odometry.hpp"
#include "robotics_control/path_follower.hpp"
#include "robotics_control/path_smoother.hpp"
#include "robotics_control/pid.hpp"
#include "robotics_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

using namespace robotics_control;

namespace {

GridMap make_map() {
    GridMap map(24, 18, 0.25);
    for (int y = 2; y < 15; ++y) {
        if (y != 8 && y != 9) {
            map.set_occupied({10, y});
        }
    }
    for (int x = 3; x < 19; ++x) {
        if (x != 15 && x != 16) {
            map.set_occupied({x, 12});
        }
    }
    return map;
}

std::vector<Vec2> metric_path(const GridMap& map, const std::vector<GridCell>& cells) {
    std::vector<Vec2> result;
    result.reserve(cells.size());
    for (const GridCell cell : cells) {
        result.push_back(map.cell_center(cell));
    }
    return result;
}

double segment_distance(const Pose2D& pose, const Vec2& a, const Vec2& b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= 1e-12) {
        return distance(pose, a);
    }
    const double wx = pose.x - a.x;
    const double wy = pose.y - a.y;
    const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
    return std::hypot(pose.x - (a.x + t * vx), pose.y - (a.y + t * vy));
}

double cross_track_error(const Pose2D& pose, const std::vector<Vec2>& path) {
    if (path.size() == 1U) {
        return distance(pose, path.front());
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1U; i < path.size(); ++i) {
        best = std::min(best, segment_distance(pose, path[i - 1U], path[i]));
    }
    return best;
}

struct Metrics {
    std::string_view controller;
    std::size_t waypoints{};
    int steps{};
    double travelled_m{};
    double max_cross_track_m{};
    double final_error_m{};
    bool collision_free{};
    bool reached{};
};

template <typename UpdateFn>
Metrics simulate(std::string_view name, const GridMap& occupancy_map, const std::vector<Vec2>& path, UpdateFn update) {
    constexpr double track_width_m = 0.42;
    constexpr double dt_s = 0.02;
    constexpr int max_steps = 5000;

    DifferentialDriveKinematics kinematics(track_width_m);
    DifferentialDriveOdometry odometry(track_width_m, {path.front().x, path.front().y, 0.0});
    Pose2D pose = odometry.pose();
    double travelled = 0.0;
    double max_cross_track = 0.0;
    bool reached = false;
    bool collision_free = true;
    int steps = 0;

    for (; steps < max_steps; ++steps) {
        const auto [twist, done] = update(pose, path, dt_s);
        max_cross_track = std::max(max_cross_track, cross_track_error(pose, path));
        if (done) {
            reached = true;
            break;
        }
        const WheelSpeeds wheels = kinematics.inverse(twist);
        const double left = wheels.left_mps * dt_s;
        const double right = wheels.right_mps * dt_s;
        travelled += 0.5 * (std::abs(left) + std::abs(right));
        pose = odometry.update(left, right);
        const GridCell cell{static_cast<int>(std::floor(pose.x / occupancy_map.resolution_m())),
                            static_cast<int>(std::floor(pose.y / occupancy_map.resolution_m()))};
        if (occupancy_map.is_occupied(cell)) {
            collision_free = false;
            break;
        }
    }

    return Metrics{name, path.size(), steps, travelled, max_cross_track, distance(pose, path.back()), collision_free, reached};
}

void print(const Metrics& metrics) {
    std::cout << metrics.controller << ',' << metrics.waypoints << ',' << metrics.steps << ','
              << metrics.travelled_m << ',' << metrics.max_cross_track_m << ',' << metrics.final_error_m << ','
              << std::boolalpha << metrics.collision_free << ',' << metrics.reached << '\n';
}

}  // namespace

int main() {
    const GridMap map = make_map();
    const GridMap planning_map = inflate_obstacles(map, 1);
    const AStarPlanner planner(true);
    const auto plan = planner.plan(planning_map, {2, 2}, {21, 15});
    if (!plan) {
        throw std::runtime_error("benchmark map unexpectedly became unreachable");
    }

    const std::vector<Vec2> raw_path = metric_path(planning_map, plan->cells);
    const GridPathSmoother smoother;
    const std::vector<GridCell> smoothed_cells = smoother.smooth(planning_map, plan->cells);
    const std::vector<Vec2> smoothed_path = metric_path(planning_map, smoothed_cells);

    PidController heading_pid({3.1, 0.05, 0.18}, {-1.8, 1.8, -0.7, 0.7});
    PathFollower follower({0.9, 1.8, 0.16, 0.10, 0.95}, std::move(heading_pid));
    const Metrics baseline = simulate("pid_waypoint", map, raw_path,
        [&follower](const Pose2D& pose, const std::vector<Vec2>& path, double dt_s) {
            const FollowCommand command = follower.update(pose, path, dt_s);
            return std::pair{command.twist, command.goal_reached};
        });

    PurePursuitController pure_pursuit({0.30, 0.95, 1.8, 0.10, 0.75});
    const Metrics pursuit = simulate("pure_pursuit_smoothed", map, smoothed_path,
        [&pure_pursuit](const Pose2D& pose, const std::vector<Vec2>& path, double) {
            const PurePursuitCommand command = pure_pursuit.update(pose, path);
            return std::pair{command.twist, command.goal_reached};
        });

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "controller,waypoints,steps,travelled_m,max_cross_track_m,final_error_m,collision_free,reached\n";
    print(baseline);
    print(pursuit);

    return baseline.reached && pursuit.reached && baseline.collision_free && pursuit.collision_free ? 0 : 1;
}
