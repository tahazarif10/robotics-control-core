#include "robotics_control/a_star.hpp"
#include "robotics_control/differential_drive.hpp"
#include "robotics_control/map_inflation.hpp"
#include "robotics_control/odometry.hpp"
#include "robotics_control/path_smoother.hpp"
#include "robotics_control/pure_pursuit.hpp"

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace robotics_control;

namespace {

void expect_true(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_near(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
    }
}

void test_map_inflation_expands_obstacles() {
    GridMap map(5, 5, 0.25);
    map.set_occupied({2, 2});
    const GridMap inflated = inflate_obstacles(map, 1);
    expect_true(inflated.is_occupied({2, 2}), "source obstacle must remain occupied");
    expect_true(inflated.is_occupied({1, 1}) && inflated.is_occupied({3, 3}), "one-cell inflation must include diagonal neighbors");
    expect_true(!inflated.is_occupied({0, 0}), "inflation radius must remain bounded");
}

void test_map_inflation_zero_is_identity() {
    GridMap map(4, 4, 0.5);
    map.set_occupied({1, 2});
    const GridMap inflated = inflate_obstacles(map, 0);
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            expect_true(inflated.is_occupied({x, y}) == map.is_occupied({x, y}), "zero inflation must preserve occupancy");
        }
    }
}

void test_path_smoother_reduces_open_path() {
    GridMap map(8, 8);
    const std::vector<GridCell> path{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}};
    const GridPathSmoother smoother;
    const auto smoothed = smoother.smooth(map, path);
    expect_true(smoothed.size() == 2U, "open diagonal path should collapse to endpoints");
    expect_true(smoothed.front() == path.front() && smoothed.back() == path.back(), "smoother must preserve endpoints");
}

void test_path_smoother_preserves_obstacle_clearance() {
    GridMap map(6, 6);
    map.set_occupied({2, 2});
    const std::vector<GridCell> path{{0, 1}, {1, 1}, {2, 1}, {3, 1}, {3, 2}, {3, 3}, {4, 3}, {5, 3}};
    const GridPathSmoother smoother;
    const auto smoothed = smoother.smooth(map, path);
    expect_true(smoothed.size() > 2U, "obstacle should prevent direct endpoint shortcut");
    for (std::size_t i = 1U; i < smoothed.size(); ++i) {
        expect_true(smoother.has_line_of_sight(map, smoothed[i - 1U], smoothed[i]), "every smoothed segment must be collision-free");
    }
}

void test_path_smoother_blocks_corner_cutting() {
    GridMap map(3, 3);
    map.set_occupied({1, 0});
    map.set_occupied({0, 1});
    const GridPathSmoother smoother;
    expect_true(!smoother.has_line_of_sight(map, {0, 0}, {1, 1}), "smoother must not cut blocked diagonal corners");
}

void test_pure_pursuit_straight_path() {
    PurePursuitController controller({0.5, 1.0, 2.0, 0.1, 0.7});
    const std::vector<Vec2> path{{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}};
    const auto command = controller.update({0.0, 0.0, 0.0}, path);
    expect_true(!command.goal_reached, "straight path should remain active");
    expect_true(command.twist.linear_mps > 0.0, "straight path should command forward motion");
    expect_near(command.twist.angular_rps, 0.0, 1e-12, "straight path angular rate");
}

void test_pure_pursuit_turn_direction() {
    PurePursuitController controller({0.4, 0.8, 2.0, 0.08, 0.7});
    const std::vector<Vec2> path{{0.0, 0.0}, {0.4, 0.2}, {0.8, 0.6}};
    const auto command = controller.update({0.0, 0.0, 0.0}, path);
    expect_true(command.curvature_per_m > 0.0, "left-side lookahead should produce positive curvature");
    expect_true(command.twist.angular_rps > 0.0, "left-side lookahead should command positive angular rate");
}

void test_pure_pursuit_goal_tolerance_stops() {
    PurePursuitController controller({0.5, 0.8, 2.0, 0.1, 0.7});
    const std::vector<Vec2> path{{0.0, 0.0}, {1.0, 0.0}};
    const auto command = controller.update({0.95, 0.0, 0.0}, path);
    expect_true(command.goal_reached, "goal tolerance should stop pure pursuit");
    expect_near(command.twist.linear_mps, 0.0, 1e-12, "goal stop linear speed");
    expect_near(command.twist.angular_rps, 0.0, 1e-12, "goal stop angular speed");
}

void test_smoothed_pure_pursuit_reaches_demo_goal() {
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

    const GridCell start{2, 2};
    const GridCell goal{21, 15};
    const GridMap planning_map = inflate_obstacles(map, 1);
    const AStarPlanner planner(true);
    const auto plan = planner.plan(planning_map, start, goal);
    expect_true(plan.has_value(), "inflated demo plan should exist");

    const GridPathSmoother smoother;
    const auto smoothed_cells = smoother.smooth(planning_map, plan->cells);
    expect_true(smoothed_cells.size() < plan->cells.size(), "smoother should reduce demo waypoint count");

    std::vector<Vec2> path;
    path.reserve(smoothed_cells.size());
    for (const GridCell cell : smoothed_cells) {
        path.push_back(planning_map.cell_center(cell));
    }

    constexpr double track_width_m = 0.42;
    constexpr double dt_s = 0.02;
    DifferentialDriveKinematics kinematics(track_width_m);
    DifferentialDriveOdometry odometry(track_width_m, {path.front().x, path.front().y, 0.0});
    PurePursuitController controller({0.30, 0.95, 1.8, 0.10, 0.75});

    Pose2D pose = odometry.pose();
    bool reached = false;
    for (int step = 0; step < 5000; ++step) {
        const auto command = controller.update(pose, path);
        if (command.goal_reached) {
            reached = true;
            break;
        }
        const WheelSpeeds wheels = kinematics.inverse(command.twist);
        pose = odometry.update(wheels.left_mps * dt_s, wheels.right_mps * dt_s);
        const GridCell occupied_check{static_cast<int>(std::floor(pose.x / map.resolution_m())),
                                      static_cast<int>(std::floor(pose.y / map.resolution_m()))};
        expect_true(!map.is_occupied(occupied_check), "pure-pursuit trajectory must remain outside occupied cells");
    }

    expect_true(reached, "smoothed pure-pursuit integration should reach the demo goal");
    expect_true(distance(pose, path.back()) <= 0.11 + 1e-9, "pure-pursuit final goal error must satisfy tolerance");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"map_inflation_expands_obstacles", test_map_inflation_expands_obstacles},
        {"map_inflation_zero_is_identity", test_map_inflation_zero_is_identity},
        {"path_smoother_reduces_open_path", test_path_smoother_reduces_open_path},
        {"path_smoother_preserves_obstacle_clearance", test_path_smoother_preserves_obstacle_clearance},
        {"path_smoother_blocks_corner_cutting", test_path_smoother_blocks_corner_cutting},
        {"pure_pursuit_straight_path", test_pure_pursuit_straight_path},
        {"pure_pursuit_turn_direction", test_pure_pursuit_turn_direction},
        {"pure_pursuit_goal_tolerance_stops", test_pure_pursuit_goal_tolerance_stops},
        {"smoothed_pure_pursuit_reaches_demo_goal", test_smoothed_pure_pursuit_reaches_demo_goal},
    };

    std::size_t passed = 0U;
    for (const auto& [name, test] : tests) {
        try {
            test();
            ++passed;
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
        }
    }

    std::cout << "v02_tests_passed=" << passed << '/' << tests.size() << '\n';
    return passed == tests.size() ? 0 : 1;
}
