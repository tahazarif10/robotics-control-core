#include "robotics_control/a_star.hpp"
#include "robotics_control/differential_drive.hpp"
#include "robotics_control/odometry.hpp"
#include "robotics_control/path_follower.hpp"
#include "robotics_control/pid.hpp"

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

void test_astar_finds_short_path() {
    GridMap map(5, 5, 0.5);
    AStarPlanner planner(false);
    const auto result = planner.plan(map, {0, 0}, {4, 0});
    expect_true(result.has_value(), "A* should find path");
    expect_true(result->cells.size() == 5U, "unexpected path length");
    expect_near(result->cost, 2.0, 1e-12, "unexpected metric cost");
}

void test_astar_blocks_corner_cutting() {
    GridMap map(3, 3);
    map.set_occupied({1, 0});
    map.set_occupied({0, 1});
    AStarPlanner planner(true);
    const auto result = planner.plan(map, {0, 0}, {1, 1});
    expect_true(!result.has_value(), "diagonal corner cutting must be rejected");
}

void test_astar_unreachable_returns_none() {
    GridMap map(5, 5);
    for (int y = 0; y < 5; ++y) {
        map.set_occupied({2, y});
    }
    AStarPlanner planner(true);
    expect_true(!planner.plan(map, {0, 2}, {4, 2}).has_value(), "unreachable goal must fail");
}


void test_astar_start_equals_goal() {
    GridMap map(3, 3, 0.25);
    AStarPlanner planner(true);
    const auto result = planner.plan(map, {1, 1}, {1, 1});
    expect_true(result.has_value(), "start=goal should succeed");
    expect_true(result->cells.size() == 1U, "start=goal should return one cell");
    expect_near(result->cost, 0.0, 1e-12, "start=goal cost");
}

void test_astar_rejects_occupied_endpoint() {
    GridMap map(3, 3);
    map.set_occupied({2, 2});
    AStarPlanner planner(true);
    expect_true(!planner.plan(map, {0, 0}, {2, 2}).has_value(), "occupied goal must fail");
}

void test_kinematics_round_trip_sweep() {
    DifferentialDriveKinematics model(0.37);
    for (int linear_i = -5; linear_i <= 5; ++linear_i) {
        for (int angular_i = -8; angular_i <= 8; ++angular_i) {
            const BodyTwist input{0.2 * static_cast<double>(linear_i), 0.15 * static_cast<double>(angular_i)};
            const BodyTwist output = model.forward(model.inverse(input));
            expect_near(output.linear_mps, input.linear_mps, 1e-12, "sweep linear round-trip");
            expect_near(output.angular_rps, input.angular_rps, 1e-12, "sweep angular round-trip");
        }
    }
}

void test_odometry_curved_motion_is_finite() {
    DifferentialDriveOdometry odom(0.5);
    const Pose2D pose = odom.update(0.5, 1.0);
    expect_true(std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw),
                "curved odometry must stay finite");
    expect_true(pose.x > 0.0 && pose.y > 0.0, "curved odometry should advance into positive x/y");
}

void test_pid_rejects_nonpositive_dt() {
    PidController pid({1.0, 0.0, 0.0}, {-1.0, 1.0, -1.0, 1.0});
    bool threw = false;
    try {
        static_cast<void>(pid.update(1.0, 0.0));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect_true(threw, "PID must reject dt <= 0");
}

void test_kinematics_round_trip() {
    DifferentialDriveKinematics model(0.42);
    const BodyTwist input{0.8, -0.6};
    const WheelSpeeds wheels = model.inverse(input);
    const BodyTwist output = model.forward(wheels);
    expect_near(output.linear_mps, input.linear_mps, 1e-12, "linear velocity round-trip");
    expect_near(output.angular_rps, input.angular_rps, 1e-12, "angular velocity round-trip");
}

void test_odometry_straight_line() {
    DifferentialDriveOdometry odom(0.5);
    const Pose2D pose = odom.update(1.0, 1.0);
    expect_near(pose.x, 1.0, 1e-12, "straight x");
    expect_near(pose.y, 0.0, 1e-12, "straight y");
    expect_near(pose.yaw, 0.0, 1e-12, "straight yaw");
}

void test_odometry_in_place_turn() {
    DifferentialDriveOdometry odom(0.5);
    const Pose2D pose = odom.update(-0.25, 0.25);
    expect_near(pose.x, 0.0, 1e-12, "turn x");
    expect_near(pose.y, 0.0, 1e-12, "turn y");
    expect_near(pose.yaw, 1.0, 1e-12, "turn yaw");
}

void test_pid_saturates_and_resets() {
    PidController pid({10.0, 2.0, 0.0}, {-1.0, 1.0, -0.2, 0.2});
    expect_near(pid.update(2.0, 0.1), 1.0, 1e-12, "positive saturation");
    expect_true(pid.integral_state() <= 0.2 + 1e-12, "integral clamp");
    pid.reset();
    expect_near(pid.integral_state(), 0.0, 1e-12, "PID reset");
}

void test_path_follower_empty_path_is_done() {
    PidController pid({1.0, 0.0, 0.0}, {-1.0, 1.0, -1.0, 1.0});
    PathFollower follower({}, std::move(pid));
    const FollowCommand command = follower.update({}, {}, 0.02);
    expect_true(command.goal_reached, "empty path should be terminal");
    expect_near(command.twist.linear_mps, 0.0, 1e-12, "empty path linear speed");
}

void test_path_follower_reaches_last_waypoint() {
    PidController pid({2.0, 0.0, 0.0}, {-1.5, 1.5, -1.0, 1.0});
    PathFollower follower({0.5, 1.5, 0.2, 0.1, 1.0}, std::move(pid));
    const std::vector<Vec2> path{{1.0, 0.0}};
    const FollowCommand command = follower.update({0.96, 0.0, 0.0}, path, 0.02);
    expect_true(command.goal_reached, "goal tolerance should stop controller");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"astar_finds_short_path", test_astar_finds_short_path},
        {"astar_blocks_corner_cutting", test_astar_blocks_corner_cutting},
        {"astar_unreachable_returns_none", test_astar_unreachable_returns_none},
        {"astar_start_equals_goal", test_astar_start_equals_goal},
        {"astar_rejects_occupied_endpoint", test_astar_rejects_occupied_endpoint},
        {"kinematics_round_trip", test_kinematics_round_trip},
        {"kinematics_round_trip_sweep", test_kinematics_round_trip_sweep},
        {"odometry_straight_line", test_odometry_straight_line},
        {"odometry_in_place_turn", test_odometry_in_place_turn},
        {"odometry_curved_motion_is_finite", test_odometry_curved_motion_is_finite},
        {"pid_saturates_and_resets", test_pid_saturates_and_resets},
        {"pid_rejects_nonpositive_dt", test_pid_rejects_nonpositive_dt},
        {"path_follower_empty_path_is_done", test_path_follower_empty_path_is_done},
        {"path_follower_reaches_last_waypoint", test_path_follower_reaches_last_waypoint},
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

    std::cout << "tests_passed=" << passed << '/' << tests.size() << '\n';
    return passed == tests.size() ? 0 : 1;
}
