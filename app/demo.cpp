#include "robotics_control/a_star.hpp"
#include "robotics_control/differential_drive.hpp"
#include "robotics_control/odometry.hpp"
#include "robotics_control/path_follower.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace robotics_control;

namespace {

GridMap make_demo_map() {
    GridMap map(24, 18, 0.25);
    for (int y = 2; y < 15; ++y) {
        if (y == 8 || y == 9) {
            continue;
        }
        map.set_occupied({10, y});
    }
    for (int x = 3; x < 19; ++x) {
        if (x == 15 || x == 16) {
            continue;
        }
        map.set_occupied({x, 12});
    }
    return map;
}

std::vector<Vec2> to_metric_path(const GridMap& map, const std::vector<GridCell>& cells) {
    std::vector<Vec2> result;
    result.reserve(cells.size());
    for (const GridCell cell : cells) {
        result.push_back(map.cell_center(cell));
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string output_path = argc > 1 ? argv[1] : "trajectory.csv";
    const std::string path_output = argc > 2 ? argv[2] : "planned_path.csv";
    const std::string map_output = argc > 3 ? argv[3] : "map.csv";
    const GridMap map = make_demo_map();
    const GridCell start{2, 2};
    const GridCell goal{21, 15};

    const AStarPlanner planner(true);
    const auto plan = planner.plan(map, start, goal);
    if (!plan) {
        std::cerr << "planning_failed\n";
        return 2;
    }

    const std::vector<Vec2> path = to_metric_path(map, plan->cells);

    std::ofstream path_csv(path_output);
    if (!path_csv) {
        std::cerr << "cannot_open_output=" << path_output << '\n';
        return 3;
    }
    path_csv << "index,x_m,y_m\n";
    for (std::size_t i = 0U; i < path.size(); ++i) {
        path_csv << i << ',' << path[i].x << ',' << path[i].y << '\n';
    }

    std::ofstream map_csv(map_output);
    if (!map_csv) {
        std::cerr << "cannot_open_output=" << map_output << '\n';
        return 3;
    }
    map_csv << "x_cell,y_cell,occupied,resolution_m\n";
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            map_csv << x << ',' << y << ',' << (map.is_occupied({x, y}) ? 1 : 0) << ',' << map.resolution_m() << '\n';
        }
    }

    const Vec2 start_point = path.front();
    Pose2D pose{start_point.x, start_point.y, 0.0};

    constexpr double track_width_m = 0.42;
    DifferentialDriveKinematics kinematics(track_width_m);
    DifferentialDriveOdometry odometry(track_width_m, pose);
    PidController heading_pid({3.1, 0.05, 0.18}, {-1.8, 1.8, -0.7, 0.7});
    PathFollower follower({0.9, 1.8, 0.16, 0.10, 0.95}, std::move(heading_pid));

    std::ofstream csv(output_path);
    if (!csv) {
        std::cerr << "cannot_open_output=" << output_path << '\n';
        return 3;
    }
    csv << "time_s,x_m,y_m,yaw_rad,target_index,linear_mps,angular_rps,left_mps,right_mps\n";

    constexpr double dt_s = 0.02;
    constexpr int max_steps = 4000;
    bool reached = false;
    int steps = 0;

    for (; steps < max_steps; ++steps) {
        const FollowCommand command = follower.update(pose, path, dt_s);
        const WheelSpeeds wheels = kinematics.inverse(command.twist);
        csv << static_cast<double>(steps) * dt_s << ',' << pose.x << ',' << pose.y << ',' << pose.yaw << ','
            << command.target_index << ',' << command.twist.linear_mps << ',' << command.twist.angular_rps << ','
            << wheels.left_mps << ',' << wheels.right_mps << '\n';

        if (command.goal_reached) {
            reached = true;
            break;
        }

        pose = odometry.update(wheels.left_mps * dt_s, wheels.right_mps * dt_s);
    }

    const double goal_error = distance(pose, path.back());
    std::cout << "planned_cells=" << plan->cells.size() << '\n';
    std::cout << "path_cost_m=" << plan->cost << '\n';
    std::cout << "expanded_nodes=" << plan->expanded_nodes << '\n';
    std::cout << "simulation_steps=" << steps << '\n';
    std::cout << "goal_error_m=" << goal_error << '\n';
    std::cout << "goal_reached=" << std::boolalpha << reached << '\n';
    std::cout << "trajectory_csv=" << output_path << '\n';
    std::cout << "planned_path_csv=" << path_output << '\n';
    std::cout << "map_csv=" << map_output << '\n';

    return reached && goal_error <= 0.11 ? 0 : 4;
}
