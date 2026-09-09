#include "robotics_control/a_star.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace robotics_control {
namespace {

struct QueueNode {
    GridCell cell;
    double f{};
    double g{};

    bool operator>(const QueueNode& other) const noexcept { return f > other.f; }
};

[[nodiscard]] double heuristic(GridCell a, GridCell b, bool diagonal) noexcept {
    const double dx = static_cast<double>(std::abs(a.x - b.x));
    const double dy = static_cast<double>(std::abs(a.y - b.y));
    if (!diagonal) {
        return dx + dy;
    }
    constexpr double sqrt2 = 1.4142135623730951;
    return (dx + dy) + (sqrt2 - 2.0) * std::min(dx, dy);
}

}  // namespace

std::optional<PlanResult> AStarPlanner::plan(const GridMap& map, GridCell start, GridCell goal) const {
    if (!map.in_bounds(start) || !map.in_bounds(goal) || map.is_occupied(start) || map.is_occupied(goal)) {
        return std::nullopt;
    }
    if (start == goal) {
        return PlanResult{{start}, 0.0, 0U};
    }

    static constexpr std::array<GridCell, 8> directions{{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    }};

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> open;
    std::unordered_map<GridCell, double, GridCellHash> g_score;
    std::unordered_map<GridCell, GridCell, GridCellHash> came_from;

    g_score.emplace(start, 0.0);
    open.push(QueueNode{start, heuristic(start, goal, allow_diagonal_), 0.0});

    std::size_t expanded = 0U;
    constexpr double sqrt2 = 1.4142135623730951;

    while (!open.empty()) {
        const QueueNode current = open.top();
        open.pop();

        const auto known = g_score.find(current.cell);
        if (known == g_score.end() || current.g > known->second + 1e-12) {
            continue;
        }

        if (current.cell == goal) {
            std::vector<GridCell> path;
            GridCell cursor = goal;
            path.push_back(cursor);
            while (!(cursor == start)) {
                cursor = came_from.at(cursor);
                path.push_back(cursor);
            }
            std::reverse(path.begin(), path.end());
            return PlanResult{std::move(path), current.g * map.resolution_m(), expanded};
        }

        ++expanded;
        for (std::size_t i = 0U; i < directions.size(); ++i) {
            if (!allow_diagonal_ && i >= 4U) {
                break;
            }
            const GridCell step = directions[i];
            const GridCell next{current.cell.x + step.x, current.cell.y + step.y};
            if (!map.in_bounds(next) || map.is_occupied(next)) {
                continue;
            }

            const bool diagonal_move = step.x != 0 && step.y != 0;
            if (diagonal_move) {
                // Prevent corner cutting through two touching obstacles.
                const GridCell side_a{current.cell.x + step.x, current.cell.y};
                const GridCell side_b{current.cell.x, current.cell.y + step.y};
                if (map.is_occupied(side_a) || map.is_occupied(side_b)) {
                    continue;
                }
            }

            const double step_cost = diagonal_move ? sqrt2 : 1.0;
            const double tentative = current.g + step_cost;
            const auto existing = g_score.find(next);
            if (existing != g_score.end() && tentative >= existing->second - 1e-12) {
                continue;
            }

            came_from[next] = current.cell;
            g_score[next] = tentative;
            open.push(QueueNode{next, tentative + heuristic(next, goal, allow_diagonal_), tentative});
        }
    }

    return std::nullopt;
}

}  // namespace robotics_control
