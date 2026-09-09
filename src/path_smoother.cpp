#include "robotics_control/path_smoother.hpp"

#include <algorithm>
#include <cmath>

namespace robotics_control {

bool GridPathSmoother::has_line_of_sight(const GridMap& map, GridCell from, GridCell to) const {
    if (!map.in_bounds(from) || !map.in_bounds(to) || map.is_occupied(from) || map.is_occupied(to)) {
        return false;
    }

    const int dx = std::abs(to.x - from.x);
    const int dy = std::abs(to.y - from.y);
    const int sx = from.x < to.x ? 1 : -1;
    const int sy = from.y < to.y ? 1 : -1;
    int error = dx - dy;
    GridCell current = from;

    while (!(current == to)) {
        const int twice_error = 2 * error;
        GridCell next = current;
        if (twice_error > -dy) {
            error -= dy;
            next.x += sx;
        }
        if (twice_error < dx) {
            error += dx;
            next.y += sy;
        }

        if (!map.in_bounds(next) || map.is_occupied(next)) {
            return false;
        }

        const bool diagonal_step = next.x != current.x && next.y != current.y;
        if (diagonal_step) {
            const GridCell side_a{next.x, current.y};
            const GridCell side_b{current.x, next.y};
            if (map.is_occupied(side_a) || map.is_occupied(side_b)) {
                return false;
            }
        }

        current = next;
    }

    return true;
}

std::vector<GridCell> GridPathSmoother::smooth(const GridMap& map, const std::vector<GridCell>& path) const {
    if (path.size() <= 2U) {
        return path;
    }

    std::vector<GridCell> result;
    result.reserve(path.size());
    result.push_back(path.front());

    std::size_t anchor = 0U;
    while (anchor + 1U < path.size()) {
        std::size_t furthest = anchor + 1U;
        for (std::size_t candidate = anchor + 2U; candidate < path.size(); ++candidate) {
            if (has_line_of_sight(map, path[anchor], path[candidate])) {
                furthest = candidate;
            }
        }
        result.push_back(path[furthest]);
        anchor = furthest;
    }

    return result;
}

}  // namespace robotics_control
