#pragma once

#include "robotics_control/grid_map.hpp"

#include <vector>

namespace robotics_control {

class GridPathSmoother {
public:
    [[nodiscard]] std::vector<GridCell> smooth(const GridMap& map, const std::vector<GridCell>& path) const;
    [[nodiscard]] bool has_line_of_sight(const GridMap& map, GridCell from, GridCell to) const;
};

}  // namespace robotics_control
