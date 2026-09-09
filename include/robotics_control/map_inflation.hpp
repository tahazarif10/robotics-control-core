#pragma once

#include "robotics_control/grid_map.hpp"

namespace robotics_control {

[[nodiscard]] GridMap inflate_obstacles(const GridMap& map, int clearance_cells);

}  // namespace robotics_control
