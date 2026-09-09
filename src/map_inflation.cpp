#include "robotics_control/map_inflation.hpp"

#include <stdexcept>

namespace robotics_control {

GridMap inflate_obstacles(const GridMap& map, int clearance_cells) {
    if (clearance_cells < 0) {
        throw std::invalid_argument("clearance_cells must be non-negative");
    }

    GridMap inflated(map.width(), map.height(), map.resolution_m());
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (!map.is_occupied({x, y})) {
                continue;
            }
            for (int oy = -clearance_cells; oy <= clearance_cells; ++oy) {
                for (int ox = -clearance_cells; ox <= clearance_cells; ++ox) {
                    const GridCell cell{x + ox, y + oy};
                    if (inflated.in_bounds(cell)) {
                        inflated.set_occupied(cell);
                    }
                }
            }
        }
    }
    return inflated;
}

}  // namespace robotics_control
