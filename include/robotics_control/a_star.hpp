#pragma once

#include "robotics_control/grid_map.hpp"

#include <optional>
#include <vector>

namespace robotics_control {

struct PlanResult {
    std::vector<GridCell> cells;
    double cost{};
    std::size_t expanded_nodes{};
};

class AStarPlanner {
public:
    explicit AStarPlanner(bool allow_diagonal = true) : allow_diagonal_(allow_diagonal) {}

    [[nodiscard]] std::optional<PlanResult> plan(const GridMap& map, GridCell start, GridCell goal) const;

private:
    bool allow_diagonal_;
};

}  // namespace robotics_control
