#pragma once

#include "robotics_control/types.hpp"

#include <cstddef>
#include <vector>

namespace robotics_control {

struct PurePursuitConfig {
    double lookahead_m{0.55};
    double max_linear_mps{0.85};
    double max_angular_rps{1.8};
    double goal_tolerance_m{0.10};
    double curvature_slowdown{0.70};
};

struct PurePursuitCommand {
    BodyTwist twist;
    std::size_t target_index{};
    double curvature_per_m{};
    bool goal_reached{};
};

class PurePursuitController {
public:
    explicit PurePursuitController(PurePursuitConfig config = {});

    [[nodiscard]] PurePursuitCommand update(const Pose2D& pose, const std::vector<Vec2>& path);
    void reset() noexcept { target_index_ = 0U; }

private:
    PurePursuitConfig config_;
    std::size_t target_index_{};
};

}  // namespace robotics_control
