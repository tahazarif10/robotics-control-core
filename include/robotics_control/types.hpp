#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace robotics_control {

struct Vec2 {
    double x{};
    double y{};
};

struct Pose2D {
    double x{};
    double y{};
    double yaw{};
};

struct GridCell {
    int x{};
    int y{};

    friend constexpr bool operator==(const GridCell&, const GridCell&) = default;
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const noexcept {
        const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.x));
        const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.y));
        return static_cast<std::size_t>((ux << 32U) ^ uy);
    }
};

struct WheelSpeeds {
    double left_mps{};
    double right_mps{};
};

struct BodyTwist {
    double linear_mps{};
    double angular_rps{};
};

[[nodiscard]] inline double normalize_angle(double angle) noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    angle = std::fmod(angle + std::numbers::pi, two_pi);
    if (angle < 0.0) {
        angle += two_pi;
    }
    return angle - std::numbers::pi;
}

[[nodiscard]] inline double distance(const Vec2& a, const Vec2& b) noexcept {
    return std::hypot(a.x - b.x, a.y - b.y);
}

[[nodiscard]] inline double distance(const Pose2D& pose, const Vec2& point) noexcept {
    return std::hypot(pose.x - point.x, pose.y - point.y);
}

}  // namespace robotics_control
