#pragma once

#include "robotics_control/types.hpp"

#include <stdexcept>
#include <vector>

namespace robotics_control {

class GridMap {
public:
    GridMap(int width, int height, double resolution_m = 1.0)
        : width_(width), height_(height), resolution_m_(resolution_m), occupied_(checked_size(width, height), false) {
        if (resolution_m <= 0.0) {
            throw std::invalid_argument("resolution must be positive");
        }
    }

    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] double resolution_m() const noexcept { return resolution_m_; }

    [[nodiscard]] bool in_bounds(GridCell cell) const noexcept {
        return cell.x >= 0 && cell.y >= 0 && cell.x < width_ && cell.y < height_;
    }

    [[nodiscard]] bool is_occupied(GridCell cell) const {
        if (!in_bounds(cell)) {
            return true;
        }
        return occupied_.at(index(cell));
    }

    void set_occupied(GridCell cell, bool occupied = true) {
        if (!in_bounds(cell)) {
            throw std::out_of_range("grid cell out of bounds");
        }
        occupied_.at(index(cell)) = occupied;
    }

    [[nodiscard]] Vec2 cell_center(GridCell cell) const {
        if (!in_bounds(cell)) {
            throw std::out_of_range("grid cell out of bounds");
        }
        return Vec2{(static_cast<double>(cell.x) + 0.5) * resolution_m_,
                    (static_cast<double>(cell.y) + 0.5) * resolution_m_};
    }

private:
    static std::size_t checked_size(int width, int height) {
        if (width <= 0 || height <= 0) {
            throw std::invalid_argument("grid dimensions must be positive");
        }
        return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }

    [[nodiscard]] std::size_t index(GridCell cell) const noexcept {
        return static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(cell.x);
    }

    int width_;
    int height_;
    double resolution_m_;
    std::vector<bool> occupied_;
};

}  // namespace robotics_control
