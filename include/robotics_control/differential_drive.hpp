#pragma once

#include "robotics_control/types.hpp"

#include <stdexcept>

namespace robotics_control {

class DifferentialDriveKinematics {
public:
    explicit DifferentialDriveKinematics(double track_width_m) : track_width_m_(track_width_m) {
        if (track_width_m <= 0.0) {
            throw std::invalid_argument("track width must be positive");
        }
    }

    [[nodiscard]] WheelSpeeds inverse(BodyTwist twist) const noexcept;
    [[nodiscard]] BodyTwist forward(WheelSpeeds wheels) const noexcept;
    [[nodiscard]] double track_width_m() const noexcept { return track_width_m_; }

private:
    double track_width_m_;
};

}  // namespace robotics_control
