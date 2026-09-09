#pragma once

#include <algorithm>
#include <stdexcept>

namespace robotics_control {

struct PidGains {
    double kp{};
    double ki{};
    double kd{};
};

struct PidLimits {
    double min_output{-1.0};
    double max_output{1.0};
    double min_integral{-1.0};
    double max_integral{1.0};
};

class PidController {
public:
    PidController(PidGains gains, PidLimits limits);

    [[nodiscard]] double update(double error, double dt_s);
    void reset() noexcept;

    [[nodiscard]] double integral_state() const noexcept { return integral_; }

private:
    PidGains gains_;
    PidLimits limits_;
    double integral_{};
    double previous_error_{};
    bool has_previous_{};
};

}  // namespace robotics_control
