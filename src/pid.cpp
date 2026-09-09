#include "robotics_control/pid.hpp"

namespace robotics_control {

PidController::PidController(PidGains gains, PidLimits limits) : gains_(gains), limits_(limits) {
    if (limits_.min_output > limits_.max_output || limits_.min_integral > limits_.max_integral) {
        throw std::invalid_argument("invalid PID limits");
    }
}

double PidController::update(double error, double dt_s) {
    if (dt_s <= 0.0) {
        throw std::invalid_argument("PID dt must be positive");
    }

    integral_ = std::clamp(integral_ + error * dt_s, limits_.min_integral, limits_.max_integral);
    const double derivative = has_previous_ ? (error - previous_error_) / dt_s : 0.0;
    previous_error_ = error;
    has_previous_ = true;

    const double raw = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    return std::clamp(raw, limits_.min_output, limits_.max_output);
}

void PidController::reset() noexcept {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_ = false;
}

}  // namespace robotics_control
