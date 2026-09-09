#include "robotics_control/differential_drive.hpp"

#include <cmath>
#include <iostream>

int main() {
    const robotics_control::DifferentialDriveKinematics model(0.42);
    const robotics_control::BodyTwist requested{0.8, -0.6};
    const auto wheels = model.inverse(requested);
    const auto recovered = model.forward(wheels);

    const bool ok = std::abs(recovered.linear_mps - requested.linear_mps) < 1e-12 &&
                    std::abs(recovered.angular_rps - requested.angular_rps) < 1e-12;

    std::cout << "consumer_package_smoke=" << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}
