#include "base_estimator.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

int main() {
    const double p0[3]{0.0, 0.0, 0.3};
    const double ground[4]{0.02, 0.02, 0.02, 0.02};
    const double feet[12]{0.2, 0.1, -0.28, 0.2, -0.1, -0.28,
                          -0.2, 0.1, -0.28, -0.2, -0.1, -0.28};
    const double joint_vel[12]{};
    const double gyro[3]{};
    const double stationary[3]{0.0, 0.0, 9.81};
    const double accelerating[3]{1.0, 0.0, 9.81};
    const double invalid[3]{0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
    const int stance[4]{1, 1, 1, 1};
    const int flight[4]{};
    constexpr double dt = 0.004;
    std::unique_ptr<void, decltype(&base_estimator_destroy)> estimator(
        base_estimator_create(p0, ground), base_estimator_destroy);
    double pose[7]{}, velocity[6]{};
    int support = 0;
    const auto update = [&](double step, const double* accel, const int* contacts) {
        return base_estimator_step(estimator.get(), step, gyro, accel, feet, joint_vel,
                                   contacts, pose, velocity, &support);
    };
    for (int i = 0; i < 250; ++i) {
        if (update(dt, stationary, stance) != 0 || support != (i < 2 ? 0 : 4)) {
            std::cerr << "contact debounce failed\n";
            return 1;
        }
    }
    if (std::abs(pose[2] - p0[2]) > 1e-10 || std::abs(velocity[2]) > 1e-10 ||
        std::abs(pose[3] - 1.0) > 1e-10) {
        std::cerr << "stationary estimate drifted\n";
        return 1;
    }
    if (update(0.0, stationary, stance) != 1 || update(dt, invalid, stance) != 1 ||
        update(dt, accelerating, flight) != 0 || support != 0 ||
        std::abs(velocity[0] - dt) > 1e-10 ||
        std::abs(pose[0] - 0.5 * dt * dt) > 1e-10) {
        std::cerr << "invalid sample handling or inertial prediction failed\n";
        return 1;
    }
    std::cout << "Stationary contact, debounce, flight propagation, and invalid input passed\n";
}
