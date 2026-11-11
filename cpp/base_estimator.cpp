#include "base_estimator.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <new>

namespace {
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Map;

constexpr double gravity = 9.81;
constexpr int foot_count = 4;

Quaterniond incremental_rotation(const Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < 1e-12) {
        return Quaterniond::Identity();
    }
    return Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

class BaseEstimator {
public:
    BaseEstimator(const Vector3d& initial_position, const double* ground_foot_z)
        : position_(initial_position) {
        std::copy_n(ground_foot_z, foot_count, ground_foot_z_.begin());
    }

    void step(double dt, const Vector3d& gyro, const Vector3d& specific_force,
              const double* feet, const double* joint_velocities, const int* contacts,
              double* pose, double* velocity, int* support_count) {
        int support = 0;
        for (int foot = 0; foot < foot_count; ++foot) {
            contact_ticks_[foot] = contacts[foot] ? std::min(contact_ticks_[foot] + 1, 3) : 0;
            support += contact_ticks_[foot] == 3;
        }

        orientation_ = (orientation_ * incremental_rotation(gyro * dt)).normalized();
        const double force_norm = specific_force.norm();
        // Trust gravity for tilt only in quasi-static, well-supported samples.
        if (support >= 3 && gyro.norm() < 0.25 && velocity_.norm() < 0.08 &&
            std::abs(force_norm - gravity) < 0.35) {
            const Vector3d measured_up = specific_force / force_norm;
            const Vector3d predicted_up = orientation_.conjugate() * Vector3d::UnitZ();
            const Vector3d tilt_error = measured_up.cross(predicted_up);
            orientation_ = (orientation_ * incremental_rotation(tilt_error * (1.2 * dt))).normalized();
        }

        const Vector3d world_acceleration =
            orientation_ * specific_force - gravity * Vector3d::UnitZ();
        position_ += velocity_ * dt + world_acceleration * (0.5 * dt * dt);
        velocity_ += world_acceleration * dt;

        Vector3d position_error = Vector3d::Zero();
        Vector3d velocity_error = Vector3d::Zero();
        for (int foot = 0; foot < foot_count; ++foot) {
            if (contact_ticks_[foot] < 3) {
                anchored_[foot] = false;
                continue;
            }
            const Map<const Vector3d> r(feet + 3 * foot);
            const Map<const Vector3d> joint_velocity(joint_velocities + 3 * foot);
            const Vector3d world_offset = orientation_ * r;
            if (!anchored_[foot]) {
                anchors_[foot] = position_ + world_offset;
                anchors_[foot].z() = ground_foot_z_[foot];
                anchored_[foot] = true;
            }

            // Stance foot velocity is approximately zero. Cap each residual's influence.
            const Vector3d position_residual = anchors_[foot] - position_ - world_offset;
            const Vector3d stance_velocity = -(orientation_ * (gyro.cross(r) + joint_velocity));
            position_error += position_residual.cwiseMax(-0.025).cwiseMin(0.025);
            velocity_error += (stance_velocity - velocity_).cwiseMax(-0.25).cwiseMin(0.25);
        }
        if (support > 0) {
            position_ += (1.0 - std::exp(-8.0 * dt)) * position_error / support;
            velocity_ += (1.0 - std::exp(-30.0 * dt)) * velocity_error / support;
        }

        Map<Vector3d> pose_position(pose);
        pose_position = position_;
        pose[3] = orientation_.w();
        pose[4] = orientation_.x();
        pose[5] = orientation_.y();
        pose[6] = orientation_.z();
        Map<Vector3d> linear_velocity(velocity);
        linear_velocity = velocity_;
        Map<Vector3d> angular_velocity(velocity + 3);
        angular_velocity = gyro;
        *support_count = support;
    }

private:
    Vector3d position_;
    Vector3d velocity_ = Vector3d::Zero();
    Quaterniond orientation_ = Quaterniond::Identity();
    std::array<Vector3d, foot_count> anchors_;
    std::array<double, foot_count> ground_foot_z_;
    std::array<int, foot_count> contact_ticks_{};
    std::array<bool, foot_count> anchored_{};
};
}

extern "C" void* base_estimator_create(const double* initial_position, const double* ground_foot_z) {
    const Map<const Vector3d> position(initial_position);
    if (!position.allFinite() || !Map<const Eigen::Vector4d>(ground_foot_z).allFinite()) {
        return nullptr;
    }
    return new (std::nothrow) BaseEstimator(position, ground_foot_z);
}

extern "C" void base_estimator_destroy(void* estimator) {
    delete static_cast<BaseEstimator*>(estimator);
}

extern "C" int base_estimator_step(void* estimator, double dt, const double* gyro,
                                   const double* specific_force, const double* feet,
                                   const double* joint_velocities, const int* contacts,
                                   double* pose, double* velocity, int* support_count) {
    using FootVector = Eigen::Matrix<double, 12, 1>;
    if (!estimator || !(dt > 0.0 && dt <= 0.05) ||
        !Map<const Vector3d>(gyro).allFinite() || !Map<const Vector3d>(specific_force).allFinite() ||
        !Map<const FootVector>(feet).allFinite() || !Map<const FootVector>(joint_velocities).allFinite()) {
        return 1;
    }
    static_cast<BaseEstimator*>(estimator)->step(
        dt, Map<const Vector3d>(gyro), Map<const Vector3d>(specific_force),
        feet, joint_velocities, contacts, pose, velocity, support_count);
    return 0;
}
