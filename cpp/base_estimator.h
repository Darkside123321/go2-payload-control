#pragma once

// Poses are world xyz + body-to-world wxyz. Velocity is world xyz + body angular xyz.
// Foot arrays contain FL, FR, RL, RR in base coordinates. No ground-truth base state is used.
extern "C" {
void* base_estimator_create(const double* initial_position, const double* ground_foot_z);
void base_estimator_destroy(void* estimator);
int base_estimator_step(void* estimator, double dt, const double* gyro,
                        const double* specific_force, const double* foot_positions,
                        const double* joint_foot_velocities, const int* contacts,
                        double* pose, double* velocity, int* support_count);
}
