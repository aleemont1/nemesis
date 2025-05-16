#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

// Macros needed for a conflict between similar macro variables names of Arduino.h and Eigen.h
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#ifdef B0
#undef B0
#endif

#define EKF_N 16 // Size of state space [3-positions, 3-velocities, 3-accelerations, 4-quaternion_rot] 
#define EKF_M 6 // Size of observation (measurement) space [3-positions, 3-accelerations, 4-quaternion_rot]

#include <tinyekf.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <random>
#include <ArduinoEigen.h>

// These vectors should be put private iside of the class
static float Q_diag[EKF_N];  // Process noise covariance
static float R[EKF_M * EKF_M];  // Measurement noise covariance
static float H[EKF_M*EKF_N];  // Measurement Jacobian
static float F[EKF_N*EKF_N];  // Jacobian with input/output relations

class KalmanFilter {
public:
    KalmanFilter();
    std::vector<std::vector<float>> step(float dt, float omega[3], float accel[3]);
    float* state();

private:
    ekf_t ekf;
    
    // initial covariances of state noise, measurement noise
    // Q matrix (model) // We obtain this value from a comparison between a model and the real data.
    const float P0 = 1e-4; 
    const float V0 = 1e-4;
    const float q_a = 1e-8;
    const float b_a = 1e-8;
    const float b_g = 1e-8;

    // R matrix (measurements)
    const float A0 = 1e-3;
    const float G0 = 1e-5;

    // Gravity vector in ENU coordinates
    const Eigen::Vector3f gravity{0, 0, -9.81};

    Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world);

    // Compute H_q^{(a)} numerically
    Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(const Eigen::Quaternionf& q_nominal, const Eigen::Vector3f& accel_world, float epsilon = 1e-5);

    void run_model(float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3]);

    void computeJacobianF_tinyEKF(float dt, float omega_x, float omega_y, float omega_z, float accel_z[3], float F_out[EKF_N * EKF_N]);
};

#endif // KALMANFILTER_HPP