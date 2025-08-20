#include "parameters_ekf.h"

// ---------------------------
// Bias de sensores (definición real)
// ---------------------------
const Eigen::Vector3f bias_a(0.05f, -0.05f, 0.025f);
const Eigen::Vector3f bias_g(0.05f, -0.05f, 0.025f);
const Eigen::Vector3f gravity(0, 0, -9.803);

float estimateBaroVar(float v) {
    float std = (std::abs(v) / 300.0f) * 29.0f + 1.0f;
    return std * std;
}

// ---------------------------
// Matrices EKF (definición real)
// ---------------------------
// Set fixed process-noise covariance matrix Q, see [1]  ---------------------

const float Q[EKF_N*EKF_N] = {

    P0, 0, 0, 0, 0, 0,
    0, V0, 0, 0, 0, 0,
    0, 0, q_a, 0, 0, 0,
    0, 0, 0, q_a, 0, 0,
    0, 0, 0, 0, q_a, 0,
    0, 0, 0, 0, 0, q_a
};

// Set fixed measurement noise covariance matrix R ----------------------------

float R[EKF_M*EKF_M] = {
    A0, 0, 0, 0, 0, 0, 0, 0,
    0, A0, 0, 0, 0, 0, 0, 0,
    0, 0, A0, 0, 0, 0, 0, 0,
    0, 0, 0, G0, 0, 0, 0, 0,
    0, 0, 0, 0, G0, 0, 0, 0,
    0, 0, 0, 0, 0, G0, 0, 0,
    0, 0, 0, 0, 0, 0, estimateBaroVar(0), 0,
    0, 0, 0, 0, 0, 0, 0, GPS_Z0
};

// Jacobian matrix
float F[EKF_N*EKF_N] = {
    1, dt, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,

    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1

};

// Initially, the acceleration is constantly zero, so it won't change
float H[EKF_M*EKF_N] = {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0
};