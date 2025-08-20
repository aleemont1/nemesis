#pragma once
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/Dense"
#include "parameters_ekf.h"

// Converts pressure in hPa to altitude in meters
float pressureToAltitude(float p_hpa);

// Rotate a vector from world frame to body frame using a quaternion
Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world);

// Computes H_q^{(a)} numerically
Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float epsilon = 1e-2
);

void computeFullHJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float H_out[EKF_M * EKF_N],
    float epsilon = 1e-3
);

// Declaración de la función
void computeJacobianF_tinyEKF(
    ekf_t* ekf, 
    float dt, 
    float omega_x, 
    float omega_y, 
    float omega_z, 
    float accel_z[3], 
    float F_out[EKF_N * EKF_N], 
    float h_pressure_sensor, 
    float z_gps
);

extern void run_model(ekf_t* ekf, float dt,
               float* x, float* z,
               float omega_x, float omega_y, float omega_z,
               float accel[3],
               float h_pressure_sensor,
               float z_gps,
               int save = 0); // Declaración de la función, sin implementación aquí