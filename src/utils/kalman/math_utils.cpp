#include "parameters_ekf.h"
#include "../lib/TinyEKF-master/src/tinyekf.h"
#include "math_utils.h"


// Function to convert pressure to altitude using the barometric formula
float pressureToAltitude(float p) {
    const float p0 = 101325.0;      // sea-level standard atmospheric pressure, Pa
    const float T0 = 288.15;        // standard temperature, K
    const float L  = 0.0065;        // temperature lapse rate, K/m
    const float g0 = 9.80665;       // gravity, m/s^2
    const float R  = 8.31447;       // universal gas constant, J/(mol*K)
    const float M  = 0.0289644;     // molar mass of Earth's air, kg/mol

    // Inverse of: p = p0 * (1 - L*h/T0)^(g0*M/(R*L))
    float exponent = R * L / (g0 * M);
    float h = T0 / L * (1.0 - std::pow(p / p0, exponent));
    return h;
}

Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world) {
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

// Compute H_q^{(a)} numerically
Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float epsilon)
{
    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen = omega - bias_g; // Subtract gyroscope bias
    Eigen::Vector3f axis = omega_eigen.normalized();
    float theta = omega_eigen.norm() * dt;
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)

    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();
    
    // Eigen::Vector3f accel_world = q_rot * (accel_z - bias_a) + gravity;  // Equivalent to q * a * q.inverse()
    Eigen::Vector3f accel_world = q_rot * (accel_z - bias_a);  // Equivalent to q * a * q.inverse().

    // std::cout << "Line: " << lineNum << ", Accel: " << accel_abs.transpose() << std::endl; 
    Eigen::Matrix<float, 3, 4> H;
    Eigen::Vector4f q_vec = q_nominal.coeffs();  // (x, y, z, w)

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4f dq = Eigen::Vector4f::Zero();
        dq(i) = epsilon;

        // Perturb positively
        Eigen::Vector4f q_plus_vec = q_vec + dq;
        Eigen::Quaternionf q_plus(q_plus_vec(3), q_plus_vec(0), q_plus_vec(1), q_plus_vec(2));
        q_plus.normalize();

        // Perturb negatively
        Eigen::Vector4f q_minus_vec = q_vec - dq;
        Eigen::Quaternionf q_minus(q_minus_vec(3), q_minus_vec(0), q_minus_vec(1), q_minus_vec(2));
        q_minus.normalize();

        // Rotate vector under perturbed quaternions
        Eigen::Vector3f a_plus = rotateToBody(q_plus, accel_world);
        Eigen::Vector3f a_minus = rotateToBody(q_minus, accel_world);

        // Central difference
        H.col(i) = (a_plus - a_minus) / (2.0 * epsilon);
    }

    return H;  // size 3x4
}

void computeFullHJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float H_out[EKF_M * EKF_N],
    float epsilon
) {
    Eigen::Matrix<float, 3, 4> Hq = computeHqAccelJacobian(q_nominal, accel_z, omega, epsilon);

    // Limpiar toda la matriz
    for(int i=0;i<EKF_M*EKF_N;i++) H_out[i] = 0.0f;

    // Rellenar las derivadas de la aceleración respecto al quaternion
    for(int row=0; row<3; row++)
        for(int col=2; col<6; col++)  // estados 2,3,4,5 = q0,q1,q2,q3
            H_out[row*EKF_N + col] = Hq(row, col-2);

    // Derivadas de velocidad angular respecto al estado (solo respecto a gyro bias o quaternion si quieres)
    // Si medimos gyro directamente y no dependen del estado, ya está en 0

    H_out[6*EKF_N + 0] = 1.0f;  
    H_out[7*EKF_N + 0] = 1.0f;  
}


void computeJacobianF_tinyEKF(ekf_t* ekf, float dt, float omega_x, float omega_y, float omega_z,
                              float accel_z[3], float F_out[EKF_N * EKF_N], 
                              float h_pressure_sensor, float z_gps) {
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf->x, ekf->x + EKF_N);

    run_model(ekf, dt, fx_base, hx_dummy, omega_x, omega_y, omega_z, accel_z, h_pressure_sensor, z_gps, 0); // Added missing int argument

    for (int i = 0; i < EKF_N; ++i) {
        float epsilon = 1e-5f * std::max(fabs(ekf->x[i]), 1.0f);
        if (epsilon < 1e-6f) epsilon = 1e-6f;

        ekf->x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(ekf, dt, fx_perturbed, hx_dummy, omega_x, omega_y, omega_z, accel_z, h_pressure_sensor, z_gps, 0); // Added missing int argument

        for (int j = 0; j < EKF_N; ++j) {
            F_out[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf->x[i] = original_state[i];
    }
}
