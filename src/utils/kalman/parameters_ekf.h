#pragma once
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/Dense"

// These must be defined before including TinyEKF.h
#define EKF_N 6 // Size of state space [1-position, 1-velocity, 4-quaternion_rot] 
#define EKF_M 8 // Size of measurement space [3-accelerations, 3-angular velocities, 1-GPS, 1-pressure]

// ---------------------------
// Constantes simples
// ---------------------------
inline constexpr float dt = 0.1f; // 10 Hz

// How much we trust the Model
inline constexpr float P0 = 1e-3f;   // position (m²)
inline constexpr float V0 = 1e-4f;   // velocity (m²/s²)
inline constexpr float q_a = 1e-6f;  // quaternion

// How much we trust the Sensors
inline constexpr float A0 = 1e-3f;       // accelerometer (m/s²)
inline constexpr float G0 = 5e-4f;       // gyroscope (rad/s)
inline constexpr float GPS_Z0 = 3.0f;    // GPS altitude (m²)


inline constexpr float SeaLevel = 165.0f;
inline constexpr float gps_bias = 3.0f;
inline constexpr float h_bias_pressure_sensor = 2.0f;

// ---------------------------
// Bias de sensores (vectores)
// ---------------------------
extern const Eigen::Vector3f bias_a; // acelerómetro
extern const Eigen::Vector3f bias_g; // giroscopio
extern const Eigen::Vector3f gravity; // Gravity vector in ENU coordinates

// ---------------------------
// Declaración de matrices EKF
// ---------------------------
extern const float Q[EKF_N*EKF_N];  // matriz de covarianza de proceso
extern float R[EKF_M*EKF_M];  // matriz de covarianza de medición
extern float H[EKF_M*EKF_N];  // matriz de observación
extern float F[EKF_N*EKF_N];  // matriz jacobiana del modelo de estado

float estimateBaroVar(float v); // Función para estimar la varianza del barómetro
float pressureToAltitude(float pressure); // Función para convertir presión a altitud
void initializeCSVFiles(); // Función para inicializar archivos CSV