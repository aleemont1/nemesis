/* SensorFusion: Sensor fusion on Arduino using TinyEKF.  
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */


// BNO055
// orientation_x, orientation_y, orientation_z,
// angVelocity_x, angVelocity_y, angVelocity_z,
// linearAccel_x, linearAccel_y, linearAccel_z,
// magnetometer_x, magnetometer_y, magnetometer_z,
// accelerometer_x, accelerometer_y, accelerometer_z,
// gravity_x, gravity_y, gravity_z, 
// quaternion_w, quaternion_x, quaternion_y, quaternion_z
// MPRLS
// pressure
#define EKF_N 24
#define EKF_M 24

static const uint8_t LM35_PIN = 0;

#include <tinyekf.h>
#include "./sensors/MPRLS/MPRLSSensor.hpp"
#include "./sensors/BNO055/BNO055Sensor.hpp"
#include <Wire.h>

static const float EPS = 1e-4;

static const float Q[EKF_N*EKF_N];
static const float R[EKF_M*EKF_M];

static const float F[EKF_N*EKF_N];
static const float H[EKF_M*EKF_N];

static MPRLSSensor mprls;
static BNO055Sensor bno;

static ekf_t _ekf;

void setup() 
{
    // Initialize the Q and R matrices
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            Q[i * EKF_N + j] = (i == j) ? EPS : 0;
        }
    }

    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_M; j++) {
            R[i * EKF_M + j] = (i == j) ? EPS : 0;
        }
    }

    // Initialize the F and H Jacobian matrices
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            F[i * EKF_N + j] = (i == j) ? 1 : 0;
        }
    }

    for (int i = 0; i < EKF_M; i++) {
        for (int j = 0; j < EKF_N; j++) {
            H[i * EKF_N + j] = (i == j) ? 1 : 0;
        }
    }

    // Use identity matrix as initiali covariance matrix
    const float Pdiag[EKF_N] = {1};
    
    for (int i = 0; i < EKF_N; i++) {
        P[i] = 1;
    }

    // Initialize the sensors
    mprls.init();
    bno.init();
    
    ekf_initialize(&_ekf, Pdiag);

    Serial.begin(115200);
}

void loop()
{
    // Read pressure from BMP180
    const SensorData baroData = mprls.getData();
    const float mprls_pressure = baroData.value("pressure");

    // Read temperature from BMP180
    const SensorData baroData = mprls.getData();
    
    const float bno_orientation = baroData.value("orientation");
    const float bno_orientation_x = bno_orientation["x"];
    const float bno_orientation_y = bno_orientation["y"];
    const float bno_orientation_z = bno_orientation["z"];

    const float bno_angVelocity = baroData.value("angular_velocity");
    const float bno_angVelocity_x = bno_angVelocity["x"];
    const float bno_angVelocity_y = bno_angVelocity["y"];
    const float bno_angVelocity_z = bno_angVelocity["z"];

    const float bno_linearAccel = baroData.value("linear_acceleration");
    const float bno_linearAccel_x = bno_linearAccel["x"];
    const float bno_linearAccel_y = bno_linearAccel["y"];
    const float bno_linearAccel_z = bno_linearAccel["z"];

    const float bno_magnetometer = baroData.value("magnetometer");
    const float bno_magnetometer_x = bno_magnetometer["x"];
    const float bno_magnetometer_y = bno_magnetometer["y"];
    const float bno_magnetometer_z = bno_magnetometer["z"];

    const float bno_accelerometer = baroData.value("accelerometer");
    const float bno_accelerometer_x = bno_accelerometer["x"];
    const float bno_accelerometer_y = bno_accelerometer["y"];
    const float bno_accelerometer_z = bno_accelerometer["z"];

    const float bno_gravity = baroData.value("gravity");
    const float bno_gravity_x = bno_gravity["x"];
    const float bno_gravity_y = bno_gravity["y"];
    const float bno_gravity_z = bno_gravity["z"];

    const float bno_temperature = baroData.value("board_temperature");

    const float bno_quaternion = baroData.value("quaternion");
    const float bno_quaternion_w = bno_quaternion["w"];
    const float bno_quaternion_x = bno_quaternion["x"];
    const float bno_quaternion_y = bno_quaternion["y"];
    const float bno_quaternion_z = bno_quaternion["z"];

    // Set the observation vector z
    const float z[EKF_M] = {
        mprls_pressure, 
        bno_orientation_x, bno_orientation_y, bno_orientation_z, 
        bno_angVelocity_x, bno_angVelocity_y, bno_angVelocity_z, 
        bno_linearAccel_x, bno_linearAccel_y, bno_linearAccel_z, 
        bno_magnetometer_x, bno_magnetometer_y, bno_magnetometer_z, 
        bno_accelerometer_x, bno_accelerometer_y, bno_accelerometer_z, 
        bno_gravity_x, bno_gravity_y, bno_gravity_z, 
        bno_quaternion_w, bno_quaternion_x, bno_quaternion_y, bno_quaternion_z, 
        bno_temperature};

    // Process model is f(x) = x
    const float fx[EKF_N] = { 
        _ekf.x[0], 
        _ekf.x[1], _ekf.x[2], _ekf.x[3],
        _ekf.x[4], _ekf.x[5], _ekf.x[6],
        _ekf.x[7], _ekf.x[8], _ekf.x[9],
        _ekf.x[10], _ekf.x[11], _ekf.x[12],
        _ekf.x[13], _ekf.x[14], _ekf.x[15],
        _ekf.x[16], _ekf.x[17], _ekf.x[18],
        _ekf.x[19], _ekf.x[20], _ekf.x[21], _ekf.x[22],
        _ekf.x[23]};

    // Run the prediction step of the DKF
    ekf_predict(&_ekf, fx, F, Q);

    // Measurement function simplifies the relationship between state
    // and sensor readings for convenience.  A more realistic
    // measurement function would distinguish between state value and
    // measured value; e.g.:
    //   hx[0] = pow(this->x[0], 1.03);
    //   hx[1] = 1.005 * this->x[1];
    //   hx[2] = .9987 * this->x[1] + .001;
    const float hx[EKF_M] = { 
        _ekf.x[0], 
        _ekf.x[1], _ekf.x[2], _ekf.x[3],
        _ekf.x[4], _ekf.x[5], _ekf.x[6],
        _ekf.x[7], _ekf.x[8], _ekf.x[9],
        _ekf.x[10], _ekf.x[11], _ekf.x[12],
        _ekf.x[13], _ekf.x[14], _ekf.x[15],
        _ekf.x[16], _ekf.x[17], _ekf.x[18],
        _ekf.x[19], _ekf.x[20], _ekf.x[21], _ekf.x[22],
        _ekf.x[23]};

    // Run the update step
    ekf_update(&_ekf, z, hx, H, R);

    // Report measured and predicte/fused values
    Serial.println("MPRLS Pressure:");
    Serial.println(z[0]);
    Serial.println();
    Serial.println("BNO055 Orientation:");
    Serial.println(z[1]);
    Serial.println(z[2]);
    Serial.println(z[3]);
    Serial.println();
    Serial.println("BNO055 Angular Velocity:");
    Serial.println(z[4]);
    Serial.println(z[5]);
    Serial.println(z[6]);
    Serial.println();
    Serial.println("BNO055 Linear Acceleration:");
    Serial.println(z[7]);
    Serial.println(z[8]);
    Serial.println(z[9]);
    Serial.println();
    Serial.println("BNO055 Magnetometer:");
    Serial.println(z[10]);
    Serial.println(z[11]);
    Serial.println(z[12]);
    Serial.println();
    Serial.println("BNO055 Accelerometer:");
    Serial.println(z[13]);
    Serial.println(z[14]);
    Serial.println(z[15]);
    Serial.println();
    Serial.println("BNO055 Gravity:");
    Serial.println(z[16]);
    Serial.println(z[17]);
    Serial.println(z[18]);
    Serial.println();
    Serial.println("BNO055 Quaternion:");
    Serial.println(z[19]);
    Serial.println(z[20]);
    Serial.println(z[21]);
    Serial.println(z[22]);
    Serial.println();
    Serial.println("BNO055 Temperature:");
    Serial.println(z[23]);
    Serial.println();
    Serial.println("EKF Pressure:");
    Serial.println(_ekf.x[0]);
    Serial.println();
    Serial.println("EKF Orientation:");
    Serial.println(_ekf.x[1]);
    Serial.println(_ekf.x[2]);
    Serial.println(_ekf.x[3]);
    Serial.println();
    Serial.println("EKF Angular Velocity:");
    Serial.println(_ekf.x[4]);
    Serial.println(_ekf.x[5]);
    Serial.println(_ekf.x[6]);
    Serial.println();
    Serial.println("EKF Linear Acceleration:");
    Serial.println(_ekf.x[7]);
    Serial.println(_ekf.x[8]);
    Serial.println(_ekf.x[9]);
    Serial.println();
    Serial.println("EKF Magnetometer:");
    Serial.println(_ekf.x[10]);
    Serial.println(_ekf.x[11]);
    Serial.println(_ekf.x[12]);
    Serial.println();
    Serial.println("EKF Accelerometer:");
    Serial.println(_ekf.x[13]);
    Serial.println(_ekf.x[14]);
    Serial.println(_ekf.x[15]);
    Serial.println();
    Serial.println("EKF Gravity:");
    Serial.println(_ekf.x[16]);
    Serial.println(_ekf.x[17]);
    Serial.println(_ekf.x[18]);
    Serial.println();
    Serial.println("EKF Quaternion:");
    Serial.println(_ekf.x[19]);
    Serial.println(_ekf.x[20]);
    Serial.println(_ekf.x[21]);
    Serial.println(_ekf.x[22]);
    Serial.println();
    Serial.println("EKF Temperature:");
    Serial.println(_ekf.x[23]);
    Serial.println();
    //delay(1000);
}



