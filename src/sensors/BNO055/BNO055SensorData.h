#ifndef BNO055_SENSOR_DATA_H
#define BNO055_SENSOR_DATA_H
#include "sensors/SensorData.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class BNO055SensorData : public SensorData
{
    public:
        BNO055SensorData() : board_temperature(-1), system_calibration(0), gyro_calibration(0), accel_calibration(0), mag_calibration(0) {}

        sensors_event_t getOrientation() const;
        sensors_event_t getAngularVelocity() const;
        sensors_event_t getLinearAcceleration() const;
        sensors_event_t getMagnetometer() const;
        sensors_event_t getAccelerometer() const;
        sensors_event_t getGravity() const;
        int8_t getBoardTemperature() const;
        uint8_t getSystemCalibration() const;
        uint8_t getGyroCalibration() const;
        uint8_t getAccelCalibration() const;
        uint8_t getMagCalibration() const;
        String toString() override;
    private:
        sensors_event_t orientation;
        sensors_event_t angularVelocity;
        sensors_event_t linearAcceleration;
        sensors_event_t magnetometer;
        sensors_event_t accelerometer;
        sensors_event_t gravity;
        int8_t board_temperature;
        uint8_t system_calibration;
        uint8_t gyro_calibration;
        uint8_t accel_calibration;
        uint8_t mag_calibration;
};
#endif // BNO055_SENSOR_DATA_H