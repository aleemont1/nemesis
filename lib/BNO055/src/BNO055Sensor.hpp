#pragma once

#include <Adafruit_BNO055.h>
#include <ISensor.hpp>
#include <BNO055SensorInterface.hpp>
#include <config.h>

class BNO055Sensor : public ISensor
{
public:
    BNO055Sensor();
    bool init() override;
    bool calibrate();
    bool hardwareTest();
    std::optional<SensorData> getData() override;
    

private:
    BNO055SensorInterface bno_interface;
};

