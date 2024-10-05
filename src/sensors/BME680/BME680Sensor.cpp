#include "BME680Sensor.h"
#include <Adafruit_BME680.h>

BME680Sensor::BME680Sensor()
{
}

BME680Sensor::~BME680Sensor()
{
    delete &data;
}

bool BME680Sensor::init()
{
    return false;
}

bool BME680Sensor::readData()
{
    return false;
}

bool BME680Sensor::readData_async()
{
    return false;
}

BME680SensorData BME680Sensor::getData()
{
    return BME680SensorData(data.getTemperature(), 
                            data.getPressure(), 
                            data.getHumidity(), 
                            data.getGasResistance());
}