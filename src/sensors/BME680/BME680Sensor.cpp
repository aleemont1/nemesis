#include "BME680Sensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "sensors/BME680/BME680Sensor.h"

bool initialized = false;

BME680Sensor::BME680Sensor()
{
    bme = Adafruit_BME680(&Wire);
}

bool BME680Sensor::init()
{
    int attempts = 0;
    while (!bme.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        delay(1000);
    }
    if(attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    // Set up oversampling and filter initialization (NOTE: need to study the optimal values)
    bme.setTemperatureOversampling(BME680_OS_8X);   // Set temperature oversampling
    bme.setHumidityOversampling(BME680_OS_2X);      // Set humidity oversampling
    bme.setPressureOversampling(BME680_OS_4X);      // Set pressure oversampling
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);     // Set filter size
    bme.setGasHeater(0, 0);                         // Disable gas heater
    initialized = true;
    return initialized;
}

bool BME680Sensor::readData()
{
    if (!initialized)
    {
        Serial.println("BME680 Sensor not initialized");
        return false;
    }
    if (!bme.performReading())
    {
        Serial.println("BME680 Failed to perform reading :(");
        return false;
    } 
    Serial.println("BME680 Reading performed successfully");
    return true;
}

BME680SensorData BME680Sensor::getData()
{
    if (!initialized)
    {
        Serial.println("BME680 Sensor not initialized");
        return BME680SensorData();
    }
    return BME680SensorData(bme.temperature, bme.pressure, bme.humidity, bme.gas_resistance);
}