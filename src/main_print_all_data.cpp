// ...existing code...
#include <Arduino.h>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <GPS.hpp>
#include <config.h>
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "utils/logger/data/LogSensorData.hpp"
#include "utils/logger/LogData.hpp"

// Sensor objects
BNO055Sensor bno;
MPRLSSensor mprls;
//GPS gps;

// Logger object
RocketLogger rocketLogger;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    
    bno.init();
    mprls.init();
    //gps.init();
}

void loop() {
    Serial.println("Test");

    // Retrieve data from all sensors
    auto mprlsDataOpt = mprls.getData();
    auto bnoDataOpt = bno.getData();
    //auto gpsDataOpt = gps.getData();

    // Create SensorData objects for each sensor
    if (mprlsDataOpt.has_value()) {
        rocketLogger.logSensorData(mprlsDataOpt.value());
    }
    if (bnoDataOpt.has_value()) {
        rocketLogger.logSensorData(bnoDataOpt.value());
    }
    /*
    if (gpsDataOpt.has_value()) {
        sensorDataList.push_back(gpsDataOpt.value());
    }
    */

    // Log the LogSensorData object
    rocketLogger.logInfo("Some info");
    rocketLogger.logWarning("Some warning");
    rocketLogger.logError("Some error");

    // Print all the logs in the logger
    Serial.println(rocketLogger.getJSONAll().dump(4).c_str());
    // Clear the logger after printing
    rocketLogger.clearData();

    delay(1000);
}