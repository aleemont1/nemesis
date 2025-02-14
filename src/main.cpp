#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <optional>
#include <vector>
#include <string>
#include "global/config.h"
#include "utils/utilities/functions.h"
#include "utils/logger/ILogger.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "sensors/ISensor.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "sensors/MPRLS/MPRLSSensor.hpp"
#include "telemetry/LoRa/E220LoRaTransmitter.hpp"
#include "utils/logger/SD/SD-master.hpp"

ILogger *rocketLogger;
ILogger *dataLogger;
SD *sdModule;
ISensor *bno055;
ISensor *mprls;
ITransmitter *loraTransmitter;
HardwareSerial loraSerial(LORA_SERIAL);

// Struct to store sensor information for initialization and logging
struct SensorInfo
{
    ISensor *sensor;            // Pointer to the sensor object
    std::string name;           // Name of the sensor
    std::optional<int> address; // I2C address of the sensor (if applicable)
};

// Vector of sensors to initialize (add the used sensors here)
std::vector<SensorInfo> sensors = {
    // {bme680, "BME680", BME680_I2C_ADDR_1},
    {mprls, "MPRLS", MPRLS_I2C_ADDR},
    {bno055, "BNO055", BNO055_I2C_ADDR}};

void logTransmitterStatus(ResponseStatusContainer &transmitterStatus);
void logTransmissionResponse(ResponseStatusContainer &response);
void logInitializationResult(const std::string &sensorName, const std::optional<int> &address, bool success);
bool initSensor(ISensor *sensor, const std::string &name, const std::optional<int> &address);
void initAllSensorsAndLogStatus();

void setup()
{
    rocketLogger = new RocketLogger();
    rocketLogger->logInfo("Setup started.");

    dataLogger = new RocketLogger();

    sdModule = new SD();

    sdModule->init() ? rocketLogger->logInfo("SD card initialized.") : rocketLogger->logError("Failed to initialize SD card.");

    loraSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.begin(SERIAL_BAUD_RATE);
    //! TODO: Delete after testing phase is over.
    delay(500);
    // bme680 = new BME680Sensor(BME680_I2C_ADDR_1);
    mprls = new MPRLSSensor();
    bno055 = new BNO055Sensor();
    loraTransmitter = new E220LoRaTransmitter(loraSerial, LORA_AUX, LORA_M0, LORA_M1);

    auto transmitterStatus = loraTransmitter->init();
    logTransmitterStatus(transmitterStatus);

    initAllSensorsAndLogStatus();

    rocketLogger->logInfo("Setup complete.");
    auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
    logTransmissionResponse(response);
    //! TODO: Delete after testing phase is over.
    delay(2000);
    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
    sdModule->writeFile("log.json", rocketLogger->getJSONAll().dump(4));
    rocketLogger->clearData();
}

void loop()
{
    // Read data from all sensors inside the sensors vector and log it.
    for (const auto &[sensor, name, address] : sensors)
    {
        auto data = sensor->getData();
        if (data.has_value())
        {
            dataLogger->logSensorData(data.value());
        }
    }

    //! TODO: Test overhead of writing to SD card.
    //* note: Implement a separate task (thread) for writing to SD card. This will prevent the main loop from blocking.
    //*         The task should be able to write data to the SD card at a fixed interval (e.g. every 500ms).

    // Write SENSORs data to SD data.json file
    sdModule->openFile("data.json");
    sdModule->writeFile("data.json", dataLogger->getJSONAll().dump(4));
    sdModule->closeFile();

    //* note: Implement a separate task (thread) for transmitting data over LoRa. This will prevent the main loop from blocking.
    //*         The task should be able to transmit data over LoRa at a fixed interval (e.g. every 500ms).
    auto response = loraTransmitter->transmit(dataLogger->getJSONAll());
    logTransmissionResponse(response);

    // Write transmission response to SD log.json file
    sdModule->openFile("log.json");
    sdModule->writeFile("log.json", rocketLogger->getJSONAll().dump(4));
    sdModule->closeFile();

    //! TODO: Delete after testing phase is over.
    Serial.println("################## SENSOR DATA START ####################");
    Serial.write((dataLogger->getJSONAll().dump(4) + "\n").c_str());
    Serial.println("################## SENSOR DATA END ######################\n");

    Serial.println("################## LOG INFO START ####################");
    Serial.write((rocketLogger->getJSONAll().dump(4) + "\n").c_str());
    Serial.println("################## LOG INFO END ######################\n");
    delay(250);     //! TODO: Delete after testing phase is over.
    dataLogger->clearData();
    rocketLogger->clearData();
}

// Log transmitter initialization status
void logTransmitterStatus(ResponseStatusContainer &transmitterStatus)
{
    if (transmitterStatus.getCode() == RESPONSE_STATUS::E220_SUCCESS)
    {
        rocketLogger->logInfo(
            ("LoRa transmitter initialized with configuration: " +
             static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                .c_str());
    }
    else
    {
        rocketLogger->logError(
            ("Failed to initialize LoRa transmitter with error: " +
             transmitterStatus.getDescription() +
             " (" + String(transmitterStatus.getCode()) + ")")
                .c_str());
        rocketLogger->logInfo(("Current configuration: " +
                               static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                                  .c_str());
    }

}

// Log data transmission response
void logTransmissionResponse(ResponseStatusContainer &response)
{
    response.getCode() != RESPONSE_STATUS::E220_SUCCESS
        ? rocketLogger->logError(("Failed to transmit data with error: " + response.getDescription() + " (" + String(response.getCode()) + ")").c_str())
        : rocketLogger->logInfo("Data transmitted successfully.");
}

// Log a sensor initialization status
void logInitializationResult(const std::string &sensorName, const std::optional<int> &address, bool success)
{
    std::string addressInfo = address.has_value() ? " on address " + std::to_string(address.value()) : "";

    if (success)
    {
        rocketLogger->logInfo(sensorName + " sensor initialized" + addressInfo);
    }
    else
    {
        rocketLogger->logError("Failed to initialize " + sensorName + " sensor" + addressInfo);
    }
}

// Initialize a sensor
bool initSensor(ISensor *sensor, const std::string &name, const std::optional<int> &address)
{
    bool initSuccess = sensor->init();
    logInitializationResult(name, address, initSuccess);
    return initSuccess;
}

// Initialize the sensors inside the sensors vector and log the initialization status
void initAllSensorsAndLogStatus()
{
    for (const auto &[sensor, name, address] : sensors)
    {
        initSensor(sensor, name, address);
    }
}
