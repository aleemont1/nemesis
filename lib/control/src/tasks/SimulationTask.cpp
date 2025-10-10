// SimulationTask implementation
#include "SimulationTask.hpp"
#include <sstream>

bool SimulationTask::firstTime = true;
unsigned long SimulationTask::startTime = millis();
SD SimulationTask::sdManager;
std::string SimulationTask::csvFilePath;

SimulationTask::SimulationTask(const std::string& csvFilePathPar,
                                                             std::shared_ptr<SharedSensorData> sensorData,
                                                             SemaphoreHandle_t mutex,
                                                             std::shared_ptr<RocketLogger> rocketLogger,
                                                             SemaphoreHandle_t loggerMutex)
        : BaseTask("SimulationTask"), sensorData(sensorData),
            dataMutex(mutex), rocketLogger(rocketLogger), loggerMutex(loggerMutex) {
    csvFilePath = csvFilePathPar;
    if (!sdManager.init()) {
        sdManager.openFile(csvFilePath);
    }
}

SimulationTask::~SimulationTask() {
    // Do not close file here, keep it open for all instances
}

void SimulationTask::onTaskStart() {
    if (firstTime) {
        startTime = millis();
    }
    started = true;
}

void SimulationTask::onTaskStop() {
    started = false;
}

void SimulationTask::reset() {
    currentIndex = 0;
    started = false;
}

void SimulationTask::taskFunction() {
    auto now = millis();
    double elapsed = now - startTime;

    // Read the next row from the file only if its timestamp <= elapsed
    if (firstTime) {
        String header = sdManager.readLine(); // skip header
        firstTime = false;
    }

    String line = sdManager.readLine();
    if (line.length() > 0) {
        std::stringstream ss(line.c_str());
        std::string cell;
        // Read time
        std::getline(ss, cell, ',');
        // Parse CSV columns into variables
        double AccBodyX_ms2, AccBodyY_ms2, AccBodyZ_ms2;
        double AccSensorX_ms2, AccSensorY_ms2, AccSensorZ_ms2;
        double OmegaBody_degs, OmegaSensor_degs;
        double OmegaBody_degs_Y, OmegaSensor_degs_Y;
        double OmegaBody_degs_Z, OmegaSensor_degs_Z;
        double Z_real_m, Z_sensor_m;
        double Pressure_real_Pa, Pressure_sensor_Pa;

        std::vector<double> values;
        while (std::getline(ss, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        
        // Assign variables from values vector
        AccBodyX_ms2 = values[0];
        AccBodyY_ms2 = values[1];
        AccBodyZ_ms2 = values[2];
        AccSensorX_ms2 = values[3];
        AccSensorY_ms2 = values[4];
        AccSensorZ_ms2 = values[5];
        OmegaBody_degs = values[6];
        OmegaSensor_degs = values[7];
        OmegaBody_degs_Y = values[8];
        OmegaSensor_degs_Y = values[9];
        OmegaBody_degs_Z = values[10];
        OmegaSensor_degs_Z = values[11];
        Z_real_m = values[12];
        Z_sensor_m = values[13];
        Pressure_real_Pa = values[14];
        Pressure_sensor_Pa = values[15];
        
        // Mappings between values and BNO055 data fields
        SensorData bnoData("BNO055");
        bnoData.setData("system_calibration", 3);
        bnoData.setData("gyro_calibration", 3);
        bnoData.setData("accel_calibration", 3);
        bnoData.setData("mag_calibration", 3);

        bnoData.setData("linear_acceleration", std::map<std::string, float>{
            {"x", static_cast<float>(AccBodyX_ms2)},
            {"y", static_cast<float>(AccBodyY_ms2)},
            {"z", static_cast<float>(AccBodyZ_ms2)}});

        bnoData.setData("angular_velocity", std::map<std::string, float>{
            {"x", static_cast<float>(OmegaBody_degs)},
            {"y", static_cast<float>(OmegaBody_degs_Y)},
            {"z", static_cast<float>(OmegaBody_degs_Z)}});

        // Mappings between MS561101BA03 data fields and columns
        SensorData baroData1 = SensorData("MS561101BA03");
        baroData1.setData("pressure", Pressure_sensor_Pa);

        SensorData baroData2 = SensorData("MS561101BA03");
        baroData2.setData("pressure", Pressure_sensor_Pa);

        // Mappings between LIS3DHTR data fields and data fields
        SensorData accData = SensorData("LIS3DHTR");
        accData.setData("accel_x", AccSensorX_ms2);
        accData.setData("accel_y", AccSensorY_ms2);
        accData.setData("accel_z", AccSensorZ_ms2);
        accData.setData("accel_magnitude", std::sqrt(AccSensorX_ms2 * AccSensorX_ms2 +
                                                    AccSensorY_ms2 * AccSensorY_ms2 +
                                                    AccSensorZ_ms2 * AccSensorZ_ms2));
        
        // Mapping between GPS data fields and data fields                                                            
        SensorData gpsData = SensorData("GPS");
        gpsData.setData("altitude",  Z_sensor_m);

        // Inject into shared sensor data
        if (sensorData && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            sensorData->imuData = bnoData;
            sensorData->baroData1 = baroData1;
            sensorData->baroData2 = baroData2;
            //sensorData->accData = accData;
            sensorData->gpsData = gpsData;
            sensorData->timestamp = static_cast<uint32_t>(now);
            sensorData->dataValid = true;
        }
    } else {
        // Not enough time has passed, rewind to previous position
        LOG_INFO("SimulationTask", "Waiting for time to pass to read next data point.");
    }

    // Sleep for 100ms, which is the time gap between simulation data points
    vTaskDelay(pdMS_TO_TICKS(100));
}