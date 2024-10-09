#ifndef MPRLS_SENSOR_DATA_H
#define MPRLS_SENSOR_DATA_H
#include "sensors/SensorData.h"

class MPRLSSensorData : public SensorData
{
    friend class MPRLSSensor;
public:
    float getPressure() const { return this->pressure; }
    String toString() override;

private:
    MPRLSSensorData() : pressure(-10000) {}
    MPRLSSensorData(float _pressure) : pressure(_pressure) {}
    float pressure;
};
#endif // MPRLS_SENSOR_DATA_H