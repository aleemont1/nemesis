#include <vector>
#include <tuple>
#include <BNO055Sensor.hpp>
 
static BNO055Sensor bno;

void setup()
{
    Serial.begin(115200);
    
    if (!bno.init()) {
        while(1) {
            Serial.println("Failed to initialize BNO055");
        }
    }
}
 
void loop()
{
    auto bnoDataOpt = bno.getData();
    if (!bnoDataOpt.has_value()) {
        Serial.println("BNO055 data not available");
        return;
    }
    auto bnoData = bnoDataOpt.value();

    float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
    float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
    float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);

    float bno_gravity_xyz = sqrt(pow(bno_gravity_x, 2) + pow(bno_gravity_y, 2) + pow(bno_gravity_z, 2));

    Serial.println("Gravity x: " + String(bno_gravity_x) + 
                   ", y: " + String(bno_gravity_y) + 
                   ", z: " + String(bno_gravity_z) +
                   ", xyz: " + String(bno_gravity_xyz));
    
    delay(10);
}