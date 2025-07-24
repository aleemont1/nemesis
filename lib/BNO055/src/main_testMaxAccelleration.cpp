#include <vector>
#include <tuple>
#include <BNO055Sensor.hpp>
 
static BNO055Sensor bno;

void setup()
{
    Serial.begin(115200);
    
    if (!bno.init()) {
        Serial.println("Failed to initialize BNO055");
        while(1);
    }
    
    // Verify we're in NDOF mode
    uint8_t mode = bno.setOperationMode(0x0C); // OPERATION_MODE_NDOF = 0x0C
    
    if (mode == 0x0C) {
        Serial.println("BNO055 is in NDOF mode - OK");
    } else {
        Serial.println("WARNING: BNO055 is NOT in NDOF mode");
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