#include <Wire.h>
#include <VL53L0X.h>

#define MY_SDA 8
#define MY_SCL 9

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(MY_SDA, MY_SCL);
  Wire.setClock(100000); // 更稳

  Serial.println("Starting VL53L0X...");

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setTimeout(500);

  // --- 稳定测距到 1.3 米 ---
  sensor.setSignalRateLimit(0.1);
  sensor.setMeasurementTimingBudget(50000); 
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  Serial.println("VL53L0X ready!");
}

void loop() {
  uint16_t distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout!");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  delay(50);
}

