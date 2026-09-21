#include <Arduino.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void scanBus() {
  Serial.println("--- I2C scan");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("    device at 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("    total %u device(s)\n", found);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== VL53L1X diagnostic");

  Wire.begin();
  Wire.setClock(100000);
  scanBus();

  uint16_t id = sensor.readReg16Bit(VL53L1X::IDENTIFICATION__MODEL_ID);
  Serial.printf("--- model id 0x%04X (expect 0xEACC), i2c status %u\n", id, sensor.last_status);

  sensor.setTimeout(500);
  bool ok = sensor.init();
  Serial.printf("--- init %s, i2c status %u\n", ok ? "OK" : "FAILED", sensor.last_status);
  if (!ok) {
    Serial.println("=== stop");
    while (1) delay(1000);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  Serial.println("--- continuous started");
}

void loop() {
  uint8_t hvStatus = sensor.readReg(VL53L1X::GPIO__TIO_HV_STATUS);
  uint8_t statusAfterRead = sensor.last_status;

  uint16_t mm = sensor.read();
  bool timeout = sensor.timeoutOccurred();

  Serial.printf("hv_status=0x%02X i2c=%u | mm=%u status=%s%s\n",
                hvStatus, statusAfterRead, mm,
                VL53L1X::rangeStatusToString(sensor.ranging_data.range_status),
                timeout ? " TIMEOUT" : "");
  delay(200);
}
