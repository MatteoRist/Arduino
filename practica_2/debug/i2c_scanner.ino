#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Scanning I2C...");
}

void loop() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found: 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(2000);
}
