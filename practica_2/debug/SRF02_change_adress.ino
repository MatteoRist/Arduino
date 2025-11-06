#include <Wire.h>

#define CURRENT_ADDR 0x70        // obecny 7-bitowy adres SRF02
#define NEW_ADDR_8BIT 0xE2       // nowy adres w formacie 8-bitowym (np. 0xF2)
#define COMMAND_REGISTER 0x00    // rejestr poleceń SRF02

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  delay(1000);

  Serial.print("Changing SRF02 address from 0x");
  Serial.print(CURRENT_ADDR, HEX);
  Serial.println("...");

  // Sekwencja zmiany adresu: 0xA0, 0xAA, 0xA5, NEW_ADDR
  sendCommand(0xA0);
  delay(50);
  sendCommand(0xAA);
  delay(50);
  sendCommand(0xA5);
  delay(50);
  sendCommand(NEW_ADDR_8BIT);
  delay(100);

  Serial.println("Address change done!");
  Serial.print("New 7-bit address: 0x");
  Serial.println(NEW_ADDR_8BIT >> 1, HEX);
}

void loop() {
  // nic nie robimy — adres już zmieniony
}

void sendCommand(byte cmd) {
  Wire.beginTransmission(CURRENT_ADDR);
  Wire.write(COMMAND_REGISTER);
  Wire.write(cmd);
  Wire.endTransmission();
}
