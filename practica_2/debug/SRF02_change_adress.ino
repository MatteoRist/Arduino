#include <Wire.h>

#define CURRENT_ADDR 0x70        // current 7 bit adress
#define NEW_ADDR_8BIT 0xE2       // new adress in 8 bit format
#define COMMAND_REGISTER 0x00    //

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  delay(1000);

  Serial.print("Changing SRF02 address from 0x");
  Serial.print(CURRENT_ADDR, HEX);
  Serial.println("...");

  // Changing adress sequence: 0xA0, 0xAA, 0xA5, NEW_ADDR
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

}

void sendCommand(byte cmd) {
  Wire.beginTransmission(CURRENT_ADDR);
  Wire.write(COMMAND_REGISTER);
  Wire.write(cmd);
  Wire.endTransmission();
}
