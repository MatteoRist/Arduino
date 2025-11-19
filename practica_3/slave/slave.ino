#include <SPI.h>
#include <LoRa.h>

const uint8_t localAddress = 0xB1;   // Slave
const uint8_t destination   = 0xA1;  // Master

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

LoRaConfig_t currentConf = { 0, 12, 8, 14 };

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  applyConfig(currentConf);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("Slave ready.");
}

void loop() {
  // Passive, waits for PING
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint16_t msgId = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t buffer[10];
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }

  if (msgId == 1) { // PING
    Serial.print("PING received from 0x");
    Serial.println(sender, HEX);

    // Reply with PONG
    uint8_t payload[10];
    uint8_t len = encodeConfig(currentConf, payload);

    LoRa.beginPacket();
    LoRa.write(destination);
    LoRa.write(localAddress);
    LoRa.write((uint8_t)(0));
    LoRa.write((uint8_t)(2)); // Msg ID for PONG
    LoRa.write(len);
    LoRa.write(payload, len);
    LoRa.endPacket();

    Serial.println("PONG sent.");
  }
}

// --- Helper functions ---

void applyConfig(LoRaConfig_t conf) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
  LoRa.setSpreadingFactor(conf.spreadingFactor);
  LoRa.setCodingRate4(conf.codingRate);
  LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(12);
}

uint8_t encodeConfig(LoRaConfig_t conf, uint8_t* payload) {
  uint8_t len = 0;
  payload[len]    = (conf.bandwidth_index << 4);
  payload[len++] |= ((conf.spreadingFactor - 6) << 1);
  payload[len]    = ((conf.codingRate - 5) << 6);
  payload[len++] |= ((conf.txPower - 2) << 1);
  payload[len++] = uint8_t(-LoRa.packetRssi() * 2);
  payload[len++] = uint8_t(148 + LoRa.packetSnr());
  return len;
}