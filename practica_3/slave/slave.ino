#include <SPI.h>
#include <LoRa.h>

// IDs for addressing
const uint8_t localAddress = 0xB1;   // Slave ID
const uint8_t destination   = 0xA1;  // Master ID

// Config struct
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

// Bandwidth lookup
double bandwidth_kHz[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

// Same robust config
LoRaConfig_t thisNodeConf = { 0, 12, 8, 14 };

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // Apply config
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(12);

  // Register callback for reception
  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("Slave ready, listening...");
}

void loop() {
  // Nothing here, reception handled by callback
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // Read header
  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint16_t msgId = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  // Read payload
  uint8_t buffer[10];
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }

  Serial.print("Received binary PING from 0x");
  Serial.print(sender, HEX);
  Serial.print(" with length ");
  Serial.println(receivedBytes);

  // Decode config if payload matches expected size
  if (receivedBytes >= 4) {
    LoRaConfig_t remoteConf;
    remoteConf.bandwidth_index = buffer[0] >> 4;
    remoteConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteConf.codingRate = 5 + (buffer[1] >> 6);
    remoteConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);

    Serial.print("Remote config: BW=");
    Serial.print(bandwidth_kHz[remoteConf.bandwidth_index]);
    Serial.print(" Hz, SF=");
    Serial.print(remoteConf.spreadingFactor);
    Serial.print(", CR=");
    Serial.print(remoteConf.codingRate);
    Serial.print(", TxPwr=");
    Serial.print(remoteConf.txPower);
    Serial.println(" dBm");
  }

  // Build reply (PONG) with own config
  uint8_t payload[10];
  uint8_t len = 0;
  payload[len]    = (thisNodeConf.bandwidth_index << 4);
  payload[len++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
  payload[len]    = ((thisNodeConf.codingRate - 5) << 6);
  payload[len++] |= ((thisNodeConf.txPower - 2) << 1);
  payload[len++] = uint8_t(-LoRa.packetRssi() * 2);
  payload[len++] = uint8_t(148 + LoRa.packetSnr());

  // Send reply
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(0));
  LoRa.write((uint8_t)(2)); // Msg ID for PONG
  LoRa.write(len);
  LoRa.write(payload, len);
  LoRa.endPacket();

  Serial.println("Binary PONG sent.");
}
