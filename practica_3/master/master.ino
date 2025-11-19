#include <SPI.h>
#include <LoRa.h>

// IDs for addressing
const uint8_t localAddress = 0xA1;   // Master ID
const uint8_t destination   = 0xB1;  // Slave ID

// Struct to hold LoRa config
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

// Bandwidth lookup table
double bandwidth_kHz[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

// Robust config: narrow BW, high SF, strong CR, moderate Tx power
LoRaConfig_t thisNodeConf = { 0, 12, 8, 14 };

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Init LoRa at 868 MHz
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // Apply chosen parameters
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(12);

  Serial.println("Master ready.");
}

void loop() {
  static unsigned long lastPing = 0;

  // Send every 2 seconds
  if (millis() - lastPing > 2000) {
    lastPing = millis();

    uint8_t payload[10];
    uint8_t len = 0;

    // Encode config into binary payload
    payload[len]    = (thisNodeConf.bandwidth_index << 4);
    payload[len++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[len]    = ((thisNodeConf.codingRate - 5) << 6);
    payload[len++] |= ((thisNodeConf.txPower - 2) << 1);

    // Add RSSI/SNR from last packet
    payload[len++] = uint8_t(-LoRa.packetRssi() * 2);
    payload[len++] = uint8_t(148 + LoRa.packetSnr());

    // Build and send packet
    LoRa.beginPacket();
    LoRa.write(destination);   // Who to send to
    LoRa.write(localAddress);  // Who is sending
    LoRa.write((uint8_t)(0));  // Msg ID high byte
    LoRa.write((uint8_t)(1));  // Msg ID low byte
    LoRa.write(len);           // Payload length
    LoRa.write(payload, len);  // Payload itself
    LoRa.endPacket();

    Serial.println("Binary PING sent.");
  }
}
