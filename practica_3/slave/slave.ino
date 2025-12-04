#include <SPI.h>
#include <LoRa.h>

// --- IDs ---
const uint8_t localAddress = 0xB1;   // Slave
const uint8_t destination  = 0xA1;   // Master

// --- Message Types ---
const uint8_t MSG_PING       = 0x01;
const uint8_t MSG_PONG       = 0x02;
const uint8_t MSG_CONFIG     = 0x03;
const uint8_t MSG_ACK        = 0x04;

// --- Timing ---
// Timeout should be slightly longer than Master's to avoid premature reverts, 
// but short enough to catch the Master if it reverts.
const unsigned long TIMEOUT_MS = 11000; 
unsigned long lastPacketTime = 0;

// --- Configuration ---
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

// 1. Default Robust Config (Fallback)
const LoRaConfig_t defaultConf = { 0, 12, 8, 3 }; 

// 2. Current Active Config
LoRaConfig_t currentConf = defaultConf;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // Initial setup
  applyConfig(currentConf);
  
  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("Slave ready. Listening...");
  lastPacketTime = millis();
}

void loop() {
  // Watchdog: If no packets received for TIMEOUT, revert to default
  if (millis() - lastPacketTime > TIMEOUT_MS) {
    // Only print/revert if we aren't already on default to avoid serial spam
    if (memcmp(&currentConf, &defaultConf, sizeof(LoRaConfig_t)) != 0) {
      Serial.println("!!! Timeout: Link Lost. Reverting to Default Config. !!!");
      currentConf = defaultConf;
      applyConfig(currentConf);
    }
    // Reset timer so we don't spam the revert logic
    lastPacketTime = millis(); 
  }
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // Read header
  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t incomingLength = LoRa.read();

  if (recipient != localAddress) return;

  // Valid packet received -> Reset Watchdog
  lastPacketTime = millis();

  // Read Payload
  uint8_t buffer[10];
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }

  Serial.print("Rx MsgID: "); Serial.println(msgId);

  // --- Handle Message Types ---
  
  if (msgId == MSG_PING) {
    sendPong();
  } 
  else if (msgId == MSG_CONFIG) {
    // 1. Decode new config
    LoRaConfig_t newConf;
    decodeConfig(buffer, &newConf);
    
    Serial.println("Config Request Received.");
    Serial.print("Proposed -> SF:"); Serial.print(newConf.spreadingFactor);
    Serial.print(" BW_IDX:"); Serial.println(newConf.bandwidth_index);

    // 2. Acknowledge receipt (using CURRENT settings so Master hears us)
    sendAck();

    // 3. Apply new settings locally
    currentConf = newConf;
    applyConfig(currentConf);
    Serial.println("*** Local Configuration Updated ***");
  }
}

// --- Transmission Functions ---

void sendPong() {
  // Get telemetry of the packet we just received
  int packetRSSI = LoRa.packetRssi();
  int packetSNR  = LoRa.packetSnr();

  // Encode telemetry into payload for Master to analyze
  // Master expects: [Abs(RSSI)*2, SNR+148]
  uint8_t payload[2];
  payload[0] = (uint8_t)(abs(packetRSSI) * 2);
  payload[1] = (uint8_t)(packetSNR + 148);

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PONG);
  LoRa.write(2); // Length
  LoRa.write(payload, 2);
  LoRa.endPacket();
  
  LoRa.receive(); // Go back to listen mode
  Serial.println("-> Sent PONG");
}

void sendAck() {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_ACK);
  LoRa.write(0);
  LoRa.endPacket();
  
  LoRa.receive(); // Go back to listen mode
  Serial.println("-> Sent ACK");
}

// --- Helper Functions ---

void applyConfig(LoRaConfig_t conf) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
  LoRa.setSpreadingFactor(conf.spreadingFactor);
  LoRa.setCodingRate4(conf.codingRate);
  LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(12);
}

// Inverse of Master's encodeConfig
void decodeConfig(uint8_t* payload, LoRaConfig_t* conf) {
  // Master Encode: 
  // payload[0] = (bw << 4) | ((sf - 6) << 1)
  // payload[1] = ((cr - 5) << 6) | ((tx - 2) << 1)

  conf->bandwidth_index = (payload[0] >> 4) & 0x0F;
  conf->spreadingFactor = ((payload[0] >> 1) & 0x07) + 6;
  conf->codingRate      = ((payload[1] >> 6) & 0x03) + 5;
  conf->txPower         = ((payload[1] >> 1) & 0x1F) + 2;
}