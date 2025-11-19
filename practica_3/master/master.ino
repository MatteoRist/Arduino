#include <SPI.h>
#include <LoRa.h>

// IDs
const uint8_t localAddress = 0xA1;   // Master
const uint8_t destination   = 0xB1;  // Slave

// Timeout in ms
const unsigned long TIMEOUT_MS = 10000;

// Thresholds for link quality
const int RSSI_THRESHOLD = -100;   // dBm
const int SNR_THRESHOLD  = 5;      // dB

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

// Start with robust config
LoRaConfig_t currentConf = { 0, 12, 8, 14 };

unsigned long pingSentTime = 0;
bool waitingForReply = false;

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

  Serial.println("Master ready.");
}

void loop() {
  // Timeout check
  if (waitingForReply && (millis() - pingSentTime > TIMEOUT_MS)) {
    Serial.println("Timeout: no PONG received.");
    waitingForReply = false;
  }

  // If not waiting, send new PING
  if (!waitingForReply) {
    sendPing();
  }
}

void sendPing() {
  uint8_t payload[10];
  uint8_t len = encodeConfig(currentConf, payload);

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(0));
  LoRa.write((uint8_t)(1)); // Msg ID for PING
  LoRa.write(len);
  LoRa.write(payload, len);
  LoRa.endPacket();

  pingSentTime = millis();
  waitingForReply = true;

  Serial.println("PING sent, waiting for PONG...");
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

  if (msgId == 2 && waitingForReply) { // PONG
    unsigned long rtt = millis() - pingSentTime;
    waitingForReply = false;

    // Decode remote RSSI/SNR
    int remoteRSSI = -int(buffer[2]) / 2;
    int remoteSNR  = int(buffer[3]) - 148;

    Serial.print("PONG received | RTT=");
    Serial.print(rtt);
    Serial.print(" ms | RSSI=");
    Serial.print(remoteRSSI);
    Serial.print(" dBm | SNR=");
    Serial.print(remoteSNR);
    Serial.println(" dB");

    // Optimization step
    if (remoteRSSI > RSSI_THRESHOLD && remoteSNR > SNR_THRESHOLD) {
      adjustParameters();
    } else {
      Serial.println("Link too weak, keeping current config.");
    }
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

void adjustParameters() {
  // Example: step down SF first, then widen BW
  if (currentConf.spreadingFactor > 7) {
    currentConf.spreadingFactor--;
    Serial.print("Adjusting: new SF=");
    Serial.println(currentConf.spreadingFactor);
  } else if (currentConf.bandwidth_index < 9) {
    currentConf.bandwidth_index++;
    Serial.print("Adjusting: new BW=");
    Serial.println(bandwidth_kHz[currentConf.bandwidth_index]);
  } else if (currentConf.codingRate > 5) {
    currentConf.codingRate--;
    Serial.print("Adjusting: new CR=");
    Serial.println(currentConf.codingRate);
  } else if (currentConf.txPower > 2) {
    currentConf.txPower--;
    Serial.print("Adjusting: new TxPower=");
    Serial.println(currentConf.txPower);
  } else {
    Serial.println("Optimization complete: cannot reduce further.");
  }

  applyConfig(currentConf);
}
