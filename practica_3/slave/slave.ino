#include <SPI.h>
#include <LoRa.h>

// --- IDs ---
const uint8_t localAddress = 0xB2;   // Slave (Must match Master's destination)
const uint8_t destination  = 0xA2;   // Master (Must match Master's localAddress)

// --- Message Types ---
const uint8_t MSG_PING       = 0x01;
const uint8_t MSG_PONG       = 0x02; // (Deprecated)
const uint8_t MSG_CONFIG     = 0x03;
const uint8_t MSG_ACK        = 0x04;
const uint8_t MSG_REPORT     = 0x05; // New Summary Report

// --- Timing ---
const unsigned long TIMEOUT_MS = 11000; 
unsigned long lastPacketTime = 0;

// --- Burst Collection State ---
bool collectingBurst = false;
unsigned long lastBurstPacketTime = 0;
const int BURST_SILENCE_TIMEOUT = 150; // ms to wait before sending report
int burstReceivedCount = 0;
long burstSumRSSI = 0;
long burstSumSNR = 0;
uint8_t currentBatchId = 0;

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
const LoRaConfig_t defaultConf = { 8, 9, 8, 5 }; 

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
  unsigned long currentMillis = millis();

  // 1. BURST LOGIC: Check "Silence Timer"
  // If we are currently collecting a burst, but haven't heard a packet in X ms,
  // we assume the Master is finished sending.
  if (collectingBurst && (currentMillis - lastBurstPacketTime > BURST_SILENCE_TIMEOUT)) {
     sendReport();
     collectingBurst = false; // Reset collection state
     burstReceivedCount = 0;
     burstSumRSSI = 0;
     burstSumSNR = 0;
  }

  // 2. WATCHDOG: If no packets received for global TIMEOUT, revert to default
  if (currentMillis - lastPacketTime > TIMEOUT_MS) {
    if (memcmp(&currentConf, &defaultConf, sizeof(LoRaConfig_t)) != 0) {
      Serial.println("!!! Timeout: Link Lost. Reverting to Default Config. !!!");
      currentConf = defaultConf;
      applyConfig(currentConf);
    }
    lastPacketTime = currentMillis; 
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

  // --- Handle Message Types ---
  
  if (msgId == MSG_PING) {
    // Payload: [BatchID, Sequence, Total]
    uint8_t batchId = buffer[0];
    
    // If this is a new batch, reset counters (just in case silence timer failed)
    if (!collectingBurst || batchId != currentBatchId) {
        collectingBurst = true;
        currentBatchId = batchId;
        burstReceivedCount = 0;
        burstSumRSSI = 0;
        burstSumSNR = 0;
    }

    // Accumulate Stats
    burstReceivedCount++;
    burstSumRSSI += LoRa.packetRssi();
    burstSumSNR  += LoRa.packetSnr();
    lastBurstPacketTime = millis(); // Reset burst silence timer
    
    Serial.print("."); // Visual indicator of burst rx
  } 
  else if (msgId == MSG_CONFIG) {
    // 1. Decode new config
    LoRaConfig_t newConf;
    decodeConfig(buffer, &newConf);
    
    Serial.println("\nConfig Request Received.");
    
    // 2. Acknowledge receipt (using CURRENT settings so Master hears us)
    sendAck();
    
    // 3. Apply new settings locally
    // IMPORTANT: Wait a tiny bit to ensure ACK transmits fully before switching context
    delay(50); 
    
    currentConf = newConf;
    applyConfig(currentConf);
    Serial.println("*** Local Configuration Updated ***");
  }
}

// --- Transmission Functions ---

void sendReport() {
  Serial.println("\n-> Sending REPORT");
  
  // Calculate Averages
  int avgRSSI = (burstReceivedCount > 0) ? (burstSumRSSI / burstReceivedCount) : 0;
  int avgSNR  = (burstReceivedCount > 0) ? (burstSumSNR / burstReceivedCount) : 0;

  // Encode telemetry [Count, Abs(RSSI), SNR]
  uint8_t payload[3];
  payload[0] = (uint8_t)burstReceivedCount;
  payload[1] = (uint8_t)abs(avgRSSI); 
  payload[2] = (uint8_t)avgSNR;

  // --- DOUBLE TAP STRATEGY ---
  // Send twice to ensure Master hears it (Redundancy)
  
  for(int i=0; i<2; i++) {
    LoRa.beginPacket();
    LoRa.write(destination);
    LoRa.write(localAddress);
    LoRa.write(MSG_REPORT);
    LoRa.write(3); 
    LoRa.write(payload, 3);
    LoRa.endPacket();
    
    if (i == 0) delay(30); // Small hop between redundant packets
  }
  
  LoRa.receive(); // Go back to listen mode
}

void sendAck() {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_ACK);
  LoRa.write(0);
  LoRa.endPacket();
  
  // No LoRa.receive() here because onReceive continues or we switch config immediately after
}

// --- Helper Functions ---

void applyConfig(LoRaConfig_t conf) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
  LoRa.setSpreadingFactor(conf.spreadingFactor);
  LoRa.setCodingRate4(conf.codingRate);
  LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x22); // Sync Word must match Master
  LoRa.setPreambleLength(14);
  Serial.print("Applied Config -> SF:"); Serial.print(conf.spreadingFactor);
  Serial.print(" BW_IDX:"); Serial.println(conf.bandwidth_index);
}

void decodeConfig(uint8_t* payload, LoRaConfig_t* conf) {
  conf->bandwidth_index = (payload[0] >> 4) & 0x0F;
  conf->spreadingFactor = ((payload[0] & 0x0E) >> 1) + 6; 
  conf->codingRate      = ((payload[1] >> 6) & 0x03) + 5;
  conf->txPower         = ((payload[1] >> 1) & 0x1F) + 2;
}