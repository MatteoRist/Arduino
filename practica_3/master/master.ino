#include <SPI.h>
#include <LoRa.h>

// --- IDs ---
const uint8_t localAddress = 0xA2;   // Master
const uint8_t destination  = 0xB2;   // Slave

// --- Message Types ---
const uint8_t MSG_PING       = 0x01; // Standard keep-alive
const uint8_t MSG_PONG       = 0x02; // (Depreciated in Burst Mode)
const uint8_t MSG_CONFIG     = 0x03; // Command to change settings
const uint8_t MSG_ACK        = 0x04; // Acknowledge config change
const uint8_t MSG_REPORT     = 0x05; // New: Summary report from Slave

// --- Timing ---
const unsigned long TIMEOUT_MS = 10000;  
const unsigned long PING_INTERVAL = 2000;   // Wait between cycles
unsigned long lastActionTime = 0;

// --- Thresholds for link quality ---
const int RSSI_THRESHOLD = -90;   // dBm 
const int SNR_THRESHOLD  = 8;     // dB

// --- Burst Configuration ---
const int BURST_SIZE = 4;         // How many packets per Ping cycle
uint8_t currentBatchId = 0;       // To identify specific bursts
bool reportReceivedThisCycle = false; // De-duplication flag

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

// Default Robust Config (Fallback)
const LoRaConfig_t defaultConf = { 8, 9, 8, 5 }; 

// Current Active Config
LoRaConfig_t currentConf = defaultConf;

// configuration memory system
LoRaConfig_t lastConf = defaultConf;

// Proposed Config (Pending application)
LoRaConfig_t nextConf;
bool hasPendingConfigChange = false;

// State tracking
bool waitingForReply = false;
unsigned long msgSentTime = 0;

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

  Serial.println("Master ready. Using Default Config.");
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. TIMEOUT CHECK / CONNECTION LOSS HANDLING
  if (waitingForReply && (currentMillis - msgSentTime > TIMEOUT_MS)) {
    
    // Reset state
    waitingForReply = false;
    hasPendingConfigChange = false;
    
    // Force revert to default robust settings
    if (memcmp(&currentConf, &lastConf, sizeof(LoRaConfig_t)) != 0){
      Serial.println("!!! TIMEOUT: Link Lost. Reverting to Last Config. !!!");
      currentConf = lastConf;
    }else{
      Serial.println("!!! TIMEOUT: Link Lost. Reverting to Default. !!!");
      currentConf = defaultConf;
    }
    
    applyConfig(currentConf);
    
    // Add a small random delay to desynchronize slightly if needed
    delay(1000); 
    return;
  }

  // 2. SENDING LOGIC (If not waiting for reply and interval passed)
  if (!waitingForReply && (currentMillis - lastActionTime > PING_INTERVAL)) {
    
    if (hasPendingConfigChange) {
      // If we calculated a better config previously, send it now
      sendConfigCommand(nextConf);
    } else {
      // Send the BURST of 4 packets
      sendPingBurst();
    }
    
    lastActionTime = currentMillis;
  }
}

// --- Transmission Functions ---

void sendPingBurst() {
  Serial.println("-> Sending BURST PING (4 pkts)");
  
  // Increment Batch ID so Slave knows these 4 belong together
  currentBatchId++;
  
  // Reset the de-duplication flag for the new cycle
  reportReceivedThisCycle = false;

  // Send 4 packets rapidly
  for (int i = 0; i < BURST_SIZE; i++) {
      LoRa.beginPacket();
      LoRa.write(destination);
      LoRa.write(localAddress);
      LoRa.write(MSG_PING); 
      
      // Payload for Burst Tracking
      LoRa.write(currentBatchId); // Which batch is this?
      LoRa.write(i);              // Which number is this? (0-3)
      LoRa.write(BURST_SIZE);     // Total size (4)
      
      LoRa.endPacket();
      
      // Small delay to prevent overwhelming the receiver buffer
      delay(20); 
  }

  msgSentTime = millis();
  waitingForReply = true;
  LoRa.receive();
}

void sendConfigCommand(LoRaConfig_t newConf) {
  Serial.println("-> Sending NEW CONFIGURATION");
  
  uint8_t payload[10];
  uint8_t len = encodeConfig(newConf, payload);

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_CONFIG); 
  LoRa.write(len);
  LoRa.write(payload, len);
  LoRa.endPacket();

  msgSentTime = millis();
  waitingForReply = true;
  LoRa.receive();
}

// --- Receive Handler ---

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // Read header
  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t incomingLength = LoRa.read();

  if (recipient != localAddress) return;

  // Read Payload
  uint8_t buffer[10];
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }

  // Handle Response Types
  if (waitingForReply) {
    
    // Case A: REPORT received
    if (msgId == MSG_REPORT) {
      
      // DE-DUPLICATION: If we already got a report for this burst, ignore this one.
      if (reportReceivedThisCycle) return;
      
      reportReceivedThisCycle = true;
      waitingForReply = false;
      unsigned long rtt = millis() - msgSentTime;

      // Extract Report Data
      // Payload: [PacketsReceived, AvgRSSI, AvgSNR]
      int packetsReceived = (int)buffer[0];
      int remoteRSSI      = -int(buffer[1]); // Assuming slave sent abs() value
      int remoteSNR       = int(buffer[2]); 

      Serial.print("<- REPORT | Pkts: "); Serial.print(packetsReceived);
      Serial.print("/"); Serial.print(BURST_SIZE);
      Serial.print(" | RSSI: "); Serial.print(remoteRSSI);
      Serial.print(" | SNR: "); Serial.println(remoteSNR);

      // Optimize based on STABILITY (packet count) and QUALITY (rssi/snr)
      checkForOptimization(packetsReceived, remoteRSSI, remoteSNR);
    }

    // Case B: ACK received (Response to CONFIG request)
    else if (msgId == MSG_ACK) {
      waitingForReply = false;
      Serial.println("<- ACK Received. Slave updated.");
      
      // The Slave accepted the config, now WE apply it
      lastConf = currentConf;
      currentConf = nextConf;
      applyConfig(currentConf);
      
      Serial.println("*** Local Configuration Updated ***");
      hasPendingConfigChange = false;  
    }
  }
}

// --- Logic Helpers ---

// UPDATED: Now takes packetCount to ensure link stability
void checkForOptimization(int packetsReceived, int rssi, int snr) {
  // Reset pending flag initially
  hasPendingConfigChange = false;
  
  // GUARD CLAUSE: If we lost packets, the link is not stable enough to upgrade.
  if (packetsReceived < BURST_SIZE) {
    Serial.println("(Stability) Packet loss detected. Holding current config.");
    return;
  }

  // Copy current to next as a base
  nextConf = currentConf; 
  bool changeNeeded = false;

  // LOGIC: If signal is very strong AND packet rate is 100%
  if (rssi > RSSI_THRESHOLD && snr > SNR_THRESHOLD) {
    
    // 1. Try to lower Spreading Factor
    if (nextConf.spreadingFactor > 7) {
      nextConf.spreadingFactor--;
      changeNeeded = true;
      Serial.print("(Plan) Reduce SF to "); Serial.println(nextConf.spreadingFactor);
    } 
    // 2. If SF is low, try to increase Bandwidth
    else if (nextConf.bandwidth_index < 9) {
      nextConf.bandwidth_index++;
      changeNeeded = true;
      Serial.print("(Plan) Increase BW to index "); Serial.println(nextConf.bandwidth_index);
    }
  }

  if (changeNeeded) {
    hasPendingConfigChange = true;
    Serial.println("Optimization found. Next packet will send Config.");
  }
}

void applyConfig(LoRaConfig_t conf) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
  LoRa.setSpreadingFactor(conf.spreadingFactor);
  LoRa.setCodingRate4(conf.codingRate);
  LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x22);
  LoRa.setPreambleLength(14);
  Serial.print("Applied Config -> SF:"); Serial.print(conf.spreadingFactor);
  Serial.print(" BW_IDX:"); Serial.println(conf.bandwidth_index);
}

uint8_t encodeConfig(LoRaConfig_t conf, uint8_t* payload) {
  uint8_t len = 0;
  payload[len]    = (conf.bandwidth_index << 4);
  payload[len++] |= ((conf.spreadingFactor - 6) << 1);
  payload[len]    = ((conf.codingRate - 5) << 6);
  payload[len++] |= ((conf.txPower - 2) << 1);
  return len;
}

// Helper function to print the struct fields
void printStruct(const LoRaConfig_t& config) {
  Serial.println(F("------ LoRa Config ------"));
  
  // We cast to (int) to ensure it prints the number, not the ASCII character
  Serial.print(F("Bandwidth Index:  "));
  Serial.println((int)config.bandwidth_index);
  
  Serial.print(F("Spreading Factor: "));
  Serial.println((int)config.spreadingFactor);
  
  Serial.print(F("Coding Rate:      "));
  Serial.println((int)config.codingRate);
  
  Serial.print(F("TX Power:         "));
  Serial.println((int)config.txPower);
  
  Serial.println(F("-------------------------"));
}