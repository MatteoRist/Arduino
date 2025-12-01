#include <SPI.h>
#include <LoRa.h>

// --- IDs ---
const uint8_t localAddress = 0xA1;   // Master
const uint8_t destination  = 0xB1;   // Slave

// --- Message Types ---
const uint8_t MSG_PING       = 0x01; // Standard keep-alive
const uint8_t MSG_PONG       = 0x02; // Response to PING
const uint8_t MSG_CONFIG     = 0x03; // Command to change settings
const uint8_t MSG_ACK        = 0x04; // Acknowledge config change

// --- Timing ---
const unsigned long TIMEOUT_MS = 10000;  
const unsigned long PING_INTERVAL = 2000;   // Wait between cycles
unsigned long lastActionTime = 0;

// --- Thresholds for link quality ---
// If signal is stronger than this, we try to optimize speed
const int RSSI_THRESHOLD = -90;   // dBm 
const int SNR_THRESHOLD  = 8;     // dB

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
const LoRaConfig_t defaultConf = { 0, 12, 8, 14 }; 

// 2. Current Active Config
LoRaConfig_t currentConf = defaultConf;

// 3. Proposed Config (Pending application)
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
    Serial.println("!!! TIMEOUT: Link Lost. Reverting to Defaults. !!!");
    
    // Reset state
    waitingForReply = false;
    hasPendingConfigChange = false;
    
    // Force revert to default robust settings
    currentConf = defaultConf;
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
      // Otherwise, just send a standard PING
      sendPing();
    }
    
    lastActionTime = currentMillis;
  }
}

// --- Transmission Functions ---

void sendPing() {
  Serial.println("-> Sending PING");
  
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PING); 
  LoRa.write(0); // Payload length 0 for basic PING
  LoRa.endPacket();

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
    
    // Case A: PONG received (Response to PING)
    if (msgId == MSG_PONG) {
      waitingForReply = false;
      unsigned long rtt = millis() - msgSentTime;

      // Extract SNR/RSSI from packet telemetry (assuming slave sent them back)
      // Note: Assuming Slave sends [RSSI, SNR] as first 2 bytes of payload
      int remoteRSSI = -int(buffer[0]) / 2; // Example decoding
      int remoteSNR  = int(buffer[1]) - 148; 

      Serial.print("<- PONG | RTT: "); Serial.print(rtt);
      Serial.print(" | Remote RSSI: "); Serial.print(remoteRSSI);
      Serial.print(" | Remote SNR: "); Serial.println(remoteSNR);

      // Check if we can improve settings for the NEXT transmission
      checkForOptimization(remoteRSSI, remoteSNR);
    }

    // Case B: ACK received (Response to CONFIG request)
    else if (msgId == MSG_ACK) {
      waitingForReply = false;
      Serial.println("<- ACK Received. Slave updated.");
      
      // The Slave accepted the config, now WE apply it
      currentConf = nextConf;
      applyConfig(currentConf);
      
      Serial.println("*** Local Configuration Updated ***");
      hasPendingConfigChange = false;
    }
  }
}

// --- Logic Helpers ---

void checkForOptimization(int rssi, int snr) {
  // Reset pending flag initially
  hasPendingConfigChange = false;
  
  // Copy current to next as a base
  nextConf = currentConf; 
  bool changeNeeded = false;

  // LOGIC: If signal is very strong, increase data rate (lower SF, higher BW)
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
  
  // You could add logic here to DECREASE speed if RSSI is bad, 
  // but usually the Timeout/Reset to Default handles the "bad link" scenario best.

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
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(12);
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