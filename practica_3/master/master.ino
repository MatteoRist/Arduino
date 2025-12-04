#include <SPI.h>
#include <LoRa.h>
#include "networking.h"


#define SERIAL_DBG 0

// Default
bool hasPendingConfigChange = false;

// ---- State machine ----
enum MasterState { IDLE, SENDING_TEST, SENDING_FINAL, ANALYZE_IDLE };
MasterState state = IDLE;
bool sendingSecondTest = false;
// ---- Mode -----------
enum MainMode { SAFE, SEND_CONFIG_WITHOUT_ACK, POSSIBLE_RECUPERATION, RECUPERATION_MODE };
MainMode mode = SAFE;

unsigned long lastPongTimestamp = 0;
unsigned long possibleRecuparationStart = 0;
unsigned long lastActionTime = 0;

unsigned long lastChangeTime = 0;

int lowestTestedRSSIPing = 256;
int lowestTestedSNRPing = 256;

// Test tracking
uint8_t test_id = TEST_ID_MAIN;
uint8_t ping_sent = 0;
uint8_t pongs_received = 0;

// Flags
volatile bool gotACK = false;
volatile bool gotPing = false;

// SENT without ack variables
unsigned long configSentTimestamp = 0;
const unsigned long CONFIG_TIMEOUT_MS = PING_INTERVAL_MS*2.5;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

    // ---- IDs ----
  localAddress = 0xA1;   // Master
  destination  = 0xB1;   // Slave

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  establishConnection();
  Serial.println("Master ready. Using Default Config.");
  state = IDLE;
  lastActionTime = millis();
}

void loop() {
  unsigned long now = millis();

  // If tx is in progress, we wait for onTxDone to schedule next time.
  // Only send if not txInProgress and     lastRecoveryAttempted = millis();now >= nextTxTime

  if(mode == RECUPERATION_MODE){
    getSignalBack();
  }
  if (!txInProgress && now >= nextTxTime) {
    if(gotPing){
      pongs_received++;
      mode = SAFE;
    }
    if(mode == SEND_CONFIG_WITHOUT_ACK ){
      if(gotACK){
      // we will need to wait for the slave to comeback to this so we should do test and then after PING_INTERVAL_MS do another one
      Serial.println("<- ACK received. Slave applied config. Applying locally.");
      if (hasPendingConfigChange) {
        lastConfig = currentConf;
        currentConf = nextConf;
        applyConfig(currentConf);
        hasPendingConfigChange = false;
        lastActionTime = 0;
      }
      }else if(millis() - configSentTimestamp > CONFIG_TIMEOUT_MS){
        Serial.println("CONFIG timeout, assuming failure. Reverting...");
        currentConf = lastConfig;
        applyConfig(currentConf);
        mode = RECUPERATION_MODE;
      }
    }
    if (state == IDLE && (now - lastActionTime > PING_INTERVAL_MS)) {
      // start a test
      Serial.println("Starting new multi-PING test");
      state = SENDING_TEST;
      test_id = (TEST_ID_MAIN + sendingSecondTest) << 7;
      ping_sent = 0;
      pongs_received = 0;
      lowestTestedRSSIPing = 255;
      lowestTestedSNRPing = 255;
    }

    if (state == SENDING_TEST) {
      if (ping_sent < PING_COUNT) {
        sendPing((ping_sent | test_id), pongs_received);
        ping_sent++;
        // after sending last ping, go to wait
        if (ping_sent == PING_COUNT) {
          state = SENDING_FINAL;
        }
      }
    } 
    else if (state == SENDING_FINAL) {
      sendPing(ping_sent, pongs_received);
      state = ANALYZE_IDLE;
    } else if (state == ANALYZE_IDLE) {
      analyzeTest();
    }
  }
}

// --- TX / RX / helpers ---


void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();


  if (recipient != localAddress) return;

#if SERIAL_DBG
  Serial.print("SNR  ==");Serial.print(LoRa.packetSnr());
  Serial.print("  RSSI =="); Serial.println(LoRa.rssi());
#endif

  // read payload
  uint8_t buffer[10];
  uint8_t rcvd = 0;
  while (LoRa.available() && rcvd < sizeof(buffer)) {
    buffer[rcvd++] = (uint8_t)LoRa.read();
  }
  if(msgId == MSG_ESTABLISH_CONN){
    ConnectionFrame.flags = buffer[0];
    ConnectionFrame.rssi = buffer[1];
    ConnectionFrame.snr = buffer[2];
    receivedConnectionFrame = true;
  }
  else if (msgId == MSG_PONG) {
    // PONG payload format: [ test_id, count_received ]
    if (rcvd >= 2) {
      
      gotPing = true;
      if(LoRa.rssi() < lowestTestedRSSIPing) lowestTestedRSSIPing = LoRa.rssi();
      if(LoRa.packetSnr() < lowestTestedSNRPing) lowestTestedSNRPing = LoRa.packetSnr();

#if SERIAL_DBG
      Serial.print("<- PONG from "); Serial.print(sender, HEX);
      Serial.print(" test_id:"); Serial.print(data[0]);
      Serial.print("  count_received: "); Serial.print(data[1]);
#endif

    }
  } else if (msgId == MSG_ACK) {
    // Slave ACK to CONFIG
    gotACK = true;
  }
}

void sendPing(uint8_t pingId, uint8_t pongReceived) {
  // payload: [testId, pingId]

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PING);
  LoRa.write(pingId);
  LoRa.write(pongReceived);
  LoRa.endPacket(true);

  txInProgress = true;
  txStartTime = millis();
#if SERIAL_DBG
  Serial.print("-> Sent PING id: "); Serial.print(pingId);
  Serial.print("   Pong received "); Serial.print(pongReceived);
#endif
}




// Encode config into payload, returning length
uint8_t encodeConfig(LoRaConfig_t conf, uint8_t* payload) {
  uint8_t len = 0;
  payload[len]    = (conf.bandwidth_index << 4);
  payload[len++] |= ((conf.spreadingFactor - 6) << 1);
  payload[len]    = ((conf.codingRate - 5) << 6);
  payload[len++] |= ((conf.txPower - 2) << 1);
  return len;
}

inline void analyzeTest(){
  // analyze results and decide config changes
      float ratio =pongs_received / (float) PING_COUNT ;
      Serial.print("Test done. PONG ratio: "); Serial.println(ratio);

      unsigned long nowt = millis();
      bool doChange = false;

      // if less than 50% pongs received do recovery
      if (ratio <= 0.5f) {
        mode = RECUPERATION_MODE;
      } 
      else if (ratio > 0.5f && ratio < 1) {
        Serial.println("50% < x < 100% -> repeat test once more before decisions.");
        // schedule a repeat of the test (do nothing now, will restart test next loop)
        if(sendingSecondTest){
          sendingSecondTest = false;
          state = IDLE;
          lastActionTime = nowt;
        } else{
          sendingSecondTest = true;
          state = RECUPERATION_MODE;
          lastActionTime = nowt;
        }
        return;
      } else { // == 100%
        Serial.println("> 100% -> candidate to increase speed");
        // try to speed up
        nextConf = currentConf;
        if (nextConf.spreadingFactor > 7) {
          nextConf.spreadingFactor--;
          doChange = true;
        } else if (nextConf.bandwidth_index < 9) {
          nextConf.bandwidth_index++;
          doChange = true;
        } else if (nextConf.txPower > 2) {
          nextConf.txPower--; // reduce power if possible
          doChange = true;
        }
      }

      // throttle changes

      if (doChange && (nowt - lastChangeTime) > MIN_TIME_BETWEEN_CFG_CHANGES_MS) {
        Serial.println("Proposing config change: will send CONFIG to slave.");
        hasPendingConfigChange = true;
        lastChangeTime = nowt;
        // send config (will be sent in next loop as part of state machine)
        LoRa.beginPacket();
        LoRa.write(destination);
        LoRa.write(localAddress);
        LoRa.write(MSG_CONFIG);
        uint8_t encodedConfig[2];
        encodeConfig(nextConf, encodedConfig);
        LoRa.write(encodedConfig[0]);
        LoRa.write(encodedConfig[1]);
        mode = SEND_CONFIG_WITHOUT_ACK;
        configSentTimestamp = millis();
        LoRa.endPacket(true);
        sendingSecondTest = false
      } else {
        Serial.println("No config change or change too recent.");
        state = IDLE;
        sendingSecondTest = false
      }
}
