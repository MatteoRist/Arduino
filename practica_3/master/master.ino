#include <SPI.h>
#include <LoRa.h>
#include "networking.h"


#define SERIAL_DBG 0

// Default
bool hasPendingConfigChange = false;

// ---- State machine ----
enum MasterState { IDLE, SENDING_TEST, SENDING_FINAL, ANALYZE_IDLE };
MasterState state = IDLE;

unsigned long lastActionTime = 0;



// Test tracking
uint8_t test_id = TEST_ID_MAIN;
uint8_t ping_sent = 0;
uint8_t pongs_received = 0;
unsigned long testStartTime = 0;


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
  // Only send if not txInProgress and now >= nextTxTime
  if (!txInProgress && now >= nextTxTime) {
    if (state == IDLE && (now - lastActionTime > PING_INTERVAL_MS)) {
      // start a test
      Serial.println("Starting new multi-PING test");
      state = SENDING_TEST;
      test_id = TEST_ID_MAIN << 7;
      ping_sent = 0;
      pongs_received = 0;
    }

    if (state == SENDING_TEST) {
      if (ping_sent < PING_COUNT) {
        sendPing((ping_sent | test_id), pongs_received);
        ping_sent++;
        // after sending last ping, go to wait
        if (ping_sent == PING_COUNT) {
          state = SENDING_FINAL;
          testStartTime = millis();
        }
      }
    } else if (state == SENDING_FINAL) {
      // send single final ping containing how many pongs we received
      sendPing(ping_sent, pongs_received);
      state = ANALYZE_IDLE;
    } else if (state == ANALYZE_IDLE) {
      // analyze results and decide config changes
      float ratio = (float)pongs_received / (float)PING_COUNT;
      Serial.print("Test done. PONG ratio: "); Serial.println(ratio);

      unsigned long nowt = millis();
      bool doChange = false;
      if (ratio < 0.5f) {
        Serial.println("Ratio < 50% -> reduce speed / increase reliability.");
        // plan to increase reliability (example: increase SF or reduce BW)
        // nextConf = currentConf;
        // if (nextConf.spreadingFactor < 12) {
          // nextConf.spreadingFactor++;
          // doChange = true;
        // } else if (nextConf.bandwidth_index > 0) {
          // nextConf.bandwidth_index--;
          // doChange = true;
        // } else {
          // increase tx power if possible
          // if (nextConf.txPower < 20) { nextConf.txPower++; doChange = true; }
        // }
      } else if (ratio >= 0.5f && ratio <= 0.9f) {
        Serial.println("50%-90% -> repeat test once more before decisions.");
        // schedule a repeat of the test (do nothing now, will restart test next loop)
        state = IDLE;
        lastActionTime = nowt;
        return;
      } else { // > 0.9
        Serial.println("> 90% -> candidate to increase speed");
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
      static unsigned long lastChangeTime = 0;
      if (doChange && (nowt - lastChangeTime) > MIN_TIME_BETWEEN_CFG_CHANGES_MS) {
        Serial.println("Proposing config change: will send CONFIG to slave.");
        hasPendingConfigChange = true;
        lastChangeTime = nowt;
        // send config (will be sent in next loop as part of state machine)
        state = SENDING_FINAL; // first let master send final ping to notify slave of results
      } else {
        Serial.println("No config change or change too recent.");
        state = IDLE;
      }
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
      uint8_t t_id = buffer[0];
      uint8_t received = buffer[1];
      pongs_received++;

#if SERIAL_DBG
      Serial.print("<- PONG from "); Serial.print(sender, HEX);
      Serial.print(" test_id:"); Serial.print(t_id);
      Serial.print(" got_count:"); Serial.println(received);
#endif

    }
  } else if (msgId == MSG_ACK) {
    // Slave ACK to CONFIG
    Serial.println("<- ACK received. Slave applied config. Applying locally.");
    // If we had a pending config, apply it locally
    if (hasPendingConfigChange) {
      currentConf = nextConf;
      applyConfig(currentConf);
      hasPendingConfigChange = false;
    }
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
