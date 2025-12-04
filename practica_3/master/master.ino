#include <SPI.h>
#include <LoRa.h>
#include "networking.h"



// ---- IDs ----
const uint8_t localAddress = 0xA1;   // Master
const uint8_t destination  = 0xB1;   // Slave

// ---- Config type ----




// Default
const LoRaConfig_t defaultConf = { 8, 9, 8, 2 };
LoRaConfig_t currentConf = defaultConf;
LoRaConfig_t nextConf;
bool hasPendingConfigChange = false;

// ---- State machine ----
enum MasterState { IDLE, SENDING_TEST, WAITING_PONGS, SENDING_FINAL, ANALYZE_IDLE };
MasterState state = IDLE;

unsigned long lastActionTime = 0;
unsigned long msgSentTime = 0;


// Test tracking
uint8_t test_id = TEST_ID_MAIN;
uint8_t ping_sent = 0;
uint8_t pongs_received = 0;
unsigned long testStartTime = 0;

// thresholds
const int RSSI_THRESHOLD = -90;
const int SNR_THRESHOLD  = -5;

// ---- function prototypes ----
void onReceive(int packetSize);
void onTxDone();
void sendPing(uint8_t testId, uint8_t pingId);
void sendFinalPing(uint8_t finalCount);
void applyConfig(LoRaConfig_t conf);
uint8_t encodeConfig(LoRaConfig_t conf, uint8_t* payload);
void checkForOptimization(int remoteRSSI, int remoteSNR);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  applyConfig(currentConf);
  Serial.println("Master ready. Using Default Config.");
  state = IDLE;
  lastActionTime = millis();
}

void loop() {
  unsigned long now = millis();

  // TIMEOUT if waiting for pongs too long
  if (state == WAITING_PONGS && (now - testStartTime > TIMEOUT_MS)) {
    Serial.println("Timeout waiting for PONGs. Analyzing what arrived.");
    state = ANALYZE_IDLE;
  }

  // If tx is in progress, we wait for onTxDone to schedule next time.
  // Only send if not txInProgress and now >= nextTxTime
  if (!txInProgress && now >= nextTxTime) {
    if (state == IDLE && (now - lastActionTime > PING_INTERVAL_MS)) {
      // start a test
      Serial.println("Starting new multi-PING test");
      state = SENDING_TEST;
      test_id = TEST_ID_MAIN;
      ping_sent = 0;
      pongs_received = 0;
    }

    if (state == SENDING_TEST) {
      if (ping_sent < PING_COUNT) {
        sendPing(test_id, ping_sent);
        ping_sent++;
        // after sending last ping, go to wait
        if (ping_sent == PING_COUNT) {
          state = WAITING_PONGS;
          testStartTime = millis();
        }
      }
    } else if (state == SENDING_FINAL) {
      // send single final ping containing how many pongs we received
      sendFinalPing(pongs_received);
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
        delay(10);
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
    } else if (state == WAITING_PONGS) {
      // nothing; wait for pongs or timeout
    }
  }
}

// --- TX / RX / helpers ---

void onTxDone() {
  // called from ISR context typically
  unsigned long now = millis();
  lastTxDuration = now - txStartTime;
  txInProgress = false;
  nextTxTime = now + lastTxDuration + EXTRA_MARGIN_MS;
  LoRa.receive(); // go back to receive mode
  Serial.print("TX done. duration(ms): "); Serial.print(lastTxDuration);
  Serial.print(" nextTxTime in(ms): "); Serial.println(lastTxDuration + EXTRA_MARGIN_MS);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t incomingLength = LoRa.read();

  if (recipient != localAddress) return;
  Serial.print("SNR  ==");Serial.print(LoRa.packetSnr());
  Serial.print("  RSSI =="); Serial.println(LoRa.rssi());
  // read payload
  uint8_t buffer[10];
  uint8_t rcvd = 0;
  while (LoRa.available() && rcvd < sizeof(buffer)) {
    buffer[rcvd++] = (uint8_t)LoRa.read();
  }

  if (msgId == MSG_PONG) {
    // PONG payload format: [ test_id, count_received ]
    if (rcvd >= 2) {
      uint8_t t_id = buffer[0];
      uint8_t received = buffer[1];
      Serial.print("<- PONG from "); Serial.print(sender, HEX);
      Serial.print(" test_id:"); Serial.print(t_id);
      Serial.print(" got_count:"); Serial.println(received);

      if (t_id == TEST_ID_MAIN) {
        pongs_received = received; // slave tells how many pings it received so far
        // after receiving first PONG we might still be waiting for more PONGs
        // If slave sends only summary PONGs (e.g. incremental), we accept the latest value.
      }
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

void sendPing(uint8_t testId, uint8_t pingId) {
  // payload: [testId, pingId]
  uint8_t payload[2];
  payload[0] = testId;
  payload[1] = pingId;

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PING);
  LoRa.write(2); // len
  LoRa.write(payload, 2);
  LoRa.endPacket(true);

  txInProgress = true;
  txStartTime = millis();
  msgSentTime = txStartTime;
  Serial.print("-> Sent PING id: "); Serial.println(pingId);
}

void sendFinalPing(uint8_t finalCount) {
  // payload: [TEST_ID_FINAL, finalCount]
  uint8_t payload[2];
  payload[0] = TEST_ID_FINAL;
  payload[1] = finalCount;

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PING);
  LoRa.write(2);
  LoRa.write(payload, 2);
  LoRa.endPacket(true);

  txInProgress = true;
  txStartTime = millis();
  Serial.print("-> Sent FINAL PING with count: "); Serial.println(finalCount);
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
  Serial.println('\n\n');
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
