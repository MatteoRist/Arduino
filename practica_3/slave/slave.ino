// ---- TEST / TIMING ----
#define PING_COUNT 4
#define EXTRA_MARGIN_MS 30
#define TIMEOUT_MS 10000UL
#define PING_INTERVAL_MS 2000UL
#define MIN_TIME_BETWEEN_CFG_CHANGES_MS 15000UL

// Message types:
const uint8_t MSG_PING   = 0x01;
const uint8_t MSG_PONG   = 0x02;
const uint8_t MSG_CONFIG = 0x03;
const uint8_t MSG_ACK    = 0x04;

// Test IDs:
const uint8_t TEST_ID_MAIN = 0;
const uint8_t TEST_ID_FINAL = 1;


#include <SPI.h>
#include <LoRa.h>

// ---- IDs ----
const uint8_t localAddress = 0xB1;   // Slave
const uint8_t destination  = 0xA1;   // Master

// ---- Config type ----
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

// Bandwidth lookup (Hz)
double bandwidth_kHz[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

// Default
const LoRaConfig_t defaultConf = { 8, 9, 8, 2 };
LoRaConfig_t currentConf = defaultConf;

// Non-blocking TX tracking
volatile bool txInProgress = false;
volatile unsigned long txStartTime = 0;
volatile unsigned long lastTxDuration = 0;
unsigned long nextTxTime = 0;
const unsigned long EXTRA_MARGIN = EXTRA_MARGIN_MS;

// Test tracking for current test_id
uint8_t current_test_id = 255;
bool received_flags[PING_COUNT];
uint8_t received_count = 0;
unsigned long lastConfigChangeTime = 0;

void onReceive(int packetSize);
void onTxDone();
void applyConfig(LoRaConfig_t conf);
void decodeConfig(uint8_t* payload, LoRaConfig_t* conf);
void sendPong(uint8_t testId, uint8_t receivedSoFar);

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
  Serial.println("Slave ready. Listening...");
}

void loop() {
  // watchdog: revert to default after long no packets
  static unsigned long lastPacket = millis();
  if (millis() - lastPacket > TIMEOUT_MS) {
    if (memcmp(&currentConf, &defaultConf, sizeof(LoRaConfig_t)) != 0) {
      Serial.println("Timeout: reverting to default config");
      currentConf = defaultConf;
      applyConfig(currentConf);
    }
    lastPacket = millis();
  }
}

// ---- callbacks ----

void onTxDone() {
  unsigned long now = millis();
  lastTxDuration = now - txStartTime;
  txInProgress = false;
  nextTxTime = now + lastTxDuration + EXTRA_MARGIN_MS;
  LoRa.receive();
  Serial.print("TX done. dur: "); Serial.println(lastTxDuration);
}

void onReceive(int packetSize) {
  Serial.println("kurwa");
  if (packetSize == 0) return;

  int recipient = LoRa .read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t incomingLength = LoRa.read();

  Serial.print("SNR  ==");Serial.print(LoRa.packetSnr());
  Serial.print("  RSSI =="); Serial.println(LoRa.rssi());

  if (recipient != localAddress) return;

  // read payload
  uint8_t buffer[10];
  uint8_t rcvd = 0;
  while (LoRa.available() && rcvd < sizeof(buffer)) {
    buffer[rcvd++] = (uint8_t)LoRa.read();
  }

  // reset packet watchdog timestamp
  static unsigned long lastPacketMillis = 0;
  lastPacketMillis = millis();

  if (msgId == MSG_PING) {
    // payload: [ testId, pingIdOrCount ]
    if (rcvd >= 2) {
      uint8_t t_id = buffer[0];
      uint8_t val  = buffer[1];

      if (t_id == TEST_ID_MAIN) {
        // mark reception of this ping id
        uint8_t pingId = val;
        if (pingId < PING_COUNT && !received_flags[pingId]) {
          received_flags[pingId] = true;
          received_count++;
        }
        // send a PONG summarizing how many pings received so far
        sendPong(TEST_ID_MAIN, received_count);
      } else if (t_id == TEST_ID_FINAL) {
        // Master final ping with count of PONGs it received
        uint8_t master_received_pongs = val;
        Serial.print("Master reports it received "); Serial.println(master_received_pongs);

        // Now slave decides whether it accepts the config change requested earlier
        // For this simple scheme, we compare our received_count to decide.
        float ratio = (float)received_count / (float)PING_COUNT;
        Serial.print("Our receive ratio: "); Serial.println(ratio);

        // Example policy: if our receive ratio < 0.5 -> increase reliability
        if (ratio < 0.5f) {
          // propose local change: increase SF or decrease BW
          LoRaConfig_t newC = currentConf;
          if (newC.spreadingFactor < 12) newC.spreadingFactor++;
          else if (newC.bandwidth_index > 0) newC.bandwidth_index--;
          // apply locally and notify master with ACK (we'll send ACK)
          unsigned long now = millis();
          if (now - lastConfigChangeTime > MIN_TIME_BETWEEN_CFG_CHANGES_MS) {
            currentConf = newC;
            applyConfig(currentConf);
            lastConfigChangeTime = now;

            // send ACK to master (notifying that we applied config)
            LoRa.beginPacket();
            LoRa.write(destination);
            LoRa.write(localAddress);
            LoRa.write(MSG_ACK);
            LoRa.write(0);
            LoRa.endPacket(true);
            txInProgress = true;
            txStartTime = millis();
            Serial.println("-> Sent ACK announcing local config change");
          } else {
            Serial.println("Config change suppressed (too soon since last change).");
          }
        } else if (ratio >= 0.9f) {
          // we can attempt to speed up if safe
          LoRaConfig_t newC = currentConf;
          if (newC.spreadingFactor > 7) newC.spreadingFactor--;
          else if (newC.bandwidth_index < 9) newC.bandwidth_index++;
          unsigned long now = millis();
          if (now - lastConfigChangeTime > MIN_TIME_BETWEEN_CFG_CHANGES_MS) {
            currentConf = newC;
            applyConfig(currentConf);
            lastConfigChangeTime = now;

            LoRa.beginPacket();
            LoRa.write(destination);
            LoRa.write(localAddress);
            LoRa.write(MSG_ACK);
            LoRa.write(0);
            LoRa.endPacket(true);
            txInProgress = true;
            txStartTime = millis();
            Serial.println("-> Sent ACK (speed-up applied locally)");
          } else {
            Serial.println("Speed-up suppressed (too soon).");
          }
        } else {
          Serial.println("No config change needed.");
        }

        // reset test tracking
        memset(received_flags, 0, sizeof(received_flags));
        received_count = 0;
      }
    }
  } else if (msgId == MSG_CONFIG) {
    // decode and apply new config (Master sent explicit CONFIG)
    LoRaConfig_t newConf;
    decodeConfig(buffer, &newConf);
    Serial.println("Received CONFIG command. Sending ACK and applying.");
    // send ACK first so master knows we received config
    LoRa.beginPacket();
    LoRa.write(destination);
    LoRa.write(localAddress);
    LoRa.write(MSG_ACK);
    LoRa.write(0);
    LoRa.endPacket(true);
    txInProgress = true;
    txStartTime = millis();

    // apply config locally
    unsigned long now = millis();
    if (now - lastConfigChangeTime > MIN_TIME_BETWEEN_CFG_CHANGES_MS) {
      currentConf = newConf;
      applyConfig(currentConf);
      lastConfigChangeTime = now;
      Serial.println("*** Local Configuration Updated ***");
    } else {
      Serial.println("CONFIG ignored (rate limited).");
    }
  }
}

void sendPong(uint8_t testId, uint8_t receivedSoFar) {
  // payload: [ testId, receivedSoFar ]
  // schedule immediate send (if radio free)
  if (txInProgress) {
    Serial.println("TX busy, cannot send PONG right now.");
    return;
  }

  uint8_t payload[2];
  payload[0] = testId;
  payload[1] = receivedSoFar;

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(MSG_PONG);
  LoRa.write(2);
  LoRa.write(payload, 2);
  LoRa.endPacket(true);

  txInProgress = true;
  txStartTime = millis();
  Serial.print("-> Sent PONG. count: "); Serial.println(receivedSoFar);
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

void decodeConfig(uint8_t* payload, LoRaConfig_t* conf) {
  conf->bandwidth_index = (payload[0] >> 4) & 0x0F;
  conf->spreadingFactor = ((payload[0] >> 1) & 0x07) + 6;
  conf->codingRate      = ((payload[1] >> 6) & 0x03) + 5;
  conf->txPower         = ((payload[1] >> 1) & 0x1F) + 2;
}
