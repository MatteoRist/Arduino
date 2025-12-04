#include <SPI.h>
#include <LoRa.h>
#include "../master/networking.h"


volatile uint8_t lastPingId = 255;
volatile uint8_t lastPingPongReceived = 255;

uint8_t received_count = 0;
unsigned long lastConfigChangeTime = 0;

// Flags
volatile bool shouldSendPong = false;
volatile bool shouldSendACK = false;
bool shouldSendACKAndChangeConfig = false;

enum MainMode {SAFE, POSSIBLE_RECUPERATION, RECUPERATION_MODE};
MainMode mode = SAFE;

long lastMsgGotTS = 0;
long possibleRecuparationStart = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // ---- IDs ----
  localAddress = 0xB1;   // Slave
  destination  = 0xA1;   // Master

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();
  Serial.println("Slave ready. Listening...");

  establishConnection();
}

void loop() {
  // Checking modes if get messages
  if(mode == SAFE){
    if(millis() - lastMsgGotTS > PING_INTERVAL_MS){
      mode = POSSIBLE_RECUPERATION;
      possibleRecuparationStart = millis();
    }
  }
  else if(mode == POSSIBLE_RECUPERATION){
    if(millis() - possibleRecuparationStart > PING_INTERVAL_MS / 4){
      mode = RECUPERATION_MODE;
      attemptedRecovery = 0;
    }
  }
  else if(mode == RECUPERATION_MODE){
    getSignalBack();
  }
  if (!txInProgress) {

    // Changing config after sending ack to master
    if(shouldSendACKAndChangeConfig){
      lastConfig = currentConf;
      currentConf = nextConf;
      applyConfig(currentConf);
      shouldSendACKAndChangeConfig =false;
      mode = POSSIBLE_RECUPERATION;
      possibleRecuparationStart = millis();
    }
    if(millis() >= nextTxTime){

      // Sending pong to master
      if(shouldSendPong){
        sendPong(lastPingId, received_count);
        shouldSendPong = false;
        lastMsgGotTS = millis();
        mode = SAFE;

      }
      // Sending acknowledge to master 
      else if(shouldSendACK){
        LoRa.beginPacket();
        LoRa.write(destination);
        LoRa.write(localAddress);
        LoRa.write(MSG_ACK);
        LoRa.write(0);
        LoRa.endPacket(true);
        txInProgress = true;
        txStartTime = millis();

        shouldSendACK = false;
        shouldSendACKAndChangeConfig = true;

        Serial.println("Received CONFIG command. Sending ACK and applying.");
      }
    }
  }
}

// ---- callbacks ----

void onReceive(int packetSize) {
#if SERIAL_DBG
  Serial.println("Got msg");
#endif
  if (packetSize == 0) return;

  int recipient = LoRa .read();
  int sender    = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t incomingLength = LoRa.read();

#if SERIAL_DBG
  Serial.print("SNR  ==");Serial.print(LoRa.packetSnr());
  Serial.print("  RSSI =="); Serial.println(LoRa.rssi());
#endif

  if (recipient != localAddress) return;

  // read payload
  uint8_t buffer[10];
  uint8_t rcvd = 0;
  while (LoRa.available() && rcvd < sizeof(buffer)) {
    buffer[rcvd++] = (uint8_t)LoRa.read();
  }


  if(msgId == MSG_ESTABLISH_CONN){
    ConnectionFrame.flags = buffer[0];
    // RSSI is in [0,-127] so multiply *-1 will give uint8_t
    ConnectionFrame.rssi = -LoRa.rssi();
    // SNR is in [0, -148] so adding will give uint8_T
    ConnectionFrame.snr = LoRa.packetSnr()+148;
    receivedConnectionFrame = true;
  }else if(msgId == MSG_PING){
    lastPingId = buffer[0];
    lastPingPongReceived = buffer[1];
    shouldSendPong = true;
  }
  else if (msgId == MSG_CONFIG) {
    decodeConfig(buffer);
    shouldSendACK = true;
  }
}

void sendPong(uint8_t testId, uint8_t receivedSoFar) {

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

inline void decodeConfig(uint8_t* payload) {
  nextConf.bandwidth_index = (payload[0] >> 4) & 0x0F;
  nextConf.spreadingFactor = ((payload[0] >> 1) & 0x07) + 6;
  nextConf.codingRate      = ((payload[1] >> 6) & 0x03) + 5;
  nextConf.txPower         = ((payload[1] >> 1) & 0x1F) + 2;
}

inline void getSignalBack(){

  if(currentConf.bandwidth_index  == lastConfig.bandwidth_index &&
     currentConf.spreadingFactor  == lastConfig.spreadingFactor &&
     currentConf.codingRate  == lastConfig.codingRate &&
     currentConf.txPower  == lastConfig.txPower){

      if(millis() - lastRecoveryAttempted > RECOVERY_INTERVAL){
        if(attemptedRecovery < RECOVERY_ATTEMPTS){
          recoveryStep();
          attemptedRecovery++;
        } else if(attemptedRecovery == RECOVERY_ATTEMPTS){
          recoveryFallback();
          attemptedRecovery++;
        } else{
          establishConnection();
        }
      }
  } else{
    currentConf = lastConfig;
    applyConfig(currentConf);
    lastRecoveryAttempted = millis();
  }
}
