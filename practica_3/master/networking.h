#include <SPI.h>
#include <LoRa.h>


#define MASTER 1


#define RSSI_THRESHOLD = -90;
#define SNR_THRESHOLD  = -5;
// ---- TEST / TIMING ----
#define PING_COUNT 4
#define EXTRA_MARGIN_MS 100
#define TIMEOUT_MS 10000UL
#define PING_INTERVAL_MS 2000UL
#define MIN_TIME_BETWEEN_CFG_CHANGES_MS 15000UL


uint8_t localAddress;
uint8_t destination;

/*------------------------------
Debug messege types
------------------------------*/
const uint8_t MSG_ESTABLISH_CONN    = 0x00;
const uint8_t MSG_PING              = 0x01;
const uint8_t MSG_PONG              = 0x02;
const uint8_t MSG_CONFIG            = 0x03;
const uint8_t MSG_ACK               = 0x04;




/*------------------------------
LoRaConfig struct
------------------------------*/

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

// Bandwidth lookup (Hz)

const double bandwidth_kHz[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

/*------------------------------
Scanning two defices
------------------------------*/

const LoRaConfig_t configs[] = {
    {6, 12, 8, 14},   // very reliable, slow
    {7, 10, 7, 10},
    {8, 9,  7, 6},
    {9, 7,  5, 2},   // fast but least reliable
};

#define MSG_HS_SYN 0x00
#define MSG_HS_SYN_ACK 0x01
#define MSG_HS_ACK 0x02


typedef struct{
    uint8_t flags;
    uint8_t rssi;
    uint8_t snr;
} HandshakeFrame;

volatile HandshakeFrame ConnectionFrame;
volatile bool receivedConnectionFrame = false;

#define CFG_COUNT (sizeof(configs)/sizeof(configs[0]))

void establishConnection(uint8_t sender, uint8_t receiver, bool master){

    int currentCfg = 0;
    unsigned long lastCfgSwitch = 0;
    unsigned int CFG_SWITCH_INTERVAL;
    unsigned long now;

    bool connectionEstablished = false;
#if MASTER
    /*------------------------------
    Master logic
    ------------------------------*/
    CFG_SWITCH_INTERVAL = 400;

    while(!connectionEstablished){

        now = millis();

        if(!txInProgress && now >= nextTxTime){

            if(receivedConnectionFrame){
                if (ConnectionFrame.flags == MSG_HS_SYN_ACK){
                    Serial.println("[MASTER] got SYN-ACK");

                    // Evaluate link quality
                    bool goodEnough = (ConnectionFrame.rssi > RSSI_THRESHOLD) && (ConnectionFrame.snr > SNR_THRESHOLD);

                    if (!goodEnough)
                    {
                        Serial.println("[MASTER] RSSI/SNR too weak -> rejecting config");
                        // Send a RST frame (just use MSG_ACK with flag=0xEE)
                        LoRa.beginPacket();
                        LoRa.write(destination);
                        LoRa.write(localAddress);
                        LoRa.write(MSG_ESTABLISH_CONN);
                        LoRa.write(0xEE); // "reset / reject"
                        LoRa.write(0x00);
                        LoRa.write(0x00);
                        txStartTime = millis();
                        txInProgress = true;
                        LoRa.endPacket(true);
                        continue;
                    }

                    // Otherwise, send ACK → connection established
                    Serial.println("[MASTER] sending ACK – connection established");

                    LoRa.beginPacket();
                    LoRa.write(destination);
                    LoRa.write(localAddress);
                    LoRa.write(MSG_ESTABLISH_CONN);
                    LoRa.write(MSG_HS_ACK);
                    LoRa.write(0x00);
                    LoRa.write(0x00);
                    txStartTime = millis();
                    txInProgress = true;
                    LoRa.endPacket(true);
                    while(txInProgress) delay(1);
                    connectionEstablished = true;
                    continue;
                }

            } else if(now - lastCfgSwitch > CFG_SWITCH_INTERVAL){
                currentCfg = (currentCfg + 1) % CFG_COUNT;

                Serial.print("[MASTER] switching cfg to index ");
                Serial.println(currentCfg);

                applyConfig(configs[currentCfg]);

                lastCfgSwitch = now;
            } else {
 
                Serial.print("[MASTER] sending SYN on cfg ");
                Serial.println(currentCfg);

                LoRa.beginPacket();
                LoRa.write(destination);
                LoRa.write(localAddress);
                LoRa.write(MSG_ESTABLISH_CONN);
                LoRa.write(MSG_HS_SYN);
                LoRa.write(0x00);
                LoRa.write(0x00);
                txStartTime = millis();
                txInProgress = true;
                LoRa.endPacket(true);

            }
            

        }
    }

    
    #else
    /*------------------------------
    slave logic
    ------------------------------*/

    while(!connectionEstablished){

        now = millis();

            if(!txInProgress && now >= nextTxTime){
            if (receivedConnectionFrame){

                receivedConnectionFrame = false;

                // MASTER → SYN
                if (ConnectionFrame.flags == MSG_HS_SYN)
                {
                    Serial.println("[SLAVE] got SYN");

                    // Prepare SYN-ACK
                    
                    Serial.println("[SLAVE] sending SYN-ACK");

                    LoRa.beginPacket();
                    LoRa.write(destination);
                    LoRa.write(localAddress);
                    LoRa.write(MSG_ESTABLISH_CONN);
                    LoRa.write(MSG_HS_SYN_ACK);
                    LoRa.write(ConnectionFrame.rssi);
                    LoRa.write(ConnectionFrame.snr);
                    txStartTime = millis();
                    txInProgress = true;
                    LoRa.endPacket(true);
                }
                // MASTER → final ACK
                else if (ConnectionFrame.flags == MSG_HS_ACK)
                {
                    Serial.println("[SLAVE] got ACK - connection ready");
                    connectionEstablished = true;
                }
                // MASTER → RST
                else if (ConnectionFrame.flags == 0xEE)
                {
                Serial.println("[SLAVE] got RST – master rejected config");
                // continue scanning
                }
            } else if (now - lastCfgSwitch > 2500){
                currentCfg = (currentCfg + 1) % CFG_COUNT;

                Serial.print("[Slave] switching cfg to index ");
                Serial.println(currentCfg);

                applyConfig(configs[currentCfg]);

                lastCfgSwitch = now;
            }
        }
    }
#endif

    Serial.println("=== Connection Established ===");
}

// Test IDs:
const uint8_t TEST_ID_MAIN = 0;
const uint8_t TEST_ID_FINAL = 1;


/*------------------------------
Tx logic
------------------------------*/

volatile bool txInProgress = false;
volatile unsigned long txStartTime = 0;
volatile unsigned long nextTxTime = 0;

void onTxDone() {
  unsigned long now = millis();
  txInProgress = false;
  nextTxTime = now + now - txStartTime + EXTRA_MARGIN_MS;
  Serial.print("TX done. duration(ms): "); Serial.print(now - txStartTime);
  Serial.print(" nextTxTime in(ms): "); Serial.println(now - txStartTime + EXTRA_MARGIN_MS);
}

/*------------------------------
helper funcitons
------------------------------*/

void applyConfig(const LoRaConfig_t conf) {
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