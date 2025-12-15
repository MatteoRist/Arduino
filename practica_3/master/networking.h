#include <SPI.h>
#include <LoRa.h>

#define TX_LAPSE_MS          10000

const uint8_t MASTER_IP = 0xB0;
const uint8_t SLAVE_IP = 0xB1;
uint8_t localAddress; 
uint8_t destination = 0xFF;           

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t defaultConfig = { 7, 9, 8, 2};
LoRaConfig_t thisNodeConf   = defaultConfig;
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;

//--------------------------NEW THINGS-------------------------------

enum Mode {STABLE, PRE_PROBING ,PROBING, INSTABLE};
Mode mode = STABLE;

#define TX_INTERVAL_BEFORE_PROBING 5
int txIntervals = 0;

#define CONFIG_CHANGE_FLAG 128
#define CONFIG_NOT_ACCEPTED 64
#define LOOSING_DATA_FLAG 32
#define INSTABLE_FLAG 16
#define INSTABLE_FLAG_ACK 8
#define SIXTH_FLAG 4
#define SEVENTH_FLAG 2
#define EIGHT_FLAG 1



#define RSSI_THRESHOLD -105
#define RSSI_THRESHOLD_UPPER -70
const float SNR_THRESHOLD[] = {-2.5, -4, -6.5, -9, -11.5, -13};
#define SNR_THRESHOLD_UPPER 7.5

volatile int last_packet_RSSI = -60;
volatile float last_packet_SNR = 0;

#define PROBING_PLANNED_TX_INTERVALS 4
uint32_t probingStarted = 0;
uint32_t probingPlannedTime = 0;
#define PROBING_WITHOUT_DATA_TX_INTERVALS 2
uint32_t probingWithoutData = 0;

#define INSTABLE_PLANNED_TX_INTERVALS 3
uint32_t instableStarted = 0;
uint16_t instablePlannedTime = 0;

volatile int lastMsgId = 255;
volatile uint8_t loosingData = false;
volatile uint8_t gotDataProbing = 0;
volatile uint32_t lastReceivedTime_ms = 0;

LoRaConfig_t previousConfig = thisNodeConf;
LoRaConfig_t nextConfig = thisNodeConf;


// Restaring variables
#define LORA_MINIMUM_TIME_BETWEEN_RESTARTS 400
uint32_t LoRaLastRestart_timestamp = 0;
// TX variables 
uint8_t PREAMBLE_LEN = 8;
uint32_t theoreticalTimeOnAir = 0;
float symbol_duration_ms = 0;     
float header_preamble_time_ms = 0; 
int cr_eff = 0;                  
bool ldro_enabled = false;

// --------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------

// Inicjalization and d
bool applyConfig(const LoRaConfig_t conf, bool check_difference_and_apply_difference);
void updateTimingParameters(const LoRaConfig_t& conf);
uint32_t getTimeOnAirBytes(uint8_t payloadLength);
typedef void (*LoRaReceiveCallback)(int);
bool init_LoRa(LoRaReceiveCallback onReceive);

// sendinf and getting
bool sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount, uint8_t flags, uint8_t async);
void TxFinished();

inline bool operator==(const LoRaConfig_t& a, const LoRaConfig_t& b);

// debbuging
void printBinaryPayload(uint8_t * payload, uint8_t payloadLength);
void printFlags(uint8_t flags);

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------

bool sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount, uint8_t flags, uint8_t async) 
{
  LoRa.idle();
  if(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    return false;                           // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(flags);
  LoRa.write(payloadLength);              // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // Añadimos el mensaje/payload 
  LoRa.endPacket(async);                   

  theoreticalTimeOnAir = getTimeOnAirBytes(payloadLength+6);
  Serial.print("Theory time on Air:  "); Serial.println(theoreticalTimeOnAir);
  return true;
}

/*--------------------------------------------------------------
on Tx done
--------------------------------------------------------------*/
void TxFinished()
{
  txDoneFlag = true;
}

/*--------------------------------------------------------------
DEBUG
--------------------------------------------------------------*/
void printBinaryPayload(uint8_t * payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void printFlags(uint8_t flags) {
  Serial.print("FLAGS [0x");
  Serial.print(flags, HEX);
  Serial.print("] -> ");

  if (flags == 0) {
    Serial.println("NONE");
    return;
  }

  if (flags & CONFIG_CHANGE_FLAG)           Serial.print("CONFIG_CHANGE ");
  if (flags & CONFIG_NOT_ACCEPTED)         Serial.print("CONFIG_NOT_ACCEPTED ");
  if (flags & LOOSING_DATA_FLAG)    Serial.print("LOOSING_DATA ");
  if (flags & INSTABLE_FLAG)        Serial.print("INSTABLE ");
  if (flags & INSTABLE_FLAG_ACK)       Serial.print("INSTABLE_FLAG_ACK ");
  if (flags & SIXTH_FLAG)       Serial.print("SIXTH_FLAG ");
  if (flags & SEVENTH_FLAG)       Serial.print("SEVENTH_FLAG ");
  if (flags & EIGHT_FLAG)        Serial.print("EIGHT_FLAG ");

  Serial.println();
}
/*--------------------------------------------------------------
Apply new config
--------------------------------------------------------------*/
bool applyConfig(const LoRaConfig_t conf, bool check_difference_and_apply_difference) {
  if(!check_difference_and_apply_difference){
    LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
    LoRa.setSpreadingFactor(conf.spreadingFactor);
    LoRa.setCodingRate4(conf.codingRate);
    LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
    updateTimingParameters(conf);
  }else{
    if(conf.bandwidth_index != thisNodeConf.bandwidth_index){
      LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
    }
    if(conf.spreadingFactor != thisNodeConf.spreadingFactor){
      LoRa.setSpreadingFactor(conf.spreadingFactor);
    }
    if(conf.codingRate != thisNodeConf.codingRate){
      LoRa.setCodingRate4(conf.codingRate);
    }
    if(conf.txPower != thisNodeConf.txPower){
      LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
    }
  }
  
  
  // LoRa.setSyncWord(0x12);
  // LoRa.setPreambleLength(50);
  bool configChanged = false;
  if(!(thisNodeConf == conf)){
    configChanged = true;
    updateTimingParameters(conf);
    if(!(conf == defaultConfig)){
      Serial.print("Applied Config -> SF:"); Serial.print(conf.spreadingFactor);
      Serial.print(" BW_IDX:"); Serial.print(conf.bandwidth_index);
      Serial.print("  CODING_RATE  "); Serial.println(conf.codingRate);
    }else{
      Serial.println("Applied basic config");
      
    }
    
  }
  return configChanged;

}

inline bool operator==(const LoRaConfig_t& a, const LoRaConfig_t& b) {
    return a.bandwidth_index == b.bandwidth_index &&
           a.spreadingFactor == b.spreadingFactor &&
           a.codingRate == b.codingRate &&
           a.txPower == b.txPower;
}


bool inline revertConfig(uint16_t &msgCount, uint8_t &flags){
    if(previousConfig == thisNodeConf){
        previousConfig = defaultConfig;
    } 
    nextConfig = previousConfig;
    applyConfig(previousConfig, true);
    thisNodeConf = previousConfig;
    loosingData = 0;
    txIntervals = 0;
    msgCount = 0;
    lastMsgId = 255;
    mode = STABLE;
    lastReceivedTime_ms = millis();
    flags = 0;
}

bool inline changeToNewConfig(uint16_t &msgCount, uint8_t &flags){
    previousConfig = thisNodeConf;
    applyConfig(nextConfig, true);
    bool isNextSLower = false;
    if( thisNodeConf.bandwidth_index >= nextConfig.bandwidth_index &&
        thisNodeConf.spreadingFactor <= nextConfig.spreadingFactor &&
        thisNodeConf.codingRate <= nextConfig.codingRate){
          isNextSLower = true;
    }
    thisNodeConf = nextConfig;
    loosingData = 0;
    txIntervals = 0;
    msgCount = 0;
    flags = 0;
    lastMsgId = 255;
    lastReceivedTime_ms = millis();
    return isNextSLower;
}

/*--------------------------------------------------------------
Calculating theory time in tx
--------------------------------------------------------------*/

void updateTimingParameters(const LoRaConfig_t& conf) {
    // symbol time
    double bw_hz = bandwidth_kHz[conf.bandwidth_index];
    symbol_duration_ms = (pow(2, conf.spreadingFactor) / bw_hz) * 1000.0;


    ldro_enabled = (symbol_duration_ms > 16.0);


    header_preamble_time_ms = (PREAMBLE_LEN + 4.25) * symbol_duration_ms;


    cr_eff = conf.codingRate;
}

uint32_t getTimeOnAirBytes(uint8_t payloadLength) {

    int de = ldro_enabled ? 1 : 0;
    int H = 0;          // Explicit header (Assumed 0 for Explicit, 1 for Implicit)
    int CRC = 0;       
    uint8_t sf = thisNodeConf.spreadingFactor;
    int CR = thisNodeConf.codingRate; // CR is 5-8

    // Numerator: (8 * PL - 4 * SF + 28 - 20 * H + 16 * CRC)
    long bits_needed = 8 * payloadLength - 4 * sf + 28 + 16 * CRC - 20 * H;
    
    // Denominator: 4 * (SF - 2*DE)
    long divisor = 4 * (sf - 2 * de);
    
    // Ceiling division of the main fraction: num_blocks = ceil(Numerator / Denominator)
    long num_blocks = (bits_needed + divisor - 1) / divisor;

    if(num_blocks < 0) num_blocks = 0;

    // Total Symbols: 8 + num_blocks * CR
    long payload_symbols = 8 + num_blocks * CR;

    float total_time_ms = header_preamble_time_ms + payload_symbols * symbol_duration_ms;
    return (uint16_t)total_time_ms;
}
/*--------------------------------------------------------------
LoRa init
--------------------------------------------------------------*/
typedef void (*LoRaReceiveCallback)(int);

bool init_LoRa(LoRaReceiveCallback onReceive){
  if(millis() - LoRaLastRestart_timestamp > LORA_MINIMUM_TIME_BETWEEN_RESTARTS){
    if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
      Serial.println("LoRa init failed. Check your connections.");            
    }
    applyConfig(thisNodeConf, false);
    LoRa.setSyncWord(0x25);      

    LoRa.setPreambleLength(8);     


    LoRa.onReceive(onReceive);
    LoRa.onTxDone(TxFinished);
    LoRaLastRestart_timestamp = millis();
    return true;
  }
  return false;
}


bool resetRadio(LoRaReceiveCallback onReceive)
{
    if(millis() - LoRaLastRestart_timestamp > LORA_MINIMUM_TIME_BETWEEN_RESTARTS){
    bool resetWorked = LoRa.reset(868E6);
    LoRa.setSyncWord(0x25);  
    LoRa.setPreambleLength(8);
    applyConfig(thisNodeConf, false);
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(TxFinished);

    LoRa.idle();
    LoRaLastRestart_timestamp = millis();
    return resetWorked;
    }
    return false;
}

/*-------------------------------
Receiving logic
-------------------------------*/

// #define RECEIVE_DEBUG_PRINT_ON
#ifdef RECEIVE_DEBUG_PRINT_ON
    #define RECEIVE_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define RECEIVE_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    #define RECEIVE_DEBUG_PRINT(...) 
    #define RECEIVE_DEBUG_PRINTLN(...) 
#endif

typedef struct{
  uint8_t     recipent;
  uint8_t     sender;
  uint16_t    incomingMsgId;
  uint8_t     flags;
  uint8_t     incomingLength;
  uint8_t     data[6];
} LoRaConfigPacket_t;
#define LORA_CONFIG_PACKET_BUFFER_SIZE 4
volatile LoRaConfigPacket_t  loraConfigPacketFIFO[LORA_CONFIG_PACKET_BUFFER_SIZE];
volatile uint8_t             loraConfigPackeSize = 0;
volatile uint8_t             loraConfigPacketRead = 0;

void inline onReceiveCommon(int packetSize){

  if (transmitting){ 
    RECEIVE_DEBUG_PRINTLN("\n----------->[BUG] radio should be idle");
  }

  if (packetSize == 0) {
    RECEIVE_DEBUG_PRINTLN("Zero packet");
    return;
  }

  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  if(sender == localAddress){
    RECEIVE_DEBUG_PRINTLN("\n----------->[BUG] Got my own message possible bounce back or hardware problem");
    return;
  }

  lastReceivedTime_ms = millis();

  if ((recipient & localAddress) != localAddress ) {
    RECEIVE_DEBUG_PRINTLN("Receiving error: This message is not for me.");
    RECEIVE_DEBUG_PRINTLN(recipient, HEX);
    return;
  } 
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();

  if(incomingMsgId == lastMsgId && lastMsgId != 0){
    RECEIVE_DEBUG_PRINTLN("\n----------->[BUG] Got twice the same msg");
    return;
  }
  // If everything alright writing data to buffer if it's empty
  
  if(loraConfigPackeSize < LORA_CONFIG_PACKET_BUFFER_SIZE){
    uint8_t next = (loraConfigPacketRead+loraConfigPackeSize) % LORA_CONFIG_PACKET_BUFFER_SIZE;
    loraConfigPackeSize++;
    RECEIVE_DEBUG_PRINTLN("Got Package");
    loraConfigPacketFIFO[next].recipent = recipient;
    loraConfigPacketFIFO[next].sender = sender;
    loraConfigPacketFIFO[next].incomingMsgId = incomingMsgId;
    loraConfigPacketFIFO[next].flags = LoRa.read();
    loraConfigPacketFIFO[next].incomingLength = min(LoRa.read(),6);
    uint8_t receivedBytes = 0; 
    while (LoRa.available() && (receivedBytes < LORA_CONFIG_PACKET_BUFFER_SIZE-1)) {            
      loraConfigPacketFIFO[next].data[receivedBytes++] = LoRa.read();
    }
    int packetRssi = LoRa.packetRssi();
    if(packetRssi < last_packet_RSSI) last_packet_RSSI = packetRssi;
    float packetSnr = LoRa.packetSnr();
    if(packetSnr < last_packet_SNR)last_packet_SNR = packetSnr;

    // if (loraConfigPacketFIFO[next].incomingLength  != receivedBytes) {// Verificamos la longitud del mensaje
      // RECEIVE_DEBUG_PRINT("Receiving error: declared message length " + String(loraConfigPacketFIFO[next].incomingLength));
      // RECEIVE_DEBUG_PRINTLN(" does not match length " + String(receivedBytes));
      // return;                             
    // }
  }else{
    RECEIVE_DEBUG_PRINTLN("[ERROR] Not enough space in buffer");
  }

  // Should be changed but do not know for what
  if(lastMsgId-incomingMsgId > 1 && lastMsgId-incomingMsgId < 100){
    RECEIVE_DEBUG_PRINT("\nLost message lastMsgId"); RECEIVE_DEBUG_PRINT(lastMsgId); RECEIVE_DEBUG_PRINT(" incoming id "); RECEIVE_DEBUG_PRINTLN(incomingMsgId);
    loosingData = true;
  } 
  gotDataProbing++;
  lastMsgId = incomingMsgId;

  return;

}

void inline LoRaPacketContentPrint(uint8_t &readPointer){
  RECEIVE_DEBUG_PRINTLN("Received from: 0x" + String(loraConfigPacketFIFO[readPointer].sender, HEX));
  RECEIVE_DEBUG_PRINTLN("Sent to: 0x" + String(loraConfigPacketFIFO[readPointer].recipent, HEX));
  RECEIVE_DEBUG_PRINTLN("Message ID: " + String(loraConfigPacketFIFO[readPointer].incomingMsgId));
  RECEIVE_DEBUG_PRINTLN("Payload length: " + String(loraConfigPacketFIFO[readPointer].incomingLength));
  RECEIVE_DEBUG_PRINT("Payload: ");
  uint8_t buffer[6];
  for(int i =0 ;i <6; i++){
    buffer[i] = loraConfigPacketFIFO[readPointer].data[i];
  }
  printBinaryPayload(buffer, loraConfigPacketFIFO[readPointer].incomingLength);
  RECEIVE_DEBUG_PRINT("RSSI: " + String(last_packet_RSSI));
  RECEIVE_DEBUG_PRINT(" dBm\nSNR: " + String(last_packet_SNR));
  RECEIVE_DEBUG_PRINTLN(" dB");

  // Actualizamos remoteNodeConf y lo mostramos
  if (loraConfigPacketFIFO[readPointer].incomingLength == 4) {
    RECEIVE_DEBUG_PRINT("Remote config: BW: ");
    RECEIVE_DEBUG_PRINT(bandwidth_kHz[remoteNodeConf.bandwidth_index]);
    RECEIVE_DEBUG_PRINT(" kHz, SPF: ");
    RECEIVE_DEBUG_PRINT(remoteNodeConf.spreadingFactor);
    RECEIVE_DEBUG_PRINT(", CR: ");
    RECEIVE_DEBUG_PRINT(remoteNodeConf.codingRate);
    RECEIVE_DEBUG_PRINT(", TxPwr: ");
    RECEIVE_DEBUG_PRINT(remoteNodeConf.txPower);
    RECEIVE_DEBUG_PRINT(" dBm, RSSI: ");
    RECEIVE_DEBUG_PRINT(remoteRSSI);
    RECEIVE_DEBUG_PRINT(" dBm, SNR: ");
    RECEIVE_DEBUG_PRINT(remoteSNR,1);
    RECEIVE_DEBUG_PRINTLN(" dB\n");
  }
}

/*----------------------------------------------
Mode logic
----------------------------------------------*/

void inline probingModeLogic(uint8_t &flags, uint16_t &msgCount, uint8_t &otherFlags){
    if((flags & CONFIG_NOT_ACCEPTED) != 0){
        mode = STABLE;
        flags = 0;
        revertConfig(msgCount, flags);
        Serial.println("---------->[LOG] new config not accepted");
        return;
    }
    if ((otherFlags & CONFIG_NOT_ACCEPTED) || millis() - lastReceivedTime_ms > (probingPlannedTime/PROBING_PLANNED_TX_INTERVALS)*2) {
        if(flags & CONFIG_NOT_ACCEPTED == 0){
        Serial.print("\n--------->[CRITICAL] Connection lost! No data received for ");
        Serial.print(probingPlannedTime);
        Serial.println(" ms. Increasing reverting to last config after timeout.");
        }
        flags = flags | CONFIG_NOT_ACCEPTED;
    }
    if(((flags & CONFIG_NOT_ACCEPTED) == 0) && (millis() - probingStarted) > probingPlannedTime){
      Serial.print(flags & CONFIG_NOT_ACCEPTED);
      Serial.println("---------->[LOG] Accepted new config");
      mode = STABLE;
      flags = 0;
      msgCount = 0;
    }
    

}

void inline startProbing(uint32_t &txInterval_ms, uint16_t &msgCount, uint8_t &flags){
    mode = PROBING;
    gotDataProbing = 0;
    probingStarted = millis();
    if(!changeToNewConfig(msgCount, flags)){
      probingPlannedTime = txInterval_ms * PROBING_PLANNED_TX_INTERVALS;
      probingWithoutData = txInterval_ms * PROBING_WITHOUT_DATA_TX_INTERVALS;
      Serial.print("Probing for faster config started  "); Serial.print(probingStarted); Serial.print("  probingPlannedTime "); Serial.println(probingPlannedTime);
      
    } else{
      probingPlannedTime = txInterval_ms * PROBING_PLANNED_TX_INTERVALS*6;
      probingWithoutData = txInterval_ms * PROBING_WITHOUT_DATA_TX_INTERVALS*6;
      Serial.print("Probing for slower config started  "); Serial.print(probingStarted); Serial.print("  probingPlannedTime "); Serial.println(probingPlannedTime);
    }
    
}

  /*-----------------------------------------------------------
  TX ended logic
  -----------------------------------------------------------*/
#define DUTY_CYCLE 40

void inline onTXCommon(uint32_t &tx_begin_ms, uint32_t &lastSendTime_ms, uint32_t &txInterval_ms, LoRaReceiveCallback onReceive, uint8_t &flags){
    uint32_t TxTime_ms = min(millis() - tx_begin_ms, theoreticalTimeOnAir);

    
    
    // Serial.print("----> TX completed in ");
    // Serial.print(TxTime_ms);
    // Serial.println(" msecs");
    
    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (1.0 * DUTY_CYCLE * TxTime_ms) / lapse_ms;
    
    // Serial.print("Duty cycle: ");
    // Serial.print(duty_cycle, 1);
    // Serial.println(" %\n");
    txInterval_ms = TxTime_ms * DUTY_CYCLE;
}
