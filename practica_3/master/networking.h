#include <SPI.h>
#include <LoRa.h>

#define TX_LAPSE_MS          10000

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

LoRaConfig_t defaultConfig = { 6, 10, 5, 2};
LoRaConfig_t thisNodeConf   = defaultConfig;
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;

//--------------------------NEW THINGS-------------------------------

enum Mode {STABLE, PROBING, INSTABLE};
Mode mode = STABLE;

#define TX_INTERVAL_BEFORE_PROBING 5
int txIntervals = 0;

#define CONFIG_CHANGE_FLAG 128
#define SECOND_FLAG 64
#define LOOSING_DATA_FLAG 32
#define INSTABLE_FLAG 16
#define INSTABLE_FLAG_ACK 8
#define SIXTH_FLAG 4
#define SEVENTH_FLAG 2
#define EIGHT_FLAG 1


static bool tried_conf[4] = {false,false,false,false};

#define RSSI_THRESHOLD -95
#define RSSI_THRESHOLD_UPPER -50
#define SNR_THRESHOLD -10
#define SNR_THRESHOLD_UPPER 10

volatile int last_packet_RSSI = -60;
volatile float last_packet_SNR = 0;

#define PROBING_PLANNED_TX_INTERVALS 5
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
void applyConfig(const LoRaConfig_t conf);
void updateTimingParameters(const LoRaConfig_t& conf);
uint32_t getTimeOnAirBytes(uint8_t payloadLength);
typedef void (*LoRaReceiveCallback)(int);
void init_LoRa(LoRaReceiveCallback onReceive);

// sendinf and getting
bool sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount, uint8_t flags);
void TxFinished();


// debbuging
void printBinaryPayload(uint8_t * payload, uint8_t payloadLength);
void printFlags(uint8_t flags);

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------

bool sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount, uint8_t flags) 
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
  LoRa.endPacket(true);                   

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
  if (flags & SECOND_FLAG)         Serial.print("SECOND_FLAG ");
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
void applyConfig(const LoRaConfig_t conf) {

  LoRa.setSignalBandwidth(long(bandwidth_kHz[conf.bandwidth_index]));
  LoRa.setSpreadingFactor(conf.spreadingFactor);
  LoRa.setCodingRate4(conf.codingRate);
  LoRa.setTxPower(conf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  // LoRa.setSyncWord(0x12);
  // LoRa.setPreambleLength(50);
  Serial.print("\nApplied Config -> SF:"); Serial.print(conf.spreadingFactor);
  Serial.print(" BW_IDX:"); Serial.print(conf.bandwidth_index);
  Serial.print("  CODING_RATE  "); Serial.println(conf.codingRate); Serial.println();
  Serial.println('\n\n');
  updateTimingParameters(conf);

}

inline bool operator==(const LoRaConfig_t& a, const LoRaConfig_t& b) {
    return a.bandwidth_index == b.bandwidth_index &&
           a.spreadingFactor == b.spreadingFactor &&
           a.codingRate == b.codingRate &&
           a.txPower == b.txPower;
}

inline bool operator>(const LoRaConfig_t& a, const LoRaConfig_t& b){
    return a.bandwidth_index > b.bandwidth_index ||
           a.spreadingFactor < b.spreadingFactor ||
           a.codingRate < b.codingRate ||
           a.txPower < b.txPower;
}

bool inline revertConfig(uint16_t &msgCount, uint8_t &flags){
    if(previousConfig == thisNodeConf){
        previousConfig = defaultConfig;
    } else if(previousConfig > thisNodeConf){
        previousConfig = thisNodeConf;
        min(12,previousConfig.spreadingFactor++);
    }

    thisNodeConf = previousConfig;
    nextConfig = previousConfig;
    applyConfig(thisNodeConf);
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
    thisNodeConf = nextConfig;
    applyConfig(thisNodeConf);
    loosingData = 0;
    txIntervals = 0;
    msgCount = 0;
    lastMsgId = 255;
    lastReceivedTime_ms = millis();
}

/*--------------------------------------------------------------
Calculating theory time in tx
--------------------------------------------------------------*/

// Wywołujemy w applyConfig()
void updateTimingParameters(const LoRaConfig_t& conf) {
    // 1. czas trwania symbolu (Ts) w ms
    double bw_hz = bandwidth_kHz[conf.bandwidth_index];
    symbol_duration_ms = (pow(2, conf.spreadingFactor) / bw_hz) * 1000.0;

    // 2. Low Data Rate Optimization
    ldro_enabled = (symbol_duration_ms > 16.0);

    // 3. Czas preambuły
    header_preamble_time_ms = (PREAMBLE_LEN + 4.25) * symbol_duration_ms;

    // 4. Skuteczny coding rate
    cr_eff = conf.codingRate; // przechowujemy raz, użyjemy później
}

// --- Funkcja zwracająca czas nadawania dla dowolnej ilości bajtów ---
uint32_t getTimeOnAirBytes(uint8_t payloadLength) {
    if (symbol_duration_ms == 0) return 0;

    int de = ldro_enabled ? 1 : 0;
    int H = 0;          // Explicit header
    int CRC = 1;        // CRC enabled
    uint8_t sf = thisNodeConf.spreadingFactor;
    int CR = thisNodeConf.codingRate - 4; // CR=5 -> 1

    long bits_needed = 8*payloadLength - 4*sf + 28 + 16*CRC - 20*H;
    long divisor = 4 * (sf - 2*de);
    long num_blocks = (bits_needed + divisor - 1)/divisor;

    if(num_blocks < 0) num_blocks = 0;

    // CR+4, bo biblioteka używa wartości 1-4 zamiast 5-8
    long payload_symbols = 8 + num_blocks * cr_eff;

    float total_time_ms = header_preamble_time_ms + payload_symbols * symbol_duration_ms;
    return (uint16_t)total_time_ms;
}
/*--------------------------------------------------------------
LoRa init
--------------------------------------------------------------*/
typedef void (*LoRaReceiveCallback)(int);

void init_LoRa(LoRaReceiveCallback onReceive){

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }
  applyConfig(thisNodeConf);
  LoRa.setSyncWord(0x12);      

  LoRa.setPreambleLength(8);     

  
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);

}


bool resetRadio(LoRaReceiveCallback onReceive)
{
    LoRa.reset(868E6);
    LoRa.setSyncWord(0x12);  
    LoRa.setPreambleLength(8);
    applyConfig(thisNodeConf);
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(TxFinished);

    LoRa.idle();
}

/*-------------------------------
Receiving logic
-------------------------------*/

#define RECEIVE_DEBUG_PRINT_ON
#ifdef RECEIVE_DEBUG_PRINT_ON
    // Tryb debugowania: MAKRA są faktycznymi wywołaniami Serial
    #define RECEIVE_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define RECEIVE_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    // Tryb produkcyjny: MAKRA są PUSTE i zostaną usunięte przez kompilator
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
  if ((recipient & localAddress) != localAddress ) {
    RECEIVE_DEBUG_PRINTLN("Receiving error: This message is not for me.");
    RECEIVE_DEBUG_PRINTLN(recipient, HEX);
    return;
  } 
  uint8_t sender = LoRa.read();
  if(sender == localAddress){
    RECEIVE_DEBUG_PRINTLN("\n----------->[BUG] Got my own message possible bounce back or hardware problem");
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
    loraConfigPacketFIFO[next].incomingLength = LoRa.read();
    uint8_t receivedBytes = 0; 
    while (LoRa.available() && (receivedBytes < uint8_t(sizeof(LORA_CONFIG_PACKET_BUFFER_SIZE)-1))) {            
      loraConfigPacketFIFO[next].data[receivedBytes++] = LoRa.read();
    }
    last_packet_RSSI = LoRa.packetRssi();
    last_packet_SNR = LoRa.packetSnr();

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
  lastReceivedTime_ms = millis();

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

void inline probingModeLogic(uint8_t &flags, uint16_t &msgCount){
    if (millis() - lastReceivedTime_ms > (probingPlannedTime/PROBING_PLANNED_TX_INTERVALS)*2) {

        Serial.print("\n[CRITICAL] Connection lost! No data received for ");
        Serial.print(probingPlannedTime);
        Serial.println(" ms. Increasing reverting to last config immediately.");
        
        revertConfig(msgCount, flags);
        mode = STABLE;
        flags = 0;
        msgCount = 0;
        
    }

    if((millis() - probingStarted) > probingPlannedTime){
      if(!loosingData && gotDataProbing > PROBING_PLANNED_TX_INTERVALS/2){
        mode = STABLE;
        flags = 0;
        tried_conf[0] = false;
        tried_conf[1] = false;
        tried_conf[2] = false;
        tried_conf[3] = false;
      } else{
        revertConfig(msgCount, flags);
        mode = STABLE;
        flags = 0;
        msgCount = 0;
      }
    }
}

void inline startProbing(uint32_t &txInterval_ms, uint16_t &msgCount, uint8_t &flags){
    gotDataProbing = 0;
    probingStarted = millis();
    probingPlannedTime = txInterval_ms * PROBING_PLANNED_TX_INTERVALS;
    probingWithoutData = txInterval_ms * PROBING_WITHOUT_DATA_TX_INTERVALS;
    Serial.print("Probing started  "); Serial.print(probingStarted); Serial.print("  probingPlannedTime "); Serial.println(probingPlannedTime);

    changeToNewConfig(msgCount, flags);
}

  /*-----------------------------------------------------------
  TX ended logic
  -----------------------------------------------------------*/

void inline onTXCommon(uint32_t &tx_begin_ms, uint32_t &lastSendTime_ms, uint32_t &txInterval_ms, LoRaReceiveCallback onReceive){
    uint32_t TxTime_ms = min(millis() - tx_begin_ms, theoreticalTimeOnAir);

    if (digitalRead(LORA_DEFAULT_DIO0_PIN) == HIGH){
      Serial.println("\n----------->[BUG] LORA DIO0 PIN should be low");
      resetRadio(onReceive);
    }
    
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");
    
    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;
    
    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
    // if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    // }
}
