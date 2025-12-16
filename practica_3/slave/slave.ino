/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_Binary
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithCallbacks,
 *  muestra cómo es posible comunicar los parámetros de 
 *  configuración del transceiver entre nodos LoRa en
 *  formato binario *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include "networking.h"

uint32_t lastReceivedMasterTime_ms = 0;
#define TIMEOUT_TO_DEFAULT 100000
#define MINIMAL_TX_TO_DEFAULT 8

// NOTA: Ajustar estas variables 



// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  // while (!Serial); 

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  localAddress = SLAVE_IP;
  init_LoRa(onReceive);
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");

  Serial.println("SLAVE started");
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() 
{
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
  static uint8_t flags = 0;
  static bool gotMsg = false;

  uint8_t masterFlags = 0;
  if(loraConfigPackeSize){
    uint8_t readPointer = loraConfigPacketRead;
    if(loraConfigPacketFIFO[loraConfigPacketRead].sender == MASTER_IP){
      masterFlags = loraConfigPacketFIFO[loraConfigPacketRead].flags;
    }
    // if(loraConfigPacketFIFO[readPointer].incomingLength == 4){
      remoteNodeConf.bandwidth_index = loraConfigPacketFIFO[readPointer].data[0] >> 4;
      remoteNodeConf.spreadingFactor = 6 + ((loraConfigPacketFIFO[readPointer].data[0] & 0x0F) >> 1);
      remoteNodeConf.codingRate = 5 + (loraConfigPacketFIFO[readPointer].data[1] >> 6);
      remoteNodeConf.txPower = 2 + ((loraConfigPacketFIFO[readPointer].data[1] & 0x3F) >> 1);
      if(remoteNodeConf.txPower != thisNodeConf.txPower){
        thisNodeConf.txPower = remoteNodeConf.txPower;
        LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
      }
      remoteRSSI = -int(loraConfigPacketFIFO[readPointer].data[2]) / 2.0f;
      remoteSNR  =  int(loraConfigPacketFIFO[readPointer].data[3]) - 148;
    // }
    #ifdef RECEIVE_DEBUG_PRINT_ON
      RECEIVE_DEBUG_PRINT("Flags received ");
      printFlags(masterFlags);    
      LoRaPacketContentPrint(readPointer);
    #endif
    Serial.print("Flags received ");
    printFlags(masterFlags);
    if(loraConfigPacketFIFO[readPointer].sender == MASTER_IP){
      lastReceivedMasterTime_ms = millis();
    }
    loraConfigPacketRead = (loraConfigPacketRead+1) % LORA_CONFIG_PACKET_BUFFER_SIZE;
    loraConfigPackeSize--;
    gotMsg = true;
  }


  if(mode == PROBING){
        probingModeLogic(flags, msgCount, masterFlags);
  }


  if(!transmitting && mode == STABLE && (millis() - lastReceivedTime_ms) > txInterval_ms * 2){
    init_LoRa(onReceive);
    LoRa.receive();
    Serial.println("Maybe init");
    txInterval_ms = getTimeOnAirBytes(10)*100;
    lastReceivedTime_ms = millis();
  }
  if(mode == STABLE && (millis() - lastReceivedMasterTime_ms) > max(TIMEOUT_TO_DEFAULT,txInterval_ms * MINIMAL_TX_TO_DEFAULT)){
    Serial.println("[LOG] Returning to the default");
    applyConfig(defaultConfig, true);
    thisNodeConf = defaultConfig;
    lastReceivedMasterTime_ms = millis();
  }
  // Flags logic
  if(masterFlags) {
        uint8_t copyMasterFlags = masterFlags;
        masterFlags = 0;


        if(copyMasterFlags & CONFIG_CHANGE_FLAG) {
            if(copyMasterFlags & CONFIG_NOT_ACCEPTED){
              nextConfig = remoteNodeConf;
              applyConfig(remoteNodeConf, true);
              thisNodeConf = nextConfig;
            }else{
              nextConfig = remoteNodeConf;
              flags = CONFIG_CHANGE_FLAG | flags;
            }
            
      }


  }

      
  if (!transmitting && gotMsg && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    Serial.print("i am here");
    gotMsg = false;
    if(!(mode == PROBING && (flags & CONFIG_NOT_ACCEPTED))){
      uint8_t payload[50];
      uint8_t payloadLength = 0;

      payload[payloadLength]    = (thisNodeConf.bandwidth_index << 4);
      payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
      payload[payloadLength]    = ((thisNodeConf.codingRate - 5) << 6);
      payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

      // Incluimos el RSSI y el SNR del último paquete recibido
      // RSSI puede estar en un rango de [0, -127] dBm
      payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
      // SNR puede estar en un rango de [20, -148] dBm
      payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());
      payloadLength = 4;
      delay(10);
      if(sendMessage(payload, payloadLength, msgCount, flags, !((flags & CONFIG_CHANGE_FLAG) || SYNC))){
      Serial.print(!(flags & CONFIG_CHANGE_FLAG));
      transmitting = true;
      txDoneFlag = ((flags & CONFIG_CHANGE_FLAG) || SYNC);
      tx_begin_ms = millis() - theoreticalTimeOnAir*(!!((flags & CONFIG_CHANGE_FLAG) || SYNC)); 

      Serial.print("Sending packet ");
      Serial.print(msgCount++);
      Serial.print(" flags sent ");
      printFlags(flags);
      Serial.print(": ");
      printBinaryPayload(payload, payloadLength);}
      else{
        Serial.println("\nBlad wysyłania pakietu!!!!!");
      }
    }
  } 

  /*-----------------------------------------------------------
  TX ended logic
  -----------------------------------------------------------*/


  if (transmitting && txDoneFlag) {

    onTXCommon(tx_begin_ms ,lastSendTime_ms, txInterval_ms, onReceive, flags);

    transmitting = false;
    if(flags & CONFIG_CHANGE_FLAG){
      startProbing(txInterval_ms, msgCount, flags);
      txInterval_ms = 0;
    }
    // Reactivamos la recepción de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    LoRa.receive();   
  }
  // hardware bug fix
  if(transmitting && millis() - tx_begin_ms > theoreticalTimeOnAir*1.5){
      Serial.print("\n----------->[BUG] Sending time is too long txTime: ");Serial.print(millis() - tx_begin_ms); Serial.print("  should be max: "); Serial.println(theoreticalTimeOnAir);
      init_LoRa(onReceive);
      txDoneFlag = true;
      // flags = flags | CONFIG_NOT_ACCEPTED;
  }
}




// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  onReceiveCommon(packetSize);
}











