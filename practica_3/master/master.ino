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

#define DEFAULT_CONFIG_DETECTION_INTERVAL 100000
uint32_t last_default_config_detection_ts = 0;

void inline optimizeConfig(uint8_t &flags);
void inline sendDetectionConfig();
// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  while (!Serial); 

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

 
  localAddress = MASTER_IP;
  init_LoRa(onReceive);
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
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
  static bool defaultConfigDetection = false;
  static bool gotMsg;
  
  uint8_t masterFlags = 0;
  if(loraConfigPackeSize){
    if(loraConfigPacketFIFO[loraConfigPacketRead].sender == SLAVE_IP){
      masterFlags = loraConfigPacketFIFO[loraConfigPacketRead].flags;
    }
    #ifdef RECEIVE_DEBUG_PRINT_ON
      uint8_t readPointer = loraConfigPacketRead;
      RECEIVE_DEBUG_PRINT("Flags received ");
      printFlags(masterFlags);
      // if(loraConfigPacketFIFO[readPointer].incomingLength == 4){
        remoteNodeConf.bandwidth_index = loraConfigPacketFIFO[readPointer].data[0] >> 4;
        remoteNodeConf.spreadingFactor = 6 + ((loraConfigPacketFIFO[readPointer].data[0] & 0x0F) >> 1);
        remoteNodeConf.codingRate = 5 + (loraConfigPacketFIFO[readPointer].data[1] >> 6);
        remoteNodeConf.txPower = 2 + ((loraConfigPacketFIFO[readPointer].data[1] & 0x3F) >> 1);
        remoteRSSI = -int(loraConfigPacketFIFO[readPointer].data[2]) / 2.0f;
        remoteSNR  =  int(loraConfigPacketFIFO[readPointer].data[3]) - 148;
      // }

      LoRaPacketContentPrint(readPointer);
    #endif
    Serial.print("Flags received ");
    printFlags(masterFlags);
    loraConfigPacketRead = (loraConfigPacketRead+1) % LORA_CONFIG_PACKET_BUFFER_SIZE;
    loraConfigPackeSize--;
    gotMsg = true;
  }


  if(mode == INSTABLE ){
    flags = flags | INSTABLE_FLAG;
    if((millis() - instableStarted) > instablePlannedTime){
      mode = STABLE;
      flags = 0;
      msgCount = 0;
      revertConfig(msgCount, flags);
    }
  }

  if(masterFlags){
    if(masterFlags & CONFIG_CHANGE_FLAG){
      startProbing(txInterval_ms, msgCount, flags);
      txInterval_ms = 0;
    }
  }

  // Restarting radio just in case
  if(!transmitting && mode == STABLE && (millis() - lastReceivedTime_ms) > txInterval_ms * 3){
    resetRadio(onReceive);
    LoRa.receive();
    lastReceivedTime_ms = millis();
  }

  

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    if(mode == STABLE && ((millis() - last_default_config_detection_ts) > DEFAULT_CONFIG_DETECTION_INTERVAL) && applyConfig(defaultConfig, true)){ 
      sendDetectionConfig();
      tx_begin_ms = millis();
      transmitting = true;
      txDoneFlag = false;
      defaultConfigDetection = true;

    }else{
      Serial.print("Want to transmit msg");

      if(gotMsg){
        optimizeConfig(flags);
        gotMsg = false;
      }


      uint8_t payload[50];
      uint8_t payloadLength = 0;
      payload[payloadLength]    = (nextConfig.bandwidth_index << 4);
      payload[payloadLength++] |= ((nextConfig.spreadingFactor - 6) << 1);
      payload[payloadLength]    = ((nextConfig.codingRate - 5) << 6);
      payload[payloadLength++] |= ((nextConfig.txPower - 2) << 1);

      // Incluimos el RSSI y el SNR del último paquete recibido
      // RSSI puede estar en un rango de [0, -127] dBm
      payload[payloadLength++] = uint8_t(-last_packet_RSSI * 2);
      // SNR puede estar en un rango de [20, -148] dBm
      payload[payloadLength++] = uint8_t(148 + last_packet_SNR);
      payloadLength = 4;


      if(sendMessage(payload, payloadLength, msgCount, flags, !(flags & CONFIG_CHANGE_FLAG))){
      
      transmitting = true;
      txDoneFlag = (flags & CONFIG_CHANGE_FLAG);
      tx_begin_ms = millis() - theoreticalTimeOnAir*(!!(flags & CONFIG_CHANGE_FLAG)); 
      
      Serial.print("Sending packet ");
      Serial.print(msgCount++);
      Serial.print(" flags sent ");
      printFlags(flags);
      Serial.print(": ");
      printBinaryPayload(payload, payloadLength);
      
      if(last_packet_RSSI < RSSI_THRESHOLD && thisNodeConf.txPower < 20){
          flags = 0;
          msgCount = 0;
          changeToNewConfig(msgCount, flags);
          mode = STABLE;
          Serial.print("RSSI back");

          printFlags(flags);
      }
      }else{
        Serial.print("Blad wysylanie pakietu");
      }

      if(mode == PROBING){
        probingModeLogic(flags, msgCount, masterFlags);
      }
    }
  }       
  
  

  if (transmitting && txDoneFlag) {
    if(defaultConfigDetection){
      Serial.print("After default transmission");
      applyConfig(thisNodeConf, false);
      defaultConfigDetection = false;
    }
    onTXCommon(tx_begin_ms ,lastSendTime_ms, txInterval_ms, onReceive, flags);

    transmitting = false;
    LoRa.receive();
    
  }

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

void inline optimizeConfig(uint8_t &flags){
  bool lowerTXPower = false;
  bool fasterBW = false;
  bool lowSnr = false; 
  if(mode == STABLE){
    if(last_packet_RSSI < RSSI_THRESHOLD && thisNodeConf.txPower < 20){
          Serial.print("Lora RSSI below threshold"); Serial.print(last_packet_RSSI); Serial.print(" < "); Serial.println(RSSI_THRESHOLD);
          nextConfig = thisNodeConf;
          nextConfig.txPower++;
            applyConfig(nextConfig, true);
            thisNodeConf = nextConfig;
            last_packet_RSSI = RSSI_THRESHOLD + 1;
        } else if (last_packet_SNR < SNR_THRESHOLD[thisNodeConf.spreadingFactor-7]){
          Serial.print("Lora SNR below threshold"); Serial.print(last_packet_SNR); Serial.print(" < "); Serial.println(SNR_THRESHOLD[thisNodeConf.spreadingFactor-7]);
          lowSnr = true;
        } else if (mode == STABLE && last_packet_RSSI > RSSI_THRESHOLD_UPPER && thisNodeConf.txPower>2){
            nextConfig = thisNodeConf;
            nextConfig.txPower--;
            applyConfig(nextConfig, true);
            thisNodeConf = nextConfig;
            last_packet_RSSI = RSSI_THRESHOLD_UPPER - 1;
        } else if (last_packet_SNR > SNR_THRESHOLD_UPPER){
          if(thisNodeConf.bandwidth_index < 9){
            fasterBW = true;
            txIntervals = min(++txIntervals,TX_INTERVAL_BEFORE_PROBING);
          } else{
          if(thisNodeConf.txPower > 2){
            lowerTXPower = true;
          }
          }
    } 
    if(txIntervals ==TX_INTERVAL_BEFORE_PROBING){
          nextConfig = thisNodeConf;

          if(lowSnr){
            if(thisNodeConf.spreadingFactor < 12){
              nextConfig.spreadingFactor ++;
            }
            flags = CONFIG_CHANGE_FLAG; 
            if(thisNodeConf.bandwidth_index > 0){
              nextConfig.bandwidth_index--;
            }
          } else if(lowerTXPower){
            flags = CONFIG_CHANGE_FLAG; nextConfig.txPower--;

          } else if(fasterBW){
            flags = CONFIG_CHANGE_FLAG;
            nextConfig.bandwidth_index++;
            nextConfig.spreadingFactor = max(7, thisNodeConf.spreadingFactor-1);
          }
          else if(thisNodeConf.spreadingFactor != 7 && last_packet_SNR > SNR_THRESHOLD[thisNodeConf.spreadingFactor-8]){
            flags = CONFIG_CHANGE_FLAG; nextConfig.spreadingFactor--;
          }
          else if(thisNodeConf.bandwidth_index != 9 && last_packet_SNR > SNR_THRESHOLD[thisNodeConf.spreadingFactor-7] + 3){
            flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index++;
          }
          else if (thisNodeConf.codingRate != 5){
            flags = CONFIG_CHANGE_FLAG; nextConfig.codingRate--;
          }else if (thisNodeConf.txPower != 2){
            flags = CONFIG_CHANGE_FLAG; nextConfig.txPower--;
          }else {
            txIntervals = 0;
          }
          printFlags(flags);
          last_packet_RSSI = 10;
          last_packet_SNR = 10;
    }
    if(flags & CONFIG_CHANGE_FLAG){
      mode = PRE_PROBING;
      resetRadio(onReceive);
    }
    txIntervals++;
    }
}

void inline sendDetectionConfig(){
  Serial.print("[LOG] default detection");
  uint8_t payload[4];
  payload[0]    = (thisNodeConf.bandwidth_index << 4);
  payload[0] |= ((thisNodeConf.spreadingFactor - 6) << 1);
  payload[1]    = ((thisNodeConf.codingRate - 5) << 6);
  payload[1] |= ((thisNodeConf.txPower - 2) << 1);
  if(sendMessage(payload, 4, 255, CONFIG_CHANGE_FLAG | CONFIG_NOT_ACCEPTED, true)){
    Serial.print("Sending packet ");
    Serial.print(255);
    Serial.print(" flags sent ");
    printFlags(CONFIG_CHANGE_FLAG | CONFIG_NOT_ACCEPTED);
    Serial.print(": ");
    printBinaryPayload(payload, 4);
  }
  last_default_config_detection_ts = millis();
}



