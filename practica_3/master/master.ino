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




void inline optimizeConfig(uint8_t &flags);

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

 
  localAddress = 0xB0;
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
  
  uint8_t masterFlags = 0;
  if(loraConfigPackeSize){
    masterFlags = loraConfigPacketFIFO[loraConfigPacketRead].flags;
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
    loraConfigPacketRead = (loraConfigPacketRead+1) % LORA_CONFIG_PACKET_BUFFER_SIZE;
    loraConfigPackeSize--;
    optimizeConfig(flags);
  }

  if(mode == PROBING){
    probingModeLogic(flags, msgCount);
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
    if(masterFlags & INSTABLE_FLAG_ACK){
      flags = flags | INSTABLE_FLAG_ACK;
    }
    if(masterFlags & INSTABLE_FLAG){
      mode = STABLE;
      flags = 0;
      msgCount = 0;
      revertConfig(msgCount, flags);
    }
  }

  // Restarting radio just in case
  if(!transmitting && mode == STABLE && (millis() - lastReceivedTime_ms) > txInterval_ms * 3){
    resetRadio(onReceive);
    LoRa.receive();
    lastReceivedTime_ms = millis();
    // mode = INSTABLE;
    // Serial.println("Starting instable modePacket");
    // mode = INSTABLE;
    // instablePlannedTime = txInterval_ms * INSTABLE_PLANNED_TX_INTERVALS;
    // instableStarted = millis();
    // nextConfig = thisNodeConf;
    // flags = flags | INSTABLE_FLAG;
    // if(flags & INSTABLE_FLAG_ACK ){
      // mode = STABLE;
      // flags = 0;
      // msgCount = 0;
      // revertConfig(msgCount, flags);
    // }

  }

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {

    Serial.print("Want to transmit msg");
    if(loosingData){
        flags = flags | LOOSING_DATA_FLAG;
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

    
    if(sendMessage(payload, payloadLength, msgCount, flags)){
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis(); 
     
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
    flags &= ~CONFIG_CHANGE_FLAG;
    }else{
      Serial.print("Blad wysylanie pakietu");
    }
  }       
  
  if(transmitting && millis() - tx_begin_ms > theoreticalTimeOnAir*1.5){
      Serial.print("\n----------->[BUG] Sending time is too long txTime: ");Serial.print(millis() - tx_begin_ms); Serial.print("  should be max: "); Serial.println(theoreticalTimeOnAir);
      init_LoRa(onReceive);
      txDoneFlag = true;
  }

  if (transmitting && txDoneFlag) {

    onTXCommon(tx_begin_ms ,lastSendTime_ms, txInterval_ms, onReceive);

    // STARTING PROBING TO GET FASTER 
    if(mode == STABLE && txIntervals > TX_INTERVAL_BEFORE_PROBING){
      Serial.println("Starting Probing mode");
      // Deciding what to do
      
      // if(txIntervals > TX_INTERVAL_BEFORE_PROBING){
        mode = PROBING;
        startProbing(txInterval_ms, msgCount, flags);
      // }
    }
    // if(mode == STABLE && loosingData){
      // Serial.println("Starting instable mode");
      // mode = INSTABLE;
      // instablePlannedTime = txInterval_ms * INSTABLE_PLANNED_TX_INTERVALS;
      // instableStarted = millis();
    // }
    transmitting = false;
    LoRa.receive();
    
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
  bool lowerBW = false; 
  if(last_packet_RSSI < RSSI_THRESHOLD && thisNodeConf.txPower < 20){
        Serial.print("Lora RSSI below threshold"); Serial.print(last_packet_RSSI); Serial.print(" < "); Serial.println(RSSI_THRESHOLD);
        nextConfig = thisNodeConf;
        nextConfig.txPower++;
        tried_conf[0] = false;
        tried_conf[1] = false;
        tried_conf[2] = false;
        tried_conf[3] = true;
        flags = flags | CONFIG_CHANGE_FLAG;
        printFlags(flags);
    } else if (mode == STABLE && last_packet_RSSI > RSSI_THRESHOLD_UPPER && thisNodeConf.txPower>2){
        lowerTXPower = true;
    } 
    else if (last_packet_SNR > SNR_THRESHOLD_UPPER){
      if(thisNodeConf.bandwidth_index < 9){
        fasterBW = true;
        txIntervals = min(++txIntervals,TX_INTERVAL_BEFORE_PROBING);
      } else{
      if(thisNodeConf.txPower > 2){
        lowerTXPower = true;
      }
      }
    } else if (last_packet_SNR < SNR_THRESHOLD){
      Serial.print("Lora SNR below threshold"); Serial.print(last_packet_SNR); Serial.print(" < "); Serial.println(SNR_THRESHOLD);
      lowerBW = true;
    }

  if(mode == STABLE && txIntervals ==TX_INTERVAL_BEFORE_PROBING){
      nextConfig = thisNodeConf;

      if(lowerBW){
        flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index--;
      }if(lowerTXPower){
        flags = CONFIG_CHANGE_FLAG; nextConfig.txPower--;
        lowerTXPower = false;
      } else if(fasterBW){
        flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index++;
      }
      else if(!tried_conf[1] && thisNodeConf.spreadingFactor != 7){
        tried_conf[1] = true;
        flags = CONFIG_CHANGE_FLAG; nextConfig.spreadingFactor--;
      }
      else if(!tried_conf[0] && thisNodeConf.bandwidth_index != 9){
        tried_conf[0] = true;
        flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index++;
      }
      else if (!tried_conf[2] && thisNodeConf.codingRate != 5){
        tried_conf[2] = true;
        flags = CONFIG_CHANGE_FLAG; nextConfig.codingRate--;
      }else{
        txIntervals = 0;
      }
      printFlags(flags);
    }

    if(mode == STABLE){
      txIntervals++;
    }
}



