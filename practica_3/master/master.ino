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
  static bool lowerTXPower = false;
  static bool lowerBW = false;
  
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
    uint8_t copyMasterFlag = masterFlags;
    masterFlags = 0;
    if(copyMasterFlag & INSTABLE_FLAG_ACK){
      flags = flags | INSTABLE_FLAG_ACK;
    }
    if(copyMasterFlag & INSTABLE_FLAG){
      mode = STABLE;
      flags = 0;
      msgCount = 0;
      revertConfig(msgCount, flags);
    }
  }

  if(mode == STABLE && (millis() - lastReceivedTime_ms) > txInterval_ms * 3){
    mode = INSTABLE;
    Serial.println("Starting instable modePacket");
    mode = INSTABLE;
    instablePlannedTime = txInterval_ms * INSTABLE_PLANNED_TX_INTERVALS;
    instableStarted = millis();
    nextConfig = thisNodeConf;
    flags = flags | INSTABLE_FLAG;
    if(flags & INSTABLE_FLAG_ACK ){
      mode = STABLE;
      flags = 0;
      msgCount = 0;
      revertConfig(msgCount, flags);
    }

  }

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    if(mode == STABLE && txIntervals ==TX_INTERVAL_BEFORE_PROBING){
      nextConfig = thisNodeConf;
      if(lowerTXPower){
        
        flags = CONFIG_CHANGE_FLAG; nextConfig.txPower--;
        lowerTXPower = false;
      } else if(lowerBW){
        flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index++;
      }
      else if(!tried_conf[1]){
        if(thisNodeConf.spreadingFactor == 7){ tried_conf[1] = true;}
        else{flags = CONFIG_CHANGE_FLAG; nextConfig.spreadingFactor--;}
      }
      else if(!tried_conf[0]){
        if(thisNodeConf.bandwidth_index == 9){ tried_conf[0] = true;}
        else{flags = CONFIG_CHANGE_FLAG; nextConfig.bandwidth_index++;}
      }
      else if (!tried_conf[2]){
        if(thisNodeConf.codingRate == 5){ tried_conf[2] = true;}
        else{flags = CONFIG_CHANGE_FLAG; nextConfig.codingRate--;}
      }else{
        txIntervals = 0;
      }
      printFlags(flags);
    }

    Serial.print("Want to transmit msg");
    if(loosingData){
        flags = flags | LOOSING_DATA_FLAG;
    }

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
        lowerBW = true;
      txIntervals = min(++txIntervals,TX_INTERVAL_BEFORE_PROBING);
      } else{
      if(thisNodeConf.txPower > 2){
        lowerTXPower = true;
      }
      }
    } else if (last_packet_SNR < SNR_THRESHOLD){
      Serial.print("Lora SNR below threshold"); Serial.print(last_packet_SNR); Serial.print(" < "); Serial.println(SNR_THRESHOLD);
        nextConfig = thisNodeConf;
        nextConfig.bandwidth_index--;
        tried_conf[0] = true;
        tried_conf[1] = false;
        tried_conf[2] = false;
        tried_conf[3] = false;
        flags = flags | CONFIG_CHANGE_FLAG;
        printFlags(flags);
    }
      

    uint8_t payload[50];
    uint8_t payloadLength = 0;
    payload[payloadLength]    = (nextConfig.bandwidth_index << 4);
    payload[payloadLength++] |= ((nextConfig.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((nextConfig.codingRate - 5) << 6);
    payload[payloadLength++] |= ((nextConfig.txPower - 2) << 1);

    // Incluimos el RSSI y el SNR del último paquete recibido
    // RSSI puede estar en un rango de [0, -127] dBm
    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    // SNR puede estar en un rango de [20, -148] dBm
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());
    

    
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

    if(mode == STABLE){
      txIntervals++;
    }
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
    if(mode == STABLE && loosingData){
      Serial.println("Starting instable mode");
      mode = INSTABLE;
      instablePlannedTime = txInterval_ms * INSTABLE_PLANNED_TX_INTERVALS;
      instableStarted = millis();
    }
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



