#ifndef NETWORKING_H
#define NETWORKING_H

#include <LoRa.h>
#include "debugging_keywords.h"

#define STANDARD_BANDWIDTH 500e3
#define STANDARD_SPREADING_FACTOR 7
#define STANDARD_CODING_FACTOR 5
#define STANDARD_TX_POWER 2


void onReceive(int packetSize);
bool init_LoRa();

bool init_LoRa(){

    if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
      Serial.println("[ERROR] LoRa init failed. Check your connections.");   
      return false;         
    }
    LoRa.setSignalBandwidth(STANDARD_BANDWIDTH);
    LoRa.setSpreadingFactor(STANDARD_SPREADING_FACTOR);
    LoRa.setCodingRate4(STANDARD_CODING_FACTOR);
    LoRa.setTxPower(STANDARD_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);

    LoRa.setSyncWord(0x25);      

    LoRa.setPreambleLength(8);     


    LoRa.onReceive(onReceive);
    return true;

}

#define MINIMAL_PACKET_SIZE 1
#define MAXIMUM_PACKET_SIZE 20
#define LORA_PACKET_BUFFER_SIZE 4
typedef struct{
    uint8_t data[MAXIMUM_PACKET_SIZE];
    uint8_t size;
} Frame_t;
volatile Frame_t  loraPacketFIFO[LORA_PACKET_BUFFER_SIZE];
volatile uint8_t  loraPackeSize = 0;
volatile uint8_t  loraPacketRead = 0;
void inline onReceive(int packetSize){
//   if (transmitting){ 
    // RECEIVE_DEBUG_PRINTLN("\n----------->[BUG] radio should be idle");
//   }

  if (packetSize < MINIMAL_PACKET_SIZE || packetSize > MAXIMUM_PACKET_SIZE) {
    DEBUG_PRINTLN("packet size is incorrect");
    return;
  }

  // If everything alright writing data to buffer if it's empty
  
  if(loraPackeSize < LORA_PACKET_BUFFER_SIZE){
    uint8_t next = (loraPacketRead+loraPackeSize) % LORA_PACKET_BUFFER_SIZE;
    loraPackeSize++;
    uint8_t receivedBytes = 0; 
    while (LoRa.available() && (receivedBytes < MAXIMUM_PACKET_SIZE-1)) {            
      loraPacketFIFO[next].data[receivedBytes++] = LoRa.read();
    }
    loraPacketFIFO[next].size = receivedBytes;
  }else{
    DEBUG_PRINTLN("Not enough space in buffer");
  }

}

#endif