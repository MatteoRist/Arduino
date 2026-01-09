#include "LoRa.h"

#define SENDING_ASYNC 0


// Frame will be send by base length is 11, sends it as a heartbeat and correction for slaves
inline bool send_correction_frame(uint8_t& minutes, uint8_t& seconds, int32_t& lat_correction, int32_t& lon_correction){
    if(!LoRa.beginPacket()){
        return false;
    }
    LoRa.write(0x00);
    LoRa.write(minutes);
    LoRa.write(seconds);
    LoRa.write((uint8_t *) &lon_correction, 4);
    LoRa.write((uint8_t *) &lat_correction, 4);
    LoRa.endPacket(SENDING_ASYNC);
    return true;
}

// Frame will be send by slaves length is 15, sends its raw fix when has connection to the base
inline bool send_localization_frame(const uint8_t ttl,const uint32_t& id,const uint8_t& minutes,const uint8_t& seconds,const int32_t& lat,const int32_t& lon){
    if(!LoRa.beginPacket()){
        return false;
    }
    if(ttl){
        LoRa.write((128 | ttl));
    }else{
        LoRa.write(0x01);
    }
    
    LoRa.write((uint8_t *) &id, 4);
    LoRa.write(minutes);
    LoRa.write(seconds);
    LoRa.write((uint8_t *) &lon, 4);
    LoRa.write((uint8_t *) &lat, 4);
    LoRa.endPacket(SENDING_ASYNC);
    return true;
}