
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
#include "networking.h"
#include "debugging_keywords.h"
#include "sending_messages.h"


int32_t base_lon = 0;
int32_t base_lat = 0;

typedef struct{
    int32_t lat_correction;
    int32_t lon_correction;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minutes;
    uint8_t seconds;
} Correction_t;

static Correction_t correctionLast;

void getAveragePosition(uint8_t gnss_frame_iterations_max){
    uint32_t lastTime = 0;
    int64_t sum_lat = 0;
    int64_t sum_lon = 0;
    uint8_t gnss_frame_iterations = 0;

    while(gnss_frame_iterations < gnss_frame_iterations_max){
        if (millis() - lastTime > 1000){
            DEBUG_PRINT("SATELLITES: "); DEBUG_PRINT_WOUT(myGNSS.getSIV()); DEBUG_PRINT_WOUT("\n");
            if(myGNSS.getSIV()>8){
                long latitude = myGNSS.getLatitude();
                sum_lat += latitude;
                long longitude = myGNSS.getLongitude();
                sum_lon += longitude;
                gnss_frame_iterations++;
            }
            lastTime = millis();
        }
        delay(10);
    }
    base_lat = (sum_lat / gnss_frame_iterations);
    base_lon = (sum_lon / gnss_frame_iterations);
    LOG_PRINT("AVG lat: " + String(base_lat));
    LOG_PRINT_WOUT("  AVG lon: " + String(base_lon) + "\n");
}

void setup(){
    Serial.begin(115200);
    while (!Serial)
      ; //Wait for user to open terminal
    Serial.println("SparkFun u-blox Example");

    Wire.begin();

    if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
    {
      Serial.println(F("[ERROR] u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
      while (1);
    }
    if(!init_LoRa()){
        while(1);
    }
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

    getAveragePosition(10);
}

void loop()
{
    static uint32_t lastTime = 0;
    uint32_t now = millis();

    if (now - lastTime > 1000)
    {
    lastTime = millis(); //Update the timer
    correctionLast.lat_correction = base_lat - myGNSS.getLatitude();
    correctionLast.lon_correction = base_lon - myGNSS.getLongitude();
    correctionLast.year = myGNSS.getYear();
    correctionLast.month = myGNSS.getMonth();
    correctionLast.day = myGNSS.getDay();
    correctionLast.hour = myGNSS.getHour();
    correctionLast.minutes = myGNSS.getMinute();
    correctionLast.seconds = myGNSS.getSecond();
    send_correction_frame(correctionLast.minutes, correctionLast.seconds, correctionLast.lat_correction, correctionLast.lon_correction);

    // byte SIV = myGNSS.getSIV();
    // Serial.print(F(" SIV: "));
    // Serial.print(SIV);
    }

    if(loraPackeSize > 0){
        uint8_t* currentData = (uint8_t*)loraPacketFIFO[loraPacketRead].data;
        if((currentData[0] == 0x01 || currentData[0] >= 128) && loraPacketFIFO[loraPacketRead].size == 15){
            uint32_t received_id; memcpy(&received_id,  &currentData[1], 4);
            uint8_t received_minutes = currentData[5];
            uint8_t received_seconds = currentData[6];
            int32_t received_lon; memcpy(&received_lon,  &currentData[7], 4);
            int32_t received_lat; memcpy(&received_lat,  &currentData[11], 4);
            Serial.print(received_id); Serial.print(";");
            Serial.print(correctionLast.year); Serial.print("-");
            Serial.print(correctionLast.month); Serial.print("-");
            Serial.print(correctionLast.day); Serial.print("-");
            Serial.print(correctionLast.hour); Serial.print("-");
            Serial.print(correctionLast.minutes); Serial.print("-");
            Serial.print(correctionLast.seconds); Serial.print(";");
            Serial.print(received_lat + correctionLast.lat_correction); Serial.print(";");
            Serial.println(received_lon + correctionLast.lon_correction);
        }
        noInterrupts();
        loraPacketRead = (loraPacketRead+1) % LORA_PACKET_BUFFER_SIZE;
        loraPackeSize--;
        interrupts();
    }
}