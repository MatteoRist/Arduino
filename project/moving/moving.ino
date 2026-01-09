#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
#include "networking.h"
#include "debugging_keywords.h"
#include "sending_messages.h"
#include "filesystem.h"

uint32_t id = 1;

static inline bool correctionValid(uint32_t other_time) {
    return (millis() - other_time) <= 3000UL;
}

typedef struct{
    int32_t lat_correction;
    int32_t lon_correction;
    uint32_t millis;
} Correction_t;

static Correction_t correctionLast;

typedef struct{
    uint32_t id;
    uint16_t distance;
    int32_t last_lat;
    int32_t last_lon;
    uint32_t millis;
} MovingDistance_t;
#define MAX_OTHER_MOVING_SIZE 8
static MovingDistance_t otherMovingKeepDistance[MAX_OTHER_MOVING_SIZE];
static uint8_t otherMovingKeepDistanceSize = 0;

typedef struct{
    int32_t lat[3];
    int32_t lon[3];
    int32_t minLat, maxLat, minLon, maxLon; 
    uint8_t must_be_out;
} Area_t;
#define MAX_AREAS_SIZE 16
static Area_t areas[MAX_AREAS_SIZE];
static uint8_t areasSize = 0;

static int32_t thisLat = 0;
static int32_t thisLon = 0;

static bool signalizeMoving = false;
static bool signalizeArea = false;

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
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to must_be_output UBX only (turn off NMEA noise)

    if(initFileSystem()){
        readConfigFile();
    }
}

void loop(){
    static uint32_t lastTimeGNSSReading = 0;
    uint32_t now = millis();

    if (now - lastTimeGNSSReading > 1000)
    {
        lastTimeGNSSReading = millis(); //Update the timer
        if(myGNSS.getSIV()>6){
            int32_t lat_read = myGNSS.getLatitude();
            int32_t lon_read = myGNSS.getLongitude();
            uint8_t minutes = myGNSS.getMinute();
            uint8_t seconds = myGNSS.getSecond();
            if(correctionValid(correctionLast.millis)){
                thisLat = lat_read + correctionLast.lat_correction;
                thisLon = lon_read + correctionLast.lon_correction;
                send_localization_frame(0,id,minutes, seconds, lat_read, lon_read);
            }else{
                thisLat = lat_read;
                thisLon = lon_read;
                send_localization_frame(129, id, minutes, seconds, thisLat, thisLon);
            }
            isInArea();
            isCloseToOtherMoving();
            
        }

    }
    if(loraPackeSize > 0){
        handleLoRaFrames();
        noInterrupts();
        loraPacketRead = (loraPacketRead+1) % LORA_PACKET_BUFFER_SIZE;
        loraPackeSize--;
        interrupts();
    }

    if(SerialUSB.available()){
        char buff[64];
        uint8_t idx = 0;
        do{
            char c = SerialUSB.read();
            if (c == '\n' || c == '\r')
            {
                buff[idx++] = '\0';
                if (idx > 0)
                    processUserCommand(buff, idx, true);
                idx = 0;
            }
            else
            {
                buff[idx++] = c;
            }
        } while(SerialUSB.available() && idx < sizeof(buff) - 1);
    }
}    

typedef struct{
    uint32_t lastLatCalculated;
    uint32_t value;
} CosCalculated_t;


void isCloseToOtherMoving() {
    static CosCalculated_t lastCosCalculated = {-90e7, 0};

    signalizeMoving = false;
    
    if(abs(lastCosCalculated.lastLatCalculated - thisLat)>9000){
        float cosVal = cos((float)thisLat * 1e-7f * 0.0174532925f);
        lastCosCalculated.value = (int32_t)(cosVal * 1024.0f);
        lastCosCalculated.lastLatCalculated = thisLat;
    }
    int32_t lonScaleInt = lastCosCalculated.value;
    

    uint32_t now = millis();

    for (uint8_t other = 0; other < otherMovingKeepDistanceSize; other++) {
        if (now - otherMovingKeepDistance[other].millis < 60000) {
            
            int32_t dLat = thisLat - otherMovingKeepDistance[other].last_lat;
            int32_t dLon = thisLon - otherMovingKeepDistance[other].last_lon;

            int32_t limitInUnits = (int32_t)otherMovingKeepDistance[other].distance * 90;

            if (abs(dLat) > limitInUnits) continue;
            

            int32_t dLonScaled = (dLon * lonScaleInt) >> 10;
            
            if (abs(dLonScaled) > limitInUnits) continue;
            
            int32_t distSq = (dLat * dLat) + (dLonScaled * dLonScaled);
            int32_t limitSq = limitInUnits * limitInUnits;

            if (distSq < limitSq) {
                signalizeMoving = true;
                return;
            }
        }   
    }
}

void updateAreaBBox(uint8_t idx) {
    areas[idx].minLat = areas[idx].lat[0];
    areas[idx].maxLat = areas[idx].lat[0];
    areas[idx].minLon = areas[idx].lon[0];
    areas[idx].maxLon = areas[idx].lon[0];

    for(uint8_t j=1; j<3; j++) {
        if(areas[idx].lat[j] < areas[idx].minLat) areas[idx].minLat = areas[idx].lat[j];
        if(areas[idx].lat[j] > areas[idx].maxLat) areas[idx].maxLat = areas[idx].lat[j];
        if(areas[idx].lon[j] < areas[idx].minLon) areas[idx].minLon = areas[idx].lon[j];
        if(areas[idx].lon[j] > areas[idx].maxLon) areas[idx].maxLon = areas[idx].lon[j];
    }
}

void isInArea(){
    signalizeArea = false;
    bool shouldBeInAnyArena = false;
    bool isInArenaWhenShould = false;

    const int32_t curLat = thisLat;
    const int32_t curLon = thisLon;

    for(uint8_t areaPointer = 0; areaPointer < areasSize; areaPointer++){
        if (curLat < areas[areaPointer].minLat || curLat > areas[areaPointer].maxLat ||
            curLon < areas[areaPointer].minLon || curLon > areas[areaPointer].maxLon) {
            
            if (areas[areaPointer].must_be_out == false) {
                shouldBeInAnyArena = true;
            }
            continue;
        }
        uint8_t crossHowMany = 0;
        bool isIn = true;
        for(uint8_t edge = 0; edge<3; edge++){
            uint8_t next = edge+1;
            if(next==3){
                next = 0;
            }
            int64_t val = (int64_t)(areas[areaPointer].lat[edge] - areas[areaPointer].lat[next]) * (int64_t)(curLon - areas[areaPointer].lon[next]) -
              (int64_t)(areas[areaPointer].lon[edge] - areas[areaPointer].lon[next]) * (int64_t)(curLat - areas[areaPointer].lat[next]);

            if (val < 0) {
                crossHowMany++;
            }
        }
        if(!(crossHowMany == 3 || crossHowMany == 0)){
            isIn = false;
        }
        if(areas[areaPointer].must_be_out == false){
                    shouldBeInAnyArena = true;
                    if(isIn){
                        isInArenaWhenShould = true;
                    }
        }
        else if(areas[areaPointer].must_be_out == isIn){
                signalizeArea = true;
        }
    }
    if(shouldBeInAnyArena != isInArenaWhenShould){
        signalizeArea = true;
    }
}

void handleLoRaFrames() {
    uint8_t* currentData = (uint8_t*)loraPacketFIFO[loraPacketRead].data;
    uint8_t msg_type = currentData[0];
    uint8_t msg_size = loraPacketFIFO[loraPacketRead].size;

    // Correction frame
    if(msg_type == 0 && msg_size == 11) {
        memcpy(&correctionLast.lon_correction, &currentData[3], 4);
        memcpy(&correctionLast.lat_correction, &currentData[7], 4);
        correctionLast.millis = millis();
    }
    // Frame for resending
    else if((msg_type >= 128 || msg_type == 0x01) && msg_size == 15) {
        uint32_t received_id;
        int32_t received_lat;
        int32_t received_lon;
        memcpy(&received_id,  &currentData[1], 4);
        uint8_t received_minutes = currentData[5];
        uint8_t received_seconds = currentData[6];
        memcpy(&received_lon, &currentData[7], 4); 
        memcpy(&received_lat, &currentData[11], 4);

        for(uint8_t i = 0; i < otherMovingKeepDistanceSize; i++) {
            if(otherMovingKeepDistance[i].id == received_id) {
                otherMovingKeepDistance[i].millis = millis();
                
                if(correctionValid(correctionLast.millis)) {
                    otherMovingKeepDistance[i].last_lat = received_lat + correctionLast.lat_correction;
                    otherMovingKeepDistance[i].last_lon = received_lon + correctionLast.lon_correction;
                } else {
                    otherMovingKeepDistance[i].last_lat = received_lat;
                    otherMovingKeepDistance[i].last_lon = received_lon;
                }
            }
        }

        if(msg_type > 128) {
            uint8_t new_ttl = (msg_type & 0x7F) - 1; 
            send_localization_frame(new_ttl, received_id, received_minutes, received_seconds, received_lat, received_lon);
        }
    }
}

inline int32_t checkIfHexTill(char* buffer, uint8_t& idx,const char& tillThis, bool& correct){
    int32_t number = 0;
    while(buffer[idx] != tillThis){
            uint8_t digit = 0;
            if((buffer[idx] >= '0' && buffer[idx] <= '9' )) digit = buffer[idx] - '0';
            else if((buffer[idx] >= 'a' && buffer[idx] <= 'f')) digit = buffer[idx] - 'a' + 10;
            else if((buffer[idx] >= 'A' && buffer[idx] <= 'F')) digit = buffer[idx] - 'A' + 10;
            else{correct = false; return 0;}
            number = (number << 4) | digit;
            idx++;
    }
    return number;
}

void processUserCommand(char* buffer, uint8_t size, bool withSaving){
    char c = buffer[0];

    if(c == 'b' && strcmp(buffer, "begin") == 0){
        resetFile();
    } else if(c == 'm'){
        if(otherMovingKeepDistanceSize == MAX_OTHER_MOVING_SIZE){
            ERR_PRINTLN("maximum movings");
            return;
        }
        uint8_t idx = 1;
        bool correct = true;
        uint32_t id_command = 0;
        uint16_t distance = 0;
        id_command = checkIfHexTill(buffer,idx, ';', correct); idx++;
        distance = checkIfHexTill(buffer,idx, '\0', correct);
        if(correct && idx == size){
            if(withSaving) appendToConfig(buffer, size);
            otherMovingKeepDistance[otherMovingKeepDistanceSize].id = id_command;
            otherMovingKeepDistance[otherMovingKeepDistanceSize++].distance = distance;
        } else{
            ERR_PRINTLN("incorrect input data");
        }
    } else if(c == 'a'){
        if(areasSize == MAX_AREAS_SIZE){
            ERR_PRINTLN("maximum areas");
            return;
        }
        uint8_t idx = 1;
        bool correct = true;
        Area_t data_buffer;
        data_buffer.lat[0] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.lon[0] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.lat[1] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.lon[1] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.lat[2] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.lon[2] = checkIfHexTill(buffer,idx, ';', correct); idx++;
        data_buffer.must_be_out = checkIfHexTill(buffer,idx, '\0', correct);
        if(correct && idx == size){
            if(withSaving) appendToConfig(buffer, size);
            areas[areasSize++] = data_buffer;
            updateAreaBBox(areasSize-1);
        }else ERR_PRINTLN("incorrect input data");

    } else{
        ERR_PRINTLN("incorrect input data"); 
    }
}

bool readConfigFile() {

    File file = filesystem.open(CONFIG_FILENAME, READ_ONLY);
    if (!file) {
        ERR_PRINTLN("Failed to open config file for reading.");
        return false;
    }

    char line[64];
    char command[64];
    uint8_t size = 0;
    uint8_t idx = 0;
    int bytes_read = file.read(line, 64);
    while (bytes_read) {
        
        if (line[idx] == '\0') {
            idx++;
            if (size != 0){
                size++;
                processUserCommand(command, size, false);
                size = 0;
            }
            
        } else {
            if (idx < sizeof(line) - 1) {
                command[size++] = line[idx++];
            }
        }
        if(idx >= sizeof(line) - 1){
            bytes_read = file.read(line, 64);
            idx = 0;
        }
    }
    file.close();
    return true;
}

