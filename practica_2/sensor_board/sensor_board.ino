#include <ChRt.h>
#include <CAN.h>
#include <Wire.h>
#include "SRFO2_commands.h"
//------------------------------------------------------------------------------
// SRF02 handler threads
//------------------------------------------------------------------------------
#define DEBUG_CAN_TX 1
mutex_t i2c_mutex;
mutex_t can_mutex;

#define MINIMAL_PERIOD 40
#define MINIMAL_DELAY 65

inline int SRF02_write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  return Wire.endTransmission();
}

inline byte SRF02_read_register(byte address,byte the_register)
{
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();
  
  // getting sure the SRF02 is not busy
  Wire.requestFrom(address,byte(1));
  // while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
} 

inline void SRF02_get_range(int address, uint8_t range_bytes[2]) {
    range_bytes[0] = SRF02_read_register(address, RANGE_HIGH_BYTE);
    range_bytes[1] = SRF02_read_register(address, RANGE_LOW_BYTE);
}

inline void SRF02_get_min(int address, uint8_t range_bytes[2]) {
    range_bytes[0] = SRF02_read_register(address, AUTOTUNE_MINIMUM_HIGH_BYTE);
    range_bytes[1] = SRF02_read_register(address, AUTOTUNE_MINIMUM_LOW_BYTE);
}




static thread_t *SRF02_threads[2];
volatile uint32_t SRF02_reading_period_ms[2] = {100, 100};
volatile uint16_t SRF02_delay[2] = {70, 70};
volatile byte SRF02_command[2] = {81, 81};
volatile byte SRF02_adress[2] = {0X79, 0X71};
volatile bool SRF02_periodic_mode[2] = {false, false};
volatile bool SRF02_works[2] = {true, true};
static uint16_t SRF02_can_id[2] = {0x10, 0x11};

static THD_WORKING_AREA(SRF02_handler1, 128);
static THD_WORKING_AREA(SRF02_handler2, 128);

static THD_FUNCTION(SRF02_handler_function, arg)
{   
    int worker_id = int(arg);
    systime_t time = chVTGetSystemTime();
    while(true){
        chEvtWaitOne(EVENT_MASK(0));

        if(!SRF02_periodic_mode[worker_id]) {
            // one-shot
            chMtxLock(&i2c_mutex);
            bool ok = (SRF02_write_command(SRF02_adress[worker_id], SRF02_command[worker_id]) == 0);
            chMtxUnlock(&i2c_mutex);

            if(ok) {
                chThdSleepMilliseconds(SRF02_delay[worker_id]);
            
                uint8_t data[2];
                chMtxLock(&i2c_mutex);
                SRF02_get_range(SRF02_adress[worker_id], data);
                chMtxUnlock(&i2c_mutex);
            
                chMtxLock(&can_mutex);
                CAN.beginPacket(SRF02_can_id[worker_id]);
                CAN.write(data[0]);
                CAN.write(data[1]);
                CAN.endPacket();
                chMtxUnlock(&can_mutex);
            } else {
                SRF02_works[worker_id] = false;
            }
        }

        // Periodic mode
        while(SRF02_periodic_mode[worker_id]){
            // Writing command to sensor
            chMtxLock(&i2c_mutex);
            bool ok = (SRF02_write_command(SRF02_adress[worker_id], SRF02_command[worker_id]) == 0);
            chMtxUnlock(&i2c_mutex);
                
            if(ok){
                SRF02_works[worker_id] = true;
                chThdSleepMilliseconds(SRF02_delay[worker_id]);
                
                // Reading range
                uint8_t data[2];
                chMtxLock(&i2c_mutex);
                SRF02_get_range(SRF02_adress[worker_id], data);
                chMtxUnlock(&i2c_mutex);

                // Writing data to CAN
                chMtxLock(&can_mutex);
                CAN.beginPacket(SRF02_can_id[worker_id]);
                CAN.write(data[0]);
                CAN.write(data[1]);
                CAN.endPacket();
                chMtxUnlock(&can_mutex);

                // Sleeping for periodic 
                chThdSleepMilliseconds(SRF02_reading_period_ms[worker_id]);
            } else{
                SRF02_periodic_mode[worker_id] = false;
                SRF02_works[worker_id] = false;
            }
        }
    }
}



//------------------------------------------------------------------------------
// OLED handler threads
//------------------------------------------------------------------------------
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define OLED_I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

static THD_WORKING_AREA(OLED_handler1, 512);
static thread_t *oled_thread;

static THD_FUNCTION(OLED_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("OLED handler");

    chMtxLock(&i2c_mutex);
    oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    oled.clear();
    chMtxUnlock(&i2c_mutex);

    const int lines = 2;
    const int line_len = 20;
    char current_screen[lines][line_len] = {{0}};
    char last_screen[lines][line_len] = {{0}};

    while(true) {
        
        int delay0, delay1;
        bool works0, works1;
        delay0 = SRF02_delay[0];
        works0 = SRF02_works[0];
        delay1 = SRF02_delay[1];
        works1 = SRF02_works[1];


        // Making text
        snprintf(current_screen[0], line_len, "ID0:%s D:%02X", works0 ? "OK" : "ER", delay0);
        snprintf(current_screen[1], line_len, "ID1:%s D:%02X", works1 ? "OK" : "ER", delay1);

        // Iterating over every char for comparing differences
        for(int i = 0; i < lines; i++) {
            for(int j = 0; j < line_len-1; j++) { 
                if(current_screen[i][j] != last_screen[i][j]) {
                    chMtxLock(&i2c_mutex);
                    oled.setCursor(j*6, i*2);
                    oled.write(current_screen[i][j]);
                    chMtxUnlock(&i2c_mutex);
                    last_screen[i][j] = current_screen[i][j];
                    chThdSleepMilliseconds(10);
                }
            }
        }

        chThdSleepMilliseconds(1000); 
    }
}



//------------------------------------------------------------------------------
// CAN handler threads
//------------------------------------------------------------------------------
#define CAN_BUFFER_SIZE 16
struct CANFrameStruct {
    uint16_t id;
    uint8_t size;
    uint8_t data[8];
};

static CANFrameStruct can_buffer[CAN_BUFFER_SIZE];
static volatile int write_ptr = 0;
static volatile int read_ptr = 0;

static thread_t * can_thread;

static THD_WORKING_AREA(CAN_handler1, 256);


static THD_FUNCTION(CAN_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("CAN handler");

    while(true) {
        chEvtWaitOne(EVENT_MASK(0));
        while(read_ptr != write_ptr) {
            CANFrameStruct frame = can_buffer[read_ptr];
            read_ptr = (read_ptr + 1) & (CAN_BUFFER_SIZE-1);

#if DEBUG_CAN_TX
            SerialUSB.print("[CAN RX] ID=0x");
            SerialUSB.print(frame.id, HEX);
            SerialUSB.print(" DATA: ");
            for (int i = 0; i < frame.size; i++) {
                if (frame.data[i] < 0x10) SerialUSB.print("0");
                SerialUSB.print(frame.data[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
#endif

            uint8_t sensor_id  = (frame.data[0] >> 6) & 0b11;    // bits 7-6
            uint8_t command    = (frame.data[0] >> 2) & 0b1111;  // bits 5-2
            uint8_t subcommand = frame.data[0] & 0b11;           // bits 1-0

            if(frame.id == 0x110){
                handle_srf02_command(sensor_id, command, subcommand, frame.data + 1, frame.size - 1);
            }
        }
    }
}


void CAN_on_receive(int packetSize) {
    int next = (write_ptr + 1) & (CAN_BUFFER_SIZE - 1);
    if(next != read_ptr) {
        chSysLockFromISR();
        can_buffer[write_ptr].id = CAN.packetId();
        can_buffer[write_ptr].size = CAN.available();
        for(uint8_t i=0; i<can_buffer[write_ptr].size; i++)
            can_buffer[write_ptr].data[i] = CAN.read();
        write_ptr = next;
        chEvtSignalI(can_thread, EVENT_MASK(0));
      chSysUnlockFromISR();
    }
}

static void handle_srf02_command(uint8_t sensor_id, uint8_t command, uint8_t subcommand, uint8_t* args, uint8_t arg_size) {
    if(sensor_id >= 2) return; // only 2 sensors should be param

    switch(command) {
        case 0x0:  // one-shot / on / off
            switch(subcommand) {
                case 0x0:  // one-shot
                    chEvtSignal(SRF02_threads[sensor_id], EVENT_MASK(0));
                    break;

                case 0x1:  // on <period_ms>
                    if(arg_size >= 1) {
                        uint16_t period = args[0];

                        if(period < MINIMAL_PERIOD) {
                            period = MINIMAL_PERIOD;
                        }
                        SRF02_reading_period_ms[sensor_id] = period;
                        SRF02_periodic_mode[sensor_id] = true;
                        chEvtSignal(SRF02_threads[sensor_id], EVENT_MASK(0));
                    }
                    break;

                case 0x2:  // off
                    SRF02_periodic_mode[sensor_id] = false;
                    break;

                default:
                    break;
            }
            break;

        case 0x1:  // unit
            if(subcommand>2){
                return;
            }
            SRF02_command[sensor_id] = REAL_RANGING_MODE_INCHES + subcommand; // 0=inc,1=cm,2=ms
            break;

        case 0x2:  // delay
            if(arg_size >= 1) {
                uint16_t delay_ms = args[0]; // lub args[0]<<8 | args[1] jeśli 16 bit
                if(delay_ms< MINIMAL_DELAY) delay_ms = MINIMAL_DELAY;
                SRF02_delay[sensor_id] = delay_ms;
            }
            break;

        case 0x3:  // status
        {
            uint8_t response[8];
            response[0] = SRF02_adress[sensor_id];
            response[1] = SRF02_delay[sensor_id] & 0xFF;
            response[2] = (SRF02_delay[sensor_id] >> 8) & 0xFF;
            response[3] = SRF02_reading_period_ms[sensor_id] & 0xFF;
            response[4] = (SRF02_reading_period_ms[sensor_id] >> 8) & 0xFF;
            response[5] = SRF02_periodic_mode[sensor_id] ? 1 : 0;
            response[6] = SRF02_command[sensor_id]- REAL_RANGING_MODE_INCHES; // unit


            chMtxLock(&can_mutex);
            CAN.beginPacket(0x111);
            CAN.write(response, 7);
            CAN.endPacket();
            chMtxUnlock(&can_mutex);
            break;
        }

        case 0x4: 
        {
            // uint8_t response[2] = {2,0}; // 2 sensory
            uint8_t active_count = 0;
            if (SRF02_works[0]) active_count++;
            if (SRF02_works[1]) active_count++;

            chMtxLock(&can_mutex);
            CAN.beginPacket(0x112);
            CAN.write(active_count);
            CAN.endPacket();
            chMtxUnlock(&can_mutex);
            break;
        }

        default:
            // unknown command
            break;
    }
}


// setup

void chSetup()
{
    
    // mutexes
    chMtxObjectInit(&i2c_mutex);
    chMtxObjectInit(&can_mutex);

    // Threads
    SRF02_threads[0] = chThdCreateStatic(
        SRF02_handler1, sizeof(SRF02_handler1),
        NORMALPRIO+1, SRF02_handler_function, (void*)0
    );

    SRF02_threads[1] = chThdCreateStatic(
        SRF02_handler2, sizeof(SRF02_handler2),
        NORMALPRIO+1, SRF02_handler_function, (void*)1
    );
    
    can_thread = chThdCreateStatic(CAN_handler1, sizeof(CAN_handler1),
                  NORMALPRIO+1, CAN_handler_function, NULL);

    

    // ISR for can        
    CAN.onReceive(CAN_on_receive);

    oled_thread = chThdCreateStatic(
        OLED_handler1, sizeof(OLED_handler1),
        NORMALPRIO, OLED_handler_function, NULL
    );
    
}

void setup(){


    SerialUSB.begin(115200);
    while(!SerialUSB);
    SerialUSB.print("Starting sensor board");
    Wire.begin();
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }
    delay(2000);
    chBegin(chSetup);
}

void loop(){

}