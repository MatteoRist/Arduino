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


inline int SRF02_write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  return Wire.endTransmission();
}

byte SRF02_read_register(byte address,byte the_register)
{
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();
  
  // getting sure the SRF02 is not busy
  Wire.requestFrom(address,byte(1));
  // while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
} 

void SRF02_get_range(int address, uint8_t range_bytes[2]) {
    range_bytes[0] = SRF02_read_register(address, RANGE_HIGH_BYTE);
    range_bytes[1] = SRF02_read_register(address, RANGE_LOW_BYTE);
}

void SRF02_get_min(int address, uint8_t range_bytes[2]) {
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
        time = chVTGetSystemTime();
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
                time += TIME_MS2I(SRF02_reading_period_ms[worker_id]);
                chThdSleepUntil(time);
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

static THD_WORKING_AREA(OLED_handler1, 192);
static thread_t *oled_thread;

static THD_FUNCTION(OLED_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("OLED handler");

    // Inicjalizacja I2C i wyświetlacza
    chMtxLock(&i2c_mutex);
    Wire.begin();
    Wire.setClock(400000L);
    oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    oled.clear();
    oled.println("OLED started");
    chMtxUnlock(&i2c_mutex);

    // Główna pętla statusu
    while(true) {
        chMtxLock(&i2c_mutex);
        oled.clear();
        oled.setCursor(0, 0);
        oled.println("SRF02 STATUS:");
        for (int i = 0; i < 2; i++) {
            oled.print("ID");
            oled.print(i);
            oled.print(": ");
            if (SRF02_works[i]) oled.print("OK ");
            else oled.print("ERR ");

            if (SRF02_periodic_mode[i]) oled.print("P ");
            else oled.print("M "); // Manual

            oled.print("D:");
            oled.print(SRF02_delay[i]);
            oled.print(" P:");
            oled.print(SRF02_reading_period_ms[i]);
            oled.println();
        }
        chMtxUnlock(&i2c_mutex);

        // Odświeżanie co 1s
        chThdSleepMilliseconds(1000);
    }
}


//------------------------------------------------------------------------------
// CAN handler threads
//------------------------------------------------------------------------------
#define CAN_BUFFER_SIZE 16
struct CANFrameStruct {
    uint16_t can_id;
    uint8_t size;
    uint8_t data[8];
};

static CANFrameStruct can_buffer[CAN_BUFFER_SIZE];
static volatile int write_ptr = 0;
static volatile int read_ptr = 0;

static thread_t * can_thread;

static THD_WORKING_AREA(CAN_handler1, 128);


static THD_FUNCTION(CAN_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("CAN handler");

    while(true) {
        chEvtWaitOne(EVENT_MASK(0));

        while(read_ptr != write_ptr) {
            CANFrameStruct frame = can_buffer[read_ptr];
            read_ptr = (read_ptr + 1) % CAN_BUFFER_SIZE;

#if DEBUG_CAN_TX
            SerialUSB.print("[CAN RX] ID=0x");
            SerialUSB.print(frame.can_id, HEX);
            SerialUSB.print(" DATA: ");
            for (int i = 0; i < frame.size; i++) {
                if (frame.data[i] < 0x10) SerialUSB.print("0");
                SerialUSB.print(frame.data[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
#endif

            uint8_t sensor_id  = (frame.data[0] >> 6) & 0b11;    // bity 7-6
            uint8_t command    = (frame.data[0] >> 2) & 0b1111;  // bity 5-2
            uint8_t subcommand = frame.data[0] & 0b11;           // bity 1-0

            if(frame.can_id == 0x110){
                handle_srf02_command(sensor_id, command, subcommand, frame.data + 1, frame.size - 1);
            }
        }
    }
}


void CAN_on_receive(int packetSize) {
    chSysLockFromISR();

    if(CAN.packetId() <= 0x7FF && CAN.available() > 0) {
        CANFrameStruct frame;
        frame.can_id = CAN.packetId();
        frame.size = CAN.available();
        for(int i=0; i<frame.size; i++)
            frame.data[i] = CAN.read();

        int next = (write_ptr + 1) % CAN_BUFFER_SIZE;
        if(next != read_ptr) {  // sprawdzanie przepełnienia
            can_buffer[write_ptr] = frame;
            write_ptr = next;
        }
    }

    chEvtSignalI(can_thread, EVENT_MASK(0));
    chSysUnlockFromISR();
}

static void handle_srf02_command(uint8_t sensor_id, uint8_t command, uint8_t subcommand, uint8_t* args, uint8_t arg_size) {
    if(sensor_id >= 2) return; // tylko 2 sensory

    switch(command) {
        case 0x0:  // one-shot / on / off
            switch(subcommand) {
                case 0x0:  // one-shot
                    chEvtSignal(SRF02_threads[sensor_id], EVENT_MASK(0));
                    break;

                case 0x1:  // on <period_ms>
                    if(arg_size >= 1) {
                        uint16_t period = args[0]; // jeśli potrzebujesz 16 bitów, w args[0] i args[1]

                        if(period < SRF02_delay[sensor_id] + 20) {
                            period = SRF02_delay[sensor_id] + 20;
                        }
                        SRF02_reading_period_ms[sensor_id] = period+ SRF02_delay[sensor_id];
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
            if(arg_size >= 1) {
                if(subcommand>2){
                    return;
                }
                SRF02_command[sensor_id] = REAL_RANGING_MODE_INCHES + subcommand; // 0=inc,1=cm,2=ms
            }
            break;

        case 0x2:  // delay
            if(arg_size >= 1) {
                uint16_t delay_ms = args[0]; // lub args[0]<<8 | args[1] jeśli 16 bit
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
        NORMALPRIO, SRF02_handler_function, (void*)0
    );

    SRF02_threads[1] = chThdCreateStatic(
        SRF02_handler2, sizeof(SRF02_handler2),
        NORMALPRIO, SRF02_handler_function, (void*)1
    );
    can_thread = chThdCreateStatic(CAN_handler1, sizeof(CAN_handler1),
                  NORMALPRIO, CAN_handler_function, NULL);

    // ISR for can        
    CAN.onReceive(CAN_on_receive);
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