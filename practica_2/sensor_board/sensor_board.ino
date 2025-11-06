#include <ChRt.h>
#include <CAN.h>
#include <Wire.h>
#include "SRFO2_commands.h"
//------------------------------------------------------------------------------
// SRF02 handler threads
//------------------------------------------------------------------------------
mutex_t i2c_mutex;
mutex_t can_mutex;


inline void SRF02_write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  Wire.endTransmission();
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

int SRF02_get_min(int SRF02_adress){
    byte high_min=SRF02_read_register(SRF02_adress,AUTOTUNE_MINIMUM_HIGH_BYTE);
    byte low_min=SRF02_read_register(SRF02_adress,AUTOTUNE_MINIMUM_LOW_BYTE);
    int((high_min<<8) | low_min);
}




static thread_t *SRF02_threads[2];
volatile uint32_t SRF02_reading_period_ms[2] = {100, 100};
volatile uint16_t SRF02_delay[2] = {70, 70};
volatile byte SRF02_command[2] = {81, 81};
volatile byte SRF02_adress[2] = {0X79, 0X79};
volatile bool SRF02_periodic_mode[2] = {false, false};
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
        while(SRF02_periodic_mode[worker_id]){
            // Writing command to sensor
            chMtxLock(&i2c_mutex);
            SRF02_write_command(SRF02_adress[worker_id], SRF02_command[worker_id]);
            chMtxUnlock(&i2c_mutex);

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
        }
    }
}



//------------------------------------------------------------------------------
// OLED handler threads
//------------------------------------------------------------------------------
static THD_WORKING_AREA(OLED_handler1, 128);


static THD_FUNCTION(OLED_handler_function, arg)
{

}

//------------------------------------------------------------------------------
// CAN handler threads
//------------------------------------------------------------------------------

struct CANFrameStruct {
    uint16_t can_id;
    uint8_t size;
    uint8_t data[8];
};


static thread_t * can_thread;

static THD_WORKING_AREA(CAN_handler1, 128);


static THD_FUNCTION(CAN_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("CAN handler");

    while(true) {
        chEvtWaitOne(EVENT_MASK(0));

        while(CAN.parsePacket()) {
            if(CAN.available() < 1) continue;

            CANFrameStruct frame;
            frame.can_id = CAN.packetId();
            frame.size = CAN.available();

            for(int i=0; i<frame.size; i++)
                frame.data[i] = CAN.read();

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
    (void)packetSize; 

    chSysLockFromISR();
    chEvtSignalI(can_thread, EVENT_MASK(0));
    chSysUnlockFromISR();
}

static void handle_srf02_command(uint8_t sensor_id, uint8_t command, uint8_t subcommand, uint8_t* args, uint8_t arg_size) {
    if(sensor_id >= 2) return; // tylko 2 sensory

    switch(command) {
        case 0x0:  // one-shot / on / off
            switch(subcommand) {
                case 0x0:  // one-shot
                    SRF02_periodic_mode[sensor_id] = false;
                    chEvtSignal(SRF02_threads[sensor_id], EVENT_MASK(0));
                    break;

                case 0x1:  // on <period_ms>
                    if(arg_size >= 1) {
                        uint16_t period = args[0]; // jeśli potrzebujesz 16 bitów, w args[0] i args[1]
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
            response[6] = SRF02_command[sensor_id]- REAL_RANGING_MODE_INCHES; // jednostka


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
            chMtxLock(&can_mutex);
            CAN.beginPacket(0x112);
            CAN.write(2);
            CAN.endPacket();
            chMtxUnlock(&can_mutex);
            break;
        }

        default:
            // nieznana komenda
            break;
    }
}


// setup

void chSetup()
{
    // Here we assume that CH_CFG_ST_TIMEDELTA is set to zero
    // All SAMD-based boards are only supported in “tick mode”
    // Check first if ChibiOS configuration is compatible
    // with a non-cooperative scheme checking the value of CH_CFG_TIME_QUANTUM
    if (CH_CFG_TIME_QUANTUM == 0) {
    SerialUSB.println("You must set CH_CFG_TIME_QUANTUM to a non-zero value in");
    #if defined(__arm__)
    SerialUSB.print("src/<board type>/chconfig<board>.h");
    #elif defined(__AVR__)
    SerialUSB.print("src/avr/chconfig_avr.h");
    #endif
    SerialUSB.println(" to enable round-robin scheduling.");
    while (true) {}
    }
    SerialUSB.print("CH_CFG_TIME_QUANTUM: ");
    SerialUSB.println(CH_CFG_TIME_QUANTUM);
    // Check we do not spawn the idle thread
    if (CH_CFG_NO_IDLE_THREAD == FALSE) {
    SerialUSB.println("You must set CH_CFG_NO_IDLE_THREAD to TRUE");
    }


    chMtxObjectInit(&i2c_mutex);
    chMtxObjectInit(&can_mutex);
    can_thread = chThdCreateStatic(CAN_handler1, sizeof(CAN_handler1),
                  NORMALPRIO, CAN_handler_function, NULL);
    CAN.onReceive(CAN_on_receive);
}

void setup(){



    
    chBegin(chSetup);
}

void loop(){

}