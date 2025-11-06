#include <ChRt.h>
#include <CAN.h>
#include <Wire.h>

#define DEBUG_CAN_TX 1

//------------------------------------------------------------------------------
// CAN handler
//------------------------------------------------------------------------------
struct CANFrameStruct {
    uint16_t id;
    uint8_t size;
    uint8_t data[8];
};

static thread_t* can_thread;
static THD_WORKING_AREA(CAN_handler, 128);

static CANFrameStruct frame;
static volatile bool frame_received = false;

static THD_FUNCTION(CAN_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("CAN handler");

    while(true) {
        chEvtWaitOne(EVENT_MASK(0));
        while(CAN.parsePacket()) {
            frame.id = CAN.packetId();
            frame.size   = CAN.available();
            for(int i = 0; i < frame.size; i++)
                frame.data[i] = CAN.read();
            if (frame.id == 0x10 || frame.id == 0x11) {
                // pomiar z SRF02 (dwa bajty: high + low)
                uint16_t distance = (frame.data[0] << 8) | frame.data[1];
                uint8_t sensor_id = (frame.id == 0x10) ? 0 : 1;
                
                SerialUSB.print("[SRF02#");
                SerialUSB.print(sensor_id);
                SerialUSB.print("] Distance: ");
                SerialUSB.print(distance);

            }
            else if (frame.id == 0x110) {
                // odpowiedź na zapytanie status/lista
                uint8_t sensor_addr = frame.data[0];
                uint16_t delay = (frame.data[2] << 8) | frame.data[1];
                uint16_t period = (frame.data[4] << 8) | frame.data[3];
                bool periodic = frame.data[5];
                uint8_t unit = frame.data[6];

                SerialUSB.println("--- SRF02 Status ---");
                SerialUSB.print("Address: 0x");
                SerialUSB.println(sensor_addr, HEX);
                SerialUSB.print("Delay: ");
                SerialUSB.print(delay);
                SerialUSB.println(" ms");
                SerialUSB.print("Period: ");
                SerialUSB.print(period);
                SerialUSB.println(" ms");
                SerialUSB.print("Mode: ");
                SerialUSB.println(periodic ? "ON" : "OFF");
                SerialUSB.print("Unit: ");
                if (unit == 0) SerialUSB.println("inches");
                else if (unit == 1) SerialUSB.println("cm");
                else if (unit == 2) SerialUSB.println("ms");
                else SerialUSB.println("unknown");
                SerialUSB.println("--------------------");
            }
            else if (frame.id == 0x112) {
                // lista dostępnych sensorów
                uint8_t count = frame.data[0];
                SerialUSB.print("Available SRF02 sensors: ");
                SerialUSB.println(count);
            }
            else {
                SerialUSB.print("[UNKNOWN CAN ID] 0x");
                SerialUSB.println(frame.id, HEX);
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

//------------------------------------------------------------------------------
// Helper: send command
//------------------------------------------------------------------------------
void send_srf02_command(uint8_t sensor, uint8_t cmd, uint8_t subcmd, uint8_t* args = nullptr, uint8_t arg_size = 0) {
    uint8_t frame[8];
    frame[0] = ((sensor & 0b11) << 6) | ((cmd & 0b1111) << 2) | (subcmd & 0b11);
    for(uint8_t i=0; i<arg_size && i<7; i++)
        frame[i+1] = args[i];
#if DEBUG_CAN_TX
    SerialUSB.print("[CAN TX] ID=0x110 Data: ");
    for(uint8_t i = 0; i < arg_size + 1; i++) {
        if(frame[i] < 0x10) SerialUSB.print("0");
        SerialUSB.print(frame[i], HEX);
        SerialUSB.print(" ");
    }
    SerialUSB.println();
#endif
    CAN.beginPacket(0x110); // ID płytki z sensorami
    CAN.write(frame, arg_size+1);
    CAN.endPacket();
}
static THD_WORKING_AREA(UserHandler, 256);
static THD_FUNCTION(UserHandlerFunction, arg) {
    (void)arg;
    while(true) {
        if(SerialUSB.available()) {
            String line = SerialUSB.readStringUntil('\n');
            process_user_command(line);
        }
        chThdSleepMilliseconds(10);
    }
}
//------------------------------------------------------------------------------
// ChibiOS setup
//------------------------------------------------------------------------------
void chSetup() {
    SerialUSB.begin(115200);
    while(!SerialUSB);

    chMtxObjectInit(nullptr); // brak mutexów w tym prostym przykładzie
    can_thread = chThdCreateStatic(CAN_handler, sizeof(CAN_handler), NORMALPRIO,
                                   CAN_handler_function, NULL);
    chThdCreateStatic(UserHandler, sizeof(UserHandler), NORMALPRIO, UserHandlerFunction, NULL);
    CAN.onReceive(CAN_on_receive);

    SerialUSB.println("Sensor interface ready. Commands:");
    SerialUSB.println("us <sensor> <mode/on/off/status/delay/unit> [param]");
}

//------------------------------------------------------------------------------
// parse user input
//------------------------------------------------------------------------------
void process_user_command(String line) {
    line.trim();
    if(line.length() == 0) return;

    if(line == "help") {
    SerialUSB.println("Available commands:");
    SerialUSB.println("help                        : show this help message");
    SerialUSB.println("us <sensor> one-shot        : trigger a single measurement");
    SerialUSB.println("us <sensor> on <period_ms>  : start periodic measurements every <period_ms> ms");
    SerialUSB.println("us <sensor> off              : stop periodic measurements");
    SerialUSB.println("us <sensor> unit {inc|cm|ms}: change sensor measurement unit");
    SerialUSB.println("us <sensor> delay <ms>       : set minimum delay between measurements");
    SerialUSB.println("us <sensor> status           : get sensor configuration info");
    SerialUSB.println("us                           : show number of available sensors");
    return;
    }

    if(line == "us"){
        send_srf02_command(0, 0x4, 0);
    }

    // proste splitowanie po spacjach
    String parts[4];
    int count = 0;
    int start = 0;
    for(int i=0;i<line.length() && count<4;i++) {
        if(line[i] == ' ') {
            parts[count++] = line.substring(start, i);
            start = i+1;
        }
    }
    if(count<4) parts[count++] = line.substring(start);

    if(parts[0] != "us") return;

    uint8_t sensor = parts[1].toInt(); // sensor 0 lub 1
    String cmd = parts[2];

    if(cmd == "one-shot") send_srf02_command(sensor, 0x0, 0x0);
    else if(cmd == "on") {
        uint8_t period = parts[3].toInt();
        send_srf02_command(sensor, 0x0, 0x1, &period, 1);
    }
    else if(cmd == "off") send_srf02_command(sensor, 0x0, 0x2);
    else if(cmd == "unit") {
        uint8_t unit = 0;
        if(parts[3] == "inc") unit = 0;
        else if(parts[3] == "cm") unit = 1;
        else if(parts[3] == "ms") unit = 2;
        send_srf02_command(sensor, 0x1, unit);
    }
    else if(cmd == "delay") {
        uint8_t delay = parts[3].toInt();
        send_srf02_command(sensor, 0x2, 0, &delay, 1);
    }
    else if(cmd == "status") send_srf02_command(sensor, 0x3, 0);
}



//------------------------------------------------------------------------------
void setup() {
    SerialUSB.begin(115200);
    while(!SerialUSB);
    chBegin(chSetup);
}

void loop() {

}
