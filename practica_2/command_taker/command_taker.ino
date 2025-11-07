#include <ChRt.h>
#include <CAN.h>

#define DEBUG_CAN_TX 1
#define DEBUG_WORK_UPLOADING 1

//------------------------------------------------------------------------------
// CAN frame structure
//------------------------------------------------------------------------------
struct CANFrameStruct {
    uint16_t id;
    uint8_t size;
    uint8_t data[8];
};

//------------------------------------------------------------------------------
// CAN handler thread
//------------------------------------------------------------------------------
static thread_t* can_thread;
static THD_WORKING_AREA(CAN_handler, 512);
static CANFrameStruct frame;

//------------------------------------------------------------------------------
// CAN receive handler thread
//------------------------------------------------------------------------------
#define CAN_BUFFER_SIZE 32
static CANFrameStruct can_buffer[CAN_BUFFER_SIZE];
static volatile int write_ptr = 0;
static volatile int read_ptr = 0;

static THD_FUNCTION(CAN_handler_function, arg) {
    (void)arg;
    chRegSetThreadName("CAN handler");

    while (true) {
        chEvtWaitOne(EVENT_MASK(0));


        while (read_ptr != write_ptr) {
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

            // Obsługa znanych pakietów
            if (frame.id == 0x10 || frame.id == 0x11) {
                uint16_t distance = (frame.data[0] << 8) | frame.data[1];
                uint8_t sensor_id = (frame.id == 0x10) ? 0 : 1;
                SerialUSB.print("[SRF02#");
                SerialUSB.print(sensor_id);
                SerialUSB.print("] Distance: ");
                SerialUSB.println(distance);

            } else if (frame.id == 0x111) {
                uint8_t sensor_addr = frame.data[0];
                uint16_t delay = (frame.data[2] << 8) | frame.data[1];
                uint16_t period = (frame.data[4] << 8) | frame.data[3];
                bool periodic = frame.data[5];
                uint8_t unit = frame.data[6];

                SerialUSB.println("--- SRF02 Status ---");
                SerialUSB.print("Address: 0x"); SerialUSB.println(sensor_addr, HEX);
                SerialUSB.print("Delay: "); SerialUSB.print(delay); SerialUSB.println(" ms");
                SerialUSB.print("Period: "); SerialUSB.print(period); SerialUSB.println(" ms");
                SerialUSB.print("Mode: "); SerialUSB.println(periodic ? "ON" : "OFF");
                SerialUSB.print("Unit: ");
                if (unit == 0) SerialUSB.println("inches");
                else if (unit == 1) SerialUSB.println("cm");
                else if (unit == 2) SerialUSB.println("ms");
                else SerialUSB.println("unknown");
                SerialUSB.println("--------------------");

            } else if (frame.id == 0x112) {
                uint8_t count = frame.data[0];
                SerialUSB.print("Available SRF02 sensors: "); SerialUSB.println(count);

            } else {
                SerialUSB.print("[UNKNOWN CAN ID] 0x"); SerialUSB.println(frame.id, HEX);
            }
        }
    }
}

//------------------------------------------------------------------------------
// CAN interrupt callback
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// Send SRF02 command helper
//------------------------------------------------------------------------------
void send_srf02_command(uint8_t sensor, uint8_t cmd, uint8_t subcmd,
                        uint8_t* args = nullptr, uint8_t arg_size = 0) {
    uint8_t frame[8];
    frame[0] = ((sensor & 0b11) << 6) | ((cmd & 0b1111) << 2) | (subcmd & 0b11);

    for (uint8_t i = 0; i < arg_size && i < 7; i++)
        frame[i + 1] = args[i];

#if DEBUG_CAN_TX
    SerialUSB.print("[CAN TX] ID=0x110 Data: ");
    for (uint8_t i = 0; i < arg_size + 1; i++) {
        if (frame[i] < 0x10) SerialUSB.print("0");
        SerialUSB.print(frame[i], HEX);
        SerialUSB.print(" ");
    }
    SerialUSB.println();
#endif

    CAN.beginPacket(0x110);
    CAN.write(frame, arg_size + 1);
    CAN.endPacket();
}

//------------------------------------------------------------------------------
// Command parser using char buffer
//------------------------------------------------------------------------------
void process_user_command(char* line) {
    // Usuń \r i \n
    int len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
        line[--len] = '\0';
    if (len == 0) return;

    if (strcmp(line, "help") == 0) {
        SerialUSB.println("Available commands:");
        SerialUSB.println("help : show this help message");
        SerialUSB.println("us <sensor> one-shot : trigger a single measurement");
        SerialUSB.println("us <sensor> on <period_ms> : start periodic measurements");
        SerialUSB.println("us <sensor> off : stop periodic measurements");
        SerialUSB.println("us <sensor> unit {inc|cm|ms}: change measurement unit");
        SerialUSB.println("us <sensor> delay <ms> : set min delay between shots");
        SerialUSB.println("us <sensor> status : get sensor configuration");
        SerialUSB.println("us : show number of sensors");
        return;
    }

    if (strcmp(line, "us") == 0) {
        send_srf02_command(0, 0x4, 0);
        return;
    }

    char parts[4][16] = {{0}};
    int count = 0;
    char* token = strtok(line, " ");
    while (token && count < 4) {
        strncpy(parts[count++], token, 15);
        token = strtok(NULL, " ");
    }
    if (count == 0 || strcmp(parts[0], "us") != 0)
        return;

    uint8_t sensor = atoi(parts[1]);
    const char* cmd = parts[2];

    if (strcmp(cmd, "one-shot") == 0) {
        send_srf02_command(sensor, 0x0, 0x0);
    } else if (strcmp(cmd, "on") == 0) {
        uint8_t period = atoi(parts[3]);
        send_srf02_command(sensor, 0x0, 0x1, &period, 1);
    } else if (strcmp(cmd, "off") == 0) {
        send_srf02_command(sensor, 0x0, 0x2);
    } else if (strcmp(cmd, "unit") == 0) {
        uint8_t unit = 0;
        if (strcmp(parts[3], "inc") == 0) unit = 0;
        else if (strcmp(parts[3], "cm") == 0) unit = 1;
        else if (strcmp(parts[3], "ms") == 0) unit = 2;
        send_srf02_command(sensor, 0x1, unit);
    } else if (strcmp(cmd, "delay") == 0) {
        uint8_t delay = atoi(parts[3]);
        send_srf02_command(sensor, 0x2, 0, &delay, 1);
    } else if (strcmp(cmd, "status") == 0) {
        send_srf02_command(sensor, 0x3, 0);
    }
}

//------------------------------------------------------------------------------
// User command handler thread
//------------------------------------------------------------------------------
static THD_WORKING_AREA(UserHandler, 256);
static THD_FUNCTION(UserHandlerFunction, arg) {
    (void)arg;
    chRegSetThreadName("User handler");

    char buf[64];
    uint8_t idx = 0;

    while (true) {
        while (SerialUSB.available()) {
            char c = SerialUSB.read();
            if (c == '\n' || c == '\r') {
                buf[idx] = '\0';
                if (idx > 0)
                    process_user_command(buf);
                idx = 0;
            } else if (idx < sizeof(buf) - 1) {
                buf[idx++] = c;
            }
        }
        chThdSleepMilliseconds(50);
    }
}

#ifdef DEBUG_WORK_UPLOADING
//------------------------------------------------------------------------------
// Debug blink thread
//------------------------------------------------------------------------------
static THD_WORKING_AREA(BlinkDebug, 128);
static THD_FUNCTION(BlinkDebugFunction, arg) {
    (void)arg;
    chRegSetThreadName("Blink debug");

    pinMode(LED_BUILTIN, OUTPUT);
    SerialUSB.println("[DEBUG] Blink thread active (heartbeat mode).");

    while (true) {
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            chThdSleepMilliseconds(10);
            digitalWrite(LED_BUILTIN, LOW);
            chThdSleepMilliseconds(10);
        }
        digitalWrite(LED_BUILTIN, LOW);
        chThdSleepMilliseconds(800);
    }
}
#endif

//------------------------------------------------------------------------------
// ChibiOS setup
//------------------------------------------------------------------------------
void chSetup() {
    SerialUSB.begin(115200);
    uint32_t start = millis();
    while (!SerialUSB && (millis() - start < 5000))
        chThdSleepMilliseconds(100);

    SerialUSB.println("Sensor interface ready. Type 'help' for commands.");

    can_thread = chThdCreateStatic(CAN_handler, sizeof(CAN_handler), NORMALPRIO+1,
                                   CAN_handler_function, NULL);
    chThdCreateStatic(UserHandler, sizeof(UserHandler), NORMALPRIO, UserHandlerFunction, NULL);

    CAN.onReceive(CAN_on_receive);

#ifdef DEBUG_WORK_UPLOADING
    chThdCreateStatic(BlinkDebug, sizeof(BlinkDebug), NORMALPRIO, BlinkDebugFunction, NULL);
#endif
}

//------------------------------------------------------------------------------
void setup() {
    if (!SerialUSB) {
        SerialUSB.begin(9600);
        uint32_t timeout = millis() + 2000;
        while (!SerialUSB && millis() < timeout) {
            delay(1);
        }
    }

    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }

    SerialUSB.begin(115200);
    delay(2000);
    chBegin(chSetup);
}

void loop() {}
