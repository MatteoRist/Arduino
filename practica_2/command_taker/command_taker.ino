//------------------------------------------
// Subject: Internet de las Cosas (IC)
// Practica: 2
// Authors: Kacper Klasen, Matteo Ristoro
//-------------------------------------------

// This file implements a concurrent CAN (Controller Area Network) bus handler
// and a serial command interface, designed for an embedded system running ChibiOS/RT.
// It manages communication with SRF02 ultrasonic sensors over CAN.

#include <ChRt.h>
#include <CAN.h>

#define DEBUG_CAN_TX 1
#define DEBUG_WORK_UPLOADING 1

// CAN frame structure definition
struct CANFrameStruct
{
    uint16_t id;
    uint8_t size;
    uint8_t data[8];
};

// CAN handler thread setup and static variables
static thread_t *can_thread;
static THD_WORKING_AREA(CAN_handler, 512);
static CANFrameStruct frame;

// CAN receive circular buffer and pointer management
#define CAN_BUFFER_SIZE 32
static CANFrameStruct can_buffer[CAN_BUFFER_SIZE];
static volatile int write_ptr = 0;
static volatile int read_ptr = 0;

// CAN handler thread function (consumer)
// This thread waits for new CAN events, processes the frames from the circular
// buffer, and prints the results to the serial console based on the CAN ID.
static THD_FUNCTION(CAN_handler_function, arg)
{
    (void)arg;
    chRegSetThreadName("CAN handler");

    while (true)
    {
        // Block and wait for a signal from the CAN receive interrupt.
        chEvtWaitOne(EVENT_MASK(0));

        // Process all frames currently in the circular buffer (read_ptr != write_ptr).
        while (read_ptr != write_ptr)
        {
            // Read and advance the read pointer safely.
            CANFrameStruct frame = can_buffer[read_ptr];
            read_ptr = (read_ptr + 1) & (CAN_BUFFER_SIZE - 1);

#if DEBUG_CAN_TX
            // Debug output of received CAN frames.
            SerialUSB.print("[CAN RX] ID=0x");
            SerialUSB.print(frame.id, HEX);
            SerialUSB.print(" DATA: ");
            for (int i = 0; i < frame.size; i++)
            {
                if (frame.data[i] < 0x10)
                    SerialUSB.print("0");
                SerialUSB.print(frame.data[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
#endif

            // Handle known packets, specifically distance readings (0x10, 0x11)
            // and sensor status/count packets (0x111, 0x112).
            if (frame.id == 0x10 || frame.id == 0x11)
            {
                // Parse 16-bit distance from the first two data bytes (MSB first).
                uint16_t distance = (frame.data[0] << 8) | frame.data[1];
                uint8_t sensor_id = (frame.id == 0x10) ? 0 : 1;
                SerialUSB.print("[SRF02#");
                SerialUSB.print(sensor_id);
                SerialUSB.print("] Distance: ");
                SerialUSB.println(distance);
            }
            else if (frame.id == 0x111)
            {
                // Parse SRF02 sensor status details (address, delay, period, mode, unit).
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
                if (unit == 0)
                    SerialUSB.println("inches");
                else if (unit == 1)
                    SerialUSB.println("cm");
                else if (unit == 2)
                    SerialUSB.println("ms");
                else
                    SerialUSB.println("unknown");
                SerialUSB.println("--------------------");
            }
            else if (frame.id == 0x112)
            {
                // Parse the count of available SRF02 sensors.
                uint8_t count = frame.data[0];
                SerialUSB.print("Available SRF02 sensors: ");
                SerialUSB.println(count);
            }
            else
            {
                // Log unknown CAN IDs.
                SerialUSB.print("[UNKNOWN CAN ID] 0x");
                SerialUSB.println(frame.id, HEX);
            }
        }
    }
}

// CAN interrupt callback (producer)
// This function is executed in the Interrupt Service Routine (ISR) context upon
// receiving a new CAN packet. It safely pushes the frame to the circular buffer
// and signals the CAN handler thread.
void CAN_on_receive(int packetSize)
{
    (void)packetSize;

    // Check if the circular buffer is full before writing.
    int next = (write_ptr + 1) & (CAN_BUFFER_SIZE - 1);
    if (next != read_ptr)
    {
        // Enter a ChibiOS critical section for ISRs to ensure thread safety.
        chSysLockFromISR();
        // Store packet details and data.
        can_buffer[write_ptr].id = CAN.packetId();
        can_buffer[write_ptr].size = CAN.available();
        for (uint8_t i = 0; i < can_buffer[write_ptr].size; i++)
            can_buffer[write_ptr].data[i] = CAN.read();
        // Advance write pointer.
        write_ptr = next;
        // Signal the CAN handler thread to wake up and process the new frame.
        chEvtSignalI(can_thread, EVENT_MASK(0));
        chSysUnlockFromISR();
    }
}

// Helper function to format and send SRF02 commands over CAN (ID 0x110)
// This function packs the sensor ID, command, and sub-command into the first
// byte of the CAN frame using bitwise manipulation, followed by optional arguments.
void send_srf02_command(uint8_t sensor, uint8_t cmd, uint8_t subcmd,
                        uint8_t *args = nullptr, uint8_t arg_size = 0)
{
    uint8_t frame[8];
    // Pack control byte: [sensor: 2 bits] [cmd: 4 bits] [subcmd: 2 bits]
    frame[0] = ((sensor & 0b11) << 6) | ((cmd & 0b1111) << 2) | (subcmd & 0b11);

    // Copy arguments if provided.
    for (uint8_t i = 0; i < arg_size && i < 7; i++)
        frame[i + 1] = args[i];

#if DEBUG_CAN_TX
    // Debug output of outgoing CAN frames.
    SerialUSB.print("[CAN TX] ID=0x110 Data: ");
    for (uint8_t i = 0; i < arg_size + 1; i++)
    {
        if (frame[i] < 0x10)
            SerialUSB.print("0");
        SerialUSB.print(frame[i], HEX);
        SerialUSB.print(" ");
    }
    SerialUSB.println();
#endif

    // Transmit the formatted CAN packet.
    CAN.beginPacket(0x110);
    CAN.write(frame, arg_size + 1);
    CAN.endPacket();
}

// Command parser function
// This function tokenizes the user's serial input and translates commands
// (e.g., 'us 0 one-shot') into the appropriate CAN command helper calls.
void process_user_command(char *line)
{
    // Trim line endings.
    int len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
        line[--len] = '\0';
    if (len == 0)
        return;

    // Handle 'help' and simple 'us' command to check sensor count.
    if (strcmp(line, "help") == 0)
    {
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

    if (strcmp(line, "us") == 0)
    {
        send_srf02_command(0, 0x4, 0);
        return;
    }

    // Tokenize the command string into parts.
    char parts[4][16] = {{0}};
    int count = 0;
    char *token = strtok(line, " ");
    while (token && count < 4)
    {
        strncpy(parts[count++], token, 15);
        token = strtok(NULL, " ");
    }
    if (count == 0 || strcmp(parts[0], "us") != 0)
        return;

    // Determine the sensor ID and command type, then call the CAN transmit helper.
    uint8_t sensor = atoi(parts[1]);
    const char *cmd = parts[2];

    if (strcmp(cmd, "one-shot") == 0)
    {
        send_srf02_command(sensor, 0x0, 0x0);
    }
    else if (strcmp(cmd, "on") == 0)
    {
        uint8_t period = atoi(parts[3]);
        send_srf02_command(sensor, 0x0, 0x1, &period, 1);
    }
    else if (strcmp(cmd, "off") == 0)
    {
        send_srf02_command(sensor, 0x0, 0x2);
    }
    else if (strcmp(cmd, "unit") == 0)
    {
        uint8_t unit = 0;
        if (strcmp(parts[3], "inc") == 0)
            unit = 0;
        else if (strcmp(parts[3], "cm") == 0)
            unit = 1;
        else if (strcmp(parts[3], "ms") == 0)
            unit = 2;
        send_srf02_command(sensor, 0x1, unit);
    }
    else if (strcmp(cmd, "delay") == 0)
    {
        uint8_t delay = atoi(parts[3]);
        send_srf02_command(sensor, 0x2, 0, &delay, 1);
    }
    else if (strcmp(cmd, "status") == 0)
    {
        send_srf02_command(sensor, 0x3, 0);
    }
}

// User command handler thread
// This thread polls the USB serial port for user input, buffers incoming
// characters, and calls the command parser when a line ending is detected.
static THD_WORKING_AREA(UserHandler, 256);
static THD_FUNCTION(UserHandlerFunction, arg)
{
    (void)arg;
    chRegSetThreadName("User handler");

    char buf[64];
    uint8_t idx = 0;

    while (true)
    {
        // Read available serial data and buffer the command.
        while (SerialUSB.available())
        {
            char c = SerialUSB.read();
            if (c == '\n' || c == '\r')
            {
                buf[idx] = '\0';
                if (idx > 0)
                    process_user_command(buf);
                idx = 0;
            }
            else if (idx < sizeof(buf) - 1)
            {
                buf[idx++] = c;
            }
        }
        // Yield execution to other threads.
        chThdSleepMilliseconds(50);
    }
}

#ifdef DEBUG_WORK_UPLOADING
// Debug blink thread (Heartbeat)
// This thread toggles the built-in LED in a fast blink pattern followed by a
// long pause, serving as a visual indicator that the RTOS kernel is active.
static THD_WORKING_AREA(BlinkDebug, 128);
static THD_FUNCTION(BlinkDebugFunction, arg)
{
    (void)arg;
    chRegSetThreadName("Blink debug");

    pinMode(LED_BUILTIN, OUTPUT);
    SerialUSB.println("[DEBUG] Blink thread active (heartbeat mode).");

    while (true)
    {
        for (int i = 0; i < 10; i++)
        {
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

// ChibiOS kernel initialization
// This function sets up the serial connection, creates and starts the CAN
// handler and user handler threads, and registers the CAN receive callback.
void chSetup()
{
    SerialUSB.begin(115200);
    uint32_t start = millis();
    // Wait for serial connection to open.
    while (!SerialUSB && (millis() - start < 5000))
        chThdSleepMilliseconds(100);

    SerialUSB.println("Sensor interface ready. Type 'help' for commands.");

    // Create the high-priority CAN handler thread.
    can_thread = chThdCreateStatic(CAN_handler, sizeof(CAN_handler), NORMALPRIO + 1,
                                   CAN_handler_function, NULL);
    // Create the normal-priority user input thread.
    chThdCreateStatic(UserHandler, sizeof(UserHandler), NORMALPRIO, UserHandlerFunction, NULL);

    // Register the function that will be called on CAN packet reception (ISR).
    CAN.onReceive(CAN_on_receive);

#ifdef DEBUG_WORK_UPLOADING
    // Create the debug heartbeat thread.
    chThdCreateStatic(BlinkDebug, sizeof(BlinkDebug), NORMALPRIO, BlinkDebugFunction, NULL);
#endif
}

// Arduino setup and loop wrappers
// This function initializes the CAN bus hardware and starts the ChibiOS kernel.
// The loop function is deliberately empty as all tasks run as RTOS threads.
void setup()
{
    if (!SerialUSB)
    {
        SerialUSB.begin(9600);
        uint32_t timeout = millis() + 2000;
        while (!SerialUSB && millis() < timeout)
        {
            delay(1);
        }
    }

    // Start the CAN bus at 500 kbps.
    if (!CAN.begin(500E3))
    {
        Serial.println("Starting CAN failed!");
        while (1)
            ; // Halt on failure
    }

    SerialUSB.begin(115200);
    delay(2000);
    // Transfer control to the ChibiOS kernel.
    chBegin(chSetup);
}

void loop() {}