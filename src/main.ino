/* ------------------------------------------------------------------------------
 * - This program demonstrates how to use SPIFFS on the memory chip.
 * - Running this program is necessary to use a SPIFFS (SPI Flash File System) 
 *   on the flash memory.
 *
 * - Uses the MKRMEM library
 *   https://github.com/arduino-libraries/Arduino_MKRMEM
 * - But it's necessary to comment the flash variable declaration as in the
 *   file src/Arduino_WQ16DV.cpp of the MKRMEM library (lines 193 - 199)
 * -------------------------------------------------------------------------------
 */

#include <Arduino_MKRMEM.h>
#include <time.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>

// RTC object
RTCZero rtc;

# define DEBUG true
// IMPORTANT: Pay attention to the volatile nature of these variables
volatile uint32_t _period_sec = 0;
volatile uint16_t _rtcFlag = 0;
const int _externalPin = 5;
volatile uint16_t _externalFlag = 0;

// Useful macro to measure elapsed time in milliseconds
#define elapsedMilliseconds(since_ms) (uint32_t)(millis() - since_ms)

// -------------------------------------------------------------------------------
// IMPORTANT: We move the flash variable declaration here to adjust
//            the SPI bus and CS pin of the FLASH
// -------------------------------------------------------------------------------
Arduino_W25Q16DV flash(SPI1, FLASH_CS);
char filename[] = "data.txt";

// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
void setup() {
  // Remember that LORA_RESET is defined in
  // .arduino15/packages/arduino/hardware/samd/1.8.13/variants/mkrwan1300/variant.h
  pinMode(LORA_RESET, OUTPUT);    // Declare LORA reset pin as output
  digitalWrite(LORA_RESET, LOW);  // Set it to low level to disable the LoRA module

  // setting pin mode and callback
  pinMode(_externalPin, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(_externalPin, externalCallback, FALLING);

  SerialUSB.begin(9600);
  while (!SerialUSB) {
    ; // Wait for serial connection
  }

  // Initialize FLASH memory
  flash.begin();
  
  // Mount the filesystem
  SerialUSB.println("Mounting filesystem...");
  int res = filesystem.mount();
  if (res != SPIFFS_OK && res != SPIFFS_ERR_NOT_A_FS) {
    SerialUSB.println("mount() failed with error code: ");
    SerialUSB.println(res);
    on_exit_with_error_do();
  }

  // Create a new file
  // We could use create(), but open() provides more flexibility (flags)
  File file = filesystem.open(filename, CREATE | TRUNCATE);
  if (!file) {
    SerialUSB.print("Creation of file ");
    SerialUSB.print(filename);
    SerialUSB.println(" failed. Aborting...");
    on_exit_with_error_do();
  }
  file.close();
  SerialUSB.println("File created successfully");

  // Record start time
  uint32_t t_start_ms = millis();
  
  // Activate LED control pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  SerialUSB.print("Program started at: ");
  SerialUSB.print(__DATE__);
  SerialUSB.print(" ");
  SerialUSB.println(__TIME__);

  // Enable RTC usage
  rtc.begin();
  
  // Analyze the two strings to extract date and time and set the RTC
  if (!setDateTime(__DATE__, __TIME__)) {
    SerialUSB.println("setDateTime() failed!\nExiting...");
    while (1) { 
      ; // Infinite loop on failure
    }
  }

  
  
  // Turn off the LED, but wait until at least 3 seconds have passed since start
  while (elapsedMilliseconds(t_start_ms) > 3000) { 
    delay(1); 
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  // Clear _rtcFlag
  _rtcFlag = 0;
  
  // Activate alarm every 10 seconds starting from 5 seconds from now
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmCallback, CHANGE);
  setPeriodicAlarm(10,5);

  LowPower.sleep();
}

// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
void loop() {
  if (_rtcFlag) {
    writeDateToFile();
    // Read and print file contents to monitor
    printFileContents();

    // Decrement _rtcFlag
    _rtcFlag=0;
    
    // Turn off the LED
    digitalWrite(LED_BUILTIN, LOW);
  }

  if(_externalFlag){
    writeDateToFile();
    printFileContents();
    _externalFlag = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }
  if(!DEBUG){
  LowPower.sleep();
  }
}

// -------------------------------------------------------------------------------
// Read and print file contents to serial monitor debug function
// -------------------------------------------------------------------------------
void printFileContents() {
  if(DEBUG){
    SerialUSB.println("--- File Contents ---");
    
    File file = filesystem.open(filename, READ_ONLY);
    if (!file) {
      SerialUSB.println("Failed to open file for reading");
      return;
    }

    // Read and print file contents using read() instead of available()
    int bytesRead;
    char buffer[64];
    while ((bytesRead = file.read(buffer, sizeof(buffer) - 1)) > 0) {
      buffer[bytesRead] = '\0'; // Null-terminate the string
      SerialUSB.print(buffer);
    }

    file.close();
    SerialUSB.println("\n--- End of File ---");
  }
}


void writeDateToFile() {
  // Get current date and time as string
  char *data_line = getDateTime();
  
  // Reopen the file in each iteration to append data
  File file = filesystem.open(filename, WRITE_ONLY | APPEND);
  if (!file) {
    SerialUSB.print("Opening file ");
    SerialUSB.print(filename);
    SerialUSB.println(" failed for appending. Aborting...");
    on_exit_with_error_do();
  }
  // Write data to file
  int const bytes_to_write = strlen(data_line);
  int const bytes_written = file.write((void *)data_line, bytes_to_write);
  if (bytes_to_write != bytes_written) {
    SerialUSB.print("write() failed with error code ");
    SerialUSB.println(filesystem.err());
    SerialUSB.println("Aborting...");
    on_exit_with_error_do();
  }
  
  // Add newline after each entry for better readability
  file.write((void *)"\n", 1);
  
  // Close the file
  file.close();
}

// -------------------------------------------------------------------------------
// Returns current date and time as a formatted string
// -------------------------------------------------------------------------------
char* getDateTime() {
  const char *weekDay[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  
  // Get Epoch time, seconds since January 1, 1970
  time_t epoch = rtc.getEpoch();
  
  // Convert to usual date and time format
  struct tm stm;
  gmtime_r(&epoch, &stm);
  
  // Generate date and time string
  static char dateTime[32]; // Static to persist after function returns
  snprintf(dateTime, sizeof(dateTime), "%s %4u/%02u/%02u %02u:%02u:%02u",
           weekDay[stm.tm_wday], 
           stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday, 
           stm.tm_hour, stm.tm_min, stm.tm_sec);

  return dateTime;
}

// -------------------------------------------------------------------------------
// Sets date and time from two strings with the format of __DATE__ and __TIME__
// -------------------------------------------------------------------------------
bool setDateTime(const char *date_str, const char *time_str) {
  char month_str[4];
  char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  uint16_t i, mday, month, hour, min, sec, year;
  
  // Parse date string (format: "MMM DD YYYY")
  if (sscanf(date_str, "%3s %hu %hu", month_str, &mday, &year) != 3) {
    return false;
  }
  
  // Parse time string (format: "HH:MM:SS")
  if (sscanf(time_str, "%hu:%hu:%hu", &hour, &min, &sec) != 3) {
    return false;
  }
  
  // Convert month abbreviation to month number
  for (i = 0; i < 12; i++) {
    if (!strncmp(month_str, months[i], 3)) {
      month = i + 1;
      break;
    }
  }
  if (i == 12) {
    return false; // Invalid month
  }
  
  // Set RTC time and date
  rtc.setTime((uint8_t)hour, (uint8_t)min, (uint8_t)sec);
  rtc.setDate((uint8_t)mday, (uint8_t)month, (uint8_t)(year - 2000));
  
  return true;
}

// -------------------------------------------------------------------------------
// Programs the RTC alarm to activate in period_sec seconds from "offset" 
// seconds from the current instant
// -------------------------------------------------------------------------------
void setPeriodicAlarm(uint32_t period_sec, uint32_t offsetFromNow_sec) {
  _period_sec = period_sec;
  rtc.setAlarmEpoch(rtc.getEpoch() + offsetFromNow_sec);
  // See enum Alarm_Match in RTCZero.h
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

// -------------------------------------------------------------------------------
// Interrupt service routine associated with the interrupt caused by the 
// alarm expiration
// -------------------------------------------------------------------------------
void alarmCallback() {
  // Increment the _rtcFlag variable
  _rtcFlag++;
  
  // Turn on the LED
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Reprogram the alarm using the same period
  rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
}

void externalCallback(){
  _externalFlag++;

  digitalWrite(LED_BUILTIN, HIGH);
}

// -------------------------------------------------------------------------------
// Utility to unmount the filesystem and terminate in case of error
// -------------------------------------------------------------------------------
void on_exit_with_error_do() {
  filesystem.unmount();
  exit(EXIT_FAILURE);
}