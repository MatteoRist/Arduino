#ifndef DEBUGGING_KEYWORDS
#define DEBUGGING_KEYWORDS

#define SERIAL_PRINTING_LEVEL 5
// #define RECEIVE_DEBUG_PRINT_ON

#if SERIAL_PRINTING_LEVEL >= 5
    #define DEBUG_PRINT(...) do { Serial.print("[DEBUG] "); Serial.print(__VA_ARGS__); } while(0)
    #define DEBUG_PRINTLN(...) do { Serial.print("[DEBUG] "); Serial.println(__VA_ARGS__); } while(0)
    #define DEBUG_PRINT_WOUT(...) Serial.print(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...) 
    #define DEBUG_PRINTLN(...) 
    #define DEBUG_PRINT_WOUT(...)
#endif

#if SERIAL_PRINTING_LEVEL >= 4
    #define LOG_PRINT(...) do { Serial.print("[LOG] "); Serial.print(__VA_ARGS__); } while(0)
    #define LOG_PRINTLN(...) do { Serial.print("[LOG] "); Serial.println(__VA_ARGS__); } while(0)
    #define LOG_PRINT_WOUT(...) Serial.print(__VA_ARGS__)
#else
    #define LOG_PRINT(...) 
    #define LOG_PRINTLN(...) 
    #define LOG_PRINT_WOUT(...)
#endif

#if SERIAL_PRINTING_LEVEL >= 3
    #define WARN_PRINT(...) do { Serial.print("[WARN] "); Serial.print(__VA_ARGS__); } while(0)
    #define WARN_PRINTLN(...) do { Serial.print("[WARN] "); Serial.println(__VA_ARGS__); } while(0)
#else
    #define WARN_PRINT(...) 
    #define WARN_PRINTLN(...) 
#endif

#if SERIAL_PRINTING_LEVEL >= 2
    #define ERR_PRINT(...) do { Serial.print("[ERROR] "); Serial.print(__VA_ARGS__); } while(0)
    #define ERR_PRINTLN(...) do { Serial.print("[ERROR] "); Serial.println(__VA_ARGS__); } while(0)
    #define ERR_PRINTLN_WOUT(...) do { Serial.println(__VA_ARGS__); } while(0)
#else
    #define ERR_PRINT(...) 
    #define ERR_PRINTLN(...) 
    #define ERR_PRINTLN_WOUT(...)
#endif

#endif
