#include <Arduino_MKRMEM.h>
#include "debugging_keywords.h"

#define END_FILENAME "end.txt"
#define CONFIG_FILENAME "config.txt"

Arduino_W25Q16DV flash(SPI1, FLASH_CS);

bool resetFile();

bool initFileSystem(){
    flash.begin();
  
  // Mount the filesystem
  DEBUG_PRINTLN("Checking filesystem...");
  if (!filesystem.mounted()) {
    DEBUG_PRINTLN("Mounting filesystem...");
    int res = filesystem.mount();

    if (res != SPIFFS_OK && res != SPIFFS_ERR_NOT_A_FS) {
      ERR_PRINT("mount() failed with error code: ");
      ERR_PRINTLN_WOUT(res);
      filesystem.unmount();
      return false;
    }

    if(res == SPIFFS_ERR_NOT_A_FS){
      res = filesystem.format();
      if (res != SPIFFS_OK){
        ERR_PRINT("format() failed with error code: ");
        ERR_PRINTLN_WOUT(res);
        filesystem.unmount();
        return false;
      }

      SerialUSB.println("Mounting filesystem...");
      res = filesystem.mount();

      if (res != SPIFFS_OK) {
        ERR_PRINT("mount() failed with error code: ");
        ERR_PRINTLN_WOUT(res);
        filesystem.unmount();
        return false;
      }
    }
  }
  File file = filesystem.open(CONFIG_FILENAME, READ_ONLY);
  if(!file){
      return resetFile();
  }
  file.close();
  return true;    
}



bool appendToConfig(char* buff, uint8_t size){
    File file = filesystem.open(CONFIG_FILENAME, WRITE_ONLY | APPEND);
    if (!file) {
      ERR_PRINTLN("Opening file failed for appending. Aborting...");
      filesystem.unmount();
      return false;
    }
    const uint8_t bytes_written = file.write((void *)buff, size);
    if(bytes_written != size){
        ERR_PRINT("write() failed with error code ");
        ERR_PRINTLN_WOUT(filesystem.err());
        file.close();
        filesystem.unmount();
        return false;
    }
    file.close();
    return true;
}

bool resetFile(){
    File file = filesystem.open(CONFIG_FILENAME, CREATE | TRUNCATE);
    if (!file) {
      ERR_PRINTLN("Reseting file failed. Aborting...");
      filesystem.unmount();
      return false;
    }
    file.close();
    return true;
}
