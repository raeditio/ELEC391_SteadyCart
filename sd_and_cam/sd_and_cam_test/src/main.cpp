#include <Arduino.h>

// MicroSD Card libraries for ESP32
#include "FS.h"
#include "SD_MMC.h"

// Camera and webserver library for ESP32-CAM
#include "esp_camera.h"
#include <WiFi.h>
#define CAMERA_MODEL_AI_THINKER // Define the camera model
#include "camera_pins.h" // Camera pin configuration
// Define wifi credentials
const char *ssid = "arshDesktop";
const char *password = "arshHotSpot";

// Define pins for Serial2 communication with Arduino Uno
//#define RXD2_PIN 3  // ESP32-CAM Pin GPIO 3 (U2_RXD) connects to Arduino Uno Pin 1 (TX)


// SD Card pin configuration
#ifdef CONFIG_IDF_TARGET_ESP32S3
 // Default pins for ESP-S3
 // Warning: ESP32-S3-WROOM-2 is using most of the default GPIOs (33-37) to interface with on-board OPI flash.
 //   If the SD_MMC is initialized with default pins it will result in rebooting loop - please
 //   reassign the pins elsewhere using the mentioned command `setPins`.
 // Note: ESP32-S3-WROOM-1 does not have GPIO 33 and 34 broken out.
 // Note: if it's ok to use default pins, you do not need to call the setPins
 int clk = 36;
 int cmd = 35;
 int d0 = 37;
 int d1 = 38;
 int d2 = 33;
 int d3 = 39;  // GPIO 34 is not broken-out on ESP32-S3-DevKitC-1 v1.1
 #endif


// SD card functions declerations:
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void configESP_SDCARD_MMC();
// Camera function declerations:
void startCameraServer();
void setupLedFlash(int pin);
void configESP32_Camera();
void startCamera_WebServer(const char *ssid, const char *password);


void setup() {
  Serial.begin(115200);
  configESP32_Camera();
  startCamera_WebServer(ssid, password);
  configESP_SDCARD_MMC();
  delay(5000); // Wait for the camera to initialize and connect to WiFi
  Serial.println("Camera and SD card setup complete.");
  Serial.println("ESP32-CAM Receiver Ready");
  
}

void loop() {
  delay(10);
  if(Serial.available()) { // Check if data is available from the Arduino on Serial2
    String receivedString = Serial.readStringUntil('\n'); // Read the incoming string until a newline character is received
    appendFile(SD_MMC, "/foo.txt", receivedString.c_str()); // Write the received string to a file on the SD card
  }
}

// function definitions:
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
   Serial.printf("Listing directory: %s\n", dirname);
 
   File root = fs.open(dirname);
   if (!root) {
     Serial.println("Failed to open directory");
     return;
   }
   if (!root.isDirectory()) {
     Serial.println("Not a directory");
     return;
   }
 
   File file = root.openNextFile();
   while (file) {
     if (file.isDirectory()) {
       Serial.print("  DIR : ");
       Serial.println(file.name());
       if (levels) {
         listDir(fs, file.path(), levels - 1);
       }
     } else {
       Serial.print("  FILE: ");
       Serial.print(file.name());
       Serial.print("  SIZE: ");
       Serial.println(file.size());
     }
     file = root.openNextFile();
   }
 }
 
 void createDir(fs::FS &fs, const char *path) {
   Serial.printf("Creating Dir: %s\n", path);
   if (fs.mkdir(path)) {
     Serial.println("Dir created");
   } else {
     Serial.println("mkdir failed");
   }
 }
 
 void removeDir(fs::FS &fs, const char *path) {
   Serial.printf("Removing Dir: %s\n", path);
   if (fs.rmdir(path)) {
     Serial.println("Dir removed");
   } else {
     Serial.println("rmdir failed");
   }
 }
 
 void readFile(fs::FS &fs, const char *path) {
   Serial.printf("Reading file: %s\n", path);
 
   File file = fs.open(path);
   if (!file) {
     Serial.println("Failed to open file for reading");
     return;
   }
 
   Serial.print("Read from file: ");
   while (file.available()) {
     Serial.write(file.read());
   }
 }
 
 void writeFile(fs::FS &fs, const char *path, const char *message) {
   Serial.printf("Writing file: %s\n", path);
 
   File file = fs.open(path, FILE_WRITE);
   if (!file) {
     Serial.println("Failed to open file for writing");
     return;
   }
   if (file.print(message)) {
     Serial.println("File written");
   } else {
     Serial.println("Write failed");
   }
 }
 
 void appendFile(fs::FS &fs, const char *path, const char *message) {
   Serial.printf("Appending to file: %s\n", path);
 
   File file = fs.open(path, FILE_APPEND);
   if (!file) {
     Serial.println("Failed to open file for appending");
     return;
   }
   if (file.print(message)) {
     Serial.println("Message appended");
   } else {
     Serial.println("Append failed");
   }
 }
 
 void renameFile(fs::FS &fs, const char *path1, const char *path2) {
   Serial.printf("Renaming file %s to %s\n", path1, path2);
   if (fs.rename(path1, path2)) {
     Serial.println("File renamed");
   } else {
     Serial.println("Rename failed");
   }
 }
 
 void deleteFile(fs::FS &fs, const char *path) {
   Serial.printf("Deleting file: %s\n", path);
   if (fs.remove(path)) {
     Serial.println("File deleted");
   } else {
     Serial.println("Delete failed");
   }
 }
 
 void testFileIO(fs::FS &fs, const char *path) {
   File file = fs.open(path);
   static uint8_t buf[512];
   size_t len = 0;
   uint32_t start = millis();
   uint32_t end = start;
   if (file) {
     len = file.size();
     size_t flen = len;
     start = millis();
     while (len) {
       size_t toRead = len;
       if (toRead > 512) {
         toRead = 512;
       }
       file.read(buf, toRead);
       len -= toRead;
     }
     end = millis() - start;
     Serial.printf("%u bytes read for %lu ms\n", flen, end);
     file.close();
   } else {
     Serial.println("Failed to open file for reading");
   }
 
   file = fs.open(path, FILE_WRITE);
   if (!file) {
     Serial.println("Failed to open file for writing");
     return;
   }
 
   size_t i;
   start = millis();
   for (i = 0; i < 2048; i++) {
     file.write(buf, 512);
   }
   end = millis() - start;
   Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
   file.close();
 }

 void configESP_SDCARD_MMC() {
   if (!SD_MMC.begin()) {
     Serial.println("Card Mount Failed");
     return;
   }
    uint8_t cardType = SD_MMC.cardType();

    if (cardType == CARD_NONE) {
      Serial.println("No SD_MMC card attached");
      return;
    }
    
    Serial.println("Card Mount Successful");

    if (cardType == CARD_MMC) {
      Serial.println("SD_MMC Card Type: MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SD_MMC Card Type: SD");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SD_MMC Card Type: SDHC");
    } else {
      Serial.println("SD_MMC Card Type: Unknown type");
    }
  }

  void configESP32_Camera() {
    // Camera pins configuration (camera model defined in camera_pins.h)
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;  // for streaming
    //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Camera quality and frame size configuration if PSRAM IC present
    if (config.pixel_format == PIXFORMAT_JPEG) {
      if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
      } else {
        // Limit the frame size when PSRAM is not available
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
      }
    } else {
      // Best option for face detection/recognition
      config.frame_size = FRAMESIZE_240X240;
  #if CONFIG_IDF_TARGET_ESP32S3
      config.fb_count = 2;
  #endif
    }

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);        // flip it back
      s->set_brightness(s, 1);   // up the brightness just a bit
      s->set_saturation(s, -2);  // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    if (config.pixel_format == PIXFORMAT_JPEG) {
      s->set_framesize(s, FRAMESIZE_QVGA);
    }

  // Setup LED FLash if LED pin is defined in camera_pins.h
  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif

  Serial.println("Camera setup complete");
}

void startCamera_WebServer(const char *ssid, const char *password) {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}
  

