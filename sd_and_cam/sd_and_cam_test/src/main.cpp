#include <Arduino.h>

// Core Libraries
#include "FS.h"                // File System
#include "SD_MMC.h"            // SD Card driver for ESP32 MMC interface
#include <WiFi.h>              // WiFi connectivity
#include "esp_camera.h"        // ESP32 Camera driver

// Configuration
#define CAMERA_MODEL_AI_THINKER // Specify camera model (ensure camera_pins.h matches)
#include "camera_pins.h"       // Pin definitions for the specified camera model

// --- Constants ---
// WiFi Credentials (Consider using WiFiManager or other methods for production)
const char *ssid = "arshDesktop";
const char *password = "arshHotSpot";

// SD Card Logging Configuration
const char* LOG_FILENAME_BASE = "/log_";      // Base name for log files
const char* LOG_FILENAME_EXT = ".csv";        // Extension for log files
const int MAX_LOG_FILES_TO_CHECK = 1000;    // Max attempts to find a unique log filename
const char* LOG_FILE_HEADER = "angle,pwm\n"; // Header row for the CSV log file

// --- Global Variables ---
String logFilename; // Stores the full path of the current log file

// --- Function Declarations ---

// SD Card Utilities
void initializeSdCard();
bool findUniqueLogFilename();
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path); // Function for testing SD card read/write speed

// Camera & Web Server
void initializeCamera();
void startCameraWebServer(const char *wifiSsid, const char *wifiPassword);
// Note: startCameraServer() is part of the ESP camera library examples, ensure it's available.
// If using the standard Arduino ESP32 CameraWebServer example, it's defined there.
void startCameraServer(); 
void setupLedFlash(int pin); // Often part of camera examples if LED flash is used

// --- Main Program ---

void setup() {
  Serial.begin(115200);
  Serial.println("Booting up...");

  initializeCamera();       // Configure and initialize the camera hardware
  initializeSdCard();       // Initialize the SD card interface

  // Only proceed with WiFi and logging if SD card is working
  if (SD_MMC.cardType() != CARD_NONE) {
    startCameraWebServer(ssid, password); // Connect to WiFi and start the web server

    Serial.println("Searching for unique log file name...");
    if (findUniqueLogFilename()) {
      Serial.printf("Using unique log file: %s\n", logFilename.c_str());
      // Write header to the newly created log file
      writeFile(SD_MMC, logFilename.c_str(), LOG_FILE_HEADER);
    } else {
      Serial.printf("Error: Could not find unique log filename within %d checks!\n", MAX_LOG_FILES_TO_CHECK);
      // Handle error appropriately - maybe stop execution or try a default name?
    }
  } else {
    Serial.println("SD Card not found or failed to initialize. Skipping WiFi and logging setup.");
    // Decide how to proceed without SD card/logging if needed
  }

  Serial.println("Setup complete. ESP32-CAM Receiver Ready.");
  // listDir(SD_MMC, "/", 0); // Uncomment for debugging SD card contents
}

void loop() {
  // Check if data has arrived from another device via Serial communication
  if (Serial.available()) {
    // Read the incoming data line by line (assuming newline termination)
    String receivedData = Serial.readStringUntil('\n');

    // Append the received data to the log file, if a valid filename was found
    if (logFilename.length() > 0) {
        // Add newline character as readStringUntil consumes it
        receivedData += "\n"; 
        appendFile(SD_MMC, logFilename.c_str(), receivedData.c_str());
    }
    // Optional small delay if receiving data very rapidly to avoid overwhelming SD write buffer
    // delay(50); 
  }

  // Add any other periodic tasks here
}

// --- Function Definitions ---

/**
 * @brief Initializes the SD_MMC card interface.
 */
void initializeSdCard() {
  Serial.println("Initializing SD card using SD_MMC...");

  // Start the SD_MMC interface (1-bit mode is default)
  // For 4-bit mode, use: if (!SD_MMC.begin("/sdcard", false)) { // false = 4-bit mode
  if (!SD_MMC.begin()) {
    Serial.println("Card Mount Failed. Check connections and card format.");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached.");
    return;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
}

/**
 * @brief Finds an unused log filename like /log_N.csv and stores it in the global logFilename.
 * @return true if a unique name was found, false otherwise.
 */
bool findUniqueLogFilename() {
  for (int i = 0; i < MAX_LOG_FILES_TO_CHECK; i++) {
    String testName = String(LOG_FILENAME_BASE) + i + String(LOG_FILENAME_EXT);
    if (!SD_MMC.exists(testName)) {
      logFilename = testName; // Store the unique name globally
      // Create the file immediately to reserve the name (optional but good practice)
       File file = SD_MMC.open(logFilename, FILE_WRITE);
       if (file) {
           file.close();
           return true; // Successfully found and reserved the name
       } else {
           Serial.printf("Error: Could not create file %s to reserve name.\n", logFilename.c_str());
           logFilename = ""; // Clear filename on error
           return false; // Failed to create the file
       }
    }
  }
  logFilename = ""; // Clear filename if no unique name found
  return false;     // Indicate failure
}


/**
 * @brief Lists directory contents recursively.
 * @param fs File system object (e.g., SD_MMC).
 * @param dirname Directory path to list.
 * @param levels Recursion depth.
 */
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    root.close();
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
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file.close(); // Close the file handle
    file = root.openNextFile();
  }
  root.close(); // Close the directory handle
}

/**
 * @brief Creates a directory.
 * @param fs File system object.
 * @param path Path of the directory to create.
 */
void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

/**
 * @brief Removes a directory.
 * @param fs File system object.
 * @param path Path of the directory to remove.
 */
void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

/**
 * @brief Reads and prints file content to Serial.
 * @param fs File system object.
 * @param path Path of the file to read.
 */
void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("Failed to open file for reading");
    if (file) file.close();
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println(); // Add a newline after printing file content
}

/**
 * @brief Writes (overwrites) content to a file.
 * @param fs File system object.
 * @param path Path of the file to write to.
 * @param message Content to write.
 */
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
  file.close();
}

/**
 * @brief Appends content to the end of a file.
 * @param fs File system object.
 * @param path Path of the file to append to.
 * @param message Content to append.
 */
void appendFile(fs::FS &fs, const char *path, const char *message) {
 // Serial.printf("Appending to file: %s\n", path); // Optional: Can be verbose

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.printf("Failed to open file %s for appending\n", path);
    return;
  }
  if (file.print(message)) {
   // Serial.println("Message appended"); // Optional: Can be verbose
  } else {
    Serial.printf("Append failed to file %s\n", path);
  }
  file.close();
}

/**
 * @brief Renames a file.
 * @param fs File system object.
 * @param path1 Original file path.
 * @param path2 New file path.
 */
void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

/**
 * @brief Deletes a file.
 * @param fs File system object.
 * @param path Path of the file to delete.
 */
void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

/**
 * @brief Tests file I/O speed (useful for debugging/benchmarking).
 * @param fs File system object.
 * @param path Path of the file to use for testing.
 */
void testFileIO(fs::FS &fs, const char *path) {
  Serial.printf("Testing file I/O with %s\n", path);
  static uint8_t buf[512]; // Buffer for read/write operations

  // --- Read Test ---
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading test");
  } else {
    size_t len = file.size();
    size_t flen = len;
    uint32_t start = millis();
    while (len) {
      size_t toRead = (len > sizeof(buf)) ? sizeof(buf) : len;
      file.read(buf, toRead);
      len -= toRead;
    }
    uint32_t end = millis() - start;
    Serial.printf("%u bytes read in %lu ms\n", flen, end);
    file.close();
  }

  // --- Write Test ---
  file = fs.open(path, FILE_WRITE); // Overwrites existing file for test
  if (!file) {
    Serial.println("Failed to open file for writing test");
    return;
  }

  size_t bytesToWrite = 2048 * sizeof(buf); // ~1MB
  Serial.printf("Writing %u bytes...\n", bytesToWrite);
  uint32_t start = millis();
  for (size_t i = 0; i < 2048; i++) {
    if(file.write(buf, sizeof(buf)) != sizeof(buf)){
        Serial.println("Write error during test!");
        break;
    }
  }
  uint32_t end = millis() - start;
  Serial.printf("%u bytes written in %lu ms\n", bytesToWrite, end);
  file.close();
}

/**
 * @brief Configures and initializes the ESP32 camera module.
 */
void initializeCamera() {
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
  config.frame_size = FRAMESIZE_UXGA;     // Highest resolution for stills
  config.pixel_format = PIXFORMAT_JPEG;   // Use JPEG for streaming/saving space
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Grab frames when buffer is free
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM frame buffer if available
  config.jpeg_quality = 12; // Lower quality (0-63 higher) for faster processing, adjust as needed
  config.fb_count = 1;     // Number of frame buffers (1 needed if grabbing one at a time)

  // Adjust settings if PSRAM is available for better performance
  if (psramFound()) {
    Serial.println("PSRAM found, optimizing camera config.");
    config.jpeg_quality = 10; // Can usually use higher quality with PSRAM
    config.fb_count = 2;     // Use two frame buffers for smoother streaming/capture
    config.grab_mode = CAMERA_GRAB_LATEST; // Discard older frames for responsiveness
  } else {
    Serial.println("No PSRAM found, using DRAM and lower resolution.");
    // Limit frame size if no PSRAM
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // Consider halting or indicating error permanently (e.g., blinking LED)
    return;
  }
  Serial.println("Camera initialized successfully.");

  // Get sensor object for settings adjustments
  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("Failed to get camera sensor handle.");
    return;
  }

  // Sensor-specific settings (Example for OV3660)
  if (s->id.PID == OV3660_PID) {
     s->set_vflip(s, 1);       // Flip sensor output vertically if needed
     s->set_brightness(s, 1);  // Adjust brightness (example)
     s->set_saturation(s, -2); // Adjust saturation (example)
  }

  // Set initial frame size for streaming (lower resolution often preferred)
  // Keep UXGA (or SVGA if no PSRAM) as default, change dynamically if needed
   if (config.pixel_format == PIXFORMAT_JPEG) {
      // Example: Start with a smaller size for the web stream initially
      // s->set_framesize(s, FRAMESIZE_QVGA); 
   }


  // Setup LED Flash if available and pin is defined
  #if defined(LED_GPIO_NUM)
    // setupLedFlash(LED_GPIO_NUM); // Assuming this function exists elsewhere
    Serial.printf("LED Flash available on GPIO %d\n", LED_GPIO_NUM);
  #else
    Serial.println("No LED Flash pin defined.");
  #endif

}

/**
 * @brief Connects to WiFi and starts the camera web server.
 * @param wifiSsid The SSID of the WiFi network.
 * @param wifiPassword The password for the WiFi network.
 */
void startCameraWebServer(const char *wifiSsid, const char *wifiPassword) {
  Serial.printf("Connecting to WiFi SSID: %s\n", wifiSsid);
  WiFi.begin(wifiSsid, wifiPassword);
  WiFi.setSleep(false); // Keep WiFi active

  // Wait for connection
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) { // ~15 seconds timeout
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println(""); // Newline after dots

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected successfully.");
    Serial.print("Camera Stream Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
    
    // Start the camera web server (assuming standard ESP example structure)
    startCameraServer(); 
  } else {
    Serial.println("WiFi connection failed.");
    // Handle WiFi failure (e.g., retry, enter configuration mode)
  }
}

// NOTE: Ensure `startCameraServer()` is defined, likely in your main sketch
// or included from the ESP32 camera web server example code.
// If it's not available, this call in startCameraWebServer will fail.
// You might need to copy the necessary http server setup code from the example.
#if 0 // Placeholder for where startCameraServer() might be defined if using example code
#include "app_httpd.cpp" // Common in ESP examples
void startCameraServer(){
  start_webserver(); // Or similar function from app_httpd
}
#endif
  

