/*
 * LISA (Learning Intelligent Servo-balanced Assistant)
 * Layer 2: Master Communications Core (ESP32)
 * 
 * Hardware Layout:
 * - Raspberry Pi 2: RX2 -> GPIO 16, TX2 -> GPIO 17 (Baud Rate: 500000)
 * - MAX98357A Amplifier (I2S TX): BCLK -> GPIO 26, LRC -> GPIO 25, DIN -> GPIO 33
 * - I2S MEMS Microphone (I2S RX): SCK -> GPIO 44, WS -> GPIO 9, DIN -> GPIO 1
 * - Slave Co-Processor: Connected via ESP-NOW Link
 */

#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <ArduinoJson.h>

// --- UART & I2S Pins Configuration ---
#define PI_UART_BAUD    500000
#define PI_UART_RX_PIN  16
#define PI_UART_TX_PIN  17

#define SPK_I2S_PORT    I2S_NUM_0
#define PIN_SPK_BCLK    26
#define PIN_SPK_LRC     25
#define PIN_SPK_DIN     33

#define MIC_I2S_PORT    I2S_NUM_1
#define PIN_MIC_SCK     44
#define PIN_MIC_WS      9
#define PIN_MIC_SD      1

#define SAMPLE_RATE     16000

// --- ESP-NOW Configuration ---
uint8_t slaveAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 

struct __attribute__((packed)) MovementPayload {
  char move[16];
  int speed;
  int track_x; // Horizontal tracking offset percentage (-100 to 100)
};

MovementPayload moveCmd = {"idle", 0, 0};
char uartRxBuffer[1024];
int uartRxIndex = 0;

void initI2SSpeaker();
void initI2SMic();
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup() {
  Serial.begin(115200); // USB Debugging
  
  // High-Speed UART to Pi
  Serial2.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  
  // Initialize Audio
  initI2SSpeaker();
  initI2SMic();

  // Set WiFi to Station mode for ESP-NOW compatibility
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(onDataSent);

  // Register Slave Peer Info
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Slave peer. Update slaveAddress MAC in code.");
  }
  
  Serial.println("LISA Master Core Initialized!");
}

void loop() {
  // 1. Read and process high-speed serial interface from Pi
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // We treat incoming data starting with '{' as JSON Commands.
    if (c == '{' || uartRxIndex > 0) {
      if (uartRxIndex < sizeof(uartRxBuffer) - 1) {
        uartRxBuffer[uartRxIndex++] = c;
        
        // If we reach the end of the JSON object
        if (c == '}') {
          uartRxBuffer[uartRxIndex] = '\0';
          
          // Parse the Movement / Tracking JSON command
          StaticJsonDocument<256> doc;
          DeserializationError error = deserializeJson(doc, uartRxBuffer);
          if (!error) {
            bool updateNeeded = false;
            
            // Check for yaw tracking commands
            if (doc.containsKey("track_x")) {
              moveCmd.track_x = doc["track_x"];
              updateNeeded = true;
            }
            
            // Check for explicit movements
            if (doc.containsKey("move")) {
              strncpy(moveCmd.move, doc["move"] | "idle", sizeof(moveCmd.move));
              moveCmd.speed = doc["speed"] | 90;
              // Reset tracking offset during walk commands unless it's static
              if (strcmp(moveCmd.move, "walk") == 0) {
                moveCmd.track_x = 0;
              }
              updateNeeded = true;
            }

            if (updateNeeded) {
              // Broadcast straight to Slave ESP32-C3 over ESP-NOW
              esp_now_send(slaveAddress, (uint8_t *) &moveCmd, sizeof(moveCmd));
              Serial.printf("Sent ESP-NOW Telemetry. Move: %s, Speed: %d, Track X: %d\n", 
                            moveCmd.move, moveCmd.speed, moveCmd.track_x);
            }
          }
          uartRxIndex = 0; // Reset Buffer
        }
      } else {
        uartRxIndex = 0; // Buffer Overflow reset
      }
    } else {
      // Treat other incoming streaming data as raw PCM audio chunks.
      size_t bytesWritten = 0;
      i2s_write(SPK_I2S_PORT, &c, 1, &bytesWritten, portMAX_DELAY);
    }
  }

  // 2. Capture microphone raw data and stream back to Pi over UART
  int16_t micBuffer[64];
  size_t bytesRead = 0;
  i2s_read(MIC_I2S_PORT, (void*)micBuffer, sizeof(micBuffer), &bytesRead, 0); // Non-blocking
  if (bytesRead > 0) {
    Serial2.write((uint8_t*)micBuffer, bytesRead);
  }
}

void initI2SSpeaker() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_SPK_BCLK,
    .ws_io_num = PIN_SPK_LRC,
    .data_out_num = PIN_SPK_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(SPK_I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(SPK_I2S_PORT, &pin_config);
}

void initI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_MIC_SCK,
    .ws_io_num = PIN_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_MIC_SD
  };

  i2s_driver_install(MIC_I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(MIC_I2S_PORT, &pin_config);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW Telemetry transmission failed!");
  }
}
