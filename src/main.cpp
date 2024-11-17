#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <HTTPClient.h>
#include "ssid.h"

const char* ssid = SSID;
const char* password = SSID_PASSWORD;

// WebSocket server details
const char* websocket_server = "172.20.6.231";
const int websocket_port = 8000;
const char* websocket_path = "/ws/audio";

// WebSocket client object
WebSocketsClient webSocket;

#define I2S_NUM         I2S_NUM_0
#define I2S_BCLK        14
#define I2S_LRCLK       15
#define I2S_DIN         32
#define I2S_SAMPLE_RATE 16000
#define CHUNK_SIZE      512
#define BUFFER_SIZE     (CHUNK_SIZE * sizeof(int16_t))

const int blueLedPin = 2;

int16_t i2s_data[CHUNK_SIZE];

// Function prototypes
void send_audio_data(int16_t* data);
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);

void setup() {
  Serial.begin(115200);
  pinMode(blueLedPin, OUTPUT);
  digitalWrite(blueLedPin, LOW);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up WebSocket connection
  webSocket.begin(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = CHUNK_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_start(I2S_NUM);
}

void loop() {
  size_t bytes_read;
  esp_err_t err = i2s_read(I2S_NUM, (void*)i2s_data, BUFFER_SIZE, &bytes_read, portMAX_DELAY);

  if (err == ESP_OK && bytes_read == BUFFER_SIZE) {
    send_audio_data(i2s_data);
  } else {
    Serial.println("Error reading I2S data");
  }

  // WebSocket event loop
  webSocket.loop();
}

void send_audio_data(int16_t* data) {
  if (webSocket.isConnected()) {
    webSocket.sendBIN((uint8_t*)data, BUFFER_SIZE);
  } else {
    Serial.println("WebSocket not connected");
  }
}

// Task handle for the blink task
TaskHandle_t blinkTaskHandle = NULL;

// Function to turn the LED ON
void turnLedOn() {
  Serial.println("Turning LED ON");
  digitalWrite(blueLedPin, HIGH);
}

// Function to turn the LED OFF
void turnLedOff() {
  Serial.println("Turning LED OFF");
  digitalWrite(blueLedPin, LOW);
}

// Blink task function
void blinkTask(void *parameter) {
  while (true) {
    digitalWrite(blueLedPin, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS); // 200ms off
    digitalWrite(blueLedPin, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS); // 200ms on
  }
}

// Function to start blinking
void startBlinking() {
  if (blinkTaskHandle == NULL) { // Check if blink task is already running
    Serial.println("Starting blink task");
    xTaskCreate(blinkTask, "BlinkTask", 1024, NULL, 1, &blinkTaskHandle);
  }
}

// Function to stop blinking
void stopBlinking() {
  if (blinkTaskHandle != NULL) {
    Serial.println("Stopping blink task");
    vTaskDelete(blinkTaskHandle);
    blinkTaskHandle = NULL;
    turnLedOff(); // Ensure LED is turned off after stopping blink
  }
}

// WebSocket event listener
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      stopBlinking(); // Stop blinking if disconnected
      break;

    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;

    case WStype_TEXT:
      if (payload != nullptr && length > 0) {
        String message = String((char *)payload).substring(0, length);
        Serial.print("Received message: ");
        Serial.println(message);

        // Handle LED control commands
        if (message == "LED_ON") {
          stopBlinking();
          turnLedOn();
        } else if (message == "LED_OFF") {
          stopBlinking();
          turnLedOff();
        } else if (message == "LED_BLINK") {
          startBlinking();
        }
      }
      break;

    case WStype_BIN:
      Serial.println("Binary message received");
      break;

    default:
      Serial.println("Unknown WebSocket event");
      break;
  }
}
