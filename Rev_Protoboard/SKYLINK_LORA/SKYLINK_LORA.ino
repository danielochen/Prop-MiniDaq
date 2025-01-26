#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include <map> 

// LoRa Parameters for North America
#define FREQUENCY           905.2
#define BANDWIDTH           250.0
#define SPREADING_FACTOR    9
#define TRANSMIT_POWER      1
#define PAUSE               300    // Transmission pause in seconds

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               300

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    12

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      22


// Global variables
String rxData;
volatile bool rxFlag = false;
long counter = 0;
std::map<int, StaticJsonDocument<512>> canDataMap;
SemaphoreHandle_t loraMutex;

// CAN TX/RX Pins
#define CAN_RX 34
#define CAN_TX 33

// Function prototypes
void canReceiveTask(void *pvParameters);
void loraSendTask(void *pvParameters);
void loraReceiveTask(void *pvParameters);
void rxCallback();

// Task handles
TaskHandle_t canReceiveTaskHandle;
TaskHandle_t loraSendTaskHandle;
TaskHandle_t loraReceiveTaskHandle;

void setup() {
  Serial.begin(921600);
  heltec_setup();  
  both.println("SkyLink init");

  // LoRa settings
  RADIOLIB_OR_HALT(radio.begin());
  radio.setFrequency(FREQUENCY);
  radio.setBandwidth(BANDWIDTH);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  radio.setOutputPower(TRANSMIT_POWER);
  radio.setDio1Action(rxCallback);
  radio.startReceive();

  // Initialize the mutex
  loraMutex = xSemaphoreCreateMutex();

  // CAN (TWAI) Configuration
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // Tasks for CAN and LoRa
  xTaskCreatePinnedToCore(canReceiveTask, "CAN Receive Task", 4096, NULL, 2, &canReceiveTaskHandle, 1);
  xTaskCreatePinnedToCore(loraSendTask, "LoRa Send Task", 4096, NULL, 1, &loraSendTaskHandle, 0);
  xTaskCreatePinnedToCore(loraReceiveTask, "LoRa Receive Task", 4096, NULL, 1, &loraReceiveTaskHandle, 0);
}

// CAN Receive Task
void canReceiveTask(void *pvParameters) {
  while (1) {
    twai_message_t rxMessage;
    if (twai_receive(&rxMessage, pdMS_TO_TICKS(50)) == ESP_OK) {
      StaticJsonDocument<512> sensorData;
      sensorData["ID"] = rxMessage.identifier;
      for (int i = 0; i < rxMessage.data_length_code; i++) {
        sensorData["Data"][i] = rxMessage.data[i];
      }
      canDataMap[rxMessage.identifier] = sensorData;
    }
  }
}

// LoRa Send Task
void loraSendTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {  // Take mutex for safe access
      StaticJsonDocument<512> messageToSend;
      JsonArray dataArray = messageToSend.createNestedArray("Messages");
      for (const auto &entry : canDataMap) {
        dataArray.add(entry.second);
      }
      
      String output;
      serializeJson(messageToSend, output);
      heltec_led(50);
      String message = "SKYLINK: " + output + " :END\n";
      radio.transmit(message.c_str());
      heltec_led(0);
      Serial.printf("LoRa Sent: %s\n", output.c_str());

      canDataMap.clear();
      radio.standby();
      radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
      xSemaphoreGive(loraMutex);  // Release mutex
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}
void loraReceiveTask(void *pvParameters) {
  while (1) {
    if (rxFlag) {
      rxFlag = false;

      if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {  // Take mutex for safe access
        vTaskDelay(pdMS_TO_TICKS(10));
        int status = radio.readData(rxData);  // Capture read status
        xSemaphoreGive(loraMutex);  // Release mutex

        if (status == RADIOLIB_ERR_NONE) {  // Check for read success
          rxData.trim();
          if (rxData.startsWith("GROUNDLINK:") && rxData.endsWith(":END")) {
            String innerData = rxData.substring(11, rxData.length() - 4);  // Extract content
            Serial.printf("Verified RX: %s\n", innerData.c_str());
          } else {
            Serial.printf("Unverified RX format: %s\n", rxData.c_str());
          }
        } else {
          Serial.printf("LoRa read error: %d\n", status);  // Handle read error
        }
      }

      if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {  // Take mutex for safe access
        radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
        xSemaphoreGive(loraMutex);  // Release mutex
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Callback for LoRa reception
void rxCallback() {
  rxFlag = true;
}

void loop() {
  heltec_loop();
}
