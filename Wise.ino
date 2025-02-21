/********************************************
* File Name: Wise.ino
* Author   : GioMonci
* Purpose  : Main sketch file for WISE project
* Updated  : 
* Version  : 1.0
* Comment  : Todo - add 
*********************************************/

// Includes
#include "accelerometer.h"
#include "lorawan.h"

TaskHandle_t accelTaskHandle;
TaskHandle_t loraTaskHandle;

// Accelerometer Task
void accelTask(void *pvParameters) {
    while (1) {
        accelerometer_loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Adjust delay as needed
    }
}

// LoRaWAN Task
void loraTask(void *pvParameters) {
    while (1) {
        lora_loop();
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Adjust delay as needed
    }
}

void setup() {
    
    accelerometer_Setup();
    lora_setup();

    // Create tasks
    xTaskCreatePinnedToCore(accelTask, "AccelTask", 4096, NULL, 1, &accelTaskHandle, 0);
    xTaskCreatePinnedToCore(loraTask, "LoraTask", 4096, NULL, 1, &loraTaskHandle, 1);
}

void loop() {
    vTaskDelete(NULL);  // Delete the default loop task (not needed)
}