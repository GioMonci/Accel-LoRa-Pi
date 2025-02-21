/********************************************
* File Name: Wise.ino
* Author   : GioMonci
* Purpose  : Main sketch file for WISE project
* Updated  : 
* Version  : 1.0
* Comment  : Todo - add 
*********************************************/

// Includes
#include <Adafruit_gps.h>
#include "accelerometer.h"
#include "lorawan.h"


void setup(void) {
  accelerometer_Setup();
  lora_setup();

}

void loop() {
  accelerometer_loop();
  lora_loop();
}