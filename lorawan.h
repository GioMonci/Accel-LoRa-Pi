#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include "accelerometer.h"

void do_send(osjob_t* j);
void lora_setup();
void lora_loop();
