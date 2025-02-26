/********************************************
* File Name: accel-LoRa-Pi.ino
* Author   : GioMonci
* Purpose  : get accel data sent to PI w/ LoRa
* Updated  : Feb 2025
* Version  : 1.0
* Comment  : Cooked
*********************************************/

/**Includes**/
#include <Arduino.h>             // mutlithreading
#include <Adafruit_ISM330DHCX.h> // for accelerometer
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

/**Definitions:**/


/**Declarations**/
void accelSetup();
void accelSerial(void *parameter);

/**Objects**/
Adafruit_ISM330DHCX ism330dhcx; // obj for accel
TaskHandle_t Core0; // Task handle for Core 0 (if you want to use it)
TaskHandle_t Core1; // Task handle for Core 1

/**MAIN SETUP**/
void setup() {

  // Serial begins at 115200 however it doesnt start if serial monitor isnt open
  Serial.begin(115200); while (!Serial){ delay(10);}

  //sets up LSM & ISM chip to get accel data
  accelSetup();

  xTaskCreatePinnedToCore(
    accelSerial,   /* Task function. */
    "Serial",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Core1,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  //sets up LSM & ISM chip to get accel data
  //accelSetup();

}

/**MAIN LOOP**/
void loop() {

}

// ========================================================================================

// Begining of Acclerometer Section 

void accelSetup(){
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip"); while (1) { delay(10);}
  }
  // Semi-Default Ranges for Accel
  // Change if they aren't acceptable
  // We'll see after testing
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_208_HZ);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_208_HZ);
  // Configuring Interrupt 1 on ISM chip, not LIS chip
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
}

void accelSerial(void *parameter){
  // sensor events for accel, gyro, temp
  // getEvent needs (gyro & temp) for whatever reason
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  while(true){
    // again dumb asf, need 3 arguments to be passed to work, kms.
    ism330dhcx.getEvent(&accel, &gyro, &temp);
    /**
    // simple print debug, change if needed
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print(accel.acceleration.z);
    Serial.println();
    **/
    String accelData = String(&accel.acceleration.x) + "," +
                       String(&accel.acceleration.y) + "," +
                       String(&accel.acceleration.z);
    Serial.println(data);
    sendLoRaData(data);
    // Add delay for task to yield control
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
// End of Accelerometer Section

// ========================================================================================

// Start of LoRaWan Section

void loRaSetup(){

}

void sendLoRaData(){
  
}