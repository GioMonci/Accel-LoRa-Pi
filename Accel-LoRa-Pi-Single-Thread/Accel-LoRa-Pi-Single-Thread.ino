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

// Joey Tensorflow
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "autoencoder_model.h"


#include "main_functions.h"
#include "constants.h"
#include "output_handler.h"


/**Definitions:**/

/**LoRa Definitions:**/
#define CFG_us915 1  // For USA


static const u1_t PROGMEM APPEUI[8]= {};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]= {};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = {};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


/**Declarations**/
void accelSetup();
void accelSerial();
void loRaSetup();
void sendLoRaData();
void printHex2(unsigned v);
void onEvent (ev_t ev);
void do_send(osjob_t* j);

/**Objects**/
Adafruit_ISM330DHCX ism330dhcx; // obj for accel

/** Globals **/
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 60;

/**LoRa Payload **/
static uint8_t mydata[12];
static osjob_t sendjob;

/**Pin Mapping**/
const lmic_pinmap lmic_pins = {
  .nss = 33,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 15,
  .dio = {27},
};

// Globals for TensorFlow Lite
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 16 * 1024;  // Increase memory for autoencoder
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

/**MAIN SETUP**/
void setup() {

  // Serial begins at 115200 however it doesnt start if serial monitor isnt open
  Serial.begin(115200); while (!Serial){ delay(10);}
  Serial.println(F("Starting"));

  loRaSetup();
  accelSetup();
}

/**MAIN LOOP**/
void loop() {
  sendLoRaData();
  accelSerial();
  tfliteSetup();
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

void accelSerial(){
  // sensor events for accel, gyro, temp
  // getEvent needs (gyro & temp) for whatever reason
  Serial.println("Accel Started");
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  unsigned long startTime = millis();

  while(millis() - startTime < 10000){
    // again dumb asf, need 3 arguments to be passed to work, kms.
    ism330dhcx.getEvent(&accel, &gyro, &temp);
    // Convert float data to bytes
    delay(10);
    memcpy(&mydata[0], &accel.acceleration.x, sizeof(float));
    memcpy(&mydata[4], &accel.acceleration.y, sizeof(float));
    memcpy(&mydata[8], &accel.acceleration.z, sizeof(float));

    //Serial.print("X: "); Serial.print(accel.acceleration.x);
    //Serial.print("Y: "); Serial.print(accel.acceleration.y);
    //Serial.print("Z: "); Serial.println(accel.acceleration.z);
  }
}
// End of Accelerometer Section

// ========================================================================================

// Start of LoRaWan Section

void loRaSetup(){
  os_init();
  LMIC_reset();
}

// Initialize TensorFlow Lite
void tfliteSetup(){
  Serial.begin(115200);
    Serial.println("ESP32 Autoencoder Inference");

    // Initialize TensorFlow Lite Error Reporter
    static tflite::MicroErrorReporter micro_error_reporter;
    error_reporter = &micro_error_reporter;

    // Load Autoencoder Model
    model = tflite::GetModel(autoencoder_model_INT8_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(error_reporter,
                             "Model provided is schema version %d not equal "
                             "to supported version %d.",
                             model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // Setup TensorFlow Lite Interpreter
    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter = &static_interpreter;

    // Allocate Tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
        return;
    }

    // Get input and output tensors
    input = interpreter->input(0);
    output = interpreter->output(0);
}

void sendLoRaData(){
   Serial.print("mydata: ");
    for (int i = 0; i < sizeof(mydata); i++) {
      Serial.print("0x");
      Serial.print(mydata[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    do_send(&sendjob);
    os_runloop_once();
    delay(20000);
}
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
  }
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        //Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void tfLoop(){
   Serial.println("Running Autoencoder Inference...");

    float sample_input[100];  // Autoencoder expects 100 values

    // Copy data into model input tensor
    memcpy(input->data.f, sample_input, sizeof(sample_input));

    // Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        Serial.println("Model inference failed!");
        return;
    }

    // Print the Denoised Output
    Serial.print("Denoised Output: ");
    for (int i = 0; i < 100; i++) {
        Serial.print(output->data.f[i]);
        Serial.print(", ");
    }
    Serial.println();

    delay(5000);  // Run inference every 5 seconds
}