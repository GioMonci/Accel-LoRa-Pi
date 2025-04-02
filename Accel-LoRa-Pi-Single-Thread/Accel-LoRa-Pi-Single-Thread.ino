#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_ISM330DHCX.h> // Include accelerometer library

// LoRa Configuration
#define CFG_us915 1

// OTAA keys – use your own keys here
static const u1_t PROGMEM APPEUI[8]  = { 0x78, 0xDE, 0xFE, 0x95, 0x46, 0x14, 0xA8, 0x68 };
static const u1_t PROGMEM DEVEUI[8]  = { 0x2B, 0x09, 0x17, 0x87, 0x97, 0xC7, 0x3D, 0x9A };
static const u1_t PROGMEM APPKEY[16] = { 0xB8, 0x46, 0x92, 0x72, 0x71, 0xC1, 0x00, 0xBE,
                                        0x94, 0x32, 0xC9, 0xC2, 0xA3, 0xF7, 0x5C, 0x0F };

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping – using your original settings
const lmic_pinmap lmic_pins = {
  .nss  = 33,
  .rxtx = LMIC_UNUSED_PIN,
  .rst  = 15,
  .dio  = {27, 14},  // If only DIO0 is used, leave it as-is.
};

// Create accelerometer object
Adafruit_ISM330DHCX ism330dhcx;

// Declarations
void setupAccelerometer();
void readAccelerometer();

static osjob_t sendjob;
static uint8_t mydata[12]; // 3 floats (x, y, z) * 4 bytes each

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }
  
  // Read accelerometer data into mydata
  readAccelerometer();
  
  // Send the data
  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
  Serial.println(F("Packet queued"));
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      // Schedule next transmission (every 60 seconds)
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
      break;
    default:
      Serial.println(F("Other event"));
      break;
  }
}

void setupAccelerometer() {
  Serial.println("Setting up accelerometer...");
  
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) { delay(10); }
  }
  
  Serial.println("ISM330DHCX Found!");
  
  // Configure accelerometer settings
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_208_HZ);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_208_HZ);
  
  // Configure interrupt if needed
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  
  Serial.println("Accelerometer setup complete");
}

void readAccelerometer() {
  // Create sensor event variables
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  
  // Get current acceleration data
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  
  // Print values to serial console
  Serial.print("Acceleration X: "); Serial.print(accel.acceleration.x);
  Serial.print(" Y: "); Serial.print(accel.acceleration.y);
  Serial.print(" Z: "); Serial.println(accel.acceleration.z);
  
  // Copy acceleration data to mydata array
  memcpy(&mydata[0], &accel.acceleration.x, sizeof(float));
  memcpy(&mydata[4], &accel.acceleration.y, sizeof(float));
  memcpy(&mydata[8], &accel.acceleration.z, sizeof(float));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("LoRa with Accelerometer Example Start"));
  
  // Setup the accelerometer
  setupAccelerometer();
  
  // Initialize LoRa
  os_init();
  LMIC_reset();
  
  // Queue first transmission
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
