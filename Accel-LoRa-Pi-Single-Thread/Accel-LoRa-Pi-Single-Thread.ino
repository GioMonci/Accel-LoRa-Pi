#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_ISM330DHCX.h>
#include <math.h>  // For standard deviation calculations
#include <cfloat>

// LoRa Configuration
#define CFG_us915 1

// OTAA keys – use your own keys here
static const u1_t PROGMEM APPEUI[8] = { 0x78, 0xDE, 0xFE, 0x95, 0x46, 0x14, 0xA8, 0x68 };
static const u1_t PROGMEM DEVEUI[8] = { 0x2B, 0x09, 0x17, 0x87, 0x97, 0xC7, 0x3D, 0x9A };
static const u1_t PROGMEM APPKEY[16] = { 0xB8, 0x46, 0x92, 0x72, 0x71, 0xC1, 0x00, 0xBE, 0x94, 0x32, 0xC9, 0xC2, 0xA3, 0xF7, 0x5C, 0x0F };

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 33,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 15,
  .dio = {27, 14}, // If only DIO0 is used, leave it as-is.
};

// Create accelerometer object
Adafruit_ISM330DHCX ism330dhcx;

// Data collection variables
struct AccelData {
  float x;
  float y;
  float z;
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
  float x_std;
  float y_std;
  float z_std;
};

AccelData accelData;

// Variables for statistics calculation
struct StatsCollector {
  float x_sum;
  float y_sum;
  float z_sum;
  float x_sum_sq;  // Sum of squares for standard deviation
  float y_sum_sq;
  float z_sum_sq;
  int sample_count;
} stats;

// Timer for sampling
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 10;  // Sample at 100Hz

// Timer for data transmission
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL_MS = 1000;  // Send every 1 second

// Declarations
void setupAccelerometer();
void sampleAccelerometer();
void calculateStatistics();
void resetStatistics();
static void do_send(osjob_t* j);

// Job for LMIC scheduler
static osjob_t sendjob;

// Data buffer - expanded to hold all metrics
// 12 floats × 4 bytes = 48 bytes
static uint8_t mydata[48];

void resetStatistics() {
  // Initialize min values to maximum possible float
  accelData.x_min = accelData.y_min = accelData.z_min = FLT_MAX;
  // Initialize max values to minimum possible float
  accelData.x_max = accelData.y_max = accelData.z_max = -FLT_MAX;
  // Reset sums and counters for standard deviation
  stats.x_sum = stats.y_sum = stats.z_sum = 0;
  stats.x_sum_sq = stats.y_sum_sq = stats.z_sum_sq = 0;
  stats.sample_count = 0;
}

void setupAccelerometer() {
  Serial.println("Setting up accelerometer...");
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ISM330DHCX Found!");
  
  // Configure accelerometer settings for higher frequency sampling
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);  
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_416_HZ); 
  
  // Configure interrupt if needed
  ism330dhcx.configInt1(false, false, true);  // accelerometer DRDY on INT1
  Serial.println("Accelerometer setup complete");
  
  // Initialize statistics
  resetStatistics();
}

void sampleAccelerometer() {
  // Create sensor event variables
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  
  // Get current acceleration data
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  
  // Store current values
  accelData.x = accel.acceleration.x;
  accelData.y = accel.acceleration.y;
  accelData.z = accel.acceleration.z;
  
  // Update min/max values
  if (accel.acceleration.x < accelData.x_min) accelData.x_min = accel.acceleration.x;
  if (accel.acceleration.y < accelData.y_min) accelData.y_min = accel.acceleration.y;
  if (accel.acceleration.z < accelData.z_min) accelData.z_min = accel.acceleration.z;
  
  if (accel.acceleration.x > accelData.x_max) accelData.x_max = accel.acceleration.x;
  if (accel.acceleration.y > accelData.y_max) accelData.y_max = accel.acceleration.y;
  if (accel.acceleration.z > accelData.z_max) accelData.z_max = accel.acceleration.z;
  
  // Collect data for standard deviation calculation
  stats.x_sum += accel.acceleration.x;
  stats.y_sum += accel.acceleration.y;
  stats.z_sum += accel.acceleration.z;
  
  stats.x_sum_sq += accel.acceleration.x * accel.acceleration.x;
  stats.y_sum_sq += accel.acceleration.y * accel.acceleration.y;
  stats.z_sum_sq += accel.acceleration.z * accel.acceleration.z;
  
  stats.sample_count++;
}

void calculateStatistics() {
  if (stats.sample_count > 1) {
    // Calculate means
    float x_mean = stats.x_sum / stats.sample_count;
    float y_mean = stats.y_sum / stats.sample_count;
    float z_mean = stats.z_sum / stats.sample_count;
    
    // Calculate standard deviations
    accelData.x_std = sqrt((stats.x_sum_sq / stats.sample_count) - (x_mean * x_mean));
    accelData.y_std = sqrt((stats.y_sum_sq / stats.sample_count) - (y_mean * y_mean));
    accelData.z_std = sqrt((stats.z_sum_sq / stats.sample_count) - (z_mean * z_mean));
  } else {
    // Not enough samples for standard deviation
    accelData.x_std = accelData.y_std = accelData.z_std = 0;
  }
}

void prepareDataPacket() {
  // Calculate final statistics
  calculateStatistics();
  
  // Copy all data to the packet
  int offset = 0;
  
  // Current acceleration values
  memcpy(&mydata[offset], &accelData.x, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.y, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.z, sizeof(float)); offset += sizeof(float);
  
  // Min values
  memcpy(&mydata[offset], &accelData.x_min, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.y_min, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.z_min, sizeof(float)); offset += sizeof(float);
  
  // Max values
  memcpy(&mydata[offset], &accelData.x_max, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.y_max, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.z_max, sizeof(float)); offset += sizeof(float);
  
  // Standard deviation values
  memcpy(&mydata[offset], &accelData.x_std, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.y_std, sizeof(float)); offset += sizeof(float);
  memcpy(&mydata[offset], &accelData.z_std, sizeof(float)); offset += sizeof(float);
  
  // Print values to serial console for debugging
  Serial.println("=== Data Packet Ready ===");
  Serial.print("Accel XYZ: "); 
  Serial.print(accelData.x); Serial.print(", ");
  Serial.print(accelData.y); Serial.print(", ");
  Serial.println(accelData.z);
  
  Serial.print("Min XYZ: ");
  Serial.print(accelData.x_min); Serial.print(", ");
  Serial.print(accelData.y_min); Serial.print(", ");
  Serial.println(accelData.z_min);
  
  Serial.print("Max XYZ: ");
  Serial.print(accelData.x_max); Serial.print(", ");
  Serial.print(accelData.y_max); Serial.print(", ");
  Serial.println(accelData.z_max);
  
  Serial.print("StdDev XYZ: ");
  Serial.print(accelData.x_std); Serial.print(", ");
  Serial.print(accelData.y_std); Serial.print(", ");
  Serial.println(accelData.z_std);
  
  Serial.print("Samples: ");
  Serial.println(stats.sample_count);
  Serial.println("========================");
}

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }
  
  // Prepare data packet with current statistics
  prepareDataPacket();
  
  // Send the data
  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
  Serial.println(F("Packet queued"));
  
  // Reset statistics for next collection period
  resetStatistics();
  
  // Update last transmit time
  lastTransmitTime = millis();
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
      break;
    default:
      Serial.println(F("Other event"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(F("LoRa with Enhanced Accelerometer Data Start"));
  
  // Setup the accelerometer
  setupAccelerometer();
  
  // Initialize LoRa
  os_init();
  LMIC_reset();
  
  // Queue first transmission
  do_send(&sendjob);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Sample acceleration at regular intervals
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    sampleAccelerometer();
    lastSampleTime = currentTime;
  }
  
  // Send data every TRANSMIT_INTERVAL_MS
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL_MS) {
    // Don't schedule directly, only if no transmission is pending
    if (!(LMIC.opmode & OP_TXRXPEND)) {
      do_send(&sendjob);
    }
  }
  
  // Run the LMIC scheduler
  os_runloop_once();
}
