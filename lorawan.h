#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

//JOIN EUI Little Endian form 0x45,0xba,0xe9,0x69,0x65,0xb7,0xca,0x79
static const u1_t PROGMEM APPEUI[8]={0x45,0xba,0xe9,0x69,0x65,0xb7,0xca,0x79};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// LSB
static const u1_t PROGMEM DEVEUI[8]={0x2d,0x80,0x2e,0x01,0x38,0xd7,0xb1,0x4a};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
//LSB
//static const u1_t PROGMEM APPKEY[16] = {0xeb,0x24,0xd0,0xe8,0x6d,0x6b,0xec,0xf5,0xad,0xc9,0x6f,0x1a,0x26,0xe4,0x69,0xf8};
//MSB
static const u1_t PROGMEM APPKEY[16] = {0xf8,0x69,0xe4,0x26,0x1a,0x6f,0xc9,0xad,0xf5,0xec,0x6b,0x6d,0xe8,0xd0,0x24,0xeb};

void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "WISE";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 33,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 15,
    .dio = {27},
};