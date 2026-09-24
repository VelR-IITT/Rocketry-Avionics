/*
  Teensy 4.1 + u-blox NEO-7M : UBX configuration + NAV-PVT parser

  Sequence
    1. Find the module's current baud rate (probe with a UBX-MON-VER poll)
    2. Optionally load factory defaults (UBX-CFG-CFG)
    3. Switch module UART1 to TARGET_BAUD (115200)
    4. Disable all NMEA sentences
    5. Enable UBX-NAV-PVT on UART1
    6. Set measurement rate to 10 Hz
    7. Optional: airborne <4g dynamic model, save config to flash/BBR
    8. Parse NAV-PVT continuously, print a summary on USB serial at 1 Hz

  Wiring (Serial7)
    Teensy pin 28 (RX7)  <-  GPS TX
    Teensy pin 29 (TX7)  ->  GPS RX
    GND                  <->  GND
*/

#include <Arduino.h>

// ============================================================ user settings
#define GPS_SERIAL Serial7                    // change to Serial1 etc. if rewired

static const uint32_t TARGET_BAUD     = 115200;   // 10 Hz NAV-PVT needs > 9600
static const uint32_t PRINT_PERIOD_MS = 1000;     // USB report interval

static const bool LOAD_DEFAULTS_FIRST = true;     // UBX-CFG-CFG clear+load defaults
static const bool AIRBORNE_MODE       = true;     // UBX-CFG-NAV5 dynModel 8 (<4g)
static const bool SAVE_CONFIG         = false;    // UBX-CFG-CFG save (flash wear!)

// ============================================================ UBX constants
enum : uint8_t {
  UBX_CLS_NAV = 0x01, UBX_CLS_ACK = 0x05, UBX_CLS_CFG = 0x06, UBX_CLS_MON = 0x0A,
  UBX_NAV_PVT = 0x07,
  UBX_ACK_NAK = 0x00, UBX_ACK_ACK = 0x01,
  UBX_CFG_PRT = 0x00, UBX_CFG_MSG = 0x01, UBX_CFG_RATE = 0x08,
  UBX_CFG_CFG = 0x09, UBX_CFG_NAV5 = 0x24,
  UBX_MON_VER = 0x04
};

// Payloads only - header, length and checksum are generated in sendUBX()

// UBX-CFG-CFG: clearMask FFFF, saveMask 0, loadMask FFFF, devices BBR+Flash
static const uint8_t PL_LOAD_DEFAULTS[] = {
  0xFF, 0xFF, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0xFF, 0xFF, 0x00, 0x00,   0x03 };

// UBX-CFG-CFG: save all to BBR + Flash + EEPROM + SPI flash
static const uint8_t PL_SAVE_CONFIG[] = {
  0x00, 0x00, 0x00, 0x00,   0xFF, 0xFF, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x17 };

// UBX-CFG-RATE: measRate 100 ms, navRate 1, timeRef GPS
static const uint8_t PL_RATE_10HZ[] = { 0x64, 0x00,  0x01, 0x00,  0x01, 0x00 };

// UBX-CFG-MSG: NAV-PVT, rate 1 on UART1 only  (order: DDC, UART1, UART2, USB, SPI, res)
static const uint8_t PL_ENABLE_PVT[] = { 0x01, 0x07,  0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };

// UBX-CFG-NAV5: dynModel 8 (airborne <4g), fixMode auto 2D/3D
static const uint8_t PL_NAV5_AIRBORNE[] = {
  0xFF, 0xFF, 0x08, 0x03,   0x00, 0x00, 0x00, 0x00,   0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00,   0xFA, 0x00, 0x64, 0x00,   0x2C, 0x01, 0x00, 0x3C,
  0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00 };

// Standard NMEA sentences (class 0xF0) to switch off on all ports
struct NmeaMsg { uint8_t id; const char *name; };
static const NmeaMsg NMEA_MSGS[] = {
  {0x00, "GGA"}, {0x01, "GLL"}, {0x02, "GSA"},
  {0x03, "GSV"}, {0x04, "RMC"}, {0x05, "VTG"} };

// ============================================================ parser state
static uint8_t  rxState = 0;
static uint8_t  rxCls, rxId, ckA, ckB;
static uint16_t rxLen, rxIdx;
static uint8_t  rxBuf[128];

static uint32_t framesOk  = 0;
static uint32_t framesBad = 0;

static uint8_t  ackCls = 0, ackId = 0;
static uint8_t  ackResult = 0;            // 0 waiting, 1 ACK, 2 NAK

static uint32_t gpsBaud = 9600;

// ============================================================ NAV-PVT data
struct Pvt {
  uint32_t iTOW;                          // ms
  uint16_t year;
  uint8_t  month, day, hour, minute, second;
  uint8_t  valid;                         // bit0 date, bit1 time, bit2 fully resolved
  uint32_t tAcc;                          // ns
  int32_t  nano;                          // ns
  uint8_t  fixType;
  uint8_t  flags;                         // bit0 gnssFixOK, bit1 diffSoln
  uint8_t  numSV;
  int32_t  lon, lat;                      // 1e-7 deg
  int32_t  height, hMSL;                  // mm
  uint32_t hAcc, vAcc;                    // mm
  int32_t  velN, velE, velD, gSpeed;      // mm/s
  int32_t  headMot;                       // 1e-5 deg
  uint32_t sAcc;                          // mm/s
  uint32_t headAcc;                       // 1e-5 deg
  uint16_t pDOP;                          // 0.01
};

static Pvt      pvt;
static bool     havePvt      = false;
static uint32_t pvtWindow    = 0;         // frames since last report
static uint32_t lastPvtMs    = 0;
static uint32_t lastReportMs = 0;

// ============================================================ helpers
static inline uint16_t rdU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rdU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rdI32(const uint8_t *p) { return (int32_t)rdU32(p); }

static inline void ckAdd(uint8_t b) { ckA += b; ckB += ckA; }

static int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

// Print an integer scaled by 10^decimals without using float printf
static void printScaled(int64_t v, uint8_t decimals) {
  if (v < 0) { Serial.print('-'); v = -v; }
  int64_t div = 1;
  for (uint8_t i = 0; i < decimals; i++) div *= 10;
  char buf[24];
  snprintf(buf, sizeof(buf), "%ld", (long)(v / div));
  Serial.print(buf);
  if (decimals) {
    snprintf(buf, sizeof(buf), ".%0*ld", (int)decimals, (long)(v % div));
    Serial.print(buf);
  }
}

// ============================================================ UBX transmit
static void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t frame[8 + 64];
  if (len > 64) return;
  frame[0] = 0xB5;
  frame[1] = 0x62;
  frame[2] = cls;
  frame[3] = id;
  frame[4] = len & 0xFF;
  frame[5] = len >> 8;
  if (len) memcpy(&frame[6], payload, len);

  uint8_t a = 0, b = 0;                   // 8-bit Fletcher over class..payload
  for (uint16_t i = 2; i < 6u + len; i++) { a += frame[i]; b += a; }
  frame[6 + len] = a;
  frame[7 + len] = b;

  GPS_SERIAL.write(frame, 8u + len);
  GPS_SERIAL.flush();
}

// ============================================================ UBX receive
static void decodePvt(const uint8_t *p) {
  pvt.iTOW    = rdU32(p + 0);
  pvt.year    = rdU16(p + 4);
  pvt.month   = p[6];
  pvt.day     = p[7];
  pvt.hour    = p[8];
  pvt.minute  = p[9];
  pvt.second  = p[10];
  pvt.valid   = p[11];
  pvt.tAcc    = rdU32(p + 12);
  pvt.nano    = rdI32(p + 16);
  pvt.fixType = p[20];
  pvt.flags   = p[21];
  pvt.numSV   = p[23];
  pvt.lon     = rdI32(p + 24);
  pvt.lat     = rdI32(p + 28);
  pvt.height  = rdI32(p + 32);
  pvt.hMSL    = rdI32(p + 36);
  pvt.hAcc    = rdU32(p + 40);
  pvt.vAcc    = rdU32(p + 44);
  pvt.velN    = rdI32(p + 48);
  pvt.velE    = rdI32(p + 52);
  pvt.velD    = rdI32(p + 56);
  pvt.gSpeed  = rdI32(p + 60);
  pvt.headMot = rdI32(p + 64);
  pvt.sAcc    = rdU32(p + 68);
  pvt.headAcc = rdU32(p + 72);
  pvt.pDOP    = rdU16(p + 76);

  havePvt   = true;
  lastPvtMs = millis();
  pvtWindow++;
}

static void onUbxFrame() {
  framesOk++;
  // NEO-7M (protocol 14) sends an 84-byte NAV-PVT, M8 and later send 92 bytes.
  // All fields used here sit in the first 84 bytes, so accept either.
  if (rxCls == UBX_CLS_NAV && rxId == UBX_NAV_PVT && rxLen >= 84 && rxLen <= sizeof(rxBuf)) {
    decodePvt(rxBuf);
  } else if (rxCls == UBX_CLS_ACK && rxLen == 2 && rxBuf[0] == ackCls && rxBuf[1] == ackId) {
    ackResult = (rxId == UBX_ACK_ACK) ? 1 : 2;
  }
}

static void ubxFeed(uint8_t b) {
  switch (rxState) {
    case 0: if (b == 0xB5) rxState = 1; break;
    case 1: if (b == 0x62) rxState = 2; else if (b != 0xB5) rxState = 0; break;
    case 2: rxCls = b; ckA = 0; ckB = 0; ckAdd(b); rxState = 3; break;
    case 3: rxId = b; ckAdd(b); rxState = 4; break;
    case 4: rxLen = b; ckAdd(b); rxState = 5; break;
    case 5:
      rxLen |= (uint16_t)b << 8;
      ckAdd(b);
      rxIdx = 0;
      if (rxLen > 512) rxState = 0;              // implausible length, resync
      else rxState = rxLen ? 6 : 7;
      break;
    case 6:
      if (rxIdx < sizeof(rxBuf)) rxBuf[rxIdx] = b;
      rxIdx++;
      ckAdd(b);
      if (rxIdx >= rxLen) rxState = 7;
      break;
    case 7:
      if (b == ckA) rxState = 8;
      else { framesBad++; rxState = 0; }
      break;
    case 8:
      if (b == ckB) onUbxFrame();
      else framesBad++;
      rxState = 0;
      break;
  }
}

static inline void pumpGps() {
  while (GPS_SERIAL.available()) ubxFeed((uint8_t)GPS_SERIAL.read());
}

// ============================================================ setup helpers
// Send a config message and wait for UBX-ACK
static bool sendAcked(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len, const char *label) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    ackCls = cls; ackId = id; ackResult = 0;
    sendUBX(cls, id, pl, len);
    uint32_t t0 = millis();
    while (millis() - t0 < 500 && ackResult == 0) pumpGps();
    if (ackResult == 1) { Serial.printf("  [ OK  ] %s\n", label); return true; }
    if (ackResult == 2) { Serial.printf("  [ NAK ] %s\n", label); return false; }
  }
  Serial.printf("  [ TIMEOUT ] %s\n", label);
  return false;
}

// Open GPS_SERIAL at `baud` and see whether any valid UBX frame comes back
static bool probeBaud(uint32_t baud) {
  GPS_SERIAL.begin(baud);
  delay(30);
  while (GPS_SERIAL.available()) GPS_SERIAL.read();
  rxState = 0;

  uint32_t before = framesOk;
  sendUBX(UBX_CLS_MON, UBX_MON_VER, nullptr, 0);         // poll: module answers if baud matches

  uint32_t timeout = (baud <= 19200) ? 1000 : 400;       // NMEA bursts delay the reply at low baud
  uint32_t t0 = millis();
  while (millis() - t0 < timeout) {
    pumpGps();
    if (framesOk != before) return true;
  }
  return false;
}

static bool detectBaud() {
  static const uint32_t candidates[] = { TARGET_BAUD, 9600, 115200, 38400, 57600, 19200, 4800, 230400 };
  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    if (i > 0 && candidates[i] == TARGET_BAUD) continue;
    Serial.printf("  probing %lu baud ... ", (unsigned long)candidates[i]);
    if (probeBaud(candidates[i])) {
      Serial.println("found");
      gpsBaud = candidates[i];
      return true;
    }
    Serial.println("no reply");
  }
  return false;
}

static void waitForModule() {
  while (!detectBaud()) {
    Serial.println("No UBX reply on any baud rate. Check TX/RX crossover, GND and power. Retrying...");
    delay(1000);
  }
  Serial.printf("Module found at %lu baud\n", (unsigned long)gpsBaud);
}

// UBX-CFG-PRT for UART1 (8N1, UBX+NMEA+RTCM in, UBX+NMEA out), then follow with the Teensy side
static bool switchBaud(uint32_t newBaud) {
  uint8_t pl[20] = {0};
  pl[0]  = 0x01;                                   // portID = UART1
  pl[4]  = 0xD0; pl[5] = 0x08;                     // mode = 0x000008D0 (8N1)
  pl[8]  = (uint8_t)(newBaud);
  pl[9]  = (uint8_t)(newBaud >> 8);
  pl[10] = (uint8_t)(newBaud >> 16);
  pl[11] = (uint8_t)(newBaud >> 24);
  pl[12] = 0x07;                                   // inProtoMask  : UBX | NMEA | RTCM
  pl[14] = 0x03;                                   // outProtoMask : UBX | NMEA

  uint32_t oldBaud = gpsBaud;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    sendUBX(UBX_CLS_CFG, UBX_CFG_PRT, pl, sizeof(pl));
    delay(100);
    if (probeBaud(newBaud)) { gpsBaud = newBaud; return true; }
    probeBaud(oldBaud);                            // module did not switch, go back and retry
  }
  return false;
}

// ============================================================ setup
void setup() {
  Serial.begin(115200);                            // USB, baud value is ignored
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}       // give the terminal time to connect

  static uint8_t gpsRxMem[2048];
  static uint8_t gpsTxMem[512];
  GPS_SERIAL.addMemoryForRead(gpsRxMem, sizeof(gpsRxMem));
  GPS_SERIAL.addMemoryForWrite(gpsTxMem, sizeof(gpsTxMem));

  Serial.println();
  Serial.println("=== NEO-7M configuration ===");
  waitForModule();

  if (LOAD_DEFAULTS_FIRST) {
    Serial.println("Loading factory defaults (module will fall back to 9600 baud)");
    sendUBX(UBX_CLS_CFG, UBX_CFG_CFG, PL_LOAD_DEFAULTS, sizeof(PL_LOAD_DEFAULTS));
    delay(1000);
    waitForModule();
  }

  if (gpsBaud != TARGET_BAUD) {
    Serial.printf("Switching module to %lu baud\n", (unsigned long)TARGET_BAUD);
    while (!switchBaud(TARGET_BAUD)) {
      Serial.println("  baud switch failed, retrying...");
      waitForModule();
    }
  }
  Serial.printf("  [ OK  ] link at %lu baud\n", (unsigned long)gpsBaud);

  Serial.println("Disabling NMEA");
  for (size_t i = 0; i < sizeof(NMEA_MSGS) / sizeof(NMEA_MSGS[0]); i++) {
    uint8_t pl[8] = { 0xF0, NMEA_MSGS[i].id, 0, 0, 0, 0, 0, 0 };   // rate 0 on every port
    char label[24];
    snprintf(label, sizeof(label), "NMEA %s off", NMEA_MSGS[i].name);
    sendAcked(UBX_CLS_CFG, UBX_CFG_MSG, pl, sizeof(pl), label);
  }

  Serial.println("Enabling output");
  sendAcked(UBX_CLS_CFG, UBX_CFG_MSG, PL_ENABLE_PVT, sizeof(PL_ENABLE_PVT), "UBX-NAV-PVT on UART1");
  sendAcked(UBX_CLS_CFG, UBX_CFG_RATE, PL_RATE_10HZ, sizeof(PL_RATE_10HZ), "Rate 10 Hz");

  if (AIRBORNE_MODE)
    sendAcked(UBX_CLS_CFG, UBX_CFG_NAV5, PL_NAV5_AIRBORNE, sizeof(PL_NAV5_AIRBORNE), "Dynamic model: airborne <4g");

  if (SAVE_CONFIG)
    sendAcked(UBX_CLS_CFG, UBX_CFG_CFG, PL_SAVE_CONFIG, sizeof(PL_SAVE_CONFIG), "Save config");

  Serial.println("Configuration done, parsing NAV-PVT...");
  lastReportMs = millis();
  pvtWindow = 0;
}

// ============================================================ report
static const char *FIX_NAMES[] = { "No fix", "Dead reckoning", "2D", "3D", "GNSS + dead reckoning", "Time only" };

static void printReport(uint32_t now) {
  uint32_t dt = now - lastReportMs;
  lastReportMs = now;
  uint32_t rate10 = dt ? (pvtWindow * 10000UL) / dt : 0;      // in 0.1 Hz units
  pvtWindow = 0;

  Serial.println();
  Serial.println("=============== UBX-NAV-PVT ===============");

  if (!havePvt) {
    Serial.printf("Waiting for NAV-PVT... (valid UBX frames: %lu, bad: %lu)\n",
                  (unsigned long)framesOk, (unsigned long)framesBad);
    return;
  }
  if (now - lastPvtMs > 2000) Serial.println("WARNING: no NAV-PVT for over 2 s, showing last data");

  Pvt p = pvt;                                                // snapshot

  Serial.printf("Time (UTC)  : %04u-%02u-%02u %02u:%02u:%02u.%03lu   [date:%c time:%c resolved:%c]\n",
                p.year, p.month, p.day, p.hour, p.minute, p.second,
                (unsigned long)(p.iTOW % 1000),
                (p.valid & 1) ? 'Y' : 'N', (p.valid & 2) ? 'Y' : 'N', (p.valid & 4) ? 'Y' : 'N');

  Serial.printf("Fix         : %s  (fixOK:%c  DGPS:%c)   Sats: %u   pDOP: ",
                (p.fixType < 6) ? FIX_NAMES[p.fixType] : "?",
                (p.flags & 1) ? 'Y' : 'N', (p.flags & 2) ? 'Y' : 'N', p.numSV);
  printScaled(p.pDOP, 2);
  Serial.println();

  Serial.print("Latitude    : ");
  printScaled(abs64(p.lat), 7);
  Serial.printf(" deg %c\n", p.lat >= 0 ? 'N' : 'S');

  Serial.print("Longitude   : ");
  printScaled(abs64(p.lon), 7);
  Serial.printf(" deg %c\n", p.lon >= 0 ? 'E' : 'W');

  Serial.print("Altitude    : MSL ");
  printScaled(p.hMSL, 3);
  Serial.print(" m   |   ellipsoid ");
  printScaled(p.height, 3);
  Serial.println(" m");

  Serial.print("Accuracy    : horiz ");
  printScaled(p.hAcc, 3);
  Serial.print(" m   vert ");
  printScaled(p.vAcc, 3);
  Serial.print(" m   speed ");
  printScaled(p.sAcc, 3);
  Serial.println(" m/s");

  Serial.print("Ground speed: ");
  printScaled(p.gSpeed, 3);
  Serial.print(" m/s  (");
  printScaled((int64_t)p.gSpeed * 36 / 100, 2);           // mm/s -> 0.01 km/h
  Serial.println(" km/h)");

  Serial.print("Heading     : ");
  printScaled(p.headMot / 1000, 2);                       // 1e-5 deg -> 0.01 deg
  Serial.println(" deg  (course over ground, only meaningful when moving)");

  Serial.print("Vel N/E/D   : ");
  printScaled(p.velN, 3); Serial.print(" / ");
  printScaled(p.velE, 3); Serial.print(" / ");
  printScaled(p.velD, 3); Serial.println(" m/s");

  Serial.print("Link        : ");
  printScaled(rate10, 1);
  Serial.printf(" Hz measured | frames ok: %lu | bad: %lu\n",
                (unsigned long)framesOk, (unsigned long)framesBad);
}

// ============================================================ loop
void loop() {
  pumpGps();

  uint32_t now = millis();
  if (now - lastReportMs >= PRINT_PERIOD_MS) printReport(now);
}
