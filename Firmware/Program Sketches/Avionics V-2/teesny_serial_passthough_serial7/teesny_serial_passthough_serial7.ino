// Teensy 4.1: USB Serial <-> Serial7 pass-through for u-blox NEO-7M / u-center
// Serial7: RX7 = pin 28, TX7 = pin 29

#define GPS_SERIAL        Serial7
#define GPS_DEFAULT_BAUD  9600      // NEO-7M factory default

static uint8_t  gpsRxBuf[2048];     // large buffers so UBX bursts aren't dropped
static uint8_t  gpsTxBuf[512];
static uint32_t gpsBaud = GPS_DEFAULT_BAUD;

void setup() {
  Serial.begin(115200);             // baud is ignored on USB, but required to start it
  GPS_SERIAL.addMemoryForRead(gpsRxBuf, sizeof(gpsRxBuf));
  GPS_SERIAL.addMemoryForWrite(gpsTxBuf, sizeof(gpsTxBuf));
  GPS_SERIAL.begin(gpsBaud);
}

void loop() {
  // Follow the baud rate set by the PC (u-center changes this when you
  // connect or send UBX-CFG-PRT)
  uint32_t usbBaud = Serial.baud();
  if (usbBaud != gpsBaud && usbBaud >= 1200 && usbBaud <= 3000000) {
    gpsBaud = usbBaud;
    GPS_SERIAL.begin(gpsBaud);
  }

  uint8_t buf[256];
  int n;

  // PC (u-center) -> GPS
  n = Serial.available();
  if (n > 0) {
    n = min(n, (int)sizeof(buf));
    n = min(n, (int)GPS_SERIAL.availableForWrite());
    if (n > 0) {
      n = Serial.readBytes((char *)buf, n);
      GPS_SERIAL.write(buf, n);
    }
  }

  // GPS -> PC (u-center)
  n = GPS_SERIAL.available();
  if (n > 0) {
    n = min(n, (int)sizeof(buf));
    n = min(n, (int)Serial.availableForWrite());
    if (n > 0) {
      n = GPS_SERIAL.readBytes((char *)buf, n);
      Serial.write(buf, n);
      Serial.send_now();            // push immediately, keeps latency low
    }
  }
}