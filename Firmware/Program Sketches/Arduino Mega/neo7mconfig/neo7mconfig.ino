// Configuration sketch for NEO-7M GPS module on Arduino Mega
// - Set baud rate to 115200
// - Set update rate to 10 Hz (100 ms)
// - Enable only $GPGGA sentences

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(1000);
  
  Serial.println("Configuring NEO-7M GPS...");

  // Step 0: Reset to factory defaults
  resetToDefaults();
  delay(2000);
  Serial.println("Reset to factory defaults");

  // Step 1: Change baud rate to 115200
  configureBaudRate();
  delay(100);
  Serial1.flush();
  Serial1.end();
  Serial1.begin(115200);
  delay(2000);
  Serial.println("Baud rate set to 115200");

  // Step 2: Set update rate to 10 Hz (100 ms)
  configureUpdateRate();
  delay(500);
  Serial.println("Update rate set to 10 Hz");

  // Step 3: Configure NMEA messages (alternative method)
  configureNMEAMessages();
  delay(500);
  Serial.println("Only GPGGA enabled");

  // Step 4: Save configuration
  saveConfiguration();
  delay(500);
  Serial.println("Configuration saved");

  Serial.println("Setup complete. Listening for GPS data...");
}

void loop() {
  // Custom buffer with $GPGSA filter
  static char buffer[128];
  static int index = 0;
  while (Serial1.available() && index < 127) {
    char c = Serial1.read();
    buffer[index++] = c;
    if (c == '\n') {
      buffer[index] = '\0';
      //if (strncmp(buffer, "$GPGGA", 6) == 0) {  // Only print GPGGA
        Serial.print(buffer);
      //}
      index = 0;
    }
  }
}

// Send UBX packet and check ACK/NAK
void sendUBX(uint8_t *packet, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    Serial1.write(packet[i]);
  }
  Serial1.flush();
  delay(100);  // Wait for response
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    Serial.print(b, HEX); Serial.print(" ");
  }
  Serial.println();
}

// Step 0: Reset to factory defaults
void resetToDefaults() {
  uint8_t packet[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xAB, 0xA0
  };
  sendUBX(packet, sizeof(packet));
}

// Step 1: Configure baud rate to 115200
void configureBaudRate() {
  uint8_t packet[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
    0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xC0, 0x7E
  };
  sendUBX(packet, sizeof(packet));
}

// Step 2: Configure update rate to 10 Hz (100 ms)
void configureUpdateRate() {
  uint8_t packet[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
    0x01, 0x00, 0x01, 0x00, 0x79, 0x5A
  };
  sendUBX(packet, sizeof(packet));
}

// Step 3: Configure NMEA messages (alternative method)
void configureNMEAMessages() {
  // Disable all NMEA messages via CFG-NMEA
  uint8_t disableAllNMEA[] = {
    0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x00,  // Filter: none
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Masks: defaults
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Reserved
    0x00, 0x00, 0x29, 0xB5  // Checksum
  };
  
  // Enable GPGGA
  uint8_t enableGPGGA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10
  };

  // Send disable all NMEA command
  sendUBX(disableAllNMEA, sizeof(disableAllNMEA));
  delay(200);
  Serial.println("Disabled all NMEA messages");

  // Send enable GPGGA command
  sendUBX(enableGPGGA, sizeof(enableGPGGA));
  delay(200);
  Serial.println("Enabled GPGGA");
}

// Step 4: Save configuration to non-volatile memory
void saveConfiguration() {
  uint8_t packet[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xAC, 0x9F
  };
  sendUBX(packet, sizeof(packet));
}