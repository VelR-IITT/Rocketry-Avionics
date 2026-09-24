#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>

// Initialize LittleFS for external QSPI flash
LittleFS_QSPIFlash qspiFS;
bool sdMounted = false;

// State machine for the Serial Terminal
enum TerminalState { CMD_MODE, WRITE_MODE };
TerminalState currentState = CMD_MODE;

String inputBuffer = "";
File activeFile;
String activeFileName = "";

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Pre-allocate memory to prevent heap fragmentation during infinite CLI loop
  inputBuffer.reserve(512);

  Serial.println("\n=============================================");
  Serial.println("  Teensy 4.1 Dual-FS Explorer CLI (Hardened)");
  Serial.println("=============================================");

  // --- Mount QSPI Flash ---
  if (!qspiFS.begin()) {
    Serial.println("[ERROR] Failed to mount LittleFS on QSPI.");
    Serial.println("Type 'format' to initialize a brand new chip.");
  } else {
    Serial.println("[SUCCESS] QSPI Filesystem mounted.");
  }

  // --- Initial SD Card Check ---
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("[WARNING] SD card not detected on boot. Insert one to use SD features.");
  } else {
    sdMounted = true;
    Serial.println("[SUCCESS] Built-in SD Card mounted.");
  }
  
  printHelp();
  Serial.print("\n> ");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    
    // Process on Newline or Carriage Return
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0 || currentState == WRITE_MODE) {
        processInput(inputBuffer);
        inputBuffer = ""; // Reset buffer length to 0 (memory remains reserved)
      }
      
      // Reprint prompt only if we are in command mode and it's a newline
      if (currentState == CMD_MODE && c == '\n') {
        Serial.print("\n> ");
      }
    } else {
      // Prevent buffer overflow from malicious/accidental paste of massive text
      if (inputBuffer.length() < 500) {
        inputBuffer += c;
      } else {
        Serial.println("\n[ERROR] Input too long. Buffer cleared.");
        inputBuffer = "";
        Serial.print("\n> ");
      }
    }
  }
}

void processInput(String input) {
  input.trim();

  // --- HANDLE WRITE MODE ---
  if (currentState == WRITE_MODE) {
    if (input == ".") {
      activeFile.close();
      currentState = CMD_MODE;
      Serial.printf("[INFO] Saved and closed '%s'.\n", activeFileName.c_str());
    } else {
      // Write the line and immediately flush to physical memory
      // This prevents data loss if power is cut before the file is closed.
      if (activeFile.println(input)) {
        activeFile.flush(); 
        Serial.print("  + "); 
      } else {
        Serial.println("\n[FATAL ERROR] Write failed. Is the flash chip full?");
        activeFile.close();
        currentState = CMD_MODE;
      }
    }
    return;
  }

  // --- PARSE COMMAND & ARGUMENTS ---
  String cmd = "";
  String arg = "";
  
  int spaceIndex = input.indexOf(' ');  
  if (spaceIndex != -1) {
    cmd = input.substring(0, spaceIndex);
    arg = input.substring(spaceIndex + 1);
    arg.trim();
  } else {
    cmd = input;
  }

  cmd.toLowerCase();

  // --- ROUTE COMMANDS ---
  if (cmd == "help")           printHelp();
  else if (cmd == "info")      printFSInfo();
  else if (cmd == "ls")        listQSPI();
  else if (cmd == "ls_sd")     listSD();
  else if (cmd == "format")    formatQSPI();
  else if (cmd == "cat")       readFile(arg);
  else if (cmd == "write")     startWriteMode(arg);
  else if (cmd == "rm")        deleteQSPIFile(arg);
  else if (cmd == "import")    copyFile(arg, true);  
  else if (cmd == "export")    copyFile(arg, false); 
  else if (cmd == "")          { /* Ignore empty enter presses */ }
  else {
    Serial.printf("[ERROR] Unknown command: '%s'\n", cmd.c_str());
  }
}

// ---------------------------------------------------------
// File Transfer & Validation Logic
// ---------------------------------------------------------

bool isValidFilename(String filename) {
  if (filename.length() == 0 || filename.length() > 255) return false;
  
  // Strict character whitelist approach is safer than a blacklist
  for (unsigned int i = 0; i < filename.length(); i++) {
    char c = filename[i];
    if (!(isalnum(c) || c == '.' || c == '_' || c == '-' || c == '/')) {
      return false;
    }
  }
  return true;
}

void copyFile(String filename, bool toQSPI) {
  if (!isValidFilename(filename)) {
    Serial.println("[ERROR] Invalid filename. Use alphanumeric, '.', '_', '-', and '/'.");
    return;
  }

  String safePath = filename;
  if (!safePath.startsWith("/")) safePath = "/" + safePath;

  File srcFile, destFile;
  String srcName, destName;

  // Setup Source and Destination
  if (toQSPI) {
    srcName = "SD Card"; destName = "QSPI Flash";
    srcFile = SD.open(safePath.c_str(), FILE_READ);
    if (!srcFile) {
      Serial.printf("[ERROR] Cannot open '%s' on SD. Is it inserted?\n", safePath.c_str());
      return;
    }
    destFile = qspiFS.open(safePath.c_str(), FILE_WRITE);
  } else {
    srcName = "QSPI Flash"; destName = "SD Card";
    srcFile = qspiFS.open(safePath.c_str(), FILE_READ);
    if (!srcFile) {
      Serial.printf("[ERROR] Cannot open '%s' on QSPI.\n", safePath.c_str());
      return;
    }
    
    // SD cards require explicit removal before overwrite for safety
    if (SD.exists(safePath.c_str())) SD.remove(safePath.c_str());
    destFile = SD.open(safePath.c_str(), FILE_WRITE);
  }

  // Handle Directory Edge Case
  if (srcFile.isDirectory()) {
    Serial.println("[ERROR] Target is a directory, not a file.");
    srcFile.close();
    if (destFile) destFile.close();
    return;
  }

  if (!destFile) {
    Serial.printf("[ERROR] Failed to create file on %s. Write-protected or full?\n", destName.c_str());
    srcFile.close();
    return;
  }

  Serial.printf("[INFO] Copying '%s' from %s to %s...\n", safePath.c_str(), srcName.c_str(), destName.c_str());
  
  // Allocate buffer on the heap safely
  const size_t bufSize = 8192;
  uint8_t* buf = (uint8_t*)malloc(bufSize);
  
  if (!buf) {
    Serial.println("[FATAL] Out of memory allocating transfer buffer.");
    srcFile.close(); destFile.close();
    return;
  }

  uint32_t fileSize = srcFile.size();
  uint32_t totalBytes = 0;
  bool writeError = false;
  int lastPercent = -1;
  unsigned long startTime = millis();

  // The actual transfer loop
  while (srcFile.available()) {
    size_t bytesRead = srcFile.read(buf, bufSize);
    size_t bytesWritten = destFile.write(buf, bytesRead);
    
    if (bytesWritten != bytesRead) {
      Serial.println("\n[ERROR] Write mismatch! Storage full or removed?");
      writeError = true;
      break;
    }
    
    totalBytes += bytesWritten;
    
    // Calculate and print percentage dynamically
    if (fileSize > 0) {
      // 100ULL ensures we don't overflow the 32-bit integer when multiplying large file sizes
      int currentPercent = (totalBytes * 100ULL) / fileSize; 
      
      // Only print if the percentage has actually changed to save serial bandwidth
      if (currentPercent != lastPercent) {
        Serial.printf("\r[INFO] Progress: %d%%  ", currentPercent);
        lastPercent = currentPercent;
      }
    }
  }
  
  // Cleanup resources
  free(buf);
  srcFile.close();
  destFile.close();

  if (!writeError) {
    float timeSec = (millis() - startTime) / 1000.0;
    // Add a newline before printing the final success message so it doesn't overwrite the 100% text
    Serial.printf("\n[SUCCESS] Transferred %lu bytes in %.2f seconds.\n", totalBytes, timeSec);
  } else {
    // If we failed mid-write, attempt to delete the corrupted destination file
    if (toQSPI) qspiFS.remove(safePath.c_str());
    else SD.remove(safePath.c_str());
    Serial.println("\n[INFO] Cleaned up corrupted destination file.");
  }
}

// ---------------------------------------------------------
// Helper & CLI Functions
// ---------------------------------------------------------

void printHelp() {
  Serial.println("\nAvailable Commands:");
  Serial.println("  info             - Show QSPI filesystem size and usage");
  Serial.println("  ls               - List files on QSPI flash");
  Serial.println("  ls_sd            - List files on SD Card");
  Serial.println("  cat <file>       - Print a QSPI file's contents to the terminal");
  Serial.println("  write <file>     - Create/overwrite a QSPI file and enter typing mode");
  Serial.println("  rm <file>        - Delete a file from QSPI flash");
  Serial.println("  import <file>    - Copy a file from SD Card -> QSPI Flash");
  Serial.println("  export <file>    - Copy a file from QSPI Flash -> SD Card");
  Serial.println("  format           - Erase and format the QSPI flash chip");
  Serial.println("  help             - Show this menu");
}

void printFSInfo() {
  uint64_t total = qspiFS.totalSize();
  uint64_t used = qspiFS.usedSize();
  
  Serial.println("\n--- QSPI Filesystem Info ---");
  Serial.printf("Total Size: %llu bytes\n", total);
  Serial.printf("Used Size:  %llu bytes\n", used);
  Serial.printf("Free Space: %llu bytes\n", total - used);
  Serial.println("----------------------------");
  
  if (sdMounted) {
    Serial.println("[INFO] SD Card is mounted and ready.");
  } else {
    Serial.println("[WARNING] SD Card is NOT mounted.");
  }
}

void listQSPI() {
  Serial.println("\n--- QSPI Flash Directory ---");
  File root = qspiFS.open("/");
  if (!root) {
    Serial.println("[ERROR] Failed to open QSPI root. Is it formatted?");
    return;
  }
  printDirectory(root, 0);
  root.close();
  Serial.println("----------------------------");
}

void listSD() {
  Serial.println("\n--- SD Card Directory ---");
  File root = SD.open("/");
  if (!root) {
    // Re-attempt to mount just in case card was swapped
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("[ERROR] Failed to open SD root. Is the card inserted and formatted FAT32/exFAT?");
      return;
    }
    sdMounted = true;
    root = SD.open("/");
  }
  printDirectory(root, 0);
  root.close();
  Serial.println("-------------------------");
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    
    for (uint8_t i = 0; i < numTabs; i++) Serial.print("  ");
    Serial.print(entry.name());
    
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      Serial.print("\t\t\t");
      Serial.print(entry.size(), DEC);
      Serial.println(" bytes");
    }
    entry.close();
  }
}

void readFile(String filename) {
  if (!isValidFilename(filename)) {
    Serial.println("[ERROR] Invalid filename.");
    return;
  }
  
  String safePath = filename;
  if (!safePath.startsWith("/")) safePath = "/" + safePath;

  File f = qspiFS.open(safePath.c_str(), FILE_READ);
  if (!f) {
    Serial.printf("[ERROR] Failed to open '%s'. Does it exist?\n", safePath.c_str());
    return;
  }

  if (f.isDirectory()) {
    Serial.println("[ERROR] Cannot read a directory using 'cat'.");
    f.close();
    return;
  }

  Serial.printf("\n--- Contents of %s (%llu bytes) ---\n", safePath.c_str(), f.size());
  while (f.available()) {
    Serial.write(f.read());
  }
  Serial.println("\n----------------------------------");
  f.close();
}

void startWriteMode(String filename) {
  if (!isValidFilename(filename)) {
    Serial.println("[ERROR] Invalid filename.");
    return;
  }

  String safePath = filename;
  if (!safePath.startsWith("/")) safePath = "/" + safePath;

  activeFile = qspiFS.open(safePath.c_str(), FILE_WRITE);
  if (!activeFile) {
    Serial.printf("[ERROR] Failed to open '%s' for writing.\n", safePath.c_str());
    return;
  }

  if (activeFile.isDirectory()) {
    Serial.println("[ERROR] Target is a directory. Cannot write text to it.");
    activeFile.close();
    return;
  }

  activeFileName = safePath;
  currentState = WRITE_MODE;
  
  Serial.printf("\n[INFO] Opened '%s' on QSPI for writing.\n", safePath.c_str());
  Serial.println("Type your text below. To save and exit, send a single period '.' on a new line.");
  Serial.print("  + ");
}

void deleteQSPIFile(String filename) {
  if (!isValidFilename(filename)) {
    Serial.println("[ERROR] Invalid filename.");
    return;
  }
  
  String safePath = filename;
  if (!safePath.startsWith("/")) safePath = "/" + safePath;

  if (qspiFS.remove(safePath.c_str())) {
    Serial.printf("[SUCCESS] Deleted '%s' from QSPI\n", safePath.c_str());
  } else {
    Serial.printf("[ERROR] Could not delete '%s'.\n", safePath.c_str());
  }
}

void formatQSPI() {
  Serial.println("\n[WARNING] Formatting QSPI Flash...");
  qspiFS.format();
  Serial.println("[SUCCESS] Format complete.");
  if (!qspiFS.begin()) {
    Serial.println("[FATAL] Still failed to mount after formatting. Hardware issue.");
  } else {
    Serial.println("[SUCCESS] Filesystem re-mounted.");
  }
}