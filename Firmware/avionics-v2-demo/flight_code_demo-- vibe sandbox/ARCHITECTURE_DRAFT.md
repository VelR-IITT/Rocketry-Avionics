# Rocketry Avionics V2 - Architecture & Documentation Draft

This document serves as a living scratchpad for architectural decisions, system design, and final documentation for the avionics firmware.

## 1. Flight State Machine

To maximize safety and minimize complexity, the system uses a Command-Controlled State Machine with a Minimum Altitude Lockout.

### States
* **`STATE_GROUND` (Default):**
  * System is safe for handling. 
  * Pyros are strictly software-locked.
  * Barometer task maintains a rolling average (e.g., 1-second window) of the current pressure.
  * Low-rate telemetry and logging.

* **`STATE_FLIGHT` (Armed):**
  * Triggered exclusively via LoRa Uplink Command (`CMD_FLIGHT`).
  * **Zeroing:** At the exact moment the command is received, the current barometer rolling average is locked in as the `ground_reference_pressure`.
  * High-speed logging is active.
  * Apogee detection algorithm actively monitors for deployment conditions.

### Safety Mechanisms
* **Minimum Altitude Lockout:** The apogee detection algorithm will NOT fire pyros unless the calculated Above Ground Level (AGL) altitude exceeds a hardcoded threshold (e.g., 50 meters), preventing pad-deployment due to weather/pressure drift.

## 2. FreeRTOS Task Topology

* **Sensor Tasks:** `IMU_Task`, `Baro_Task`, `ADXL_Task`, `GPS_Task`
  * Poll hardware sensors and push data to the `Data_Queue`.
  * Update global telemetry structures.
* **Logging Tasks (Dual Redundancy):** 
  * `Data_Log_Task` (High Priority): Reads from the primary `Flash_Queue` and writes high-speed binary data to LittleFS (Flash). Once safely in Flash, it passes a copy of the data into a secondary `SD_Queue`.
  * `SD_Log_Task` (Low Priority): Reads from the `SD_Queue` and streams data directly to the SD card in real-time (saving as `/flight_X.bin`). This ensures data survives if power is lost before the post-flight transfer.
  * `Transfer_SD`: Triggered post-flight to dump Flash data to the SD card. Saves the flash dump as `/flight_X_f.bin` to distinguish it from the real-time log. It asserts an `SD_Transfer_Active` flag while running, forcing the `SD_Log_Task` to pause and preventing SD bus collisions.
* **Comm Tasks:**
  * `LoRa_Task`: Transmits global telemetry states and listens for uplink commands.
  * `Cmd_Task`: Processes uplink commands from the `Cmd_Queue`.

## 3. Data Concurrency & IPC
* **Data Tearing Prevention:** Global sensor structs (`latest_baro_data`, `latest_imu_data`, etc.) will be updated and read using FreeRTOS **Critical Sections** (`taskENTER_CRITICAL()` and `taskEXIT_CRITICAL()`).
* **Why:** Disabling interrupts for the nanoseconds it takes to copy a tiny struct is significantly faster and less complex than managing Mutexes or Double-Buffering flags.
* **Golden Rule:** Critical sections must *strictly* contain only memory copy operations. Doing anything slow inside them (like `Serial.print`, SPI transfers, or `delay`) will crash the RTOS or drop hardware interrupts.

## 4. Hardware Interfaces
* **MCU:** Teensy 4.1 (ARM Cortex-M7 with Double-Precision FPU)
* **Barometer:** BMP280 (SPI) - *Note: Utilizes 64-bit `double` compensation math natively on the FPU to bypass compiler bit-shift bugs and maintain max precision.*
* **IMU:** GY-91 / MPU9250 (SPI)
* **High-G Accel:** ADXL (I2C)
* **GPS:** UBLOX (Serial7)
* **Radio:** RFM95 LoRa 866MHz (SPI)
* **Storage:** Onboard LittleFS + Built-in SD Card

## 5. Dual Telemetry & Command Routing (LoRa + GSM)
To ensure maximum communication redundancy, the system uses a Multi-Producer / Single-Consumer queue architecture for commands:
* **Downlink (Telemetry):** Both `LoRa_Task` and `GSM_Task` read the global sensor variables independently (using Critical Sections) and transmit them over their respective links.
* **Uplink (Commands):** Both tasks listen for incoming commands. If a valid command is received on *either* link, it is pushed to the shared `Cmd_Queue`. 
* FreeRTOS natively handles multiple tasks writing to the same queue simultaneously, meaning the `Cmd_Task` doesn't care whether a command arrived via LoRa or GSM—it simply executes it.

## 6. Pyro Management
To prevent blocking critical flight loops, pyro deployment is handled by a dedicated `Pyro_Task`.
* **Mechanism:** The apogee detection logic triggers the pyro by sending a FreeRTOS Task Notification (`xTaskNotifyGive()`) to the `Pyro_Task`.
* **Execution:** Once notified, the `Pyro_Task` double-checks the safety lockouts, sets the ignition pin HIGH, uses `vTaskDelay()` to hold the current for the required burn duration (e.g., 2 seconds), and then pulls the pin LOW. After firing, the task suspends itself permanently.
* **Continuity:** Prior to deployment, the `Pyro_Task` periodically wakes up (e.g., at 1Hz) to read the continuity sense pins and update the global telemetry state.
