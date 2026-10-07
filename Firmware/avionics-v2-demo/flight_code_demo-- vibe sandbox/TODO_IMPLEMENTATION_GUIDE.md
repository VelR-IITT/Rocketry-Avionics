# Avionics V2 - Implementation Guide & Code Reference

This document outlines the specific code changes required to bring the current `main.cpp` up to the finalized architecture.

## 1. Barometer Math Fix  --- > unresolved implemented this . but the readings are still around 610 
**Problem:** The 64-bit integer formula in `Read_Baro()` is prone to compiler shift bugs on 32-bit ARM, resulting in ~617 hPa.
**Implementation:** Replace the `int64_t` math with native `double` (64-bit FPU) math.
```cpp
// Inside Read_Baro()
double var1, var2, p;
var1 = ((double)t_fine / 2.0) - 64000.0;
var2 = var1 * var1 * ((double)cal.P6) / 32768.0;
var2 = var2 + var1 * ((double)cal.P5) * 2.0;
var2 = (var2 / 4.0) + (((double)cal.P4) * 65536.0);
var1 = (((double)cal.P3) * var1 * var1 / 524288.0 + ((double)cal.P2) * var1) / 524288.0;
var1 = (1.0 + var1 / 32768.0) * ((double)cal.P1);
if (var1 == 0.0) return data; 
p = 1048576.0 - (double)adc_P;
p = (p - (var2 / 4096.0)) * 6250.0 / var1;
var1 = ((double)cal.P9) * p * p / 2147483648.0;
var2 = p * ((double)cal.P8) / 32768.0;
p = p + (var1 + var2 + ((double)cal.P7)) / 16.0;
data.baro.pressure = (float)(p / 100.0);
```

## 2. Pyro Task & Continuity
**Implementation:** Create a dedicated task utilizing `ulTaskNotifyTake` for a hybrid timeout/trigger loop.
```cpp
void Pyro_Task(void *pvParameters) {
    while(1) {
        uint32_t trigger = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); // 1Hz timeout
        if (trigger > 0) {
            digitalWrite(PYRO_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(2000)); // 2 second burn
            digitalWrite(PYRO_PIN, LOW);
            vTaskDelete(NULL); 
        } else {
            // No trigger -> Check continuity
            int status = analogRead(SENSE_PIN);
            taskENTER_CRITICAL();
            latest_board_data.pyro_state = status;
            taskEXIT_CRITICAL();
        }
    }
}
```

## 3. Data Concurrency
**Implementation:** Wrap all updates to global sensor structs in Critical Sections.
```cpp
taskENTER_CRITICAL();
latest_baro_data = data.baro;
taskEXIT_CRITICAL();
```

## 4. Dual Logging Pipeline
**Implementation:** 
1. Create `QueueHandle_t SD_Queue;`
2. Sensor tasks push to `Data_Queue` (which acts as the Flash queue).
3. `Data_Log_Task` reads from `Data_Queue`, writes to LittleFS, then calls `xQueueSend(SD_Queue, ...)`
4. `SD_Log_Task` reads from `SD_Queue` and writes to `/flight_X.bin`.

## 5. GSM Integration
**Implementation:** 
1. Create `GSM_Task(void *pvParameters)`.
2. Route incoming GSM commands to the existing queue: `xQueueSend(Cmd_Queue, &rx_pkt, 0)`.

## 6. Buzzer Task
**Implementation:** 
Create a low-priority task that reads `latest_board_data.error_code` and `latest_board_data.state`, using `vTaskDelay()` to beep patterns (e.g., 3 fast beeps = No GPS, Solid Tone = Armed).

## 7. Safety Fixes (To Implement)
* **I2C Timeout:** Add `Wire1.setWireTimeout(3000, true);` to prevent the ADXL from freezing the RTOS.
* **Internal Watchdog:** Use the Teensy's internal silicon watchdog (`WDT_T4` library) to reboot if a task deadlocks.
