#ifndef GSM_H
#define GSM_H

// gsm.h  —  GSM SMS driver for Rocketry Avionics
//
// Important Requirements: (Must be checked)
// Port: tsandmann/freertos-teensy
// Include: arduino_freertos.h (not Arduino.h + FreeRTOS.h)
// Scheduler: vTaskStartScheduler() called manually in setup()
// Serial:    Serial.begin(0) for USB CDC on Teensy 4.1 (if in case if you need to test the code)

#include "arduino_freertos.h"   // single header for this port

// ─── hardware binding ────────────────────────────────

#define GSM_SERIAL          Serial8     // hardware UART on Teensy (Hardware_configuration)

// tunables 

#define GSM_LINE_MAX        200         // max chars in one AT response line
#define GSM_TX_QUEUE_DEPTH  8           // outbound SMS queue depth
#define GSM_RX_QUEUE_DEPTH  8           // pending-RX index queue depth
#define GSM_MAX_RETRIES     3           // max retries before dropping a TX
#define GSM_TIMEOUT_MS      10000UL     // per-state timeout in ms

// return codes 

typedef enum {
    GSM_OK              =  1,
    GSM_ERR_QUEUE_FULL  = -1,
    GSM_ERR_BAD_PARAM   = -2
} gsm_err_t;

// public API

/**
 * gsm_init()
 *   Call once from setup(), BEFORE vTaskStartScheduler().
 *   Creates queues and spawns GSM_IO (pri 2) + GSM_CB (pri 1).
 */
void gsm_init(void);

/**
 * gsm_send_sms()
 *   Non-blocking enqueue. Safe from any task or from setup().
 *   Returns GSM_OK, GSM_ERR_QUEUE_FULL, or GSM_ERR_BAD_PARAM.
 */
gsm_err_t gsm_send_sms(const char *number,
                        const char *message);

// ─── callbacks — implement in main.cpp ───────────────

void on_sms_received(const char *number,
                     const char *message);

void on_sms_sent(int msg_id);

#endif // GSM_H
