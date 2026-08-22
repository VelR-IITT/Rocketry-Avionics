#include <Arduino.h>
#include "data.h"
#include "hardware.h"

#define DEBUG   true
#define BUZZER  true
#define TELEMETRY_BUFFER 10
#define LOG_BUFFER 100

volatile float roll ,pitch, yaw;

log_t log_buf[LOG_BUFFER] = {};
volatile  log_head = 0;
volatile  log_tail = 0;
gsm_pkt_t gsm_buf[TELEMETRY_BUFFER] = {};
volatile  gsm_head = 0;
volatile  gsm_tail = 0;
telemetry_pkt_t  telemetry_buf[TELEMETRY_BUFFER] = {};
volatile  telemetry_head = 0;
volatile  telemetry_tail = 0;



void setup() 
{
  
}

void loop() 
{
 
}

