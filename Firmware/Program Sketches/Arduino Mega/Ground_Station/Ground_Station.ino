#include <SPI.h>
#include <RH_RF95.h>
#include <string.h>

#define RF95_CS  10
#define RF95_INT 2
#define RF95_RST 9
#define RF95_FREQ 866.0

RH_RF95 rf95(RF95_CS, RF95_INT);

#pragma pack(push, 1)

typedef struct {
    uint32_t time;
    uint32_t utc_time;
    int32_t  lat, lon;
    int32_t    gps_alt, baro_alt;
    float    vx, vy, vz;
    float    roll, pitch, yaw;
    float    pressure;    
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  hx, hy, hz; // high g accel
    uint8_t  temp;
    uint8_t  v_batt;       
    uint8_t  state;      // first 3 bits for state, next 5 bits for sats count
    uint8_t  error_code; 
    uint8_t  pyro_state;
    uint8_t  RSSI;
    uint16_t CRC16;
} telemetry_pkt_t;

#define CMD_NONE       0x00
#define CMD_LED_START  0x01
#define CMD_LED_STOP   0x02
#define CMD_LOG_START  0x03
#define CMD_LOG_STOP   0x04
#define CMD_TRIG_PYRO  0x05

typedef struct {
    uint8_t  flags;      // bit 0 = has_command
    uint8_t  cmd_type;
    uint8_t  cmd_param;
    uint16_t CRC16;
} ack_pkt_t;

ack_pkt_t cmd_que[8] = {0};
uint8_t   cmd_que_count  = 0;

#pragma pack(pop)

void setup() {
    pinMode(RF95_RST, OUTPUT);
    digitalWrite(RF95_RST, HIGH);
    delay(100);
    digitalWrite(RF95_RST, LOW);
    delay(10);
    digitalWrite(RF95_RST, HIGH);
    delay(10);

    Serial.begin(115200);
    while (!Serial);

    if (!rf95.init()) {
        Serial.println("LoRa init failed");
        while (1);
    }

    rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
    // Bw125Cr45Sf128 	   ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
	// Bw500Cr45Sf128      ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	// Bw31_25Cr48Sf512	   ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	// Bw125Cr48Sf4096     ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
    

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }

    Serial.println("Receiver ready");
    Serial.print("Expecting packet size: ");
    Serial.println(sizeof(telemetry_pkt_t));
}

void Handle_LoRa()
{
  if (rf95.available()) {
        telemetry_pkt_t pkt;
        uint8_t len = sizeof(pkt);

        if (rf95.recv((uint8_t*)&pkt, &len)) {

            if (len != sizeof(telemetry_pkt_t)) {
                Serial.print("Size mismatch! got=");
                Serial.print(len);
                Serial.print(" expected=");
                Serial.println(sizeof(telemetry_pkt_t));
                return;
            }

            Serial.println("--- Packet ---");
            Serial.print("time:      "); Serial.print(pkt.time);
            Serial.print(", utc:       "); Serial.println(pkt.utc_time);
            Serial.print("lat:       "); Serial.print(pkt.lat);
            Serial.print(", lon:       "); Serial.println(pkt.lon);
            Serial.print("gps_alt:   "); Serial.print(pkt.gps_alt);
            Serial.print(", baro_alt:  "); Serial.println(pkt.baro_alt);
            Serial.print("vx vy vz:  ");
            Serial.print(pkt.vx); Serial.print(" ");
            Serial.print(pkt.vy); Serial.print(" ");
            Serial.println(pkt.vz);
            Serial.print("roll:      "); Serial.print(pkt.roll);
            Serial.print(", pitch:     "); Serial.print(pkt.pitch);
            Serial.print(", yaw:       "); Serial.println(pkt.yaw);
            Serial.print("pressure:  "); Serial.println(pkt.pressure);
            
            Serial.print("accel:     ");
            Serial.print(pkt.ax); Serial.print(" ");
            Serial.print(pkt.ay); Serial.print(" ");
            Serial.println(pkt.az);
            
            Serial.print("gyro:      ");
            Serial.print(pkt.gx); Serial.print(" ");
            Serial.print(pkt.gy); Serial.print(" ");
            Serial.println(pkt.gz);
            
            Serial.print("high_g:    ");
            Serial.print(pkt.hx); Serial.print(" ");
            Serial.print(pkt.hy); Serial.print(" ");
            Serial.println(pkt.hz);
            
            Serial.print("temp:      "); Serial.print(pkt.temp);
            Serial.print(", v_batt:    "); Serial.println(pkt.v_batt);
            
            // Decode State (last 3 bits) and Sats (first 5 bits)
            uint8_t current_state = pkt.state >>5; 
            uint8_t sats_count = (pkt.state) & 0x1F;
            Serial.print("state:     "); Serial.print(current_state);
            Serial.print(", sats:      "); Serial.println(sats_count);
            
            Serial.print("pyro:      "); Serial.print(pkt.pyro_state, BIN);
            Serial.print(", RSSI:      "); Serial.println(rf95.lastRssi());
            
            // -----------------------------------------------------
            // Decoded State Bools & Errors
            // -----------------------------------------------------
            // Bit 5 is just a status bool for SD logging
            bool is_logging = (pkt.error_code & 0x20) ? 1 : 0;
            Serial.print("Logging:   "); Serial.println(is_logging);
            
            // Display hardware errors strictly as 0 or 1
            Serial.print("Errors:    ");
            Serial.print("IMU : "); Serial.print((pkt.error_code & 0x01) ? 1 : 0);
            Serial.print(" , BARO : "); Serial.print((pkt.error_code & 0x02) ? 1 : 0);
            Serial.print(" , GPS : "); Serial.print((pkt.error_code & 0x04) ? 1 : 0);
            Serial.print(" , GSM : "); Serial.print((pkt.error_code & 0x08) ? 1 : 0);
            Serial.print(" , SD : "); Serial.println((pkt.error_code & 0x10) ? 1 : 0);
            
            // Comms feedback bits
            Serial.print((pkt.error_code & 0x80) ? "GOOD ACK " : "NO ACK ");
            Serial.println((pkt.error_code & 0x40) ? " CMD Received" : " ");
            Serial.println("--------------");

            // Queue processing logic
            if((pkt.error_code & 0x40))
            {
                if(cmd_que_count != 0)
                    cmd_que_count--;
            }

            if(cmd_que_count == 0)
            {
                ack_pkt_t ack;
                ack = {0};
                ack.flags = 0x00;
                rf95.send((uint8_t*) &ack, sizeof(ack));
                rf95.waitPacketSent();
            }
            else
            {
                rf95.send((uint8_t*) &cmd_que[cmd_que_count-1], sizeof(ack_pkt_t));
                rf95.waitPacketSent();
                char buf[80];
                snprintf(buf, sizeof(buf), "command sent: type=0x%02X param=0x%02X queue=%d",
                             cmd_que[cmd_que_count-1].cmd_type,
                             cmd_que[cmd_que_count-1].cmd_param,
                             cmd_que_count);
                Serial.println(buf);            
            }

        } 
        else 
        {
            Serial.println("recv failed");
        }
    }
}

void loop() 
{
    Handle_LoRa();
    
    if(cmd_que_count < 8)
    {
        if(Serial.available())
        {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if(cmd == "ON")
            {
                cmd_que[cmd_que_count].flags = 0x01;
                cmd_que[cmd_que_count].cmd_type = CMD_LED_START;
                cmd_que_count++;
                Serial.print("ON added to queue, queue size: ");
                Serial.println(cmd_que_count);
            }
            else if(cmd == "OFF")
            {
                cmd_que[cmd_que_count].flags = 0x01;
                cmd_que[cmd_que_count].cmd_type = CMD_LED_STOP;
                cmd_que_count++;
                Serial.print("OFF added to queue, queue size: ");
                Serial.println(cmd_que_count);
            }
            else if(cmd == "LOG")
            {
                cmd_que[cmd_que_count].flags = 0x01;
                cmd_que[cmd_que_count].cmd_type = CMD_LOG_START;
                cmd_que_count++;
                Serial.print("LOG added to queue, queue size: ");
                Serial.println(cmd_que_count);
            }
            else if(cmd == "SLG")
            {
                cmd_que[cmd_que_count].flags = 0x01;
                cmd_que[cmd_que_count].cmd_type = CMD_LOG_STOP;
                cmd_que_count++;
                Serial.print("SLG added to queue, queue size: ");
                Serial.println(cmd_que_count);
            }
            else
            {
                Serial.println("unknown command");
            }
        }
    }
}