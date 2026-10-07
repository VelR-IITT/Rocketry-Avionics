#include <SPI.h>
#include <RH_RF95.h>
#include <string.h>

#define RF95_CS   10
#define RF95_INT  2
#define RF95_RST  9
#define RF95_FREQ 866.0

// unit scaling for raw packet values (change if your firmware differs)
#define LATLON_DIV    10000000UL   // degrees * 1e7
#define GPS_ALT_DIV   1000UL       // mm -> m
#define BARO_ALT_DIV  1000UL       // mm -> m (use 100UL if cm)

RH_RF95 rf95(RF95_CS, RF95_INT);

#pragma pack(push, 1)

typedef struct {
    uint32_t time;
    uint32_t utc_time;
    int32_t  lat, lon;
    int32_t  gps_alt, baro_alt;
    float    vx, vy, vz;
    float    roll, pitch, yaw;
    float    pressure;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  hx, hy, hz;      // high g accel
    uint8_t  temp;
    uint8_t  v_batt;
    uint8_t  state;           // first 3 bits state, next 5 bits sats count
    uint8_t  error_code;
    uint8_t  flags;
    uint8_t  flight_number;
    uint8_t  pyro_state;      // bits 0-3 continuity 1-4, bits 4-7 pyro 1-4 active
    uint8_t  RSSI;
    uint16_t CRC16;
} telemetry_pkt_t;

typedef struct {
    uint8_t  flags;           // bit 0 = has_command
    uint8_t  cmd_type;
    uint8_t  cmd_param;
    uint16_t CRC16;
} ack_pkt_t;

#pragma pack(pop)

static_assert(sizeof(telemetry_pkt_t) == 80, "telemetry packet size changed, update flight code too");

// error_code bits
#define ERR_IMU        0x01
#define ERR_BARO       0x02
#define ERR_GPS        0x04
#define ERR_GSM        0x08
#define ERR_SD         0x10
#define ERR_FLASH      0x20
#define ERR_XFER       0x40   // file transfer error
#define ERR_ACK_RX     0x80   // ack received in last cycle

// flags bits
#define FLG_XFER_ACTIVE 0x01
#define FLG_XFER_DONE   0x02
#define FLG_LOGGING     0x04
#define FLG_CMD_RX      0x08  // command received in last cycle
#define FLG_DATA_DROP   0x10
#define FLG_SD_DROP     0x20
#define FLG_GPS_FIX_OK  0x40
#define FLG_GPS_FIX_3D  0x80  // 1 = 3D, 0 = 2D

// commands
#define CMD_NONE          0x00
#define CMD_LED_BLINK     0x01
#define CMD_LED_OFF       0x02
#define CMD_LOG_START     0x03
#define CMD_LOG_STOP      0x04
#define CMD_SET_CON_MODE  0x05
#define CMD_SET_PYRO_MODE 0x06
#define CMD_TRIG_PYRO     0x07

// pyro modes
#define PYRO_1     1
#define PYRO_2     2
#define PYRO_3     4
#define PYRO_4     8
#define PYRO_12    3
#define PYRO_34    12
#define PYRO_12_34 15

#define CMD_QUE_SIZE 8
#define MAX_RETRIES  10

ack_pkt_t cmd_que[CMD_QUE_SIZE];
uint8_t   cmd_sent[CMD_QUE_SIZE];   // transmissions per queued command
uint8_t   cmd_que_count = 0;

bool sent_something = false;

// ---------- helpers ----------

const char* cmdName(uint8_t t)
{
    switch (t)
    {
        case CMD_LED_BLINK:     return "LED_BLINK";
        case CMD_LED_OFF:       return "LED_OFF";
        case CMD_LOG_START:     return "LOG_START";
        case CMD_LOG_STOP:      return "LOG_STOP";
        case CMD_SET_CON_MODE:  return "SET_CON_MODE";
        case CMD_SET_PYRO_MODE: return "SET_PYRO_MODE";
        case CMD_TRIG_PYRO:     return "TRIG_PYRO";
        default:                return "UNKNOWN";
    }
}

bool validPyroMode(uint8_t m)
{
    return m == PYRO_1 || m == PYRO_2 || m == PYRO_3 || m == PYRO_4 ||
           m == PYRO_12 || m == PYRO_34 || m == PYRO_12_34;
}

// integer fixed-point print, e.g. (123456789, 1e7) -> 12.3456789
void printScaled(int32_t v, uint32_t div)
{
    bool     neg = v < 0;
    uint32_t a   = neg ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    if (neg) Serial.print('-');
    Serial.print(a / div);
    Serial.print('.');
    uint32_t f = a % div;
    for (uint32_t d = div / 10; d > 1 && f < d; d /= 10)
        Serial.print('0');
    Serial.print(f);
}

// prints |b0|b1|b2|b3|  (channel 1 first)
void printPyroBits(uint8_t nibble)
{
    Serial.print('|');
    for (uint8_t i = 0; i < 4; i++)
    {
        Serial.print((nibble >> i) & 1);
        Serial.print('|');
    }
}

void printCmdDesc(const ack_pkt_t &c)
{
    Serial.print(cmdName(c.cmd_type));
    if (c.cmd_type == CMD_SET_CON_MODE || c.cmd_type == CMD_SET_PYRO_MODE)
    {
        Serial.print(F(" param="));
        Serial.print(c.cmd_param);
    }
}

// "0b0011" -> binary, otherwise decimal
bool parseParam(const String &s, uint8_t &out)
{
    if (s.length() == 0) return false;
    long v = 0;
    if (s.startsWith("0b"))
    {
        String b = s.substring(2);
        if (b.length() == 0 || b.length() > 8) return false;
        for (unsigned i = 0; i < b.length(); i++)
        {
            if (b[i] != '0' && b[i] != '1') return false;
            v = (v << 1) | (b[i] - '0');
        }
    }
    else
    {
        for (unsigned i = 0; i < s.length(); i++)
            if (!isDigit(s[i])) return false;
        v = s.toInt();
    }
    if (v < 0 || v > 255) return false;
    out = (uint8_t)v;
    return true;
}

bool enqueue(uint8_t type, uint8_t param)
{
    if (cmd_que_count >= CMD_QUE_SIZE)
    {
        Serial.println(F("queue full, command rejected"));
        return false;
    }
    ack_pkt_t &c = cmd_que[cmd_que_count];
    memset(&c, 0, sizeof(c));
    c.flags     = 0x01;
    c.cmd_type  = type;
    c.cmd_param = param;
    cmd_sent[cmd_que_count] = 0;
    cmd_que_count++;

    printCmdDesc(c);
    Serial.print(F(" added to queue, queue size: "));
    Serial.println(cmd_que_count);
    return true;
}

// FIFO pop
void dequeue()
{
    for (uint8_t i = 1; i < cmd_que_count; i++)
    {
        cmd_que[i - 1]  = cmd_que[i];
        cmd_sent[i - 1] = cmd_sent[i];
    }
    if (cmd_que_count) cmd_que_count--;
}

void printHelp()
{
    Serial.println(F("--- HELP ---"));
    Serial.println(F("ON / OFF      green LED blink on / off"));
    Serial.println(F("LOG / SLG     start / stop logging"));
    Serial.println(F("CON <mode>    set pyro continuity-check mode"));
    Serial.println(F("PYM <mode>    set pyro fire mode"));
    Serial.println(F("FIRE          queue pyro trigger (sent immediately, once)"));
    Serial.println(F("<mode> = decimal or 0b binary, valid values:"));
    Serial.println(F("  1=P1  2=P2  4=P3  8=P4  3=P1+2  12=P3+4  15=all"));
    Serial.println(F("  e.g.  CON 3   PYM 0b1100"));
    Serial.println(F("Commands resend every cycle until the flight confirms (max 10)."));
    Serial.println(F("TRIG_PYRO is sent once, never retried."));
    Serial.println(F("------------"));
}

// ---------- setup ----------

void setup()
{
    pinMode(RF95_RST, OUTPUT);
    digitalWrite(RF95_RST, HIGH);
    delay(100);
    digitalWrite(RF95_RST, LOW);
    delay(10);
    digitalWrite(RF95_RST, HIGH);
    delay(10);

    Serial.begin(115200);
    while (!Serial);

    if (!rf95.init())
    {
        Serial.println(F("LoRa init failed"));
        while (1);
    }

    rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println(F("setFrequency failed"));
        while (1);
    }

    Serial.println(F("Receiver ready"));
    Serial.print(F("Expecting packet size: "));
    Serial.println(sizeof(telemetry_pkt_t));
    Serial.println(F("Type HELP for commands"));
}

// ---------- telemetry print ----------

void printPacket(const telemetry_pkt_t &pkt)
{
    Serial.println(F("--- Packet ---"));
    Serial.print(F("time:      ")); Serial.print(pkt.time);
    Serial.print(F(", utc:       ")); Serial.println(pkt.utc_time);
    Serial.print(F("lat:       ")); printScaled(pkt.lat, LATLON_DIV);
    Serial.print(F(", lon:       ")); printScaled(pkt.lon, LATLON_DIV);
    Serial.println();
    Serial.print(F("gps_alt:   ")); printScaled(pkt.gps_alt, GPS_ALT_DIV);
    Serial.print(F(" m, baro_alt:  ")); printScaled(pkt.baro_alt, BARO_ALT_DIV);
    Serial.println(F(" m"));
    Serial.print(F("vx vy vz:  "));
    Serial.print(pkt.vx); Serial.print(' ');
    Serial.print(pkt.vy); Serial.print(' ');
    Serial.println(pkt.vz);
    Serial.print(F("roll:      ")); Serial.print(pkt.roll);
    Serial.print(F(", pitch:     ")); Serial.print(pkt.pitch);
    Serial.print(F(", yaw:       ")); Serial.println(pkt.yaw);
    Serial.print(F("pressure:  ")); Serial.println(pkt.pressure);

    Serial.print(F("accel:     "));
    Serial.print(pkt.ax); Serial.print(' ');
    Serial.print(pkt.ay); Serial.print(' ');
    Serial.println(pkt.az);

    Serial.print(F("gyro:      "));
    Serial.print(pkt.gx); Serial.print(' ');
    Serial.print(pkt.gy); Serial.print(' ');
    Serial.println(pkt.gz);

    Serial.print(F("high_g:    "));
    Serial.print(pkt.hx); Serial.print(' ');
    Serial.print(pkt.hy); Serial.print(' ');
    Serial.println(pkt.hz);

    Serial.print(F("temp:      ")); Serial.print(pkt.temp);
    Serial.print(F(", v_batt:    ")); Serial.println(pkt.v_batt);

    Serial.print(F("state:     ")); Serial.print(pkt.state >> 5);
    Serial.print(F(", sats:      ")); Serial.println(pkt.state & 0x1F);

    Serial.print(F("pyro (1-4) cont: "));  printPyroBits(pkt.pyro_state & 0x0F);
    Serial.print(F("  state: "));          printPyroBits(pkt.pyro_state >> 4);
    Serial.println();

    Serial.print(F("RSSI:      ")); Serial.print(rf95.lastRssi());
    Serial.print(F(" (gnd), "));    Serial.print((int8_t)pkt.RSSI);
    Serial.println(F(" (onboard)"));

    Serial.print(F("Flight no: ")); Serial.println(pkt.flight_number);

    Serial.print(F("Logging:   ")); Serial.println((pkt.flags & FLG_LOGGING) ? 1 : 0);
    Serial.print(F("Transfer:  "));
    Serial.print(F("in progress : ")); Serial.print((pkt.flags & FLG_XFER_ACTIVE) ? 1 : 0);
    Serial.print(F(" , transferred : ")); Serial.print((pkt.flags & FLG_XFER_DONE) ? 1 : 0);
    Serial.print(F(" , error : ")); Serial.println((pkt.error_code & ERR_XFER) ? 1 : 0);

    Serial.print(F("Q drops:   "));
    Serial.print(F("data : ")); Serial.print((pkt.flags & FLG_DATA_DROP) ? 1 : 0);
    Serial.print(F(" , sd : ")); Serial.println((pkt.flags & FLG_SD_DROP) ? 1 : 0);

    Serial.print(F("GPS fix:   "));
    Serial.print(F("ok : "));  Serial.print((pkt.flags & FLG_GPS_FIX_OK) ? 1 : 0);
    Serial.print(F(" , 3D : ")); Serial.println((pkt.flags & FLG_GPS_FIX_3D) ? 1 : 0);

    Serial.print(F("Errors:    "));
    Serial.print(F("IMU : "));      Serial.print((pkt.error_code & ERR_IMU)   ? 1 : 0);
    Serial.print(F(" , BARO : "));  Serial.print((pkt.error_code & ERR_BARO)  ? 1 : 0);
    Serial.print(F(" , GPS : "));   Serial.print((pkt.error_code & ERR_GPS)   ? 1 : 0);
    Serial.print(F(" , GSM : "));   Serial.print((pkt.error_code & ERR_GSM)   ? 1 : 0);
    Serial.print(F(" , SD : "));    Serial.print((pkt.error_code & ERR_SD)    ? 1 : 0);
    Serial.print(F(" , FLASH : ")); Serial.println((pkt.error_code & ERR_FLASH) ? 1 : 0);

    Serial.print((pkt.error_code & ERR_ACK_RX) ? F("GOOD ACK ") : F("NO ACK "));
    Serial.println((pkt.flags & FLG_CMD_RX) ? F(" CMD Received") : F(" "));
    Serial.println(F("--------------"));
}

// ---------- LoRa ----------

void Handle_LoRa()
{
    if (!rf95.available()) return;

    telemetry_pkt_t pkt;
    uint8_t len = sizeof(pkt);

    if (!rf95.recv((uint8_t*)&pkt, &len))
    {
        Serial.println(F("recv failed"));
        return;
    }

    if (len != sizeof(telemetry_pkt_t))
    {
        Serial.print(F("Size mismatch! got="));
        Serial.print(len);
        Serial.print(F(" expected="));
        Serial.println(sizeof(telemetry_pkt_t));
        return;
    }

    printPacket(pkt);

    bool cmd_rx = pkt.flags & FLG_CMD_RX;
    bool ack_rx = pkt.error_code & ERR_ACK_RX;

    if (sent_something && !ack_rx)
        Serial.println(F("[LINK] flight did not receive our last transmission"));

    // outcome of the command sent last cycle
    if (cmd_que_count > 0 && cmd_sent[0] > 0)
    {
        if (cmd_rx)
        {
            Serial.print(F("[CMD] "));
            printCmdDesc(cmd_que[0]);
            Serial.print(F(" confirmed after "));
            Serial.print(cmd_sent[0]);
            Serial.println(F(" attempt(s)"));
            dequeue();
        }
        else if (cmd_que[0].cmd_type == CMD_TRIG_PYRO)
        {
            Serial.println(F("[CMD] TRIG_PYRO not confirmed, NOT retrying - check pyro state above"));
            dequeue();
        }
        else if (cmd_sent[0] >= MAX_RETRIES)
        {
            Serial.print(F("[CMD] "));
            printCmdDesc(cmd_que[0]);
            Serial.print(F(" no confirmation after "));
            Serial.print(cmd_sent[0]);
            Serial.println(F(" attempts, dropped"));
            dequeue();
        }
        else
        {
            Serial.print(F("[CMD] "));
            printCmdDesc(cmd_que[0]);
            Serial.println(F(" not confirmed, resending"));
        }
    }

    // transmit
    if (cmd_que_count == 0)
    {
        ack_pkt_t ack;
        memset(&ack, 0, sizeof(ack));
        rf95.send((uint8_t*)&ack, sizeof(ack));
        rf95.waitPacketSent();
        sent_something = true;
    }
    else
    {
        rf95.send((uint8_t*)&cmd_que[0], sizeof(ack_pkt_t));
        rf95.waitPacketSent();
        cmd_sent[0]++;
        sent_something = true;

        Serial.print(F("command sent: "));
        printCmdDesc(cmd_que[0]);
        Serial.print(F(" type=0x"));  Serial.print(cmd_que[0].cmd_type, HEX);
        Serial.print(F(" param=0x")); Serial.print(cmd_que[0].cmd_param, HEX);
        Serial.print(F(" attempt="));  Serial.print(cmd_sent[0]);
        Serial.print(F(" queue="));    Serial.println(cmd_que_count);
    }
}

// ---------- serial ----------

void Handle_Serial()
{
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    String cmd = line;
    String arg = "";
    int sp = line.indexOf(' ');
    if (sp > 0)
    {
        cmd = line.substring(0, sp);
        arg = line.substring(sp + 1);
        arg.trim();
    }
    cmd.toUpperCase();
    arg.toLowerCase();

    if      (cmd == "HELP" || cmd == "?") printHelp();
    else if (cmd == "ON")   enqueue(CMD_LED_BLINK, 0);
    else if (cmd == "OFF")  enqueue(CMD_LED_OFF,   0);
    else if (cmd == "LOG")  enqueue(CMD_LOG_START, 0);
    else if (cmd == "SLG")  enqueue(CMD_LOG_STOP,  0);
    else if (cmd == "FIRE") enqueue(CMD_TRIG_PYRO, 0);
    else if (cmd == "CON" || cmd == "PYM")
    {
        uint8_t mode;
        if (!parseParam(arg, mode) || !validPyroMode(mode))
        {
            Serial.println(F("invalid mode, valid: 1,2,4,8,3,12,15 (type HELP)"));
            return;
        }
        enqueue(cmd == "CON" ? CMD_SET_CON_MODE : CMD_SET_PYRO_MODE, mode);
    }
    else
    {
        Serial.println(F("unknown command, type HELP"));
    }
}

void loop()
{
    Handle_LoRa();
    Handle_Serial();
}