#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <RH_RF95.h>
#include <RHHardwareSPI1.h>

#include <LittleFS.h>
#include <SD.h>


#define DEBUG_MSG_LEN 80
#define DEBUG_QUE_LEN 16

typedef char Debug_Msg[DEBUG_MSG_LEN];
QueueHandle_t Debug_Queue;

static LittleFS_QSPI lfs;
static File          log_file;
static File          SD_file;
static bool          lfs_ready = false;
static uint32_t      flight_count = 0;

char filename[32] = {0};



bool flash_init();



#define LED_RED    10
#define LED_GREEN   9
#define LED_BLUE    8


#define LORA_MISO 39
#define LORA_CS  32
#define LORA_RST 41
#define LORA_IRQ 14
#define LORA_FREQ 866.0
#define LORA_POWER_DB 23


RH_RF95 rf95(LORA_CS, LORA_IRQ, hardware_spi1);

#pragma pack(push, 1)
typedef struct
{
    uint32_t time;
    int32_t  lat, lon;
    float    gps_alt, baro_alt;
    float    vx, vy, vz;
    float    roll, pitch, yaw;
    float    pressure;    

    
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;

    
    uint8_t   temp;
    uint8_t   v_batt;      
    uint8_t   state;
    uint8_t   error_code; 
    uint8_t   pyro_state;
    uint8_t    RSSI;

    uint16_t  CRC16;


} telemetry_pkt_t;

#define CMD_NONE            0x00
#define CMD_LED_BLINK       0x01
#define CMD_LED_OFF         0x02
#define CMD_LOG_START       0x03
#define CMD_LOG_STOP        0x04
#define CMD_TRIG_PYRO       0x05  

typedef struct {
    uint8_t flags;
    uint8_t cmd_type;
    uint8_t cmd_param;
    uint16_t CRC16;
} ack_pkt_t;

#pragma pack(pop)

#define GY_91       SPI
#define IMU_CS      37
#define BMP_CS      36

#define ACCEL_H     Wire1
#define ADS_ADDR     0x48
static constexpr uint16_t CFG_BASE =
    (1     << 15) |   // OS: start conversion
    (0b001 <<  9) |   // PGA: ±4.096V
    (1     <<  8) |   // MODE: single-shot
    (0b111 <<  5) |   // DR: 860 SPS
    0x03;             // comparator disable (COMP_QUE=11 disables comparator)
uint16_t cfg_x = CFG_BASE | (0b100 << 12);  // MUX = AIN0 vs GND
uint16_t cfg_y = CFG_BASE | (0b101 << 12);
uint16_t cfg_z = CFG_BASE | (0b110 << 12);

typedef struct 
{
    uint16_t T1, T2, T3;
    uint16_t P1, P2, P3, P4, P5, P6, P7, P8, P9;

}bmp_cal_data_t;

bmp_cal_data_t cal;

#define LOG_TYPE_IMU  0x01
#define LOG_TYPE_BARO 0x02
#define LOG_TYPE_ADXL 0x03

#define DATA_QUE_LEN 128

#pragma pack(push, 1)
typedef struct 
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

typedef struct 
{
    float pressure;
    float temp;
} baro_data_t;

typedef struct 
{
    int16_t ax, ay, az;

} adxl_data_t;

typedef struct 
{
   uint8_t type;
   uint32_t time; //ms
   union 
   {  
      imu_data_t  imu;
      baro_data_t baro;
      adxl_data_t adxl;
   };
} Data_t;
#pragma pack(pop)



QueueHandle_t Data_Queue;
SemaphoreHandle_t SPI_Mutex;

static const SPISettings spi_20M(20000000, MSBFIRST, SPI_MODE0);
static const SPISettings spi_10M(10000000, MSBFIRST, SPI_MODE0);
static const SPISettings spi_1M(1000000, MSBFIRST, SPI_MODE0);


static void spi_reg_write(uint8_t reg, uint8_t val,uint8_t cs_pin);
static uint8_t spi_reg_read(uint8_t reg,uint8_t cs_pin);
static void spi_reg_read_burst_imu(uint8_t reg, uint8_t *buf, size_t len);
static void spi_reg_read_burst_bmp(uint8_t reg, uint8_t *buf, size_t len);

bool IMU_Init();
bool BARO_Init();
bool ADXL_Init();
Data_t Read_IMU();
Data_t Read_Baro();
Data_t Read_ADXL();





#define CMD_QUE_LEN 8
QueueHandle_t Cmd_Queue;

volatile bool Green_Blink = false;
volatile bool Log_Enabled = false;
volatile bool Pyro_Triggered = false;
volatile bool File_Transferred = false;

#define RX_TIMEOUT_MS 2000


void debugPrint(const char *fmt , ...);

void Blink_Red(void *pvParameters)
{
    pinMode(LED_RED, OUTPUT);
    
    TickType_t lastWake = xTaskGetTickCount();

    while (1)
    {
        digitalToggle(LED_RED);
        debugPrint("Red toggled %d \n" ,2);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));

    }
    
}

void Blink_Green(void *pvParameters)
{
    pinMode(LED_GREEN, OUTPUT);
    vTaskDelay(pdMS_TO_TICKS(1000));
    TickType_t lastWake = xTaskGetTickCount();
    while (1)
    {
        if(Green_Blink)
        {
        digitalToggle(LED_GREEN);
        }
        else
        {
            digitalWrite(LED_GREEN, LOW);
        }
        debugPrint("Green toggled %d \n" ,5);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
    
}

void debugPrint(const char *fmt , ...)
{
    char buf[DEBUG_MSG_LEN];
    va_list args;
    va_start(args, fmt);
    
    vsnprintf(buf, DEBUG_MSG_LEN, fmt ,args);

    va_end(args);
    
    xQueueSend(Debug_Queue, &buf, 0); 
}

void DebugTask(void *pvParameters)
{
    
    Debug_Msg buf;
    while(1)
    {
        if(xQueueReceive(Debug_Queue, &buf, portMAX_DELAY))
        {
            Serial.println(buf);
        }
    }
}

int Lora_Init()
{
    SPI1.setMISO(39);
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(LORA_RST, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    if( !rf95.init())
    {
        debugPrint("LoRa init failed \n");
        return -1;
    }

    rf95.setFrequency(LORA_FREQ);
    rf95.setTxPower(LORA_POWER_DB, false);
    rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
    
    debugPrint("LoRa initialized ! \n");
    return 0;

}

void LoRa_Task(void *pvParameters)
{
    Lora_Init();

    telemetry_pkt_t pkt = {0};
    uint32_t count = 0;
    bool CMD_Recieved_Last_Cycle = false;
    bool ACK_Recieved_Last_Cycle = false;
    

    while(1)
    {
        pkt.time = millis();
        pkt.error_code &= 0x3F;
        pkt.error_code |= ((ACK_Recieved_Last_Cycle << 7) | (CMD_Recieved_Last_Cycle << 6));
        ACK_Recieved_Last_Cycle = false;
        CMD_Recieved_Last_Cycle = false;
        rf95.send((uint8_t*)&pkt, sizeof(pkt));
        rf95.waitPacketSent();
        count++;
        debugPrint("packet sent no :%d ,time %d = \n", count,millis()-pkt.time);
        vTaskDelay(pdMS_TO_TICKS(100));
        bool channel_active = rf95.isChannelActive();

        if(channel_active)
        {
            debugPrint("attempting to receive \n");
            uint32_t Close_Time  = millis() + RX_TIMEOUT_MS;

            bool got_packet = false;

            
            while(millis() < Close_Time)
            {
                if(rf95.available())
                {
                    ack_pkt_t rx_pkt;
                    uint8_t len = sizeof(rx_pkt);
                    if(rf95.recv((uint8_t*)&rx_pkt, &len) && len == sizeof(rx_pkt))
                    {
                        ACK_Recieved_Last_Cycle = true;
                        if(rx_pkt.flags & 0x01)
                        {
                            if(xQueueSend(Cmd_Queue,&rx_pkt ,0) == pdTRUE)
                            { 
                                CMD_Recieved_Last_Cycle = true;
                                debugPrint("CMD received: type=0x%02X param=0x%02X rssi=%d",
                                           rx_pkt.cmd_type, rx_pkt.cmd_param, rf95.lastRssi());

                            }
                            else
                            {
                                debugPrint("CMD queue full !");
                            }
                        }
                        got_packet = true;

                    }
                    else
                    {
                        debugPrint("RX: size mismatch (got %d, want %d)", len, sizeof(ack_pkt_t));
                    }
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            
            }
            if(!got_packet)
            {
                debugPrint("RX: timeout");
                ACK_Recieved_Last_Cycle = false;
                CMD_Recieved_Last_Cycle = false;
            }
            else
            {
                debugPrint("RX: success");
                
            }
        }
        else
        {
            ACK_Recieved_Last_Cycle = false;
            CMD_Recieved_Last_Cycle = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    
    }

}

void Cmd_Task(void *pvParameters)
{
    ack_pkt_t cmd;
    while(1)
    {
        if(xQueueReceive(Cmd_Queue, &cmd, portMAX_DELAY))
        {
            if(!(cmd.flags &0x01))
            {
                debugPrint("ACK recieved");
                continue;
            }

            switch (cmd.cmd_type)
            {
                case CMD_LED_BLINK :
                    Green_Blink = true;
                    debugPrint("CMD: green blinking on");
                    break;

                case CMD_LED_OFF:
                    Green_Blink = false;
                    debugPrint("CMD: green blinking off");
                    break;

                case CMD_LOG_START:
                    Log_Enabled = true;
                    debugPrint("CMD: log start");
                    break;

                case CMD_LOG_STOP:
                    Log_Enabled = false;
                    debugPrint("CMD: log stop");
                    break;

                case CMD_TRIG_PYRO:
                    Pyro_Triggered = true;
                    debugPrint("CMD: pyro trigger");
                    break;

                default:
                    debugPrint("CMD: unknown type 0x%02X", cmd.cmd_type);
                    break;
           }
        }
    }
}

void IMU_Task(void *pvParameters)
{
    xSemaphoreTake(SPI_Mutex,portMAX_DELAY);
    if(!IMU_Init())
    {
        xSemaphoreGive(SPI_Mutex);
        debugPrint("IMU init failed, task exiting");
        vTaskDelete(NULL);
    }
    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {
        xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
        Data_t data = Read_IMU();
        xSemaphoreGive(SPI_Mutex);
        if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
        {
            debugPrint("Data queue full, dropping IMU data");
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
    }
}

void Baro_Task(void *pvParameters)
{
    xSemaphoreTake(SPI_Mutex,portMAX_DELAY);
    if(!BARO_Init())
    {
        xSemaphoreGive(SPI_Mutex);
        debugPrint("Barometer init failed, task exiting");
        vTaskDelete(NULL);
    }
    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {
        Data_t data = Read_Baro();
        if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
        {
            debugPrint("Data queue full, dropping baro data");
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void ADXL_Task(void *pvParameters)
{
    ADXL_Init();
    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {
        Data_t data = Read_ADXL();
        if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
        {
            debugPrint("Data queue full, dropping ADXL data");
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void Transfer_SD(void *pvParameters)
{
    if(!SD.begin(BUILTIN_SDCARD))
    {
        debugPrint("SD card not ready, cannot transfer log");
        vTaskDelete(NULL);
    }
    snprintf(filename, sizeof(filename), "/flight_%ld.bin", flight_count);
    SD_file = SD.open(filename, FILE_WRITE);
    if(!SD_file)
    {
        debugPrint("Failed to open file on SD card for writing");
        vTaskDelete(NULL);
    }

    log_file = lfs.open(filename, FILE_READ);
    if(!log_file)
    {
        debugPrint("Failed to read log file !");
        vTaskDelete(NULL);
    }
    uint8_t buf[512];
    while(log_file.available())
    {
        size_t to_read = min(sizeof(buf), log_file.available());
        log_file.read(buf, to_read);
        SD_file.write(buf, to_read);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    SD_file.flush();
    SD_file.close();
    log_file.close();
    debugPrint("Log file transferred to SD card as %s", filename);
    char index_buf[16];
    snprintf(index_buf, sizeof(index_buf), "%lu_%c\n", flight_count,'T');
    log_file = lfs.open("/INDEX.txt", FILE_WRITE);
    log_file.write(index_buf);
    log_file.flush();
    log_file.close();

    File_Transferred = true;
    flight_count++;

    lfs.remove(filename);

    snprintf(filename, sizeof(filename), "/flight_%ld.bin", flight_count);

    vTaskDelete(NULL);
}

void Data_Log_Task(void *pvParameters)
{

    if(!flash_init())
    {
        debugPrint("Flash init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    while(1)
    {
        if(Log_Enabled)
        {
            Data_t data;
            int count = 0;
            uint32_t lastSync = xTaskGetTickCount();
            while(Log_Enabled)
            {
                if(xQueueReceive(Data_Queue, &data, portMAX_DELAY) == pdTRUE)
                {   
                    log_file.write((const uint8_t*)&data, sizeof(data));
                    count++;

                    bool by_count = (count % 50 == 0);
                    bool by_time  = (millis() - lastSync >= 500);

                    if (by_count || by_time) 
                    {
                        log_file.flush();
                        lastSync = millis();
                        if (count % 2000 == 0)
                        debugPrint("[LOG] %lu records, q=%u",
                                   count, uxQueueMessagesWaiting(Data_Queue));
                    }
           // else
           // {
             //   vTaskDelay(pdMS_TO_TICKS(10));
           // }
                }

            }

            log_file.flush();
            log_file.close();
            snprintf(filename, sizeof(filename), "/flight_%ld.bin", flight_count);
            log_file = lfs.open(filename, FILE_WRITE);
            char index_buf[16];
            snprintf(index_buf, sizeof(index_buf), "%lu_%c\n", flight_count,'N');
            log_file.write(index_buf);
            log_file.flush();
            log_file.close();
            File_Transferred = false;
            debugPrint("Logging stopped");

            xTaskCreate(Transfer_SD, "SD_TRANSFER", 4096, nullptr, 8, nullptr);
        }
        else
        vTaskDelay(pdMS_TO_TICKS(100));

    }
        
} 








void setup()
{
  Serial.begin(115200);

  delay(100);


  Debug_Queue = xQueueCreate(DEBUG_QUE_LEN, sizeof(Debug_Msg));
  Cmd_Queue = xQueueCreate(CMD_QUE_LEN, sizeof(ack_pkt_t));
  Data_Queue = xQueueCreate(DATA_QUE_LEN, sizeof(Data_t));

  SPI_Mutex = xSemaphoreCreateMutex();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  xTaskCreate(Blink_Green   , "GREEN"   , 512   , nullptr, 1, nullptr);
  xTaskCreate(Blink_Red     , "RED"     , 512   , nullptr, 1, nullptr);
  xTaskCreate(DebugTask     , "DEBUG"   , 512   , nullptr, 1, nullptr);
  xTaskCreate(LoRa_Task     , "LORA"    , 2048  , nullptr, 3, nullptr);
  xTaskCreate(Cmd_Task      , "CMD"     , 1024  , nullptr, 2, nullptr);
  //xTaskCreate(IMU_Task      , "IMU"     , 1024  , nullptr, 7, nullptr);
  //xTaskCreate(Baro_Task     , "BARO"    , 1024  , nullptr, 6, nullptr);
  xTaskCreate(ADXL_Task     , "ADXL"    , 1024  , nullptr, 5, nullptr);
  xTaskCreate(Data_Log_Task , "LOG"     , 8192  , nullptr, 4, nullptr);

  Serial.println("INIT");

  vTaskStartScheduler();
}

void loop()
{
  
} 



static void spi_reg_write(uint8_t reg, uint8_t val,uint8_t cs_pin)
{
    GY_91.beginTransaction(spi_1M);
    digitalWrite(cs_pin, LOW);
    GY_91.transfer(reg & 0x7F); // unSet MSB for write operation
    GY_91.transfer(val);
    digitalWrite(cs_pin, HIGH);
    GY_91.endTransaction();

}
static uint8_t spi_reg_read(uint8_t reg,uint8_t cs_pin)
{
    uint8_t val;
    GY_91.beginTransaction(spi_1M);
    digitalWrite(cs_pin, LOW);
    GY_91.transfer(reg | 0x80); // Set MSB for read operation
    val = GY_91.transfer(0x00); // Dummy byte to read data
    digitalWrite(cs_pin, HIGH);
    GY_91.endTransaction();
    return val;

}
static void spi_reg_read_burst_imu(uint8_t reg, uint8_t *buf, size_t len)
{
    GY_91.beginTransaction(spi_20M);
    digitalWrite(IMU_CS, LOW);
    GY_91.transfer(reg | 0x80); // Set MSB for read operation
    for(size_t i = 0; i < len; i++)
    {
        buf[i] = GY_91.transfer(0x00); // Dummy byte to read data
    }
    digitalWrite(IMU_CS, HIGH);
    GY_91.endTransaction();
}
static void spi_reg_read_burst_bmp(uint8_t reg, uint8_t *buf, size_t len)
{
    GY_91.beginTransaction(spi_10M);
    digitalWrite(BMP_CS, LOW);
    GY_91.transfer(reg | 0x80); // Set MSB for read operation
    for(size_t i = 0; i < len; i++)
    {
        buf[i] = GY_91.transfer(0x00); // Dummy byte to read data
    }
    digitalWrite(BMP_CS, HIGH);
    GY_91.endTransaction();
}

bool IMU_Init()
{
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    spi_reg_write(0x6B, 0x80 , IMU_CS); // Reset device
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t who_am_i = spi_reg_read(0x75, IMU_CS);
    if(who_am_i != 0x70)
    {
        debugPrint("IMU init failed: WHO_AM_I = 0x%02X", who_am_i);
        return false;
    }
    debugPrint("IMU found ID : 0x%02X", who_am_i);

    spi_reg_write(0x6B, 0x01, IMU_CS); // pwr management, clock source = gyro X, sleep disabled
    spi_reg_write(0x1A, 0x03, IMU_CS); // config, DLPF 44Hz
    spi_reg_write(0x1B, 0x18, IMU_CS); // gyro config, +-2000dps
    spi_reg_write(0x1C, 0x18, IMU_CS); // accel config, +-16g
    spi_reg_write(0x19, 0x04, IMU_CS); // 1000/4+1 = 200hz
    debugPrint("IMU initialized successfully");
    return true;


}
bool BARO_Init()
{
    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    spi_reg_write(0xE0, 0xB6, BMP_CS); // reset
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t id = spi_reg_read(0xD0, BMP_CS);
    if(id != 0x60)
    {
        debugPrint("Barometer not found: ID = 0x%02X", id);
        return false;
    }
    debugPrint("Barometer found ID: 0x%02X", id);
    return true;
    uint8_t cb[24];
    spi_reg_read_burst_bmp(0x88, cb, 24);

    auto u16 = [&](int i) { return (uint16_t)(cb[i] | (cb[i+1] << 8)); };
    auto s16 = [&](int i) { return  (int16_t)(cb[i] | (cb[i+1] << 8)); };

    cal.T1 = u16(0);   // unsigned
    cal.T2 = s16(2);   // signed
    cal.T3 = s16(4);   // signed
    cal.P1 = u16(6);   // unsigned
    cal.P2 = s16(8);   // signed 
    cal.P3 = s16(10);
    cal.P4 = s16(12);
    cal.P5 = s16(14);
    cal.P6 = s16(16);
    cal.P7 = s16(18);
    cal.P8 = s16(20);
    cal.P9 = s16(22);

    spi_reg_write(0xF4, 0x57, BMP_CS);  //0x57 = 0b01010111:    set freq osrs p = 2x :0x37
    spi_reg_write(0xF5, 0x08, BMP_CS);  //0xA0 = 0b10100000: IIR off, 20hz sampling
    // rn ~ 25hz
    debugPrint("Barometer initialized successfully");
    return true;


}

bool ADXL_Init()
{
    ACCEL_H.begin();
    ACCEL_H.setClock(400000);

    ACCEL_H.beginTransmission(ADS_ADDR);
    ACCEL_H.write(0x01);
    uint8_t err = ACCEL_H.endTransmission();
    if(err != 0)
    {
        debugPrint("ADS init failed: I2C error %d", err);
        return false;
    }
    return true;
}

Data_t Read_IMU()
{
    uint8_t buf[14];

    Data_t data;
    data.type = LOG_TYPE_IMU; 
    data.time = millis();

    spi_reg_read_burst_imu(0x3B, buf, 14);

    data.imu.ax = (int16_t)((buf[0] << 8) | buf[1]);
    data.imu.ay = (int16_t)((buf[2] << 8) | buf[3]);
    data.imu.az = (int16_t)((buf[4] << 8) | buf[5]);// BYTES 6 AND 7 ARE TEMP, NOT USED CURRENTLY
    data.imu.gx = (int16_t)((buf[8] << 8) | buf[9]);
    data.imu.gy = (int16_t)((buf[10] << 8) | buf[11]);
    data.imu.gz = (int16_t)((buf[12] << 8) | buf[13]);
    return data;
}
Data_t Read_Baro()
{
    uint8_t raw[6];

    Data_t data = {0};
    data.type = LOG_TYPE_BARO;
    data.time = millis();
    spi_reg_read_burst_bmp(0xF7, raw, 6);
    int32_t adc_P = ((int32_t)raw[0] << 12) |
                    ((int32_t)raw[1] <<  4) |
                    (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) |
                    ((int32_t)raw[4] <<  4) |
                    (raw[5] >> 4);

    static int32_t t_fine;

    int32_t var_1, var_2;
    var_1 = ((((adc_T >> 3) - ((int32_t)cal.T1 << 1))) * cal.T2) >> 11;
    var_2 = (((((adc_T >> 4) - (int32_t)cal.T1) *
              ((adc_T >> 4) - (int32_t)cal.T1)) >> 12) * cal.T3) >> 14;
    t_fine = var_1 + var_2;

    float temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t var1, var2, p;
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)cal.P6;
    var2 += (var1 * (int64_t)cal.P5) << 17;
    var2 += ((int64_t)cal.P4) << 35;
    var1  = ((var1 * var1 * (int64_t)cal.P3) >> 8) +
            ((var1 * (int64_t)cal.P2) << 12);
    var1  = ((((int64_t)1 << 47) + var1) * (int64_t)cal.P1) >> 33;
    if (var1 == 0) 
    {
        debugPrint("Barometer error: var1 is zero");
        return data;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)cal.P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)cal.P7 << 4);
    
    float pressure = (float)(uint32_t)p / 25600.0f;

    
    data.baro.pressure = pressure;
    data.baro.temp = temperature;
    return data;
}
Data_t Read_ADXL()
{
    Data_t data;
    data.type = LOG_TYPE_ADXL;
    data.time = millis();
    int16_t *a[3] = {&data.adxl.ax, &data.adxl.ay, &data.adxl.az};

    for(int i = 0; i < 3; i++)
    {
        uint16_t cfg = CFG_BASE | ((0b100 + i) << 12);

        ACCEL_H.beginTransmission(ADS_ADDR);
        ACCEL_H.write(0x01); // config register
        ACCEL_H.write((uint8_t)(cfg >> 8));
        ACCEL_H.write((uint8_t)(cfg & 0xFF));
        ACCEL_H.endTransmission();

        vTaskDelay(pdMS_TO_TICKS(2));

        ACCEL_H.beginTransmission(ADS_ADDR);
        ACCEL_H.write(0x00); // conversion register
        ACCEL_H.endTransmission(false);
        ACCEL_H.requestFrom(ADS_ADDR, 2);
        uint8_t high_byte = ACCEL_H.read();
        uint8_t low_byte = ACCEL_H.read();

        *a[i] = (int16_t)((high_byte << 8) | low_byte);
    }
    return data;    
}

bool flash_init()
{
    if(!lfs.begin())
    {
        debugPrint("LittleFS init failed !");
        return false;
    }
    debugPrint("LittleFS initialized successfully");
    log_file = lfs.open("/INDEX.txt", FILE_READ);
    flight_count = log_file.parseInt();
    log_file.read();
    char c = log_file.read();
    log_file.close();

    if(c == 'T')
    {
    snprintf(filename, sizeof(filename), "/flight_%ld.bin", flight_count);
    log_file = lfs.open(filename, FILE_WRITE);
    if(!log_file)
    {
        debugPrint("Failed to create log file !");
        return false;
    }
    lfs_ready = true;
    debugPrint("Log file created successfully");
    return true;
   }
   else
   {
    debugPrint("Current log hasn't been transferred");
    xTaskCreate(Transfer_SD, "SD_TRANSFER", 4096, nullptr, 8, nullptr);
    return true;
   }

}