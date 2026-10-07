


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//     fix bmp 280
// to do: fix adxl offset 
//        fix logging system
//        add gps
//        add gsm
//        test pyro circuit  
//        recovery logic


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                                                                              BOARD ORIENTATION
//                                                                                                ^ +z (UP)
//                                                                                                |    
//                                                                                                |
//                                                                                                |   
//                                                                                                . -------------- >+Y
//                                                                                                + X (TOWARDS YOU)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

#include "GPS.h"
#include "PYRO.h"


#define DEBUG_MSG_LEN 80
#define DEBUG_QUE_LEN 16

typedef char Debug_Msg[DEBUG_MSG_LEN];
QueueHandle_t Debug_Queue;

static LittleFS_QSPI lfs;
static File          log_file;
static File          SD_file;
static bool          SD_Ready = false;
static uint32_t      flight_count = 0;

char Filename_Flash[32] = {0};
char Filename_SD[32] = {0};


bool flash_init();
bool SD_Init();


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
    uint32_t utc_time;
    int32_t  lat, lon;
    int32_t  gps_alt, baro_alt;
    float    vx, vy, vz;
    float    roll, pitch, yaw;
    float    pressure;    

    
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  hx, hy, hz;

    
    uint8_t   temp;
    uint8_t   v_batt;      
    uint8_t   state;
    uint8_t   error_code; 
    uint8_t   flags;
    uint8_t   flight_number;
    uint8_t   pyro_state;
    uint8_t   RSSI;

    uint16_t  CRC16;


} telemetry_pkt_t;
 // first 3 bits for state, next 5 bits for sats count

//       error codes     //
// bit 0 - IMU 
// bit 1 - BARO
// bit 2 - GPS
// bit 3 - GSM
// bit 4 - SD Card
// bit 5 - Flash
// bit 6 - file transfer error
// bit 7 - Ack received in last cycle
//

//      flags           //
//
//  bit 0 - file transfer in progress 
//  bit 1 - file transferred
//  bit 2 - logging enabled
//  bit 3 - Command received in last cycle
//  bit 4 - data que dropping packets
//  bit 5 - sd que dropping packets
//  bit 6 - GPS time lock
//  bit 7 - GPS pos lock
//


//      pyro states     //
// bit 0 - pyro 1 contuinutiy
// bit 1 - pyro 2 contuinutiy
// ..
// .
//bit 4 - pyro 1 
//bit 5 - pyro 2 
//..
//.

typedef struct {
    uint8_t flags;
    uint8_t cmd_type;
    uint8_t cmd_param;
 
 
    uint16_t CRC16;
} ack_pkt_t;

#pragma pack(pop)

volatile bool IMU_Error                     = false;
volatile bool BARO_Error                    = false;
volatile bool GPS_Error                     = false;
volatile bool GSM_Error                     = false;
volatile bool SD_Error                      = false;
volatile bool Flash_Error                   = false;
volatile bool File_Transfer_Error           = false;




#define CMD_NONE            0x00
#define CMD_LED_BLINK       0x01
#define CMD_LED_OFF         0x02
#define CMD_LOG_START       0x03
#define CMD_LOG_STOP        0x04
#define CMD_SET_CON_MODE    0x05
#define CMD_SET_PYRO_MODE   0x06
#define CMD_TRIG_PYRO       0x07  

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

typedef struct {
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
   } bmp_cal_data_t;

bmp_cal_data_t cal;

#define LOG_TYPE_IMU        0x01
#define LOG_TYPE_BARO       0x02
#define LOG_TYPE_ADXL       0x03
#define LOG_TYPE_GPS_POS    0x04
#define LOG_TYPE_GPS_VEL    0x05
#define LOG_TYPE_GPS_ACC    0x06
#define LOG_TYPE_GPS_LOW    0x07
#define LOG_TYPE_BOARD      0x08

#define DATA_QUE_LEN    512
#define SD_DATA_QUE_LEN 512

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
    uint8_t   cmd;
    uint8_t   cmd_param;
    uint8_t   v_batt;      
    uint8_t   state;
    uint8_t   error_code;
    uint8_t   flags; 
    uint8_t   pyro_state; // pyro continuity and trigger states
    uint8_t   RSSI;

} board_data_t;
 // first 3 bits for state, next 5 bits for sats count

//       error codes     //
// bit 0 - IMU 
// bit 1 - BARO
// bit 2 - GPS
// bit 3 - GSM
// bit 4 - SD Card
// bit 5 - Flash
// bit 6 - file transfer error
// bit 7 - Ack received in last cycle
//

//      flags           //
//
//  bit 0 - file transfer in progress 
//  bit 1 - file transferred
//  bit 2 - logging enabled
//  bit 3 - Command received in last cycle
//  bit 4 - data que dropping packets
//  bit 5 - sd que dropping packets
//  bit 6 - GPS time lock
//  bit 7 - GPS pos lock
//

//      pyro states     //
// bit 0 - pyro 1 contuinutiy
// bit 1 - pyro 2 contuinutiy
// ..
// .
//bit 4 - pyro 1 mode
//bit 5 - pyro 2 mode
//..
//.

typedef struct 
{
   uint8_t type;
   uint32_t time; //ms
   union 
   {  
        imu_data_t          imu;
        baro_data_t         baro;
        adxl_data_t         adxl;
        gps_pos_data_t      gps_pos;
        gps_vel_data_t      gps_vel;
        gps_accuracy_data_t gps_acc;
        gps_low_freq_data_t gps_low;
        board_data_t        board;
   };
} Data_t;
#pragma pack(pop)


volatile imu_data_t latest_imu_data;

volatile baro_data_t latest_baro_data;  

volatile adxl_data_t latest_adxl_data;

volatile gps_pos_data_t latest_gps_pos_data;

volatile gps_vel_data_t latest_gps_vel_data;

volatile gps_accuracy_data_t latest_gps_acc_data;

volatile gps_low_freq_data_t latest_gps_low_data;

volatile board_data_t latest_board_data;

imu_data_t imu_offsets = {0, 0, 0, 0, 0, 0};
adxl_data_t adxl_offsets = {13200, 13200, 13200};



QueueHandle_t Data_Queue;
QueueHandle_t SD_Data_Queue;
SemaphoreHandle_t SPI_Mutex;
SemaphoreHandle_t SD_Mutex;
SemaphoreHandle_t Flash_Mutex;

static const SPISettings spi_20M(20000000, MSBFIRST, SPI_MODE0);
static const SPISettings spi_10M(10000000, MSBFIRST, SPI_MODE0);
static const SPISettings spi_1M(1000000, MSBFIRST, SPI_MODE0);


static void spi_reg_write(uint8_t reg, uint8_t val,uint8_t cs_pin);
static uint8_t spi_reg_read(uint8_t reg,uint8_t cs_pin);
static void spi_reg_read_burst_imu(uint8_t reg, uint8_t *buf, size_t len);
static void spi_reg_read_burst_bmp(uint8_t reg, uint8_t *buf, size_t len);

bool Lora_Init();
bool GPS_Init();
bool IMU_Init();
bool BARO_Init();
bool ADXL_Init();

void Read_GPS();
Data_t Read_IMU();
Data_t Read_Baro();
Data_t Read_ADXL();


void Transfer_SD(void *pvParameters);





#define CMD_QUE_LEN 8
QueueHandle_t Cmd_Queue;

volatile bool Green_Blink                               = false;

volatile bool Log_Enabled                               = false;
volatile bool File_Transferred                          = false;
volatile bool File_Transfer_In_Progress                 = false;
volatile bool Data_Que_Pkt_Drop                         = false;
volatile bool SD_Que_Pkt_Drop                           = false;

////////////// copies for telemetry //////////////////////////////
volatile uint8_t Flags_Tele                             = 0;
volatile uint8_t Error_Code_Tele                        = 0;
volatile uint8_t CMD_Recieved_Tele                      = 0;
volatile uint8_t CMD_Param_Tele                         = 0;
/////////////////////////////////////////////////////////////////

volatile bool CMD_Recieved_Last_Cycle                   = false;
volatile bool ACK_Recieved_Last_Cycle                   = false;

#define RX_TIMEOUT_MS 2000


void debugPrint(const char *fmt , ...);

void Blink_Red(void *pvParameters)
{
    pinMode(LED_RED, OUTPUT);
    
    TickType_t lastWake = xTaskGetTickCount();

    while (1)
    {
        if(good_continuity)
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




void LoRa_Task(void *pvParameters)
{
    Lora_Init();

    telemetry_pkt_t pkt = {0};
    uint32_t count = 0;
    
    

    while(1)
    {
        pkt.time            = millis();
        pkt.error_code      = (uint8_t)(latest_board_data.error_code|Error_Code_Tele);

        pkt.flags           = (uint8_t)(latest_board_data.flags|Flags_Tele);

        Error_Code_Tele     = 0;
        Flags_Tele          = 0;

        pkt.flight_number   = flight_count;
        pkt.RSSI            = rf95.lastRssi();
        pkt.utc_time        = latest_gps_low_data.UTC_time;
        pkt.lat             = latest_gps_pos_data.lat;
        pkt.lon             = latest_gps_pos_data.lon;
        pkt.gps_alt         = latest_gps_pos_data.gps_alt;
        pkt.state           = latest_board_data.state;
        pkt.ax              = latest_imu_data.ax;
        pkt.ay              = latest_imu_data.ay;
        pkt.az              = latest_imu_data.az;    
        pkt.gx              = latest_imu_data.gx;
        pkt.gy              = latest_imu_data.gy;
        pkt.gz              = latest_imu_data.gz;
        pkt.hx              = latest_adxl_data.ax;
        pkt.hy              = latest_adxl_data.ay;
        pkt.hz              = latest_adxl_data.az;
        pkt.pressure        = latest_baro_data.pressure;
        pkt.temp            = latest_baro_data.temp;
        // pkt.pyro_state

        
  

    
        debugPrint("%d %d %d",pkt.ax, pkt.ay, pkt.az);
        ACK_Recieved_Last_Cycle = false;
        CMD_Recieved_Last_Cycle = false;
        rf95.send((uint8_t*)&pkt, sizeof(pkt));
        rf95.waitPacketSent();
        count++;
        debugPrint("packet sent no :%d ,time %d = \n", count,millis()-pkt.time);

        CMD_Recieved_Tele = 0;
        CMD_Param_Tele    = 0;

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
                    CMD_Recieved_Tele = CMD_LED_BLINK;
                    debugPrint("CMD: green blinking on");
                    break;

                case CMD_LED_OFF:
                    Green_Blink = false;
                    CMD_Recieved_Tele = CMD_LED_OFF;
                    debugPrint("CMD: green blinking off");
                    break;

                case CMD_LOG_START:
                    CMD_Recieved_Tele = CMD_LOG_START;
                    if(File_Transferred)
                    {          
                        Log_Enabled = true;
                        debugPrint("CMD: log start");
                    }
                    else
                        debugPrint("CMD: log start ignored, file not transferred yet");
                    
                    break;

                case CMD_LOG_STOP:
                    CMD_Recieved_Tele = CMD_LOG_STOP;
                    Log_Enabled = false;
                    debugPrint("CMD: log stop");
                    break;

                case CMD_SET_CON_MODE:
                    CMD_Recieved_Last_Cycle =CMD_SET_CON_MODE;
                    CMD_Param_Tele = cmd.cmd_param;
                    Pyro_Con_Mode = cmd.cmd_param;
                    debugPrint("CMD: continuity mode : %b",Pyro_Con_Mode);
                    break;
                
                case CMD_SET_PYRO_MODE:
                    CMD_Recieved_Last_Cycle = CMD_SET_PYRO_MODE;
                    CMD_Param_Tele = cmd.cmd_param;
                    Pyro_Mode = cmd.cmd_param;
                    debugPrint("CMD: pyro mode : %b",Pyro_Mode);

                case CMD_TRIG_PYRO:
                    CMD_Recieved_Tele = CMD_TRIG_PYRO;
                    xTaskCreate(Pyro_Trigger_Task, "PYRO_TRIG"    , 512   , nullptr, 1, nullptr);
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
        IMU_Error = true;
        vTaskDelete(NULL);
    }
    xSemaphoreGive(SPI_Mutex);
    IMU_Error = false;

    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {

        Data_t data = Read_IMU();
        //debugPrint("IMU read: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d", data.imu.ax, data.imu.ay, data.imu.az, data.imu.gx, data.imu.gy, data.imu.gz);
        if(Log_Enabled)
        {
             if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
             {
                  debugPrint("Data queue full, dropping IMU data");
                  Data_Que_Pkt_Drop = true;
                                   
             }
             
        }

        taskENTER_CRITICAL();
        latest_imu_data = {data.imu.ax, data.imu.ay, data.imu.az,
                           data.imu.gx, data.imu.gy, data.imu.gz};
        taskEXIT_CRITICAL();


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
    xSemaphoreGive(SPI_Mutex);
    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {
        Data_t data = Read_Baro();
        if(Log_Enabled)
        {
            if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
            {
                debugPrint("Data queue full, dropping baro data");
                Data_Que_Pkt_Drop = true;
            }
        }
        taskENTER_CRITICAL();
        latest_baro_data = {data.baro.pressure, data.baro.temp};
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void ADXL_Task(void *pvParameters)
{
    if(!ADXL_Init())
    {
        debugPrint("ADXL init failed, task exiting");
        vTaskDelete(NULL);
    }

    TickType_t lastWake = xTaskGetTickCount();
    while(1)
    {
        Data_t data = Read_ADXL();
        if(Log_Enabled)
        {
            if (xQueueSend(Data_Queue, &data, 0) != pdTRUE) 
            {
                 debugPrint("Data queue full, dropping ADXL data");
                 Data_Que_Pkt_Drop = true;
            }
        }
        taskENTER_CRITICAL();
        latest_adxl_data = {data.adxl.ax, data.adxl.ay, data.adxl.az};
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void GPS_Task(void *pvParameters)
{
    debugPrint("Initializing GPS...");
    GPS_Init();

    TickType_t lastWake = xTaskGetTickCount();

    while(1)
    {
        Read_GPS();
        Data_t data_pos, data_vel, data_acc, data_low;
        data_pos.type               = LOG_TYPE_GPS_POS;
        data_vel.type               = LOG_TYPE_GPS_VEL;
        data_acc.type               = LOG_TYPE_GPS_ACC;
        data_low.type               = LOG_TYPE_GPS_LOW;
        data_pos.time               = millis();
        data_vel.time               = data_pos.time;
        data_acc.time               = data_pos.time; 
        data_low.time               = data_pos.time;

        data_pos.gps_pos.lat        = PVT_Data.lat;
        data_pos.gps_pos.lon        = PVT_Data.lon;    
        data_pos.gps_pos.gps_alt    = PVT_Data.height;

        data_vel.gps_vel.ground_velocity = PVT_Data.gSpeed;
        data_vel.gps_vel.vertical_velocity = PVT_Data.velD;
        data_vel.gps_vel.heading = PVT_Data.heading;

        data_acc.gps_acc.hAcc = PVT_Data.hAcc;
        data_acc.gps_acc.vAcc = PVT_Data.vAcc;
        data_acc.gps_acc.headAcc = PVT_Data.headAcc;

        data_low.gps_low.UTC_time = PVT_Data.iTOW;
        data_low.gps_low.hMSL = PVT_Data.hMSL;
        data_low.gps_low.numSV = PVT_Data.numSV;
        data_low.gps_low.valid = PVT_Data.valid;
        data_low.gps_low.flags = PVT_Data.flags;
        data_low.gps_low.fixType = PVT_Data.fixType;


        //debugPrint("GPS data: %d %d %d %ld", PVT_Data.hour, PVT_Data.minute, PVT_Data.second, PVT_Data.lat);

        taskENTER_CRITICAL();
        latest_gps_pos_data = data_pos.gps_pos;
        latest_gps_vel_data = data_vel.gps_vel;
        latest_gps_acc_data = data_acc.gps_acc;
        latest_gps_low_data = data_low.gps_low;
        latest_board_data.state &= 0xE0; // clear lower 5 bits
        latest_board_data.state |= PVT_Data.numSV ;
        taskEXIT_CRITICAL();


        if(Log_Enabled)
        {
            if (xQueueSend(Data_Queue, &data_pos, 0) != pdTRUE) 
            {
                 debugPrint("Data queue full, dropping GPS data");
                 Data_Que_Pkt_Drop = true;
            }
            if (xQueueSend(Data_Queue, &data_vel, 0) != pdTRUE) 
            {
                 debugPrint("Data queue full, dropping GPS data");
                 Data_Que_Pkt_Drop = true;
            }
            if (xQueueSend(Data_Queue, &data_acc, 0) != pdTRUE) 
            {
                 debugPrint("Data queue full, dropping GPS data");
                 Data_Que_Pkt_Drop = true;
            }
            if (xQueueSend(Data_Queue, &data_low, 0) != pdTRUE) 
            {
                 debugPrint("Data queue full, dropping GPS data");
                 Data_Que_Pkt_Drop = true;
            }
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

void Board_task(void *pvParameters)
{
    
    TickType_t lastWake = xTaskGetTickCount();

    while(1)
    {
        Data_t data;
        data.type = LOG_TYPE_BOARD;
        data.time = millis(); 
        data.board.error_code       =  (uint8_t)((ACK_Recieved_Last_Cycle << 7) | (File_Transfer_Error << 6)
                                        |(IMU_Error << 0) | (BARO_Error << 1) | (GPS_Error << 2) | (GSM_Error << 3) | (SD_Error << 4)| (Flash_Error << 5));

        data.board.flags            = (uint8_t)((File_Transfer_In_Progress<<0)|(File_Transferred <<1)|(Log_Enabled<<2)|(CMD_Recieved_Last_Cycle<<3)|(Data_Que_Pkt_Drop<<4)
                                        |(SD_Que_Pkt_Drop<<5)|((latest_gps_low_data.flags & 0x01)<<6)|((latest_gps_low_data.fixType == 3)<<7));

        Flags_Tele                  |= data.board.flags;
        Error_Code_Tele             |= data.board.error_code;

        uint8_t Pyro_State = 0;
        for(int i =0 ; i<4;i++)
        {
            Pyro_State |= digitalRead(pyro_continuity[i])<<i;
        }
        data.board.pyro_state       = (uint8_t)(Pyro_Con_Mode|(Pyro_State<<4));
        data.board.RSSI             = rf95.lastRssi();
        
                
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

void SD_Log_Task(void *pvParameters)
{
   

    while(1)
    {
        if(Log_Enabled)
        {
            xSemaphoreTake(SD_Mutex, portMAX_DELAY);
            Data_t data;
            int count = 0;
            uint32_t lastSync = xTaskGetTickCount();
            snprintf(Filename_SD, sizeof(Filename_SD), "/flight_%ld.bin", flight_count);

            SD_file = SD.open(Filename_SD, FILE_WRITE);
            if(!SD_file)
            {
                debugPrint("Failed to open log file in SD for logging !");
            }
            while(Log_Enabled)
            {
                if((xQueueReceive(SD_Data_Queue, &data, 1000) == pdTRUE))
                {   
                    SD_file.write((const uint8_t*)&data, sizeof(data));
                    count++;

                    bool by_count = (count % 200 == 0);
                    bool by_time  = (millis() - lastSync >= 500);

                    if (by_count || by_time) 
                    {
                        SD_file.flush();
                        lastSync = millis();
                        if (count % 2000 == 0)
                        debugPrint("SD -- [LOG] %lu records, q=%u",
                                count, uxQueueMessagesWaiting(SD_Data_Queue));
                    }
        // else
        // {
            //   vTaskDelay(pdMS_TO_TICKS(10));
        // }
                }

            }

            SD_file.flush();
            SD_file.close();      
            debugPrint("Logging stopped");
            xSemaphoreGive(SD_Mutex);  
        }
        else
        vTaskDelay(pdMS_TO_TICKS(100));

        
    }

}

void Data_Log_Task(void *pvParameters)
{

    if(!flash_init())
    {
        debugPrint("Flash init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    if(SD_Ready)
    {
    debugPrint("Flash init successful, starting SD log task");
    xTaskCreate(SD_Log_Task           , "SD_LOG"  , 8192  , nullptr, 4, nullptr);
    }

    while(1)
    {
        if(Log_Enabled)
        {
            // logging just began
            xSemaphoreTake(Flash_Mutex, portMAX_DELAY);
            
            snprintf(Filename_Flash, sizeof(Filename_Flash), "/INDEX.tmp");
            lfs.remove(Filename_Flash);
            log_file = lfs.open(Filename_Flash, FILE_WRITE);
            if(!log_file)
            {
                debugPrint("Failed to open index file ,retrying...");
                continue;
            }
            log_file.write((uint8_t)flight_count); 
            log_file.write("_");
            log_file.write("N");                  // setting 'N' for not transferred
            log_file.flush();
            log_file.close();
            lfs.rename("/INDEX.tmp","/INDEX.txt");
            File_Transferred = false;            // reset file transferred flag

        
            Data_t data;
            int count = 0;
            uint32_t lastSync = xTaskGetTickCount();
            snprintf(Filename_Flash, sizeof(Filename_Flash), "/flight_%ld.bin", flight_count);
            log_file = lfs.open(Filename_Flash, FILE_WRITE);
            if(!log_file)
            {
                debugPrint("Failed to open log file for loggging !");
            }
            while(Log_Enabled)
            {
                if((xQueueReceive(Data_Queue, &data, 1000) == pdTRUE))
                {   
                    log_file.write((const uint8_t*)&data, sizeof(data));

                    // if (xQueueSend(SD_Data_Queue, &data, 0) != pdTRUE) 
                    // {
                    //     debugPrint("SD Data queue full, dropping data");
                    //     SD_Que_Pkt_Drop = true;
                    // }

                    count++;

                    bool by_count = (count % 200 == 0);
                    bool by_time  = (millis() - lastSync >= 500);

                    if (by_count || by_time) 
                    {
                        log_file.flush();
                        lastSync = millis();
                        if (count % 2000 == 0)
                        debugPrint("flash -- [LOG] %lu records, q=%u",
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

            xSemaphoreGive(Flash_Mutex);
            
            debugPrint("Logging stopped");
            debugPrint("Initiating transfer of %s", Filename_Flash);
            xTaskCreate(Transfer_SD, "SD_TRANSFER", 4096, nullptr, 8, nullptr);
        }
        else
        vTaskDelay(pdMS_TO_TICKS(100));

    }
        
} 



void Transfer_SD(void *pvParameters)
{   
        xSemaphoreTake(SD_Mutex, portMAX_DELAY);
        xSemaphoreTake(Flash_Mutex, portMAX_DELAY);
        snprintf(Filename_SD, sizeof(Filename_SD), "/flight_%ld_f.bin", flight_count);


        if(SD.exists(Filename_SD)) SD.remove(Filename_SD);

        SD_file = SD.open(Filename_SD, FILE_WRITE);
        if(!SD_file)
        {
            debugPrint("Failed to open file on SD card for writing");
            xSemaphoreGive(Flash_Mutex);
            xSemaphoreGive(SD_Mutex);
            SD_Error = true;
            vTaskDelete(NULL);
        }


        snprintf(Filename_Flash, sizeof(Filename_Flash), "/flight_%ld.bin", flight_count);
        log_file = lfs.open(Filename_Flash, FILE_READ);
        if(!log_file)
        {
            debugPrint("Failed to read log file !");
            xSemaphoreGive(Flash_Mutex);
            xSemaphoreGive(SD_Mutex);
            SD_file.close();
            Flash_Error = true;
            vTaskDelete(NULL);
        }
        debugPrint("Transferring log file to SD card as %s", Filename_SD);
        File_Transferred = false;   
        uint8_t buf[512];
        File_Transfer_In_Progress = true;

           
        while(log_file.available())
        {
            size_t to_read = min(sizeof(buf), log_file.available());
            log_file.read(buf, to_read);
            SD_file.write(buf, to_read);
            vTaskDelay(pdMS_TO_TICKS(1));
        }


        File_Transfer_In_Progress = false;
        SD_Error    |= SD_file.getWriteError();
        Flash_Error |= log_file.getReadError();
        File_Transfer_Error = SD_file.getWriteError() || log_file.getReadError();
        debugPrint(File_Transfer_Error?"error while transfering":"all fine with transfer");

        SD_file.flush();
        SD_file.close();
        log_file.close();

        if(File_Transfer_Error)
        {
            debugPrint("Error occurred during file transfer");
        }
        //////////////////////////////////////////////////////////////////////
        
         // THIS PART SHOULD CHECK FOR THE GOOD TRANSFER IMPLEMENT THAT AFTER TESTING !
        debugPrint("Log file transferred to SD card as %s", Filename_SD);
        char index_buf[16];
        flight_count++;
        snprintf(index_buf, sizeof(index_buf), "%c_%c\n", flight_count,'T');

        //////////////////////////////////////////////////////////////////////

        snprintf(Filename_Flash, sizeof(Filename_Flash), "/INDEX.tmp");
        lfs.remove(Filename_Flash);
        log_file = lfs.open(Filename_Flash, FILE_WRITE);  
        if(!log_file)
        {
            debugPrint("Failed to open index file for writing");
            xSemaphoreGive(Flash_Mutex);
            xSemaphoreGive(SD_Mutex);
            Flash_Error = true;
            vTaskDelete(NULL);
        }
        log_file.write(index_buf);
        log_file.flush();                // power loss upto this point , will redo the transfer next time and after this point this point file will be marked as transferred
        log_file.close();
        lfs.rename("/INDEX.tmp","/INDEX.txt");
        File_Transferred = true;
        debugPrint("Updated index file with flight %lu", flight_count);
        lfs.remove(Filename_Flash);


        snprintf(Filename_Flash, sizeof(Filename_Flash), "/flight_%ld.bin", flight_count);
        snprintf(Filename_SD, sizeof(Filename_SD), "/flight_%ld.bin", flight_count);

        xSemaphoreGive(Flash_Mutex);
        xSemaphoreGive(SD_Mutex);

        vTaskDelete(NULL);

}

void setup()
{
  Serial.begin(115200);
    while (!Serial && millis() < 4000); 
    if (CrashReport) {
    Serial.print(CrashReport);
    }



  GY_91.begin();

  delay(100);


  Debug_Queue   = xQueueCreate(DEBUG_QUE_LEN, sizeof(Debug_Msg));
  Cmd_Queue     = xQueueCreate(CMD_QUE_LEN, sizeof(ack_pkt_t));
  Data_Queue    = xQueueCreate(DATA_QUE_LEN, sizeof(Data_t));
  SD_Data_Queue = xQueueCreate(SD_DATA_QUE_LEN, sizeof(Data_t));

  SPI_Mutex     = xSemaphoreCreateMutex();
  SD_Mutex      = xSemaphoreCreateMutex();
  Flash_Mutex   = xSemaphoreCreateMutex();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  xTaskCreate(Blink_Green           , "GREEN"   , 512   , nullptr, 1, nullptr);
  xTaskCreate(Blink_Red             , "RED"     , 512   , nullptr, 1, nullptr);
  xTaskCreate(DebugTask             , "DEBUG"   , 512   , nullptr, 1, nullptr);
  xTaskCreate(LoRa_Task             , "LORA"    , 2048  , nullptr, 3, nullptr);
  xTaskCreate(Cmd_Task              , "CMD"     , 1024  , nullptr, 2, nullptr);
  xTaskCreate(IMU_Task              , "IMU"     , 2048  , nullptr, 7, nullptr);
  xTaskCreate(Baro_Task             , "BARO"    , 2048  , nullptr, 6, nullptr);
  xTaskCreate(ADXL_Task             , "ADXL"    , 1024  , nullptr, 5, nullptr);
  xTaskCreate(GPS_Task              , "GPS"     , 2048  , nullptr, 5, nullptr);
  xTaskCreate(Board_task            , "BOARD"   , 1024  , nullptr, 5, nullptr);
  xTaskCreate(Data_Log_Task         , "LOG"     , 8192  , nullptr, 4, nullptr);
  xTaskCreate(Pyro_Continuity_Task  , "PYRO"    , 512   , nullptr, 2, nullptr);
    

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

static void spi_reg_read_burst_slow_bmp(uint8_t reg, uint8_t *buf, size_t len)
{
    GY_91.beginTransaction(spi_1M);
    digitalWrite(BMP_CS, LOW);
    GY_91.transfer(reg | 0x80); // Set MSB for read operation
    for(size_t i = 0; i < len; i++)
    {
        buf[i] = GY_91.transfer(0x00); // Dummy byte to read data
    }
    digitalWrite(BMP_CS, HIGH);
    GY_91.endTransaction();
}

bool flash_init()  // Initialize LittleFS and SD card , priority LittleFS
{
    if(!lfs.begin())
    {
        debugPrint("LittleFS init failed !");
        Flash_Error = true;
        return false;
    }
    debugPrint("LittleFS initialized successfully");

    SD_Ready = SD_Init();
    if(SD_Ready)
    {
        debugPrint("SD init successful");
        SD_Error = false;
    }
    else
    {
        debugPrint("SD init failed");
        SD_Error = true;
    }
    
    log_file = lfs.open("/INDEX.txt", FILE_READ);

    if(!log_file)       // creating new index file if it doesn't exist
    {
        debugPrint("No index file found, ");
        log_file = lfs.open("/INDEX.txt", FILE_WRITE);
        if(!log_file)       
        {
            debugPrint("Failed to create index file !");
            Flash_Error = true;
            return false;
        }
        log_file.write(0x01); // default flight count = 1
        log_file.write("_");
        log_file.write("T");
        log_file.flush();
        log_file.close();
        log_file = lfs.open("/INDEX.txt", FILE_READ); // reopen for reading
        debugPrint("Index file created with flight count 1");

    }
    
    flight_count = (uint8_t)log_file.read();
    
    char c = log_file.read();
    c = log_file.read();
    log_file.close();
    debugPrint("flight count: %lu, transferred: %c", flight_count, c);
    if(c == 'T')
    {
    File_Transferred = true;
    return true;
    }
    else
    {
        debugPrint("Current log hasn't been transferred");
        File_Transferred = false;
        if(SD_Ready)
        {
            xTaskCreate(Transfer_SD, "SD_TRANSFER", 4096, nullptr, 8, nullptr);
        }
        else
        {
            debugPrint("SD card not ready, cannot transfer log");
            File_Transferred = false;
            File_Transfer_Error = true;
        }
        return true;
    }

}

bool SD_Init()
{
    if(!SD.begin(BUILTIN_SDCARD))
    {
        debugPrint("SD card not ready, cannot log to SD");
        return false;
    }
    debugPrint("SD card initialized successfully");
    snprintf(Filename_SD, sizeof(Filename_SD), "/flight_%ld.bin", flight_count);
    return true;
}

bool Lora_Init()
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
    rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096 );
    // Bw125Cr45Sf128 	   ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
	// Bw500Cr45Sf128      ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	// Bw31_25Cr48Sf512	   ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	// Bw125Cr48Sf4096     ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
    
    
    debugPrint("LoRa initialized ! \n");
    return 0;

}

bool GPS_Init()
{
    
    GPS.begin(9600);
    
    GPS_Send_Cmd(setbaud,sizeof(setbaud));
    GPS.end();
    vTaskDelay(pdMS_TO_TICKS(100));
    GPS.begin(115200);
    GPS.addMemoryForRead(gps_buffer, sizeof(gps_buffer));

    GPS_Send_Cmd(disableGPGLL,sizeof(disableGPGLL));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(disableGPGSV,sizeof(disableGPGSV));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(disableGPGSA,sizeof(disableGPGSA));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(disableGPGGA,sizeof(disableGPGGA));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(disableGPVTG,sizeof(disableGPVTG));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(disableGPRMC,sizeof(disableGPRMC));
    vTaskDelay(pdMS_TO_TICKS(1));

    GPS.clear();

    GPS_Send_Cmd(NAV5_Airborne,sizeof(NAV5_Airborne));
    vTaskDelay(pdMS_TO_TICKS(1));
    GPS_Send_Cmd(Enable_PVT,sizeof(Enable_PVT));
    vTaskDelay(pdMS_TO_TICKS(1));
    debugPrint("GPS initialized successfully");
    return true;
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
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0x1A, 0x03, IMU_CS); // config, DLPF 44Hz
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0x1B, 0x18, IMU_CS); // gyro config, +-2000dps
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0x1C, 0x18, IMU_CS); // accel config, +-16g
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0x19, 0x04, IMU_CS); // 1000/4+1 = 200hz
    vTaskDelay(pdMS_TO_TICKS(1));
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
    if(id != 0x58)
    {
        debugPrint("Barometer not found: ID = 0x%02X", id);
        BARO_Error = true;
        return false;
    }
    debugPrint("Barometer found ID: 0x%02X", id);
    BARO_Error = false;
    uint8_t cb[24];
    spi_reg_read_burst_slow_bmp(0x88, cb, 24);

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

    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0xF4, 0x57, BMP_CS);  //0x57 = 0b01010111:    set freq osrs p = 2x :0x37
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_reg_write(0xF5, 0x08, BMP_CS);  //0xA0 = 0b10100000: IIR off, 20hz sampling
    vTaskDelay(pdMS_TO_TICKS(1));
    //Serial.printf("T: %u %d %d\n", cal.T1, cal.T2, cal.T3);
    //Serial.printf("P: %u %d %d %d %d %d %d %d %d\n",cal.P1, cal.P2, cal.P3, cal.P4, cal.P5, cal.P6, cal.P7, cal.P8, cal.P9);
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
        IMU_Error = true;
        return false;
    }

    IMU_Error = false;
    debugPrint("ADS initialized successfully");
    return true;
}



void Read_GPS()
{
    while(GPS.available()>90)
    {
        
        if(GPS.read() == 0xB5 && GPS.read() == 0x62)
        {
            
         if(GPS.read() == 0x01 && GPS.read() == 0x07)
         {
            uint8_t buf[100] = {0};
            int n = GPS.read();
            GPS.read();
            if(n == 92 | n == 84)
            {
              //debugPrint("GPS: PVT message received");
              GPS.readBytes((uint8_t*)buf, n+2);

              memcpy(&PVT_Data, buf, sizeof(PVT_t));
            }

         }

        }
    }
    return;
}
Data_t Read_IMU()
{
    uint8_t buf[14] = {0};

    Data_t data;
    data.type = LOG_TYPE_IMU; 
    data.time = millis();

    xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
    spi_reg_read_burst_imu(0x3B, buf, 14);
    xSemaphoreGive(SPI_Mutex);

    data.imu.ay = -(int16_t)((buf[0] << 8) | buf[1]) - imu_offsets.ay;
    data.imu.az = -(int16_t)((buf[2] << 8) | buf[3]) - imu_offsets.az;
    data.imu.ax = (int16_t)((buf[4] << 8) | buf[5]) - imu_offsets.ax;// BYTES 6 AND 7 ARE TEMP, NOT USED CURRENTLY
    data.imu.gy = -(int16_t)((buf[8] << 8) | buf[9]) - imu_offsets.gy;
    data.imu.gz = -(int16_t)((buf[10] << 8) | buf[11]) - imu_offsets.gz;
    data.imu.gx = (int16_t)((buf[12] << 8) | buf[13]) - imu_offsets.gx;   /// realignment to board orientation


    return data;
}
Data_t Read_Baro()
{
    uint8_t raw[6];

    Data_t data = {0};
    data.type = LOG_TYPE_BARO;
    data.time = millis();
    xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
    spi_reg_read_burst_bmp(0xF7, raw, 6);
    xSemaphoreGive(SPI_Mutex);

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

    
    double var1, var2, p;
    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)cal.P6) / 32768.0;
    var2 = var2 + var1 * ((double)cal.P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)cal.P4) * 65536.0);
    var1 = (((double)cal.P3) * var1 * var1 / 524288.0 + ((double)cal.P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)cal.P1);
    if (var1 == 0) 
    {
        debugPrint("Barometer error: var1 is zero");
        return data;
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)cal.P9) * p * p / 2147483648.0;
    var2 = p * ((double)cal.P8) / 32768.0;
    p = p + (var1 + var2 + ((double)cal.P7)) / 16.0;
    
    float pressure = (float)(p / 100.0);

    
    data.baro.pressure = pressure;
    data.baro.temp = temperature;
    //debugPrint("raw: %02X %02X %02X adc_P=%ld\n", raw[0], raw[1], raw[2],(long)adc_P);
    return data;
}
Data_t Read_ADXL()
{
    Data_t data;
    data.type = LOG_TYPE_ADXL;
    data.time = millis();
    int16_t *a[3] = {&data.adxl.ax, &data.adxl.az, &data.adxl.ay};
    int sign[3] = {-1, 1, -1};

    for(int i = 0; i < 3; i++)
    {
        uint16_t cfg = CFG_BASE | ((0b100 + i) << 12); // 0100 0000 0000 0000
                                                       
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

        *a[i] = sign[i]*(((int16_t)((high_byte << 8) | low_byte)) - *(&adxl_offsets.ax + i));
    }
    return data;    
}

