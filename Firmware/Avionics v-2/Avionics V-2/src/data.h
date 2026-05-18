typedef struct 
{
    uint32_t time;
    int32_t  lat, lon;
    float    gps_alt, baro_alt;
    float    roll, pitch, yaw;
    float    pressure;    


    uint8_t   temp;
    uint8_t   v_batt;      
    uint8_t   state;
    uint8_t   error_code; 
    uint8_t   pyro_state;

    uint16_t  CRC16;
 
}   gsm_pkt_t;
// expected freq 0.5 to 0.25 hz

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

// expected freq 1 - 5 hz

typedef struct
{
    uint32_t time;
    int32_t  lat, lon;
    float    gps_alt, baro_alt;
    float    vx, vy, vz;
    float    roll, pitch, yaw;
    float    pressure;  
    float    temp;  

    
    int16_t  ax, ay, az;
    int16_t  ax_h, ay_h, az_h;
    int16_t  gx, gy, gz;

    
    float     cpu_temp;
    float     v_batt;      
    uint8_t   state;
    uint8_t   error_code; 
    uint8_t   pyro_state;
} log_t;

// 100+ hz


enum state_t {
    SAFE,
    IDLE,
    PRE_LAUNCH,
    BOOST,
    COAST,
    RECOVERY,
    DESCENT,
    LANDED
};  // first 3 bits for state, next 5 bits for sats count

//       error codes     //
// bit 0 - IMU 
// bit 1 - BARO
// bit 2 - GPS
// bit 3 - GSM
// bit 4 - SD Card
// bit 5 - Reserved for future use
// bit 6 - Reserved for future use
// bit 7 - Reserved for future use
//

//      pyro states     //
// bit 0 - pyro 1 contuinutiy
// bit 1 - pyro 1 trigger
// ..
// .
//
