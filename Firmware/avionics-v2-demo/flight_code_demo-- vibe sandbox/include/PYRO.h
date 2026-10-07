#include <Arduino.h>

#include<FreeRTOS.h>
#include <task.h>

#define PYRO_1_TRIG 0
#define PYRO_2_TRIG 1
#define PYRO_3_TRIG 21
#define PYRO_4_TRIG 23

#define PYRO_1_CONT 18
#define PYRO_2_CONT 19
#define PYRO_3_CONT 20
#define PYRO_4_CONT 22

#define CONTIN_THRESHOLD 8/3 - 1



int pyro_continuity[4] = {PYRO_1_CONT, PYRO_2_CONT, PYRO_3_CONT, PYRO_4_CONT};
int Pyro_Trigger[4] = {PYRO_1_TRIG, PYRO_2_TRIG, PYRO_3_TRIG, PYRO_4_TRIG};

uint8_t Pyro_Contin = 0x00; // 0b00001111 continuity

bool good_continuity = false;

#define IGNITION_DURATION 1000

enum pyro_mode_t
{
    PYRO_1      =   1,  // 0b00000001
    PYRO_2      =   2,  // 0b00000010
    PYRO_3      =   4,  // 0b00000100
    PYRO_4      =   8,  // 0b00001000
    PYRO_12     =   3,  // 0b00000011
    PYRO_34     =   12, // 0b00001100
    PYRO_12_34  =   15  // 0b00001111
};

pyro_mode_t Pyro_Con_Mode = PYRO_1;
pyro_mode_t Pyro_Mode     = PYRO_1;


void Pyro_Continuity_Task(void *pvParameters)
{
    for(int i = 0; i < 4; i++)
    {
        pinMode(Pyro_Trigger[i], OUTPUT);
        digitalWrite(Pyro_Trigger[i], HIGH);
    }
    for(int i = 0; i < 4; i++)
    {
        pinMode(pyro_continuity[i], INPUT);
    }
    
    TickType_t lastWake = xTaskGetTickCount();

    while(1)
    {
        for(int i = 0; i < 4; i++)
        {
            if(analogRead(pyro_continuity[i]) >= CONTIN_THRESHOLD)
            {
                Pyro_Contin |= (1 << i);
            }
            else
            {
                Pyro_Contin &= ~(1 << i);
            }
        }

      if((uint8_t)Pyro_Contin == (uint8_t)Pyro_Con_Mode)
        good_continuity = true;
      else
        good_continuity = false;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
        
    }

}

void Pyro_Trigger_Task(void *pvParameters)
{
    

    for(int i = 0; i < 4; i++)
        digitalWrite(Pyro_Trigger[i], !(Pyro_Mode & (1 << i)));

    vTaskDelay(pdMS_TO_TICKS(IGNITION_DURATION));

    for(int i = 0; i < 4; i++)
        digitalWrite(Pyro_Trigger[i], HIGH);
}