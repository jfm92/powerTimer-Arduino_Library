#include "powerTimer.h"
#include "Wire.h"

#define SDA_PIN 6
#define SCL_PIN 7
#define POWER_OFF_PIN 5

#define TIMER_REPEAT 1
#define TIMER_INT 1
#define TIMER_START 1

powerTimer powerTimer;

void setup(){
    Serial.begin(115200);

    if(!Wire.begin(SDA_PIN, SCL_PIN)){
        Serial.println("I2C Intialization failed.");
        return;
    }

    if(!powerTimer.begin(POWER_OFF_PIN)){
        Serial.println("powerTimer Intialization failed.");
        return;
    }

    if(!powerTimer.dateIntegrity()){
        Serial.println("RTC power integrity compromised, cleaning interruptions.");
        //Reset RTC
        powerTimer.clearInterrupts();
    }

    //Check if wake-up reason was an alarm interruption
    if(powerTimer.readTimerInterruptFlag()){
        Serial.println("Device wake by alarm interrupt.");
        powerTimer.clearAlarmInterruptFlag();
    }

    //Set a timer in 60 seconds
    powerTimer.setTimer(TIMER_REPEAT,60,TIMER_INT,TIMER_START);
    Serial.println("Set timer alarm in 60 seconds.");
}

void loop(){
    //This should cut the energy to the MCU 
    powerTimer.powerOff();
}