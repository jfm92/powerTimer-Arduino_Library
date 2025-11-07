#include "powerTimer.h"
#include "Wire.h"

#define SDA_PIN 6
#define SCL_PIN 7
#define POWER_OFF_PIN 5

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

    //Check if wake-up reason was an alarm interruption
    if(powerTimer.readAlarmInterruptFlag()){
        Serial.println("Device wake by alarm interrupt.");
        powerTimer.clearAlarmInterruptFlag();
    }

    //If RTC date integrity is compromised, probably we lost the energy or we have a fresh start, so is necessary to set the date. 
    if(!powerTimer.dateIntegrity()){
        Serial.println("RTC power integrity compromised, setting new date.");
        //Reset RTC
        powerTimer.clearInterrupts();
        if(powerTimer.setTime(0,0,0,1,1,1,2025)){
            Serial.println("Error setting date.");
            return;
        }
        Serial.println("Set date successfully. 01/01/2025 00:00:00");
    }

    powerTimer.enableAlarmInterrupt(1, 0, 1, false);
    Serial.println("Alarm interrupt enable in 1 minute.");
}

void loop(){
    //This should cut the energy to the MCU 
    powerTimer.powerOff();
}