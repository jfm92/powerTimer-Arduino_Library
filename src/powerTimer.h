#ifndef powerTimer_h
#define powerTimer_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <time.h>

#define ADDR						(uint8_t)0x52

//RV-3028-C7 Registers
//Config Registers
#define STATUS				0x0E
#define CTRL1					0x0F
#define CTRL2					0x10
#define GPBITS				0x11
#define INT_MASK			0x12

//Periodic Countdown Timer registers
#define TIMERVAL_0				0x0A
#define TIMERVAL_1				0x0B
#define TIMERSTAT_0				0x0C
#define TIMERSTAT_1				0x0D

//Alarm registers
#define RV3028_MINUTES_ALM     			0x07
#define RV3028_HOURS_ALM       			0x08
#define RV3028_DATE_ALM        			0x09

//Bits in Status Register
#define STATUS_EEBUSY	7
#define STATUS_CLKF		6
#define STATUS_BSF		5
#define STATUS_UF		4
#define STATUS_TF		3
#define STATUS_AF		2
#define STATUS_EVF		1
#define STATUS_PORF		0

//Bits in Control2 Register
#define CTRL2_TSE		7
#define CTRL2_CLKIE		6
#define CTRL2_UIE		5
#define CTRL2_TIE		4
#define CTRL2_AIE		3
#define CTRL2_EIE		2
#define CTRL2_12_24		1
#define CTRL2_RESET		0

//Bits in Control1 Register
#define CTRL1_TRPT		7
#define CTRL1_WADA		5//Bit 6 not implemented
#define CTRL1_USEL		4
#define CTRL1_EERD		3
#define CTRL1_TE		2
#define	CTRL1_TD1		1
#define CTRL1_TD0		0

enum interruptType
{
  ALARM_INT = 0,
  TIMER_INT = 1,
};

typedef uint8_t interruptType_t;

class powerTimer
{
  private:
    uint8_t _powerOffPin;
    TwoWire *_wirePort;

    uint8_t readReg(uint8_t regAddr);
    bool writeReg(uint8_t regAddr, uint8_t regVal);
    bool readMultipleReg(uint8_t regAddr, uint8_t * dest, uint8_t len);
    bool writeMultipleReg(uint8_t regAddr, uint8_t * values, uint8_t len);
    bool setBit(uint8_t regAddr, uint8_t bitNum);
    bool clearBit(uint8_t regAddr, uint8_t bitNum);
    bool readBit(uint8_t regAddr, uint8_t bitNum);

    uint8_t BCDtoDEC(uint8_t val);
    uint8_t DECtoBCD(uint8_t val);

  public:
    powerTimer(){};

    bool begin(uint8_t powerOffPin, TwoWire &wirePort = Wire);

    void clearInterrupt(interruptType_t intType);
    bool readInterrupt(interruptType_t intType);
    void disableInterrupt(interruptType_t intType);
    void enableInterrupt(interruptType_t intType);

    bool setTimerPeriod(uint16_t secondsPeriod);    
    bool enableTimerPeriod(void);
    bool disableTimerPeriod(void);
)
    bool setAlarmDate(const struct tm &newAlarmTime);
    bool enableAlarmDate(void);
    bool disableAlarmDate(void);

    bool setRTCDate(const struct tm &dateTime);
    void getRTCDate(struct tm &dateTime);

    void powerOff(void);
};

#endif