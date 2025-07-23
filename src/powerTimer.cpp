#include "Arduino.h"
#include "powerTimer.h"

bool powerTimer::begin(uint8_t powerOffPin, TwoWire &wirePort)
{
  bool result = false;

  _powerOffPin = powerOffPin;
  _wirePort = &wirePort;

  _wirePort->beginTransmission(ADDR);
  if (_wirePort->endTransmission() != 0)
  {
    //Device not found
    return result;
  }

  //TODO: Disable clock output, 24hours mode, trickcharge backupswitchovermode

  return result;
}

void powerTimer::clearInterrupt(interruptType_t intType)
{
  uint8_t regPosition = (intType == ALARM_INT) ? STATUS_AF : STATUS_TF;
  clearBit(STATUS, regPosition);
}

bool powerTimer::readInterrupt(interruptType_t intType)
{
  return readBit(STATUS, (intType == ALARM_INT) ? STATUS_AF : STATUS_TF);
}

void powerTimer::disableInterrupt(interruptType_t intType)
{
  clearBit(CTRL2, (intType == ALARM_INT) ? CTRL2_AIE : CTRL2_TIE);
}

void powerTimer::enableInterrupt(interruptType_t intType)
{
  setBit(CTRL2, (intType == ALARM_INT) ? CTRL2_AIE : CTRL2_TIE);
}

bool powerTimer::setTimerPeriod(uint16_t secondsPeriod)
{
  bool result = false;

  if((secondsPeriod > 0) && (secondsPeriod > 3932100)){
    //Out of valid range, the timer must between 1 second and 3932100 seconds (45,5 days)
    //This is a restriction of the available RTC frequency and the type of data used (uint16_t)
    return result;
  }

  disableTimerPeriod();
  disableInterrupt(TIMER_INT);
  clearInterrupt(TIMER_INT);

  //Set required frequency configuration.
  uint8_t ctrl1Val = readReg(CTRL1);
  if(secondsPeriod > 65535 ) //Max period allowed by 1Hz frequency (65535 secs == 18h 12min), set 1/60Hz freq
  {
    ctrl1Val &= ~3;
		ctrl1Val |= 2;
  }
  else{
    ctrl1_val |= 3;
  }

  if(!writeReg(CTRL1, ctrl1Val)){
    return result;
  }

  // Set period value
  if(!writeReg(TIMERVAL_0, secondsPeriod & 0xff) || !writeReg(TIMERVAL_1, secondsPeriod >> 8)){
    return result;
  }

  result = true;

  return result;
}

bool powerTimer::enableTimerPeriod(void)
{
  bool result = false
  enableInterrupt(TIMER_INT);

  if(!setBit(CTRL1, CTRL1_TE)){
    return result;
  }

  result = true;
  return result;
}

bool powerTimer::disableTimerPeriod(void)
{
  bool result = false;
  disableInterrupt(TIMER_INT);
  clearInterrupt(TIMER_INT);

  if(!clearBit(CTRL1, CTRL1_TE)){
    return result;
  }

  result = true;
  return result;

}

bool powerTimer::setAlarmDate(const struct tm &newAlarmTime)
{
  bool result = false;
  
  disableInterrupt(ALARM_INT);
  clearInterrupt(ALARM_INT);

  uint8_t alarmTime[3];
  alarmTime[0] = DECtoBCD(newAlarmTime.tm_min);
  alarmTime[1] = DECtoBCD(newAlarmTime.tm_hour);
  alarmTime[2] = DECtoBCD(newAlarmTime.tm_mday);

  if(!writeMultipleReg(RV3028_MINUTES_ALM, alarmTime, 3)){  
    return result;
  }

  result = true;
  return result;
}

bool powerTimer::enableAlarmDate(void)
{
  bool result = false;
  enableInterrupt(ALARM_INT);

  if(!setBit(CTRL2, CTRL2_AIE)){
    return result;
  }

  result = true;
  return result;
}

bool powerTimer::disableAlarmDate(void)
{
  bool result = false;
  disableInterrupt(ALARM_INT);
  clearInterrupt(ALARM_INT);

  if(!clearBit(CTRL2, CTRL2_AIE)){
    return result;
  }

  result = true;
  return result;
}

bool powerTimer::setRTCDate(const struct tm &dateTime)
{
  bool result = false;
  uint8_t date[7];
  date[0] = DECtoBCD(dateTime.tm_sec);
  date[1] = DECtoBCD(dateTime.tm_min);
  date[2] = DECtoBCD(dateTime.tm_hour);
  date[3] = DECtoBCD(dateTime.tm_wday);
  date[4] = DECtoBCD(dateTime.tm_mday);
  date[5] = DECtoBCD(dateTime.tm_mon + 1);
  date[6] = DECtoBCD(dateTime.tm_year - 2000);

  if(!writeMultipleReg(RV3028_SECONDS, date, 7)){
    return result;
  }

  result = true;
  return result;
}

void powerTimer::getRTCDate(struct tm &dateTime)
{
  bool result = false;
  uint8_t date[7];

  if(!readMultipleReg(RV3028_SECONDS, date, 7)){
    return result;
  }

  dateTime.tm_sec = BCDtoDEC(date[0]);
  dateTime.tm_min = BCDtoDEC(date[1]);
  dateTime.tm_hour = BCDtoDEC(date[2]);
  dateTime.tm_wday = BCDtoDEC(date[3]);
  dateTime.tm_mday = BCDtoDEC(date[4]);
  dateTime.tm_mon = BCDtoDEC(date[5]) - 1;
  dateTime.tm_year = BCDtoDEC(date[6]) + 2000;

  result = true;
  return result;
}

void powerTimer::powerOff(void)
{
  digitalWrite(_powerOffPin, HIGH);
}

uint8_t powerTimer::readReg(uint8_t regAddr)
{
  uint8_t result = 0xFF;//Error code

  _wirePort->beginTransmission(ADDR);
  _wirePort->write(regAddr);
  _wirePort->endTransmission();

  _wirePort->requestFrom(ADDR, 1);
  if (_wirePort->available())
  {
    result = _wirePort->read();
  }

  return result;
}

bool powerTimer::writeReg(uint8_t regAddr, uint8_t regVal)
{
  uint8_t result = false;

  _wirePort->beginTransmission(ADDR);
  _wirePort->write(regAddr);
  _wirePort->write(regVal);
  if (_wirePort->endTransmission() == 0)
  {
    result = true;
  }

  return result;
}

bool powerTimer::readMultipleReg(uint8_t regAddr, uint8_t *dest, uint8_t len)
{
  bool result = false;
  _wirePort->beginTransmission(ADDR);
  _wirePort->write(regAddr);
  if(_wirePort->endTransmission() != 0){
    return result;
  }

  _wirePort->requestFrom(ADDR, len);
  if (_wirePort->available() == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      dest[i] = _wirePort->read();
    }
    result = true;
  }

  return result;
}

bool powerTimer::writeMultipleReg(uint8_t regAddr, uint8_t *values, uint8_t len)
{
  bool result = false;
  _wirePort->beginTransmission(ADDR);
  _wirePort->write(regAddr);
  for (uint8_t i = 0; i < len; i++)
  {
    _wirePort->write(values[i]);
  }
  if (_wirePort->endTransmission() == 0)
  {
    result = true;
  }

  return result;
}

bool powerTimer::setBit(uint8_t regAddr, uint8_t bitNum) 
{ 
  bool result = false;
  uint8_t val = readReg(regAddr);
  val |= (1 << bitNum);
  if(writeReg(regAddr, val)){
    result = true;
  }
  return result;
}

bool powerTimer::clearBit(uint8_t regAddr, uint8_t bitNum) 
{ 
  bool result = false;
  uint8_t val = readReg(regAddr);
  val &= ~(1 << bitNum);
  if(writeReg(regAddr, val)){
    result = true;
  }
  return result;
}

bool powerTimer::readBit(uint8_t regAddr, uint8_t bitNum) 
{
  bool result = false;
  uint8_t val = readReg(regAddr);
  val &= (1 << bitNum);
  if(val == 0){
    result = true;
  }
  return result;
}

uint8_t powerTimer::BCDtoDEC(uint8_t val) { return ((val / 0x10) * 10) + (val % 0x10); }
uint8_t powerTimer::DECtoBCD(uint8_t val) { return ((val / 10) * 0x10) + (val % 10); }

#endif // powerTimer_h
