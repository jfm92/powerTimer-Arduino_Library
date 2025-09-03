#include "Arduino.h"
#include "powerTimer.h"

bool powerTimer::begin(uint8_t powerOffPin, TwoWire &wirePort)
{
	bool result = false;
	_wirePort = &wirePort;
	_powerOffPin = powerOffPin;

	//Check if the device is connected
	_wirePort->beginTransmission(RV3028_ADDR);
	if (_wirePort->endTransmission() != 0)
	{
		return result;
	}

	pinMode(_powerOffPin, OUTPUT);

	delay(1);
	set24Hour(); 

	result = true;

	return result;
}

bool powerTimer::setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year)
{
	bool result = false;

	_time[TIME_SECONDS] = DECtoBCD(sec);
	_time[TIME_MINUTES] = DECtoBCD(min);
	_time[TIME_HOURS] = DECtoBCD(hour);
	_time[TIME_WEEKDAY] = DECtoBCD(weekday);
	_time[TIME_DATE] = DECtoBCD(date);
	_time[TIME_MONTH] = DECtoBCD(month);
	_time[TIME_YEAR] = DECtoBCD(year - 2000);

	if(!writeMultipleRegisters(RV3028_SECONDS, _time, TIME_ARRAY_LENGTH)){
		return result;
	}

	uint8_t _timeRead[TIME_ARRAY_LENGTH];
	if(!readMultipleRegisters(RV3028_SECONDS, _timeRead, TIME_ARRAY_LENGTH)){
		return result;
	}

	result = (_time == _timeRead) ? true : false; //Check if saved data is correct.

	return result;
}


void powerTimer::enableAlarmInterrupt(uint8_t min, uint8_t hour, uint8_t dateOrWeekDay, bool setWeekDayAlarm)
{
	disableAlarmInterrupt();
	clearAlarmInterruptFlag();

	set24Hour();

	if (setWeekDayAlarm){
		clearBit(RV3028_CTRL1, CTRL1_WADA);
	}
	else{
		setBit(RV3028_CTRL1, CTRL1_WADA);
	}

	uint8_t alarmTime[3];

	alarmTime[0] = DECtoBCD(min);
	alarmTime[1] = DECtoBCD(hour);
	alarmTime[2] = DECtoBCD(dateOrWeekDay);

	writeMultipleRegisters(RV3028_MINUTES_ALM, alarmTime, 3);

	setBit(RV3028_CTRL2, CTRL2_AIE);
}

void powerTimer::disableAlarmInterrupt()
{
	clearBit(RV3028_CTRL2, CTRL2_AIE);
}

bool powerTimer::readAlarmInterruptFlag()
{
	return readBit(RV3028_STATUS, STATUS_AF);
}

void powerTimer::clearAlarmInterruptFlag()
{
	clearBit(RV3028_STATUS, STATUS_AF);
}

/*********************************
Countdown Timer Interrupt
********************************/
bool powerTimer::setTimer(bool timerRepeat, uint16_t timerValue, bool setInterrupt, bool startTimer)
{
	bool result = false;

	if((timerValue > 3932100) || (timerValue == 0)){
		Serial.println("outof range");
		return result;
	}

	disableTimer();
	disableTimerInterrupt();
	clearTimerInterruptFlag();

	if(!writeRegister(RV3028_TIMERVAL_0, timerValue & 0xff)){
		return result;
	}

	if(!writeRegister(RV3028_TIMERVAL_1, timerValue >> 8)){
		return result;
	}

	uint8_t ctrl1Val = readRegister(RV3028_CTRL1);
	if (timerRepeat){
		ctrl1Val |= 1 << CTRL1_TRPT;
	}
	else{
		ctrl1Val &= ~(1 << CTRL1_TRPT);
	}

	if(timerValue <= 65535){
		//1Hz frequency
		ctrl1Val &= ~3; // Clear both the bits
		ctrl1Val |= 2;
	}
	else{
		// 1/60Hz
		ctrl1Val &= ~3; // Clear both the bits
	}

	//Enable Timer interrupt
	setBit(RV3028_CTRL2, CTRL2_TIE);

	if (startTimer){
		ctrl1Val |= (1 << CTRL1_TE);
	}

	if(!writeRegister(RV3028_CTRL1, ctrl1Val)){
		return result;
	}

	setBit(RV3028_CTRL1, CTRL1_TE);

	result = true;
	return result;
}

void powerTimer::disableTimerInterrupt()
{
	clearBit(RV3028_CTRL2, CTRL2_TIE);
}

bool powerTimer::readTimerInterruptFlag()
{
	return readBit(RV3028_STATUS, STATUS_TF);
}

void powerTimer::clearTimerInterruptFlag()
{
	clearBit(RV3028_STATUS, STATUS_TF);
}

void powerTimer::disableTimer()
{
	clearBit(RV3028_CTRL1, CTRL1_TE);
}

bool powerTimer::dateIntegrity()
{
	constexpr uint8_t PORF_MASK = 0b00000001; // Bit 0
	constexpr uint8_t BSF_MASK = 0b00100000; // Bit 5

	uint8_t statusRegister = status();

	bool porf = statusRegister & PORF_MASK;
	bool bsf = statusRegister & BSF_MASK;

	return !(porf || bsf);
}

uint8_t powerTimer::status(void)
{
	return(readRegister(RV3028_STATUS));
}

void powerTimer::clearInterrupts()
{
	writeRegister(RV3028_STATUS, 0);
}

void powerTimer::powerOff()
{
	//Configuration to reduce the consumption to the min
	delay(1);
	disableTrickleCharge();
	delay(1);
	setBackupSwitchoverMode(0x00);
	delay(1);

	digitalWrite(_powerOffPin, HIGH);
}

/*********************************
PRIVATE FUNCTIONS
********************************/
uint8_t powerTimer::BCDtoDEC(uint8_t val)
{
	return ((val / 0x10) * 10) + (val % 0x10);
}

uint8_t powerTimer::DECtoBCD(uint8_t val)
{
	return ((val / 10) * 0x10) + (val % 10);
}

uint8_t powerTimer::readRegister(uint8_t addr)
{
	uint8_t result = 0XFF;

	_wirePort->beginTransmission(RV3028_ADDR);
	_wirePort->write(addr);
	_wirePort->endTransmission();

	_wirePort->requestFrom(RV3028_ADDR, (uint8_t)1);

	if (_wirePort->available()) {
		result = _wirePort->read();
	}

	return result;
}

bool powerTimer::writeRegister(uint8_t addr, uint8_t val)
{
	bool result = false;

	_wirePort->beginTransmission(RV3028_ADDR);
	_wirePort->write(addr);
	_wirePort->write(val);

	if (_wirePort->endTransmission() == 0){
		result = true;
	}

	return result;
}

bool powerTimer::readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len)
{
	bool result = false;

	_wirePort->beginTransmission(RV3028_ADDR);
	_wirePort->write(addr);
	if (_wirePort->endTransmission() != 0){
	return result;
	}

	_wirePort->requestFrom(RV3028_ADDR, len);
	for (uint8_t i = 0; i < len; i++)
	{
		dest[i] = _wirePort->read();
	}

	result = true;
	return result;
}

bool powerTimer::writeMultipleRegisters(uint8_t addr, uint8_t * values, uint8_t len)
{
	bool result = false;

	_wirePort->beginTransmission(RV3028_ADDR);
	_wirePort->write(addr);
	for (uint8_t i = 0; i < len; i++)
	{
		_wirePort->write(values[i]);
	}

	if (_wirePort->endTransmission() == 0){
	result = true;
	}

	return result;
}

bool powerTimer::writeConfigEEPROM_RAMmirror(uint8_t eepromaddr, uint8_t val)
{
	bool success = waitforEEPROM();

	//Disable auto refresh by writing 1 to EERD control bit in CTRL1 register
	uint8_t ctrl1 = readRegister(RV3028_CTRL1);
	ctrl1 |= 1 << CTRL1_EERD;
	if (!writeRegister(RV3028_CTRL1, ctrl1)) success = false;
	//Write Configuration RAM Register
	writeRegister(eepromaddr, val);
	//Update EEPROM (All Configuration RAM -> EEPROM)
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_First);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_Update);
	if (!waitforEEPROM()) success = false;
	//Reenable auto refresh by writing 0 to EERD control bit in CTRL1 register
	ctrl1 = readRegister(RV3028_CTRL1);
	if (ctrl1 == 0x00)success = false;
	ctrl1 &= ~(1 << CTRL1_EERD);
	writeRegister(RV3028_CTRL1, ctrl1);
	if (!waitforEEPROM()) success = false;

	return success;
}

uint8_t powerTimer::readConfigEEPROM_RAMmirror(uint8_t eepromaddr)
{
	bool success = waitforEEPROM();

	//Disable auto refresh by writing 1 to EERD control bit in CTRL1 register
	uint8_t ctrl1 = readRegister(RV3028_CTRL1);
	ctrl1 |= 1 << CTRL1_EERD;
	if (!writeRegister(RV3028_CTRL1, ctrl1)) success = false;
	//Read EEPROM Register
	writeRegister(RV3028_EEPROM_ADDR, eepromaddr);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_First);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_ReadSingle);
	if (!waitforEEPROM()) success = false;
	uint8_t eepromdata = readRegister(RV3028_EEPROM_DATA);
	if (!waitforEEPROM()) success = false;
	//Reenable auto refresh by writing 0 to EERD control bit in CTRL1 register
	ctrl1 = readRegister(RV3028_CTRL1);
	if (ctrl1 == 0x00)success = false;
	ctrl1 &= ~(1 << CTRL1_EERD);
	writeRegister(RV3028_CTRL1, ctrl1);

	if (!success) return 0xFF;
	return eepromdata;
}

bool powerTimer::writeUserEEPROM(uint8_t eepromaddr, uint8_t val)
{
	bool success = waitforEEPROM();

	//Disable auto refresh by writing 1 to EERD control bit in CTRL1 register
	uint8_t ctrl1 = readRegister(RV3028_CTRL1);
	ctrl1 |= 1 << CTRL1_EERD;
	if (!writeRegister(RV3028_CTRL1, ctrl1)) success = false;
	//Write addr to EEADDR
	writeRegister(0x25, eepromaddr);
	//Write value to EEDATA
	writeRegister(0x26, val);
	//Update EEPROM (All Configuration RAM -> EEPROM)
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_First);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_WriteSingle);
	if (!waitforEEPROM()) success = false;
	//Reenable auto refresh by writing 0 to EERD control bit in CTRL1 register
	ctrl1 = readRegister(RV3028_CTRL1);
	if (ctrl1 == 0x00)success = false;
	ctrl1 &= ~(1 << CTRL1_EERD);
	writeRegister(RV3028_CTRL1, ctrl1);
	if (!waitforEEPROM()) success = false;

	return success;
}

uint8_t powerTimer::readUserEEPROM(uint8_t eepromaddr)
{
	bool success = waitforEEPROM();

	//Disable auto refresh by writing 1 to EERD control bit in CTRL1 register
	uint8_t ctrl1 = readRegister(RV3028_CTRL1);
	ctrl1 |= 1 << CTRL1_EERD;
	if (!writeRegister(RV3028_CTRL1, ctrl1)) success = false;
	//Read EEPROM Register
	writeRegister(RV3028_EEPROM_ADDR, eepromaddr);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_First);
	writeRegister(RV3028_EEPROM_CMD, EEPROMCMD_ReadSingle);
	if (!waitforEEPROM()) success = false;
	uint8_t eepromdata = readRegister(RV3028_EEPROM_DATA);
	if (!waitforEEPROM()) success = false;
	//Reenable auto refresh by writing 0 to EERD control bit in CTRL1 register
	ctrl1 = readRegister(RV3028_CTRL1);
	if (ctrl1 == 0x00)success = false;
	ctrl1 &= ~(1 << CTRL1_EERD);
	writeRegister(RV3028_CTRL1, ctrl1);

	if (!success) return 0xFF;
	return eepromdata;
}

bool powerTimer::waitforEEPROM()
{
	unsigned long timeout = millis() + 500;
	while ((readRegister(RV3028_STATUS) & 1 << STATUS_EEBUSY) && millis() < timeout);

	return millis() < timeout;
}

void powerTimer::reset()
{
	setBit(RV3028_CTRL2, CTRL2_RESET);
}

void powerTimer::setBit(uint8_t regAddr, uint8_t bitNum)
{
	uint8_t value = readRegister(regAddr);
	value |= (1 << bitNum); //Set the bit
	writeRegister(regAddr, value);
}

void powerTimer::clearBit(uint8_t regAddr, uint8_t bitNum)
{
	uint8_t value = readRegister(regAddr);
	value &= ~(1 << bitNum); //Clear the bit
	writeRegister(regAddr, value);
}

bool powerTimer::readBit(uint8_t regAddr, uint8_t bitNum)
{
	uint8_t value = readRegister(regAddr);
	value &= (1 << bitNum);
	return value;
}

void powerTimer::disableTrickleCharge()
{
	//Read EEPROM Backup Register (0x37)
	uint8_t EEPROMBackup = readConfigEEPROM_RAMmirror(EEPROM_Backup_Register);
	//Write 0 to TCE Bit
	EEPROMBackup &= ~(1 << EEPROMBackup_TCE_BIT);
	//Write EEPROM Backup Register
	writeConfigEEPROM_RAMmirror(EEPROM_Backup_Register, EEPROMBackup);
}

void powerTimer::set24Hour()
{
	uint8_t hour = readRegister(RV3028_HOURS); //Get the current 12 hour formatted time in BCD

	bool pm = false;

	if (hour & (1 << HOURS_AM_PM)) //Is the AM/PM bit set?
	{
	pm = true;
	hour &= ~(1 << HOURS_AM_PM); //Clear the bit
	}

	//Change to 24 hour mode
	uint8_t setting = readRegister(RV3028_CTRL2);
	setting &= ~(1 << CTRL2_12_24); //Clear the 12/24 hr bit
	writeRegister(RV3028_CTRL2, setting);

	//Given a BCD hour in the 1-12 range, make it 24
	hour = BCDtoDEC(hour); //Convert core of register to DEC

	if (pm == true) hour += 12; //2PM becomes 14
	if (hour == 12) hour = 0; //12AM stays 12, but should really be 0
	if (hour == 24) hour = 12; //12PM becomes 24, but should really be 12

	hour = DECtoBCD(hour); //Convert to BCD

	writeRegister(RV3028_HOURS, hour); //Record this to hours register
}

void powerTimer::disableClockOut()
{
	//Read EEPROM CLKOUT Register (0x35)
	uint8_t EEPROMClkout = readConfigEEPROM_RAMmirror(EEPROM_Clkout_Register);
	//Clear CLKOE Bit
	EEPROMClkout &= ~(1 << EEPROMClkout_CLKOE_BIT);
	//Write EEPROM CLKOUT Register
	writeConfigEEPROM_RAMmirror(EEPROM_Clkout_Register, EEPROMClkout);

	//Clear CLKIE Bit
	clearBit(RV3028_CTRL2, CTRL2_CLKIE);
}

/*********************************
0 = Switchover disabled
1 = Direct Switching Mode
2 = Standby Mode
3 = Level Switching Mode
*********************************/
bool powerTimer::setBackupSwitchoverMode(uint8_t val)
{
	bool result = true;

	if (val > 3){
	result = false;
	return result;
	}

	//Read EEPROM Backup Register (0x37)
	uint8_t EEPROMBackup = readConfigEEPROM_RAMmirror(EEPROM_Backup_Register);
	if (EEPROMBackup == 0xFF){
	result = false;
	}
	//Ensure FEDE Bit is set to 1
	EEPROMBackup |= 1 << EEPROMBackup_FEDE_BIT;
	//Set BSM Bits (Backup Switchover Mode)
	EEPROMBackup &= EEPROMBackup_BSM_CLEAR;		//Clear BSM Bits of EEPROM Backup Register
	EEPROMBackup |= val << EEPROMBackup_BSM_SHIFT;	//Shift values into EEPROM Backup Register
	//Write EEPROM Backup Register
	if (!writeConfigEEPROM_RAMmirror(EEPROM_Backup_Register, EEPROMBackup)){
	result = false;
	}

	return result;
}