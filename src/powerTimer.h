#ifndef POWERTIMER_H
#define POWERTIMER_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Arduino.h"
#include "Wire.h"

//The 7-bit I2C ADDRESS of the RV3028
#define RV3028_ADDR						(uint8_t)0x52

//REGISTERS
//Clock registers
#define RV3028_SECONDS      			0x00
#define RV3028_MINUTES      			0x01
#define RV3028_HOURS        			0x02
//Calendar registers
#define RV3028_WEEKDAY					0x03
#define RV3028_DATE         			0x04
#define RV3028_MONTHS        			0x05
#define RV3028_YEARS        			0x06

//Alarm registers
#define RV3028_MINUTES_ALM     			0x07
#define RV3028_HOURS_ALM       			0x08
#define RV3028_DATE_ALM        			0x09

//Periodic Countdown Timer registers
#define RV3028_TIMERVAL_0				0x0A
#define RV3028_TIMERVAL_1				0x0B
#define RV3028_TIMERSTAT_0				0x0C
#define RV3028_TIMERSTAT_1				0x0D

//Configuration registers
#define RV3028_STATUS					0x0E
#define RV3028_CTRL1					0x0F
#define RV3028_CTRL2					0x10
#define RV3028_GPBITS					0x11
#define RV3028_INT_MASK					0x12

//Eventcontrol/Timestamp registers
#define RV3028_EVENTCTRL				0x13
#define RV3028_COUNT_TS					0x14
#define RV3028_SECONDS_TS				0x15
#define RV3028_MINUTES_TS				0x16
#define RV3028_HOURS_TS					0x17
#define RV3028_DATE_TS					0x18
#define RV3028_MONTH_TS					0x19
#define RV3028_YEAR_TS					0x1A

//Unix Time registers
#define RV3028_UNIX_TIME0				0x1B
#define RV3028_UNIX_TIME1				0x1C
#define RV3028_UNIX_TIME2				0x1D
#define RV3028_UNIX_TIME3				0x1E

//RAM registers
#define RV3028_USER_RAM1				0x1F
#define RV3028_USER_RAM2				0x20

//Password registers
#define RV3028_PASSWORD0				0x21
#define RV3028_PASSWORD1				0x22
#define RV3028_PASSWORD2				0x23
#define RV3028_PASSWORD3				0x24

//EEPROM Memory Control registers
#define RV3028_EEPROM_ADDR				0x25
#define RV3028_EEPROM_DATA				0x26
#define RV3028_EEPROM_CMD				0x27

//ID register
#define RV3028_ID						0x28

//EEPROM Registers
#define EEPROM_Clkout_Register			0x35
#define RV3028_EEOffset_8_1				0x36	//bits 8 to 1 of EEOffset. Bit 0 is bit 7 of register 0x37 
#define EEPROM_Backup_Register			0x37


//BITS IN IMPORTANT REGISTERS

//Bits in Status Register
#define STATUS_EEBUSY	7
#define STATUS_CLKF		6
#define STATUS_BSF		5
#define STATUS_UF		4
#define STATUS_TF		3
#define STATUS_AF		2
#define STATUS_EVF		1
#define STATUS_PORF		0

//Bits in Control1 Register
#define CTRL1_TRPT		7
#define CTRL1_WADA		5//Bit 6 not implemented
#define CTRL1_USEL		4
#define CTRL1_EERD		3
#define CTRL1_TE		2
#define	CTRL1_TD1		1
#define CTRL1_TD0		0

//Bits in Control2 Register
#define CTRL2_TSE		7
#define CTRL2_CLKIE		6
#define CTRL2_UIE		5
#define CTRL2_TIE		4
#define CTRL2_AIE		3
#define CTRL2_EIE		2
#define CTRL2_12_24		1
#define CTRL2_RESET		0

//Bits in Hours register
#define HOURS_AM_PM			5

//Bits in Alarm registers
#define MINUTESALM_AE_M		7
#define HOURSALM_AE_H		7
#define DATE_AE_WD			7

//Commands for EEPROM Command Register (0x27)
#define EEPROMCMD_First					0x00
#define EEPROMCMD_Update				0x11
#define EEPROMCMD_Refresh				0x12
#define EEPROMCMD_WriteSingle			0x21
#define EEPROMCMD_ReadSingle			0x22

//Bits in EEPROM Backup Register
#define EEPROMBackup_TCE_BIT			5				//Trickle Charge Enable Bit
#define EEPROMBackup_FEDE_BIT			4				//Fast Edge Detection Enable Bit (for Backup Switchover Mode)
#define EEPROMBackup_BSM_SHIFT			2				//Backup Switchover Mode shift
#define EEPROMBackup_TCR_SHIFT			0				//Trickle Charge Resistor shift

#define EEPROMBackup_BSM_CLEAR			0b11110011		//Backup Switchover Mode clear
#define EEPROMBackup_TCR_CLEAR			0b11111100		//Trickle Charge Resistor clear
#define	TCR_3K							0b00			//Trickle Charge Resistor 3kOhm
#define	TCR_5K							0b01			//Trickle Charge Resistor 5kOhm
#define	TCR_9K							0b10			//Trickle Charge Resistor 9kOhm
#define	TCR_15K							0b11			//Trickle Charge Resistor 15kOhm


// Clock output register (0x35)
#define EEPROMClkout_CLKOE_BIT			7				//1 = CLKOUT pin is enabled. – Default value on delivery 
#define EEPROMClkout_CLKSY_BIT			6
// Bits 5 and 4 not implemented
#define EEPROMClkout_PORIE				  3				//0 = No interrupt, or canceled, signal on INT pin at POR. – Default value on delivery
																		//1 = An interrupt signal on INT pin at POR. Retained until the PORF flag is cleared to 0 (no automatic cancellation). 
#define EEPROMClkout_FREQ_SHIFT			0				//frequency shift
#define FD_CLKOUT_32k					  0b000			  //32.768 kHz – Default value on delivery 
#define FD_CLKOUT_8192					0b001 			//8192 Hz 
#define FD_CLKOUT_1024					0b010			  //1024 Hz
#define FD_CLKOUT_64					  0b011 		  //64 Hz 
#define FD_CLKOUT_32					  0b100			  //32 Hz
#define FD_CLKOUT_1						  0b101 		 	//1 Hz 
#define FD_CLKOUT_TIMER					0b110			  //Predefined periodic countdown timer interrupt 
#define FD_CLKOUT_LOW					  0b111 			//CLKOUT = LOW 


#define IMT_MASK_CEIE					3				//Clock output when Event Interrupt bit. 
#define IMT_MASK_CAIE					2				//Clock output when Alarm Interrupt bit.
#define IMT_MASK_CTIE					1				//Clock output when Periodic Countdown Timer Interrupt bit.
#define IMT_MASK_CUIE					0				//Clock output when Periodic Time Update Interrupt bit.


#define TIME_ARRAY_LENGTH 7 // Total number of writable values in device

enum time_order {
	TIME_SECONDS,    // 0
	TIME_MINUTES,    // 1
	TIME_HOURS,      // 2
	TIME_WEEKDAY,    // 3
	TIME_DATE,       // 4
	TIME_MONTH,      // 5
	TIME_YEAR,       // 6
};

class powerTimer
{
    private:
        uint8_t _time[TIME_ARRAY_LENGTH];
        TwoWire *_wirePort;
        uint8_t _powerOffPin = 0;

        /**
         * @brief Converts a Binary-Coded Decimal (BCD) value to a decimal value.
         * @param val The BCD value to convert.
         * @return The decimal value.
         */
        uint8_t BCDtoDEC(uint8_t val);
        /**
         * @brief Converts a decimal value to a Binary-Coded Decimal (BCD) value.
         * @param val The decimal value to convert.
         * @return The BCD value.
         */
        uint8_t DECtoBCD(uint8_t val);

        /**
         * @brief Reads a single byte from a given register address.
         * @param addr The register address to read from.
         * @return The value read from the register.
         */
        uint8_t readRegister(uint8_t addr);
        /**
         * @brief Writes a single byte to a given register address.
         * @param addr The register address to write to.
         * @param val The value to write.
         * @return True if the write was successful, false otherwise.
         */
        bool writeRegister(uint8_t addr, uint8_t val);
        /**
         * @brief Reads multiple bytes from a starting register address.
         * @param addr The starting register address.
         * @param dest Pointer to the destination buffer.
         * @param len The number of bytes to read.
         * @return True if the read was successful, false otherwise.
         */
        bool readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len);
        /**
         * @brief Writes multiple bytes to a starting register address.
         * @param addr The starting register address.
         * @param values Pointer to the source buffer.
         * @param len The number of bytes to write.
         * @return True if the write was successful, false otherwise.
         */
        bool writeMultipleRegisters(uint8_t addr, uint8_t * values, uint8_t len);

        /**
         * @brief Writes a value to a configuration register in RAM and then updates the corresponding EEPROM cell.
         * @param eepromaddr The EEPROM/RAM address to write to.
         * @param val The value to write.
         * @return True if successful, false otherwise.
         */
        bool writeConfigEEPROM_RAMmirror(uint8_t eepromaddr, uint8_t val);
        /**
         * @brief Reads a value from a configuration register in EEPROM.
         * @param eepromaddr The EEPROM address to read from.
         * @return The value read from the EEPROM.
         */
        uint8_t readConfigEEPROM_RAMmirror(uint8_t eepromaddr);
        /**
         * @brief Writes a value to a user EEPROM address.
         * @param eepromaddr The user EEPROM address to write to.
         * @param val The value to write.
         * @return True if successful, false otherwise.
         */
        bool writeUserEEPROM(uint8_t eepromaddr, uint8_t val);
        /**
         * @brief Reads a value from a user EEPROM address.
         * @param eepromaddr The user EEPROM address to read from.
         * @return The value read from the EEPROM.
         */
        uint8_t readUserEEPROM(uint8_t eepromaddr);
        /**
         * @brief Waits for the EEPROM to finish its current operation.
         * @return True if the EEPROM is ready, false if a timeout occurred.
         */
        bool waitforEEPROM();
        
        /**
         * @brief Sets a specific bit in a register.
         * @param regAddr The address of the register.
         * @param bitNum The bit number to set (0-7).
         */
        void setBit(uint8_t regAddr, uint8_t bitNum);
        /**
         * @brief Clears a specific bit in a register.
         * @param regAddr The address of the register.
         * @param bitNum The bit number to clear (0-7).
         */
        void clearBit(uint8_t regAddr, uint8_t bitNum);
        /**
         * @brief Reads a specific bit from a register.
         * @param regAddr The address of the register.
         * @param bitNum The bit number to read (0-7).
         * @return True if the bit is set, false otherwise.
         */
        bool readBit(uint8_t regAddr, uint8_t bitNum);
        /**
         * @brief Configures the backup battery switchover mode.
         * @param val The mode to set (0-3).
         * @return True if successful, false otherwise.
         */
        bool setBackupSwitchoverMode(uint8_t val);
        /**
         * @brief Disables the clock output pin.
         */
        void disableClockOut();
        /**
         * @brief Sets the RTC to 24-hour mode.
         */
        void set24Hour();
        /**
         * @brief Disables the trickle charger for the backup battery.
         */
        void disableTrickleCharge();

    public:
        /**
         * @brief Construct a new powerTimer object
         */
        powerTimer(){};

        /**
         * @brief Initializes the powerTimer.
         * @param powerOffPin The GPIO pin used to control the power-off functionality.
         * @param wirePort The I2C interface to use. Defaults to Wire.
         * @return True if initialization is successful, false otherwise.
         */
        bool begin(uint8_t powerOffPin, TwoWire &wirePort = Wire);

        /**
         * @brief Sets the time and date on the RTC.
         * @param sec Seconds (0-59).
         * @param min Minutes (0-59).
         * @param hour Hours (0-23).
         * @param weekday Day of the week (0-6, where 0 is Sunday).
         * @param date Day of the month (1-31).
         * @param month Month (1-12).
         * @param year Year (e.g., 2023).
         * @return True if setting the time was successful, false otherwise.
         */
        bool setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year);
 
        /**
         * @brief Configures and enables the alarm interrupt.
         * @param min Minute for the alarm (0-59).
         * @param hour Hour for the alarm (0-23).
         * @param dateOrWeekDay Date (1-31) or weekday (0-6) for the alarm.
         * @param setWeekDayAlarm True to use weekday for the alarm, false to use date.
         */
        void enableAlarmInterrupt(uint8_t min, uint8_t hour, uint8_t dateOrWeekDay, bool setWeekDayAlarm);
        /**
         * @brief Disables the alarm interrupt.
         */
        void disableAlarmInterrupt();
        /**
         * @brief Checks if the alarm interrupt flag is set.
         * @return True if the alarm flag is set, false otherwise.
         */
        bool readAlarmInterruptFlag();
        /**
         * @brief Clears the alarm interrupt flag in the status register.
         */
        void clearAlarmInterruptFlag();

        /**
         * @brief Configures the countdown timer.
         * @param timerRepeat True to make the timer repeat automatically.
         * @param timerValue The countdown value (depends on timer frequency, not set here).
         * @param setInterrupt True to enable the timer interrupt.
         * @param startTimer True to start the timer immediately after configuration.
         */
        bool setTimer(bool timerRepeat, uint16_t timerValue, bool setInterrupt, bool startTimer);

        /**
         * @brief Disables the countdown timer.
         */
        void disableTimer();
        /**
         * @brief Disables the countdown timer interrupt.
         */
        void disableTimerInterrupt();
        /**
         * @brief Checks if the timer interrupt flag is set.
         * @return True if the timer flag is set, false otherwise.
         */
        bool readTimerInterruptFlag();
        /**
         * @brief Clears the timer interrupt flag in the status register.
         */
        void clearTimerInterruptFlag();

        /**
         * @brief Checks the integrity of the RTC time and date.
         * @return True if a power-on reset or backup switchover has occurred, indicating potential data loss. False otherwise.
         */
        bool dateIntegrity();
        /**
         * @brief Returns the value of the status register.
         * @return The 8-bit status register value.
         */
        uint8_t status(); //Returns the status byte
        /**
         * @brief Clears all interrupt flags in the status register.
         */
        void clearInterrupts();

        /**
         * @brief Triggers the power-off mechanism.
         */
        void powerOff();

        /**
         * @brief Resets the RTC device.
         */
        void reset();
};

#endif