/*
 * RTC.h
 *
 *  Created on: 13.05.2019
 *      Author: ziga.miklosic
 */

#ifndef ACCESSORIES_RTC_H_
#define ACCESSORIES_RTC_H_

#include "stm32l0xx.h"
#include "stdbool.h"

#include "Drivers/SpiDrv.h"
#include "Drivers/ClockDrv.h"

/*******************************************************
 *
 * 	RTC is used for timestamp of collected data
 * 	when storing it on SD Card.
 *
 *	RTC type: Microchip MCP795W10
 *
 *******************************************************/


/*
 * 	Port/Pins
 */

// Watchdog timer
#define RTC_WDT_bp			( 10ul )
#define RTC_WDT_msk			( 0x01u << RTC_WDT_bp )

// Alarm interrupt
#define RTC_IRQ_bp			( 11ul )
#define RTC_IRQ_msk			( 0x01u << RTC_IRQ_bp )



/*
 *		SPI Specifics
 */
#define RTC_SPI					( SPI2 )




/*
 * 	RTC data
 */
typedef struct{

	uint16_t	year;	// 1970..2106
	uint8_t		month;	// 1..12
	uint8_t		mday;	// 1..31
	uint8_t		hour;	// 0..23
	uint8_t		min;	// 0..59
	uint8_t		sec;	// 0..59
	uint8_t		wday;	// 0..6 (Sun..Sat)
}RtcTimeTypeDef;

// RTC hardcoded presnet year
#define RTC_PRESET_YEAR 		( uint16_t ) ( 2019u )

// Calibration sign
#define RTC_CAL_SIGN_POS		( uint8_t ) ( 0x00u )
#define RTC_CAL_SIGN_NEG		( uint8_t ) ( 0x01u )


/*
 * 	Functions
 */

// Initialize RTC
void RtcInit(RtcTimeTypeDef*);

// Configure SPI for RTC
void RtcSetupSpi(void);

// Send byte
uint8_t RtcSendByte(uint8_t);

// Set write enable latch
void RtcSetWriteEnableLatch(bool);

// Write byte to SRAM
void RtcWriteByteSram(uint8_t, uint8_t);

// Write block to SRAM
void RtcWriteBlockSram(uint8_t, uint8_t*, uint8_t);

// Read byte from SRAM
uint8_t RtcReadByteSram(uint8_t);

// Read block of SRAM
uint8_t* RtcReadBlockSram(uint8_t, uint8_t);

// Write byte to EEPROM
void RtcWriteByteEeprom(uint8_t, uint8_t);

// Read byte from EEPROM
uint8_t RtcReadByteEeprom(uint8_t);

// Read status register
uint8_t RtcReadStatusReg(void);

// Start/Stop on board oscillator
void RtcEnableOnBoardOscillator(bool);

// Binary-coded decimal encoding
uint8_t RtcBcdEncoding(uint8_t);

// Binary-coded decimal decoding
uint8_t RtcBcdDecoding(uint8_t);

// Set time
void RtcSetTime(RtcTimeTypeDef*);

// Get time
void RtcGetTime(RtcTimeTypeDef*);

// Get power-up time
void RtcGetPowerUpTime(RtcTimeTypeDef*);

// Get power-down time
void RtcGetPowerDownTime(RtcTimeTypeDef*);

// Parse time packet config
void RtcParsePCSetTimeCommand(uint8_t*, RtcTimeTypeDef*);

// Set calibration factor
void RtcSetCalibrationFactor(uint8_t, uint8_t);

// Get calibration factor
uint8_t RtcGetCalibrationFactor(void);

// Get calibration factor sign
uint8_t RtcGetCalibrationFactorSign(void);



/*
 *		MCP795 RTC Register Addresses
 */

// 		Time and Configuration Registers
// ----------------------------------------------

// Hundredth of seconds
#define MCP795_HUNDREDTH_OF_SEC_addr		( uint8_t ) ( 0x00u )

// Seconds
#define MCP795_SECONDS_addr					( uint8_t )	( 0x01u )
#define MCP795_SECONDS_VAL_msk				( uint8_t ) ( 0x7Fu )
#define MCP795_SECONDS_CTRL_msk				( uint8_t ) ( 0x80u )

#define MCP795_SECONDS_START_OSC_bp			( uint8_t )	( 7u )											// Start/Stop oscillator options. (1 - start, 0 - stop)
#define MCP795_SECONDS_START_OSC_msk		( uint8_t ) ( 0x01u << MCP795_SECONDS_START_OSC_bp )

// Minutes
#define MCP795_MINUTES_addr					( uint8_t ) ( 0x02u )

// Hours
#define MCP795_HOURS_addr					( uint8_t )	( 0x03u )
#define MCP795_HOURS_VAL_msk				( uint8_t ) ( 0x3Fu )
#define MCP795_HOURS_CTRL_msk				( uint8_t ) ( 0xE0u )

#define MCP795_HOURS_CALCSGN_bp				( uint8_t ) ( 7u )											// Calibration Sign bit
#define MCP795_HOURS_CALCSGN_msk			( uint8_t ) ( 0x01u << MCP795_HOURS_CALCSGN_bp )

#define MCP795_HOURS_12_24_bp				( uint8_t ) ( 6u )											// 24h or 12h format (0 - 24-hour format, 1 - 12-hour format)
#define MCP795_HOURS_12_24_msk				( uint8_t ) ( 0x01u << MCP795_HOURS_12_24_bp )

// Day (0-7)
#define MCP795_DAY_addr						( uint8_t ) ( 0x04u )
#define MCP795_DAY_VAL_msk					( uint8_t ) ( 0x07u )
#define MCP795_DAY_CTRL_msk					( uint8_t ) ( 0x38u )

#define MCP795_DAY_OSCON_bp					( uint8_t )	( 5u )											// Oscillator ON bit (set/clear by HW)
#define MCP795_DAY_OSCON_msk				( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )

#define MCP795_DAY_VBAT_bp					( uint8_t ) ( 4u )											// External battery switch flag (1 - supplied via VBAT, 0 - supplied via VCC - set/clear by HW )
#define MCP795_DAY_VBAT_msk					( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )

#define MCP795_DAY_VBATEN_bp				( uint8_t ) ( 3u )											// Switch to external battery (1 - supply via VBAT, 0 - supply via VCC)
#define MCP795_DAY_VBATEN_msk				( uint8_t ) ( 0x01u << MCP795_DAY_VBATEN_bp )

// Date
#define MCP795_DATE_addr					( uint8_t ) ( 0x05u )

// Month
#define MCP795_MONTH_addr					( uint8_t ) ( 0x06u )
#define MCP795_MONTH_VAL_msk				( uint8_t )	( 0x1Fu )

#define MCP795_MONTH_LP_bp					( uint8_t ) ( 5u )											// Leap year ( set during leap year )
#define MCP795_MONTH_LP_msk					( uint8_t ) ( 0x01u << MCP795_MONTH_LP_bp )

// Year
#define MCP795_YEAR_addr					( uint8_t )	( 0x07u )

// Control register
#define MCP795_CONTROL_REG_addr				( uint8_t )	( 0x08u )

#define MCP795_CONTROL_REG_OUT_bp			( uint8_t ) ( 7u )											// Output polarity of CLKOUT pin
#define MCP795_CONTROL_REG_OUT_msk			( uint8_t ) ( MCP795_CONTROL_REG_OUT_bp )

#define MCP795_CONTROL_REG_SQWE_bp			( uint8_t ) ( 6u )											// Squarewave enable bit
#define MCP795_CONTROL_REG_SQWE_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_SQWE_bp )

#define MCP795_CONTROL_REG_ALM_0_bp			( uint8_t ) ( 4u )											// Alarm Configuration bits
#define MCP795_CONTROL_REG_ALM_0_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_ALM_0_bp )
#define MCP795_CONTROL_REG_ALM_1_bp			( uint8_t ) ( 5u )
#define MCP795_CONTROL_REG_ALM_1_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_ALM_1_bp )
#define MCP795_CONTROL_REG_ALM_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_ALM_0_bp )

#define MCP795_CONTROL_REG_EXTOSC_bp		( uint8_t ) ( 3u )											// External Oscillator Input bit
#define MCP795_CONTROL_REG_EXTOSC_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_EXTOSC_bp )

#define MCP795_CONTROL_REG_RS_0_bp			( uint8_t ) ( 0u )											// Calibration Mode bits
#define MCP795_CONTROL_REG_RS_0_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_0_bp )
#define MCP795_CONTROL_REG_RS_1_bp			( uint8_t ) ( 1u )
#define MCP795_CONTROL_REG_RS_1_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_1_bp )
#define MCP795_CONTROL_REG_RS_2_bp			( uint8_t ) ( 2u )
#define MCP795_CONTROL_REG_RS_2_msk			( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_RS_2_bp )
#define MCP795_CONTROL_REG_RS_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_RS_0_bp )

// Calibration
#define MCP795_CALIBRATION_addr				( uint8_t ) ( 0x09u )

// Watchdog
#define MCP795_WATCHDOG_addr				( uint8_t ) ( 0x0Au )

// Event detect
#define MCP795_EVENT_DETECT_addr			( uint8_t ) ( 0x0Bu )

// 		Alarm 0 Registers
// ----------------------------------------------

// Seconds
#define MCP795_ALARM0_SECONDS_addr			( uint8_t ) ( 0x0Cu )

// Minutes
#define MCP795_ALARM0_MINUTES_addr			( uint8_t ) ( 0x0Du )

// Hours
#define MCP795_ALARM0_HOURS_addr			( uint8_t ) ( 0x0Eu )

// Day
#define MCP795_ALARM0_DAY_addr				( uint8_t ) ( 0x0Fu )

#define MCP795_ALARM0_DAY_ALM0PIN_bp		( uint8_t ) ( 7u )											// Alarm 0 Output Pin Configuration bit

#define MCP795_ALARM0_DAY_ALM0PIN_msk		( uint8_t ) ( 0x01u << MCP795_ALARM0_DAY_ALM0PIN_bp )

#define MCP795_ALARM0_DAY_ALM0C_0_bp		( uint8_t ) ( 4u )											// Alarm 0 Configuration bits
#define MCP795_ALARM0_DAY_ALMOC_0_msk		( uint8_t ) ( 0x01u << MCP795_ALARM0_DAY_ALM0C_0_bp )
#define MCP795_ALARM0_DAY_ALM0C_1_bp		( uint8_t ) ( 5u )
#define MCP795_ALARM0_DAY_ALMOC_1_msk		( uint8_t ) ( 0x01u << MCP795_ALARM0_DAY_ALM0C_1_bp )
#define MCP795_ALARM0_DAY_ALM0C_2_bp		( uint8_t ) ( 6u )
#define MCP795_ALARM0_DAY_ALMOC_2_msk		( uint8_t ) ( 0x01u << MCP795_ALARM0_DAY_ALM0C_2_bp )
#define MCP795_ALARM0_DAY_ALM0C_msk			( uint8_t ) ( 0x07u << MCP795_ALARM0_DAY_ALM0C_0_bp )

#define MCP795_ALARM0_DAY_ALM0IF_bp			( uint8_t ) ( 3u )											// Alarm 0 Interrupt Flag (set by HW)
#define MCP795_ALARM0_DAY_ALM0IF_msk		( uint8_t ) ( 0x01u << MCP795_ALARM0_DAY_ALM0IF_bp )

// Date
#define MCP795_ALARM0_DATE_addr				( uint8_t ) ( 0x10 )

// Month
#define MCP795_ALARM0_MONTH_addr			( uint8_t ) ( 0x11 )


// 		Alarm 1 Registers
// ----------------------------------------------

// Hundredth of seconds
#define MCP795_ALARM1_HUNDREDTH_OF_SEC_addr	( uint8_t ) ( 0x12u )

// Seconds
#define MCP795_ALARM1_SECONDS_addr			( uint8_t ) ( 0x13u )

// Minutes
#define MCP795_ALARM1_MINUTES_addr			( uint8_t ) ( 0x14u )

// Hours
#define MCP795_ALARM1_HOURS_addr			( uint8_t ) ( 0x15u )

// Day
#define MCP795_ALARM1_DAY_addr				( uint8_t ) ( 0x16u )

#define MCP795_ALARM1_DAY_ALM1PIN_bp		( uint8_t ) ( 7u )											// Alarm 1 Output Pin Configuration bit
#define MCP795_ALARM1_DAY_ALM1PIN_msk		( uint8_t ) ( 0x01u << MCP795_ALARM1_DAY_ALM1PIN_bp )

#define MCP795_ALARM1_DAY_ALM1C_bp			( uint8_t ) ( 4u )											// Alarm 1 Configuration bits
#define MCP795_ALARM1_DAY_ALM1C_msk			( uint8_t ) ( 0x07u << MCP795_ALARM1_DAY_ALM1C_bp )

#define MCP795_ALARM1_DAY_ALM1IF_bp			( uint8_t ) ( 3u )											// Alarm 1 Interrupt Flag (set by HW)
#define MCP795_ALARM1_DAY_ALM1IF_msk		( uint8_t ) ( 0x01u << MCP795_ALARM1_DAY_ALM1IF_bp )

// Date
#define MCP795_ALARM1_DATE_addr				( uint8_t ) ( 0x17 )


// 		Power Down Time Stamp Registers
// ----------------------------------------------

// Minutes
#define MCP795_PWRDOWN_MINUTES_addr			( uint8_t ) ( 0x18u )

// Hours
#define MCP795_PWRDOWN_HOURS_addr			( uint8_t ) ( 0x19u )

// Date
#define MCP795_PWRDOWN_DATE_addr			( uint8_t ) ( 0x1Au )

// Month
#define MCP795_PWRDOWN_MONTH_addr			( uint8_t ) ( 0x1Bu )


// 		Power Up Time Registers
// ----------------------------------------------

// Minutes
#define MCP795_PWRUP_MINUTES_addr			( uint8_t ) ( 0x1Cu )

// Hours
#define MCP795_PWRUP_HOURS_addr				( uint8_t ) ( 0x1Du )

// Date
#define MCP795_PWRUP_DATE_addr				( uint8_t ) ( 0x1Eu )

// Month
#define MCP795_PWRUP_MONTH_addr				( uint8_t ) ( 0x1Fu )


// 		Status Register (addressed by SRREAD cmd)
// ----------------------------------------------

// Write In Protection bit
#define MCP795_STATUS_REG_WIP_bp			( uint8_t ) ( 0u )
#define MCP795_STATUS_REG_WIP_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_WIP_bp )

// Write Enable Latch
#define MCP795_STATUS_REG_WEL_bp			( uint8_t ) ( 1u )
#define MCP795_STATUS_REG_WEL_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_WEL_bp )

// Block protection
#define MCP795_STATUS_REG_BP0_bp			( uint8_t ) ( 2u )
#define MCP795_STATUS_REG_BP0_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_BP0_bp )

#define MCP795_STATUS_REG_BP1_bp			( uint8_t ) ( 3u )
#define MCP795_STATUS_REG_BP1_msk			( uint8_t ) ( 0x01u << MCP795_STATUS_REG_BP1_bp )




/*
 *		Instruction Set
 */

// Read from EEPROM
#define MCP795_ISA_EEREAD_cmd				( uint8_t ) ( 0x03u )

// Write from EEPROM
#define MCP795_ISA_EEWRITE_cmd				( uint8_t ) ( 0x02u )

// Reset write enable latch ( disable write operation )
#define MCP795_ISA_EEWRDI_cmd				( uint8_t ) ( 0x04u )

// Set the write enable latch ( enable write operation )
#define MCP795_ISA_EEWREN_cmd				( uint8_t ) ( 0x06u )

// Read STATUS register
#define MCP795_ISA_SPREAD_cmd				( uint8_t ) ( 0x05u )

// Write STATUS register
#define MCP795_ISA_SPWRITE_cmd				( uint8_t ) ( 0x01u )

// Read RTCC/SRAM array
#define MCP795_ISA_READ_cmd					( uint8_t ) ( 0x13u )

// Write RTC/SRAM array
#define MCP795_ISA_WRITE_cmd				( uint8_t ) ( 0x12u )

// Unlock ID location
#define MCP795_ISA_UNLOCK_cmd				( uint8_t ) ( 0x14u )

// Write to the ID locations
#define MCP795_ISA_IDWRITE_cmd				( uint8_t ) ( 0x34u )

// Read the ID locations
#define MCP795_ISA_IDREAD_cmd				( uint8_t ) ( 0x33u )

// Clear watchdog timer
#define MCP795_ISA_CLRWDT_cmd				( uint8_t ) ( 0x44u )

// Clear RAM location '0'
#define MCP795_ISA_CLRRAM					( uint8_t ) ( 0x54u )


#endif /* ACCESSORIES_RTC_H_ */
