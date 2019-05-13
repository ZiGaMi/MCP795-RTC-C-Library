/*
 * RTC.h
 *
 *  Created on: 13.05.2019
 *      Author: ziga.miklosic
 */


/*
 *		MCP795 RTC Register Addresses
 */

// 		Time and Configuration Registers
// ----------------------------------------------

// Hundredth of seconds
#define MCP795_HUNDREDTH_OF_SEC_addr		( uint8_t ) ( 0x00u )

// Seconds
#define MCP795_SECONDS_addr					( uint8_t )	( 0x01u )

#define MCP795_SECONDS_START_OSC_bp			( uint8_t )	( 7u )											// Start/Stop oscillator options. (1 - start, 0 - stop)
#define MCP795_SECONDS_START_OSC_msk		( uint8_t ) ( 0x01u << MCP795_SECONDS_START_OSC_bp )

// Minutes
#define MCP795_MINUTES_addr					( uint8_t ) ( 0x02u )

// Hours
#define MCP795_HOURS_addr					( uint8_t )	( 0x03u )

// Day (0-7)
#define MCP795_DAY_addr						( uint8_t ) ( 0x04u )

#define MCP795_DAY_OSCON_bp					( uint8_t )	( 5u )											// Oscillator ON bit (set/clear by HW)
#define MCP795_DAY_OSCON_msk				( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )

#define MCP795_DAY_VBAT_bp					( uint8_t ) ( 4u )											// External battery switch flag (1 - supplied via VBAT, 0 - supplied via VCC - set/clear by HW )
#define MCP795_DAY_VBAT_msk					( uint8_t ) ( 0x01u << MCP795_DAY_OSCON_bp )

#define MCP795_DAY_VBATEN_bp				( uint8_t ) ( 3u )											// Switch to external battery (1 - supply via VBAT, 0 - supply via VCC)
#define MCP795_DAY_VBATEN_bp				( uint8_t ) ( 0x01u << MCP795_DAY_VBATEN_bp )

// Date
#define MCP795_DATE_addr					( uint8_t ) ( 0x05u )

// Month
#define MCP795_MONTH_addr					( uint8_t ) ( 0x06u )

#define MCP795_MONTH_LP_bp					( uint8_t ) ( 5u )											// Leap year ( set during leap year )
#define MCP795_MONTH_LP_msk					( uint8_t ) ( 0x01u << MCP795_MONTH_LP_bp )

// Year
#define MCP795_YEAR_addr					( uint8_t )	( 0x07u )

// Control register
#define MCP795_CONTROL_REG_addr				( uint8_t )	( 0x08u )

#define MCP795_CONTROL_REG_OUT_bp			( uint8_t ) ( 7u )											// Output polarity of CLKOUT pin
#define MCP795_CONTROL_REG_OUT_msk			( uint8_t ) ( MCP795_CONTROL_REG_OUT_bp )

#define MCP795_CONTROL_REG_SQWE_bp			( uint8_t ) ( 6u )											// Squarewave enable bit
#define MCP795_CONTROL_REG_SQWE_msk			( uint8_t ) ( MCP795_CONTROL_REG_SQWE_bp )

#define MCP795_CONTROL_REG_ALM_bp			( uint8_t ) ( 4u )											// Alarm Configuration bits
#define MCP795_CONTROL_REG_ALM_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_ALM_bp )

#define MCP795_CONTROL_REG_EXTOSC_bp		( uint8_t ) ( 3u )											// External Oscillator Input bit
#define MCP795_CONTROL_REG_EXTOSC_msk		( uint8_t ) ( 0x01u << MCP795_CONTROL_REG_EXTOSC_bp )

#define MCP795_CONTROL_REG_RS_bp			( uint8_t ) ( 0u )											// Calibration Mode bits
#define MCP795_CONTROL_REG_RS_msk			( uint8_t ) ( 0x03u << MCP795_CONTROL_REG_RS_bp )

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

#define MCP795_ALARM0_DAY_ALM0C_bp			( uint8_t ) ( 4u )											// Alarm 0 Configuration bits
#define MCP795_ALARM0_DAY_ALM0C_msk			( uint8_t ) ( 0x07u << MCP795_ALARM0_DAY_ALM0C_bp )

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
