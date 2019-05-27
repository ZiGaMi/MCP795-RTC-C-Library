/*
 * RTC.c
 *
 *  Created on: 13.05.2019
 *      Author: ziga.miklosic
 */

#include "RTC.h"


// Initialize RTC
void RtcInit(RtcTimeTypeDef *time){

	// Enable oscillator
	RtcEnableOnBoardOscillator(true);

	// Read current day (that day content will not be erased)
	uint8_t wday = RtcReadByteSram(MCP795_DAY_addr);

	// Enable external battery
	if (( wday & MCP795_DAY_VBATEN_msk ) != MCP795_DAY_VBATEN_msk ){
		RtcWriteByteSram(MCP795_DAY_addr, ( MCP795_DAY_VBATEN_msk | ( wday & MCP795_DAY_VAL_msk )));
	}

	// Read control register
	uint8_t ctrl_reg = RtcReadByteSram(MCP795_CONTROL_REG_addr);

	// Enable CLKOUT and enable CAL output function
	ctrl_reg |= ( MCP795_CONTROL_REG_SQWE_msk | MCP795_CONTROL_REG_RS_2_msk );

	// Enable alarm 0
	ctrl_reg |= ( MCP795_CONTROL_REG_ALM_0_msk );

	// Set up control register
	RtcWriteByteSram(MCP795_CONTROL_REG_addr, ctrl_reg);

	// Alarm 0 on hours match
	//uint8_t alarm_0_day_reg = ( MCP795_ALARM0_DAY_ALMOC_1_msk );


}


// Configure SPI for RTC
void RtcSetupSpi(){

	// Wait until communication is done
	while (( RTC_SPI -> SR & SPI_SR_BSY ) == SPI_SR_BSY );

	// Master configuration
	// fPCLK/8 = 1 MHz
	// CPHA=0, CPOL=0
	// Software driven SS
	RTC_SPI -> CR1 = 0u;
	RTC_SPI -> CR1 |= ( SPI_CR1_BR_1 | SPI_CR1_MSTR );

	// NOTE: This configuration is suitable for RTC device.
	// SD Card SPI Mode 0
	// Data size -> 8 bit
	// SS output enable
	RTC_SPI -> CR2 = 0;
	RTC_SPI -> CR2 |= ( SPI_CR2_SSOE );

	// Enable SPI
	RTC_SPI -> CR1 |= SPI_CR1_SPE;

}


// Send byte
uint8_t RtcSendByte(uint8_t val){
	*( uint8_t* ) &( RTC_SPI -> DR ) = ( uint8_t ) ( val );
	while(( RTC_SPI -> SR & SPI_SR_RXNE ) != SPI_SR_RXNE );
	return ( uint8_t ) ( RTC_SPI -> DR );
}


// Read status register
uint8_t RtcReadStatusReg(){

	uint8_t status;

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_SPREAD_cmd );
	status = RtcSendByte( 0x00u );
	RTC_CS_high();

	return status;
}



// Set write enable latch
// NOTE: Write Enable Latch is only for non-volatile memory(EEPROM, Unique ID and STATUS register)
void RtcSetWriteEnableLatch(bool state){

	RTC_CS_low();
	if ( state ){
		RtcSendByte( MCP795_ISA_EEWREN_cmd );
	}
	else{
		RtcSendByte( MCP795_ISA_EEWRDI_cmd );
	}
	RTC_CS_high();
}


// Write byte to SRAM
void RtcWriteByteSram(uint8_t addr, uint8_t val){

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_WRITE_cmd );
	RtcSendByte( addr & 0xffu );
	RtcSendByte( val & 0xffu );
	RTC_CS_high();
}

// Write page to SRAM
void RtcWriteBlockSram(uint8_t addr, uint8_t *buf, uint8_t size){

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_WRITE_cmd );
	RtcSendByte( addr & 0xffu );

	while(size--){
		RtcSendByte(( *buf++ ) & 0xffu );
	}

	RTC_CS_high();
}


// Read byte from SRAm
uint8_t RtcReadByteSram(uint8_t addr){

	uint8_t data;

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_READ_cmd );
	RtcSendByte( addr & 0xffu );
	data = RtcSendByte( 0x00u );
	RTC_CS_high();

	return ( uint8_t ) ( data );
}

// Read block of SRAM
uint8_t* RtcReadBlockSram(uint8_t addr, uint8_t size){

	// Reception buffer
	static uint8_t buf[10];

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_READ_cmd );
	RtcSendByte( addr & 0xffu );

	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( RtcSendByte( 0x00u ) );
	}

	RTC_CS_high();

	return (uint8_t*) ( &buf );
}


// Write byte to EEPROM
void RtcWriteByteEeprom(uint8_t addr, uint8_t val){

	// Enable latch
	while (( RtcReadStatusReg() & MCP795_STATUS_REG_WEL_msk ) != MCP795_STATUS_REG_WEL_msk ){
		RtcSetWriteEnableLatch( true );
	}

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_EEWRDI_cmd );
	RtcSendByte( addr & 0xffu );
	RtcSendByte( val & 0xffu );
	RTC_CS_high();
}


// Read byte from EEPROM
uint8_t RtcReadByteEeprom(uint8_t addr){

	uint8_t data;

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_EEREAD_cmd );
	RtcSendByte( addr & 0xffu );
	data = RtcSendByte( 0x00u );
	RTC_CS_high();

	return ( uint8_t ) ( data );
}


// Start/Stop on board oscillator
void RtcEnableOnBoardOscillator(bool state){

	if ( state ){

		// Read control register
		uint8_t ctrl_reg = RtcReadByteSram(MCP795_CONTROL_REG_addr);

		// Enable external oscillator input
		ctrl_reg |= ( MCP795_CONTROL_REG_SQWE_msk );

		// Set control register
		RtcWriteByteSram(MCP795_CONTROL_REG_addr, ctrl_reg );

		// Read seconds register that enabling OSC bit doesn't clear up seconds register
		uint8_t sec_reg = RtcReadByteSram(MCP795_SECONDS_addr);

		// Start oscillator
		sec_reg |= ( MCP795_SECONDS_START_OSC_msk );

		// Start oscillator
		RtcWriteByteSram(MCP795_SECONDS_addr, sec_reg);

		// Wait oscillator to start
		delay_ms(10);
	}
	else{

		// Stop oscillator
		RtcWriteByteSram(MCP795_SECONDS_addr, 0x00u);
	}
}

// Binary-coded decimal encoding
uint8_t RtcBcdEncoding(uint8_t val){

	if ( val >= 10 ){
		val = ((( val / 10 ) << 4u ) & 0xf0u ) | (( val % 10 ) & 0x0fu );
	}
	return val;
}

// Binary-coded decimal decoding
uint8_t RtcBcdDecoding(uint8_t val){
	return ( uint8_t ) (((( val >> 4u ) & 0x0fu ) * 10u ) + ( val & 0x0fu ));
}


// Set time
// ERRATA: Write month in separate write session as this is chips bug!!!
void RtcSetTime(RtcTimeTypeDef *T){

	// Stop oscillator
	RtcEnableOnBoardOscillator( false );

	// Read time registers
	uint8_t *time_reg = RtcReadBlockSram(MCP795_SECONDS_addr, 7u);
	uint8_t sec_reg 	= time_reg[0];
	uint8_t hour_reg 	= time_reg[2];
	uint8_t wday_reg 	= time_reg[3];
	uint8_t month_reg	= time_reg[5];

	// 24 h format
	hour_reg &= ~( MCP795_HOURS_12_24_msk );

	// Encode time
	uint8_t buf[10] = 	{ 	RtcBcdEncoding( T -> sec ) | ( sec_reg & MCP795_SECONDS_START_OSC_msk ),
							RtcBcdEncoding( T -> min ),
							(( RtcBcdEncoding( T -> hour ) & MCP795_HOURS_VAL_msk ) | ( hour_reg & MCP795_HOURS_CTRL_msk )),
							(( T -> wday & MCP795_DAY_VAL_msk ) | ( wday_reg & MCP795_DAY_CTRL_msk )),
							RtcBcdEncoding( T -> mday ),
							(( RtcBcdEncoding( T -> month ) & MCP795_MONTH_VAL_msk ) | ( month_reg & MCP795_MONTH_LP_msk )),
							RtcBcdEncoding( T -> year - RTC_PRESET_YEAR )
						};

	// Set time registers
	RtcWriteBlockSram(MCP795_SECONDS_addr, (uint8_t*) &buf, 5u);

	// Separate month write -> ERRATA
	// Leap year
	if ( T -> year == 2020u || T -> year == 2024u ){
		RtcWriteByteSram(MCP795_MONTH_addr, ( buf[5] | MCP795_MONTH_LP_msk ));
	}
	else{
		RtcWriteByteSram(MCP795_MONTH_addr, buf[5]);
	}

	// Year
	RtcWriteByteSram(MCP795_YEAR_addr, buf[6]);

	// Start oscillator
	RtcEnableOnBoardOscillator( true );
	delay_ms(10);
}


// Get time
void RtcGetTime(RtcTimeTypeDef *T){

	uint8_t *buf;

	// Read current time
	buf = RtcReadBlockSram(MCP795_SECONDS_addr, 7u);

	// Set current time
	T -> sec 	= RtcBcdDecoding( *buf++ & MCP795_SECONDS_VAL_msk);		// Ignore ST bit
	T -> min 	= RtcBcdDecoding( *buf++ );
	T -> hour 	= RtcBcdDecoding( *buf++ & MCP795_HOURS_VAL_msk );
	T -> wday	= (( *buf++ ) & MCP795_DAY_VAL_msk );
	T -> mday	= RtcBcdDecoding( *buf++ );
	T -> month 	= RtcBcdDecoding( *buf++ & MCP795_MONTH_VAL_msk );
	T -> year 	= RtcBcdDecoding( *buf++ ) + RTC_PRESET_YEAR;
}


// Parse time packet config
void RtcParsePCSetTimeCommand(uint8_t *cmd, RtcTimeTypeDef *T){
	T -> sec	= ( *cmd++ );
	T -> min	= ( *cmd++ );
	T -> hour	= ( *cmd++ );
	T -> wday	= ( *cmd++ );
	T -> mday	= ( *cmd++ );
	T -> month	= ( *cmd++ );
	T -> year	= ( *cmd++ + RTC_PRESET_YEAR );
}


// Set calibration factor
void RtcSetCalibrationFactor(uint8_t sign, uint8_t factor){

	// Get hour reg
	uint8_t hour_reg = RtcReadByteSram(MCP795_HOURS_addr);

	// Positive calibration
	if ( sign == RTC_CAL_SIGN_POS ){

		// Clear calibration sign
		hour_reg &= ~( MCP795_HOURS_CALCSGN_msk );
		RtcWriteByteSram(MCP795_HOURS_addr, hour_reg );
	}

	// Negative calibration
	else if ( sign == RTC_CAL_SIGN_NEG ){

		// Set calibration sign
		hour_reg |= ( MCP795_HOURS_CALCSGN_msk );
		RtcWriteByteSram(MCP795_HOURS_addr, hour_reg);
	}

	// Set calibration factor
	RtcWriteByteSram(MCP795_CALIBRATION_addr, factor);

}


// Get calibration factor
uint8_t RtcGetCalibrationFactor(void){
	return ( uint8_t ) ( RtcReadByteSram(MCP795_CALIBRATION_addr) );
}


// Get calibration factor sign
uint8_t RtcGetCalibrationFactorSign(void){
	uint8_t hour_reg = RtcReadByteSram(MCP795_HOURS_addr);
	return ( uint8_t ) ((( hour_reg & MCP795_HOURS_CALCSGN_msk ) == MCP795_HOURS_CALCSGN_msk ) ? ( RTC_CAL_SIGN_NEG ) : ( RTC_CAL_SIGN_POS ) );
}

