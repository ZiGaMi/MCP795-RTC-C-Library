/*
 * RTC.c
 *
 *  Created on: 13.05.2019
 *      Author: ziga.miklosic
 */

#include "RTC.h"


// Initialize RTC
void RtcInit(){

	// TODO:

}


// Configure SPI for RTC
void RtcSetupSpi(){

	// RTC SPI specifics
	// 8 - bit mode
	// CPOL = 0 (low inactive)
	// CPHA = 0 (leading edge)
	// fPCLK / 32 = 1,5MHz
	// FIFO reception threshold (8-bit)

	// Wait until communication is done
	while (( RTC_SPI -> SR & SPI_SR_BSY ) == SPI_SR_BSY );

	// Change SPI settings
	RTC_SPI -> CR1 = ( SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_SPE );
	RTC_SPI -> CR2 = ( SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_SSOE | SPI_CR2_FRXTH );
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

	//RTC_CS_low();
	SPI_SS_low();

	RtcSendByte( MCP795_ISA_SPREAD_cmd );
	status = RtcSendByte( 0x00u );

	//RTC_CS_high();
	SPI_SS_high();

	return status;
}



// Set write enable latch
// NOTE: Write Enable Latch is only for non-volatile memory(EEPROM, Unique ID and STATUS register)
void RtcSetWriteEnableLatch(bool state){

	SPI_SS_low();
	if ( state ){
		RtcSendByte( MCP795_ISA_EEWREN_cmd );
	}
	else{
		RtcSendByte( MCP795_ISA_EEWRDI_cmd );
	}
	SPI_SS_high();
}


// Write byte to SRAM
void RtcWriteByteSram(uint8_t addr, uint8_t val){

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_WRITE_cmd );
	RtcSendByte( addr & 0xffu );
	RtcSendByte( val & 0xffu );
	SPI_SS_high();
}

// Write page to SRAM
void RtcWriteBlockSram(uint8_t addr, uint8_t *buf, uint8_t size){

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_WRITE_cmd );
	RtcSendByte( addr & 0xffu );

	while(size--){
		RtcSendByte(( *buf++ ) & 0xffu );
	}

	SPI_SS_high();
}


// Read byte from SRAm
uint8_t RtcReadByteSram(uint8_t addr){

	uint8_t data;

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_READ_cmd );
	RtcSendByte( addr & 0xffu );
	data = RtcSendByte( 0x00u );
	SPI_SS_high();

	return ( uint8_t ) ( data );
}

// Read block of SRAM
uint8_t* RtcReadBlockSram(uint8_t addr, uint8_t size){

	// Reception buffer
	static uint8_t buf[10];

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_READ_cmd );
	RtcSendByte( addr & 0xffu );

	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( RtcSendByte( 0x00u ) );
	}

	SPI_SS_high();

	return (uint8_t*) ( &buf );
}


// Write byte to EEPROM
void RtcWriteByteEeprom(uint8_t addr, uint8_t val){

	// Enable latch
	while (( RtcReadStatusReg() & MCP795_STATUS_REG_WEL_msk ) != MCP795_STATUS_REG_WEL_msk ){
		RtcSetWriteEnableLatch( true );
	}

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_EEWRDI_cmd );
	RtcSendByte( addr & 0xffu );
	RtcSendByte( val & 0xffu );
	SPI_SS_high();
}


// Read byte from EEPROM
uint8_t RtcReadByteEeprom(uint8_t addr){

	uint8_t data;

	SPI_SS_low();
	RtcSendByte( MCP795_ISA_EEREAD_cmd );
	RtcSendByte( addr & 0xffu );
	data = RtcSendByte( 0x00u );
	SPI_SS_high();

	return ( uint8_t ) ( data );
}


// Start/Stop on board oscillator
void RtcEnableOnBoardOscillator(bool state){

	if ( state ){

		// Enable external oscillator input
		RtcWriteByteSram(MCP795_CONTROL_REG_addr, MCP795_CONTROL_REG_EXTOSC_msk);

		// Start oscillator
		RtcWriteByteSram(MCP795_SECONDS_addr, MCP795_SECONDS_START_OSC_msk);
	}
	else{

		// Stop oscillator
		RtcWriteByteSram(MCP795_SECONDS_addr, 0x00u);
	}
}

// Binary-coded decimal encoding
uint8_t RtcBcdEncoding(uint8_t val){

	if ( val > 10 ){
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

	// Encode time
	uint8_t buf[10] = 	{ 	RtcBcdEncoding( T -> sec ),
							RtcBcdEncoding( T -> min ),
							( RtcBcdEncoding( T -> hour ) & ~( MCP795_HOURS_12_24_msk )),	// 24-hour format
							T -> wday,
							RtcBcdEncoding( T -> mday ),
							RtcBcdEncoding( T -> month ),
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
}


// Get time
void RtcGetTime(RtcTimeTypeDef *T){

	uint8_t *buf;

	// Read current time
	buf = RtcReadBlockSram(MCP795_SECONDS_addr, 7u);

	// Set current time
	T -> sec 	= RtcBcdDecoding( *buf++ );
	T -> min 	= RtcBcdDecoding( *buf++ );
	T -> hour 	= RtcBcdDecoding( *buf++ );
	T -> wday	= (( *buf++ ) & 0x07u );
	T -> mday	= RtcBcdDecoding( *buf++ );
	T -> month 	= RtcBcdDecoding( *buf++ );
	T -> year 	= RtcBcdDecoding( *buf++ ) + RTC_PRESET_YEAR;
}

