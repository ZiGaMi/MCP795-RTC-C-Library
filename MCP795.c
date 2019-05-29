/*
 * RTC.c
 *
 *  Created on: 13.05.2019
 *      Author: ziga.miklosic
 */

#include "RTC.h"


// Initialize RTC specific drivers
void RtcInit(RtcTimeTypeDef *time){


	/*
	 * 	Setup SPI for RTC
	 */
	SpiAccessInit();


	/*
	 * 		Setup IRQ and WDT line
	 */

	// Enable clock for GPIOB
	RCC -> IOPENR |= ( RCC_IOPENR_GPIOBEN );

	// Input mode
	RTC_port -> MODER &= ~(( 0x03u << ( 2ul * RTC_IRQ_bp )) |
						   ( 0x03u << ( 2ul * RTC_WDT_bp )));

	// System configure clock
	RCC -> APB2ENR |= ( RCC_APB2ENR_SYSCFGEN );

	// External interrupt on PB11 and PB10
	SYSCFG -> EXTICR[2] |= (( 0x01u << ( 4ul * ( RTC_IRQ_bp - 8u ))) |
							( 0x01u << ( 4ul * ( RTC_WDT_bp - 8u ))));

	// Unmask interrupt
	EXTI -> IMR |= ( RTC_IRQ_msk | RTC_WDT_msk );

	// Falling edge only
	EXTI -> FTSR |= ( RTC_IRQ_msk | RTC_WDT_msk );

	// Clear pending interrupt
	EXTI -> PR |= ( RTC_IRQ_msk | RTC_WDT_msk );



	/*
	 * 		RTC Initialization
	 */

	// Enable oscillator
	RtcEnableOnBoardOscillator(true);

	// Read current day (that day content will not be erased)
	uint8_t wday = RtcReadByteSram(MCP795_DAY_addr);

	// Enable external battery
	if (( wday & MCP795_DAY_VBATEN_msk ) != MCP795_DAY_VBATEN_msk ){
		RtcWriteByteSram(MCP795_DAY_addr, ( MCP795_DAY_VBATEN_msk | ( wday & MCP795_DAY_VAL_msk )));
	}

	// Enable CLKOUT and enable CAL output function
	uint8_t ctrl_reg = ( MCP795_CONTROL_REG_SQWE_msk | MCP795_CONTROL_REG_RS_2_msk );

	// Set up control register
	RtcWriteByteSram(MCP795_CONTROL_REG_addr, ctrl_reg);

	// Disable event detect
	RtcWriteByteSram(MCP795_EVENT_DETECT_addr, 0x00u);

	// Read from EEPROM to check if calibration factors are already there
	uint8_t *cal_factor = RtcReadCalibrationFactor();

	// Device was once calibrated, therefore move calibration factors from EEPROM to CAL registers
	if ( *cal_factor == RTC_CAL_FACTOR_STORE_STATUS_calibrated ){
		RtcSetCalibrationFactor( *(cal_factor + 1u), *(cal_factor + 2u));
	}

	// Device was never calibrated, therefore calibration routine must perform
	else if ( *cal_factor == RTC_CAL_FACTOR_STORE_STATUS_none ){
		// TODO:
	}

	// Enable Alarm 0
	RtcSetAlarm0(true, DATA_COLLECTOR_START_HOUR);

	// Configure WDT
	RtcResetWdt();

	// Enable interrupt on alarm 0
	NVIC_EnableIRQ(EXTI4_15_IRQn);
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

	// Wait until write is finished
	while (( RtcReadStatusReg() & MCP795_STATUS_REG_WIP_msk ) == MCP795_STATUS_REG_WIP_msk ){
		delay_ms(1);
	}
}

// Write block to EEPROM
void RtcWriteBlockEeprom(uint8_t addr, uint8_t *buf, uint8_t size){

	// Enable latch
	while (( RtcReadStatusReg() & MCP795_STATUS_REG_WEL_msk ) != MCP795_STATUS_REG_WEL_msk ){
		RtcSetWriteEnableLatch( true );
	}

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_EEWRITE_cmd );
	RtcSendByte( addr & 0xffu );

	while(size--){
		RtcSendByte(( *buf++ ) & 0xffu );
	}

	RTC_CS_high();

	// Wait until write is finished
	while (( RtcReadStatusReg() & MCP795_STATUS_REG_WIP_msk ) == MCP795_STATUS_REG_WIP_msk ){
		delay_ms(1);
	}
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

// Read block from EEPROm
uint8_t* RtcReadBlockEeprom(uint8_t addr, uint8_t size){

	// Reception buffer
	static uint8_t buf[10];

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	RtcSendByte( MCP795_ISA_EEREAD_cmd );
	RtcSendByte( addr & 0xffu );

	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( RtcSendByte( 0x00u ) );
	}

	RTC_CS_high();

	return (uint8_t*) ( &buf );
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
	hour_reg &= ~( MCP795_HOURS_12_24_msk | ( 0x01u << 5u ));

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

// Store calibration factor
void RtcStoreCalibrationFactor(uint8_t sign, uint8_t factor){

	uint8_t buf[3] = 	{	RTC_CAL_FACTOR_STORE_STATUS_calibrated,
							sign,
							factor
						};

	// Store to EEPROM
	RtcWriteBlockEeprom(RTC_CAL_FACTOR_STATUS_STORE_addr, (uint8_t*) &buf, 3u);
}

// Read calibration factor
uint8_t* RtcReadCalibrationFactor(){

	uint8_t *buf = RtcReadBlockEeprom(RTC_CAL_FACTOR_STATUS_STORE_addr, 3u);
	return (uint8_t*) buf;
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


// Set alarm 0
// NOTE: This alarm will be set to match hours
void RtcSetAlarm0(bool state, uint8_t hour){

	// Alarm 0 on hours match
	RtcWriteByteSram(MCP795_ALARM0_DAY_addr, MCP795_ALARM0_DAY_ALMOC_1_msk);

	// Alarm 0 hour reg
	RtcWriteByteSram(MCP795_ALARM0_HOURS_addr, RtcBcdEncoding(hour));

	// Read control register
	uint8_t ctrl_reg = RtcReadByteSram(MCP795_CONTROL_REG_addr);

	// Enable alarm
	if ( state ){
		ctrl_reg |= ( MCP795_CONTROL_REG_ALM_0_msk );
	}

	// Disable alarm
	else{
		ctrl_reg &= ~( MCP795_CONTROL_REG_ALM_0_msk );
	}

	// Set control register
	RtcWriteByteSram(MCP795_CONTROL_REG_addr, ctrl_reg);
}


// Alarm 0 status flag
volatile bool alarm0Status_f = false;

// Set/Get alarm 0 status flag
void RtcSetAlarm0StatusFlag(bool status){
	alarm0Status_f = status;
}

bool RtcGetAlarm0StatusFlag(){
	return alarm0Status_f;
}

// Clear IRQ flag
void RtcClearAlarm0IrqFlag(){

	// Read alarm 0 day reg
	uint8_t alarm0_day_reg = RtcReadByteSram(MCP795_ALARM0_DAY_addr);

	// Clear alarm 0 interrupt flag
	alarm0_day_reg &= ~( MCP795_ALARM0_DAY_ALM0IF_msk );

	// Set day reg
	RtcWriteByteSram(MCP795_ALARM0_DAY_addr, alarm0_day_reg);
}


// WDT status flag
volatile bool wdtStatus_f = false;

// Set WatchDog timer
void RtcSetWdt(bool state){

	// Enable WDT
	if ( state ){

		// Set to 16 seconds
		RtcWriteByteSram(MCP795_WATCHDOG_addr, ( MCP795_WATCHDOG_WDTEN_msk | MCP795_WATCHDOG_WD_2_msk | MCP795_WATCHDOG_WD_0_msk ));
	}

	// Disable WDT
	else{
		uint8_t wdt_reg = RtcReadByteSram(MCP795_WATCHDOG_addr);
		wdt_reg &= ~( MCP795_WATCHDOG_WDTEN_msk );
		RtcWriteByteSram(MCP795_WATCHDOG_addr, wdt_reg);
	}
}

// Reset WDT
// NOTE: For reseting internal logic of WDT
// special command is present
void RtcResetWdt(){
	RTC_CS_low();
	RtcSendByte(MCP795_ISA_CLRWDT_cmd);
	RTC_CS_high();
}

// Set/Get WDT status flag
void RtcSetWdtStatusFlag(bool state){
	wdtStatus_f = state;
}

bool RtcGetWdtStatusFlag(){
	return wdtStatus_f;
}

// Clear WDT flag
void RtcClearWdtIrqFlag(){

	// Read WDT reg
	uint8_t wdt_reg = RtcReadByteSram(MCP795_WATCHDOG_addr);

	// Clear iterrupt flag
	wdt_reg &= ~( MCP795_WATCHDOG_WDTIF_msk );

	// Set WDT reg
	RtcWriteByteSram(MCP795_WATCHDOG_addr, wdt_reg);
}



// External interrupt ISR
void EXTI4_15_IRQHandler(){

	// Alarm 0 interrupt
	if (( EXTI -> PR & RTC_IRQ_msk ) == RTC_IRQ_msk ){

		// Clear pending interrupt
		EXTI -> PR |= ( RTC_IRQ_msk );

		// Set alarm 0 flag
		alarm0Status_f = true;
	}

	// WDT interrupt
	if (( EXTI -> PR & RTC_WDT_msk ) == RTC_WDT_msk ){

		// Clear pending interrupt
		EXTI -> PR |= ( RTC_WDT_msk );

		// Set WDT flag
		wdtStatus_f = true;

		// Clear IRQ flag in RTC chip
		RtcClearWdtIrqFlag();
	}
}




