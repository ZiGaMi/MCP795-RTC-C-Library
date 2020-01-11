//////////////////////////////////////////////////////////////
// 
//	project:		sleep monitor
//	date:			27.12.2019
//	
//	author:			Ziga Miklosic
//
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//	INCLUDES
//////////////////////////////////////////////////////////////
#include "rtc.h"
#include "spi.h"
#include "hw_def.h"

#include "cmsis_os2.h"

//////////////////////////////////////////////////////////////
//	DEFINITIONS
//////////////////////////////////////////////////////////////


#define RTC_CS_low()			( RTC_CS__PORT -> ODR &= ~( RTC_CS__MSK ))
#define RTC_CS_high()			( RTC_CS__PORT -> ODR |= ( RTC_CS__MSK ))


//////////////////////////////////////////////////////////////
//	VARIABLES
//////////////////////////////////////////////////////////////

rtc_time_t g_rtcTime = { 	.wday_name[0] = "Mon",
							.wday_name[1] = "Tue",
							.wday_name[2] = "Wed",
							.wday_name[3] = "Thu",
							.wday_name[4] = "Fri",
							.wday_name[5] = "Sat",
							.wday_name[6] = "Sun"
						};




//////////////////////////////////////////////////////////////
// FUNCTIONS 
//////////////////////////////////////////////////////////////



// ~~~~~~~~~~~~~ RTC PRIVATE FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Send byte via SPI
static uint8_t rtc_send_byte(uint8_t val){
	return (uint8_t) ( spi_acc_send_byte( val ));
}


// Read status register
static uint8_t rtc_read_status_reg(void){

	uint8_t status;

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_SPREAD_cmd );
	status = rtc_send_byte( 0x00u );
	RTC_CS_high();

	return status;
}



// Set write enable latch
// NOTE: Write Enable Latch is only for non-volatile memory(EEPROM, Unique ID and STATUS register)
static void rtc_set_write_enable_latch(bool state){

	RTC_CS_low();
	if ( state ){
		rtc_send_byte( MCP795_ISA_EEWREN_cmd );
	}
	else{
		rtc_send_byte( MCP795_ISA_EEWRDI_cmd );
	}
	RTC_CS_high();
}




// Write byte to SRAM
static void rtc_write_byte_sram(uint8_t addr, uint8_t val){

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_WRITE_cmd );
	rtc_send_byte( addr & 0xffu );
	rtc_send_byte( val & 0xffu );
	RTC_CS_high();
}



// Write page to SRAM
static void rtc_write_blobk_sram(uint8_t addr, uint8_t *buf, uint8_t size){

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_WRITE_cmd );
	rtc_send_byte( addr & 0xffu );

	while(size--){
		rtc_send_byte(( *buf++ ) & 0xffu );
	}

	RTC_CS_high();
}



// Read byte from SRAm
static uint8_t rtc_read_byte_sram(uint8_t addr){

	uint8_t data;

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_READ_cmd );
	rtc_send_byte( addr & 0xffu );
	data = rtc_send_byte( 0x00u );
	RTC_CS_high();

	return ( uint8_t ) ( data );
}




// Read block of SRAM
static uint8_t* rtc_read_block_sram(uint8_t addr, uint8_t size){

	// Reception buffer
	static uint8_t buf[10];

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_READ_cmd );
	rtc_send_byte( addr & 0xffu );

	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( rtc_send_byte( 0x00u ) );
	}

	RTC_CS_high();

	return (uint8_t*) ( &buf );
}



// Write byte to EEPROM
static void rtc_write_byte_eeprom(uint8_t addr, uint8_t val){

	// Enable latch
	while (( rtc_read_status_reg() & MCP795_STATUS_REG_WEL_msk ) != MCP795_STATUS_REG_WEL_msk ){
		rtc_set_write_enable_latch( true );
	}

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_EEWRDI_cmd );
	rtc_send_byte( addr & 0xffu );
	rtc_send_byte( val & 0xffu );
	RTC_CS_high();

	// Wait until write is finished
	while (( rtc_read_status_reg() & MCP795_STATUS_REG_WIP_msk ) == MCP795_STATUS_REG_WIP_msk ){
		osDelay(1);
	}
}


// Write block to EEPROM
static void rtc_write_block_eeprom(uint8_t addr, uint8_t *buf, uint8_t size){

	// Enable latch
	while (( rtc_read_status_reg() & MCP795_STATUS_REG_WEL_msk ) != MCP795_STATUS_REG_WEL_msk ){
		rtc_set_write_enable_latch( true );
	}

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_EEWRITE_cmd );
	rtc_send_byte( addr & 0xffu );

	while(size--){
		rtc_send_byte(( *buf++ ) & 0xffu );
	}

	RTC_CS_high();

	// Wait until write is finished
	while (( rtc_read_status_reg() & MCP795_STATUS_REG_WIP_msk ) == MCP795_STATUS_REG_WIP_msk ){
		osDelay(1);
	}
}


// Read byte from EEPROM
static uint8_t rtc_read_byte_eeprom(uint8_t addr){

	uint8_t data;

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_EEREAD_cmd );
	rtc_send_byte( addr & 0xffu );
	data = rtc_send_byte( 0x00u );
	RTC_CS_high();

	return ( uint8_t ) ( data );
}


// Read block from EEPROM
static uint8_t* rtc_read_block_eeprom(uint8_t addr, uint8_t size){

	// Reception buffer
	static uint8_t buf[10];

	// Check size
	// In case of size is larger than page, reduce size to fit page
	size = ( (( addr & 0x07u ) + size ) > 0x08u  ) ? ( size - ( addr & 0x07u )) : ( size );

	RTC_CS_low();
	rtc_send_byte( MCP795_ISA_EEREAD_cmd );
	rtc_send_byte( addr & 0xffu );

	for ( uint8_t i = 0; i < size; i++ ){
		buf[i] = ( uint8_t ) ( rtc_send_byte( 0x00u ) );
	}

	RTC_CS_high();

	return (uint8_t*) ( &buf );
}


// Start/Stop on board oscillator
static void rtc_enable_on_board_oscillator(bool state){

	if ( state ){

		// Read control register
		uint8_t ctrl_reg = rtc_read_byte_sram(MCP795_CONTROL_REG_addr);

		// Enable external oscillator input
		ctrl_reg |= ( MCP795_CONTROL_REG_SQWE_msk );

		// Set control register
		rtc_write_byte_sram(MCP795_CONTROL_REG_addr, ctrl_reg );

		// Read seconds register that enabling OSC bit doesn't clear up seconds register
		uint8_t sec_reg = rtc_read_byte_sram(MCP795_SECONDS_addr);

		// Start oscillator
		sec_reg |= ( MCP795_SECONDS_START_OSC_msk );

		// Start oscillator
		rtc_write_byte_sram(MCP795_SECONDS_addr, sec_reg);

		// Wait oscillator to start
		osDelay(10);
	}
	else{

		// Stop oscillator
		rtc_write_byte_sram(MCP795_SECONDS_addr, 0x00u);
	}
}



// Binary-coded decimal encoding
static uint8_t rtc_bcd_encoding(uint8_t val){

	if ( val >= 10 ){
		val = ((( val / 10 ) << 4u ) & 0xf0u ) | (( val % 10 ) & 0x0fu );
	}
	return val;
}



// Binary-coded decimal decoding
static uint8_t rtc_bcd_decoding(uint8_t val){
	return ( uint8_t ) (((( val >> 4u ) & 0x0fu ) * 10u ) + ( val & 0x0fu ));
}



// Write calibration factor to eeprom
static void rtc_write_calibration_factor(uint8_t sign, uint8_t factor)
{
	uint8_t buf[3];

	// Calibration factor stored, but device not calibrated jet
	buf[0] = ( RTC_CAL_FACTOR_STATIS_CAL_STORE_MSK );
	
	// Value
	buf[1] = factor;
	
	// Sign
	buf[2] = sign;
	
	// Store calibration factor
	rtc_write_block_eeprom(RTC_CAL_FACTOR_STATUS_STORE_addr, (uint8_t*)&buf, 3u);
}



// Set calibration factor
static void rtc_set_calibration_factor(uint8_t sign, uint8_t factor){

	// Get hour reg
	uint8_t hour_reg = rtc_read_byte_sram(MCP795_HOURS_addr);
	uint8_t eeprom_data = 0;

	// Positive calibration
	if ( sign == RTC_CAL_SIGN_POS ){

		// Clear calibration sign
		hour_reg &= ~( MCP795_HOURS_CALCSGN_msk );
		rtc_write_byte_sram(MCP795_HOURS_addr, hour_reg );
	}

	// Negative calibration
	else if ( sign == RTC_CAL_SIGN_NEG ){

		// Set calibration sign
		hour_reg |= ( MCP795_HOURS_CALCSGN_msk );
		rtc_write_byte_sram(MCP795_HOURS_addr, hour_reg);
	}
	
	// Set calibration factor
	rtc_write_byte_sram(MCP795_CALIBRATION_addr, factor);
	
	// Store calibration factor to eeprom
	rtc_write_calibration_factor(sign, factor);
	
	// Device calibrated store
	eeprom_data = rtc_read_byte_eeprom(RTC_CAL_FACTOR_STATUS_STORE_addr);
	rtc_write_byte_eeprom(RTC_CAL_FACTOR_STATUS_STORE_addr, ( eeprom_data | RTC_CAL_FACTOR_STATUS_DEV_CAL_MSK ));
}



// Read calibration factor
static uint8_t* rtc_read_calibration_factor(){

	uint8_t *buf = rtc_read_block_eeprom(RTC_CAL_FACTOR_STATUS_STORE_addr, 3u);
	return (uint8_t*) buf;
}



//// Get calibration factor
//static uint8_t rtc_get_calibration_factor(void){
//	return ( uint8_t ) ( rtc_read_byte_sram(MCP795_CALIBRATION_addr) );
//}



//// Get calibration factor sign
//static uint8_t rtc_get_calibration_factor_sign(void){
//	uint8_t hour_reg = rtc_read_byte_sram(MCP795_HOURS_addr);
//	return ( uint8_t ) ((( hour_reg & MCP795_HOURS_CALCSGN_msk ) == MCP795_HOURS_CALCSGN_msk ) ? ( RTC_CAL_SIGN_NEG ) : ( RTC_CAL_SIGN_POS ) );
//}





// ~~~~~~~~~~~~~~~ END OF PRIVATE FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~






// ~~~~~~~~~~~~~ RTC PUBLIC FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Initialize RTC specific drivers
void rtc_init(){

	uint8_t ctrl_reg;
	uint8_t *cal;
	
	// Enable oscillator
	rtc_enable_on_board_oscillator(true);

	// Read current day (that day content will not be erased)
	uint8_t wday = rtc_read_byte_sram(MCP795_DAY_addr);

	// Enable external battery
	if (( wday & MCP795_DAY_VBATEN_msk ) != MCP795_DAY_VBATEN_msk ){
		rtc_write_byte_sram(MCP795_DAY_addr, ( MCP795_DAY_VBATEN_msk | ( wday & MCP795_DAY_VAL_msk )));
	}

	// Enable CLKOUT and enable CAL output function
	ctrl_reg = ( MCP795_CONTROL_REG_SQWE_msk | MCP795_CONTROL_REG_RS_2_msk );

	// Set up control register
	rtc_write_byte_sram(MCP795_CONTROL_REG_addr, ctrl_reg);

	// Disable event detect
	rtc_write_byte_sram(MCP795_EVENT_DETECT_addr, 0x00u);

	// Read from EEPROM to check if calibration factors are already there
	cal = rtc_read_calibration_factor();

	// Device is not calibrated yet
	if (( cal[0] & RTC_CAL_FACTOR_STATUS_DEV_CAL_MSK ) != RTC_CAL_FACTOR_STATUS_DEV_CAL_MSK )
	{
		// Device has not been calibrated before
		if (( cal[0] & RTC_CAL_FACTOR_STATIS_CAL_STORE_MSK ) != RTC_CAL_FACTOR_STATIS_CAL_STORE_MSK )
		{
			// Perform calibration
			// TODO: 
			// HW fix -> connect CLKOUT on TIMxCHx pin
		}
		
		// Device has been calibrated before
		else
		{
			// Move calibration info into sram
			rtc_set_calibration_factor(cal[2], cal[1]);
		}
	}
	
	// Device is calibrated
	else
	{
		// No action...
	}




////////////////////////////////////////////////
// 	Manualy set time
////////////////////////////////////////////////
//	g_rtcTime.year 		= 2020;
//	g_rtcTime.month 	= 1;
//	g_rtcTime.mday 		= 11;
//	g_rtcTime.hour 		= 9;
//	g_rtcTime.min 		= 52;
//	g_rtcTime.sec 		= 30;
//	g_rtcTime.wday 		= 5;
//	rtc_set_time();


////////////////////////////////////////////////
// 	Manualy calibrate 
//
//	This is empirically defined factor and sign -> watching CLKOUT line
//	Here calibration factors are stored into eeprom	
//
////////////////////////////////////////////////
	//rtc_write_calibration_factor(RTC_CAL_SIGN_POS, 3);	// With rugh encoder
	//rtc_write_calibration_factor(RTC_CAL_SIGN_POS, 2);	// With smooth encoder
	

}







// Set time
// ERRATA: Write month in separate write session as this is chips bug!!!
void rtc_set_time(void){

	// Stop oscillator
	rtc_enable_on_board_oscillator( false );

	// Read time registers
	uint8_t *time_reg = rtc_read_block_sram(MCP795_SECONDS_addr, 7u);
	uint8_t sec_reg 	= time_reg[0];
	uint8_t hour_reg 	= time_reg[2];
	uint8_t wday_reg 	= time_reg[3];
	uint8_t month_reg	= time_reg[5];

	// 24 h format
	hour_reg &= ~( MCP795_HOURS_12_24_msk | ( 0x01u << 5u ));

	// Encode time
	uint8_t buf[10] = 	{ 	rtc_bcd_encoding( g_rtcTime.sec ) | ( sec_reg & MCP795_SECONDS_START_OSC_msk ),
							rtc_bcd_encoding( g_rtcTime.min ),
							(( rtc_bcd_encoding( g_rtcTime.hour ) & MCP795_HOURS_VAL_msk ) | ( hour_reg & MCP795_HOURS_CTRL_msk )),
							(( g_rtcTime.wday & MCP795_DAY_VAL_msk ) | ( wday_reg & MCP795_DAY_CTRL_msk )),
							rtc_bcd_encoding( g_rtcTime.mday ),
							(( rtc_bcd_encoding( g_rtcTime.month ) & MCP795_MONTH_VAL_msk ) | ( month_reg & MCP795_MONTH_LP_msk )),
							rtc_bcd_encoding( g_rtcTime.year - RTC_PRESET_YEAR )
						};

	// Set time registers
	rtc_write_blobk_sram(MCP795_SECONDS_addr, (uint8_t*) &buf, 5u);

	// Separate month write -> ERRATA
	// Leap year
	if ( 		g_rtcTime.year == 2020u 
			|| 	g_rtcTime.year == 2024u 
			|| 	g_rtcTime.year == 2028u 
		)
	{
		rtc_write_byte_sram(MCP795_MONTH_addr, ( buf[5] | MCP795_MONTH_LP_msk ));
	}
	else{
		rtc_write_byte_sram(MCP795_MONTH_addr, buf[5]);
	}

	// Year
	rtc_write_byte_sram(MCP795_YEAR_addr, buf[6]);

	// Start oscillator
	rtc_enable_on_board_oscillator( true );

//	delay_ms(10);
	osDelay(10);
}



// Update time struct
void rtc_update_time(){

	uint8_t *buf;

	// Read current time
	buf = rtc_read_block_sram(MCP795_SECONDS_addr, 7u);

	// Set current time
	g_rtcTime.sec 	= rtc_bcd_decoding( *buf++ & MCP795_SECONDS_VAL_msk);		// Ignore ST bit
	g_rtcTime.min 	= rtc_bcd_decoding( *buf++ );
	g_rtcTime.hour 	= rtc_bcd_decoding( *buf++ & MCP795_HOURS_VAL_msk );	
	g_rtcTime.wday	= (( *buf++ ) & MCP795_DAY_VAL_msk );
	g_rtcTime.mday	= rtc_bcd_decoding( *buf++ );
	g_rtcTime.month = rtc_bcd_decoding( *buf++ & MCP795_MONTH_VAL_msk );
	g_rtcTime.year 	= rtc_bcd_decoding( *buf++ ) + RTC_PRESET_YEAR;
}


// Return rtc time
rtc_time_t *rtc_get_time()
{
	return (rtc_time_t*) ( &g_rtcTime );
}





//// Set alarm 0
//// NOTE: This alarm will be set to match hours
//void RtcSetAlarm0(bool state, uint8_t hour){

//	// Alarm 0 on hours match
//	rtc_write_byte_sram(MCP795_ALARM0_DAY_addr, MCP795_ALARM0_DAY_ALMOC_1_msk);

//	// Alarm 0 hour reg
//	rtc_write_byte_sram(MCP795_ALARM0_HOURS_addr, rtc_bcd_encoding(hour));

//	// Read control register
//	uint8_t ctrl_reg = rtc_read_byte_sram(MCP795_CONTROL_REG_addr);

//	// Enable alarm
//	if ( state ){
//		ctrl_reg |= ( MCP795_CONTROL_REG_ALM_0_msk );
//	}

//	// Disable alarm
//	else{
//		ctrl_reg &= ~( MCP795_CONTROL_REG_ALM_0_msk );
//	}

//	// Set control register
//	rtc_write_byte_sram(MCP795_CONTROL_REG_addr, ctrl_reg);
//}


//// Alarm 0 status flag
//volatile bool alarm0Status_f = false;

//// Set/Get alarm 0 status flag
//void RtcSetAlarm0StatusFlag(bool status){
//	alarm0Status_f = status;
//}

//bool RtcGetAlarm0StatusFlag(){
//	return alarm0Status_f;
//}

//// Clear IRQ flag
//void RtcClearAlarm0IrqFlag(){

//	// Read alarm 0 day reg
//	uint8_t alarm0_day_reg = rtc_read_byte_sram(MCP795_ALARM0_DAY_addr);

//	// Clear alarm 0 interrupt flag
//	alarm0_day_reg &= ~( MCP795_ALARM0_DAY_ALM0IF_msk );

//	// Set day reg
//	rtc_write_byte_sram(MCP795_ALARM0_DAY_addr, alarm0_day_reg);
//}


//// WDT status flag
//volatile bool wdtStatus_f = false;

//// Set WatchDog timer
//void RtcSetWdt(bool state){

//	// Enable WDT
//	if ( state ){

//		// Set to 16 seconds
//		rtc_write_byte_sram(MCP795_WATCHDOG_addr, ( MCP795_WATCHDOG_WDTEN_msk | MCP795_WATCHDOG_WD_2_msk | MCP795_WATCHDOG_WD_0_msk ));
//	}

//	// Disable WDT
//	else{
//		uint8_t wdt_reg = rtc_read_byte_sram(MCP795_WATCHDOG_addr);
//		wdt_reg &= ~( MCP795_WATCHDOG_WDTEN_msk );
//		rtc_write_byte_sram(MCP795_WATCHDOG_addr, wdt_reg);
//	}
//}

//// Reset WDT
//// NOTE: For reseting internal logic of WDT
//// special command is present
//void RtcResetWdt(){
//	RTC_CS_low();
//	rtc_send_byte(MCP795_ISA_CLRWDT_cmd);
//	RTC_CS_high();
//}

//// Set/Get WDT status flag
//void RtcSetWdtStatusFlag(bool state){
//	wdtStatus_f = state;
//}

//bool RtcGetWdtStatusFlag(){
//	return wdtStatus_f;
//}

//// Clear WDT flag
//void RtcClearWdtIrqFlag(){

//	// Read WDT reg
//	uint8_t wdt_reg = rtc_read_byte_sram(MCP795_WATCHDOG_addr);

//	// Clear iterrupt flag
//	wdt_reg &= ~( MCP795_WATCHDOG_WDTIF_msk );

//	// Set WDT reg
//	rtc_write_byte_sram(MCP795_WATCHDOG_addr, wdt_reg);
//}



//// External interrupt ISR
//void EXTI4_15_IRQHandler(){

//	// Alarm 0 interrupt
//	if (( EXTI -> PR & RTC_IRQ_msk ) == RTC_IRQ_msk ){

//		// Clear pending interrupt
//		EXTI -> PR |= ( RTC_IRQ_msk );

//		// Set alarm 0 flag
//		alarm0Status_f = true;
//	}

//	// WDT interrupt
//	if (( EXTI -> PR & RTC_WDT_msk ) == RTC_WDT_msk ){

//		// Clear pending interrupt
//		EXTI -> PR |= ( RTC_WDT_msk );

//		// Set WDT flag
//		wdtStatus_f = true;

//		// Clear IRQ flag in RTC chip
//		RtcClearWdtIrqFlag();
//	}
//}




//////////////////////////////////////////////////////////////
// END OF FILE
//////////////////////////////////////////////////////////////



