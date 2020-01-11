## MCP795-RTC-C-Library
C library for Microship RTC MCP795W1X/W2X.

#### Library has one global variable called g_rtcTime.

Definition of g_rtcTime:
```
typedef struct
{
	const char*   wday_name[7];		
	uint16_t	    year;			  	  
	uint8_t		    month;				  
	uint8_t		    mday;				   
	uint8_t		    hour;				   
	uint8_t		    min;				    
	uint8_t		    sec;				    
	uint8_t		    wday;				    
}rtc_time_t;
```

#### Setting time
Set g_rtcTime with current time and then calling rtc_set_time(). It will store current time into device registers and start RTC calender. 

#### Getting time 
Execute rtc_get_time() which reads time from device and stores it into g_rtcTime.

