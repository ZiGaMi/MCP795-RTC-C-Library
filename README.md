## MCP795-RTC-C-Library
C library for Microship RTC MCP795W1X/W2X.

#### Library has one global variable called g_rtcTime.

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

rtc_time_t g_rtcTime = { 	.wday_name[0] = "Mon",
				.wday_name[1] = "Tue",
				.wday_name[2] = "Wed",
				.wday_name[3] = "Thu",
				.wday_name[4] = "Fri",
				.wday_name[5] = "Sat",
				.wday_name[6] = "Sun"
			};
```

#### Setting time
Set g_rtcTime with current time and then calling rtc_set_time(). It will store current time into device registers and start RTC calender. 

#### Getting time 
Execute rtc_get_time() which reads time from device and stores it into g_rtcTime.

