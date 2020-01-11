# MCP795-RTC-C-Library
C library for Microship RTC MCP795W1X/W2X.

Library has one global structure called g_rtcTime.

-> set up time: setting g_rtcTime with current time and then calling rtc_set_time()
-> reading time: execute rtc_get_time() and check g_rtcTime

