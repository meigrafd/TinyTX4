TinyTX4
=======

RFM12B based TinyTX Transmitter


Projekt Page: http://www.forum-raspberrypi.de/Thread-messen-steuern-regeln-batteriebetriebene-funk-sensoren


# Compiled Sketch size

### with RFM12B lib:

Send_DHT22.ino --> 5.254 Bytes.  
Send_DHT22_Watchdog.ino --> 5.512 Bytes.  
Send_DHT22.ino & encrypt --> 6.896 Bytes.  
Send_DHT22_Watchdog.ino & encrypt --> 7.154 Bytes.  

Send_DS18B20.ino --> 7.342 Bytes.  
Send_DS18B20_Watchdog.ino --> 7.590 Bytes.  
Send_DS18B20.ino & encrypt --> ... 768 bytes to big.  
Send_DS18B20_Watchdog.ino & encrypt --> ... 1012 bytes to big.  

Send_ReedSwitch_Watchdog.ino --> 4.312 Bytes.  
Send_ReedSwitch_Watchdog.ino & encrypt --> 5.954 Bytes.  

### with JeeLib:

Send_DHT22_Watchdog_JeeLib.ino --> 5.082 Bytes.  
Send_DHT22_Watchdog_JeeLib.ino & encrypt --> 6.746 Bytes.  

Send_DS18B20_Watchdog_JeeLib.ino --> 7.160 Bytes.  
Send_DS18B20_Watchdog_JeeLib.ino & encrypt --> ... 578 bytes to big.  

Send_ReedSwitch_Watchdog_JeeLib.ino --> 3.870 Bytes
Send_ReedSwitch_Watchdog_JeeLib.ino & encrypt --> 5.534 Bytes

Send_BMP085_Watchdog_JeeLib.ino --> 6.266 Bytes
Send_BMP085_Watchdog_JeeLib.ino & encrypt --> 7.894 Bytes
