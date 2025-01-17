# MatrixClock

This is an 64x32 RGB LED matrix (hub75 compatible) clock which uses NTP as the primary time source with an DS3231 RTC backup. 
If the RTC is not detected then this is considered a fatal error and an error will continuously be displayed.

This uses an [Adafruit Matrix Portal S3](https://www.adafruit.com/product/5778), but may work with other devices. 

![Matrix Clock demo](images/matrix_clock.gif?raw=true)


### Building

At time of writing, there is an isssue with PlatformIO and it fails to compile this project. 

You need to install the following libraries. Newer versions may work, but these are the versions I used. 
 * Adafruit Protomatter 1.7.0 (must be 1.7.0 or higher)
 * ArduinoJson  7.2.1
 * ArduinoOTA  1.1.0
 * ESPNtpClient  0.2.7
 * ESP32Time  2.0.6
 * MQTT  2.5.2
 * uRTCLib  6.9.2
 * WiFiManager  2.0.17
 * Adafruit NeoPixel 1.12.3



### Usage

To configure, connect to WIFI AP matrix-portal-[last 4 of mac addr]. The default AP password is `MatrixPortal`.

Once booted, get the device IP either from the serial output or your DHCP server and connect to http://[host ip]/ to configure wifi and the various other settings.

To activate DST press the up button, or the down button to deactivate DST.


It optionally can connect to an MQTT broker to receive and display messages. The JSON payload should be:
```
{  
  "message": "your message", 
  "repeat": number_of_time_to_repeat(int) (optional), 
  "color", "0x prefixed rgb565 color (optional)"
}
```

For example:  `{"message": "Doorbell was rang", "color": "0x181f", "repeat": 1}`


### Notes

This is the font used (released under a generic free license): https://www.1001freefonts.com/homespun.font


This project is a continuation of my [RPI Pico Clock Project](https://github.com/jjfalling/RPI-Pico-Clock). However due to performance issues, I had to switch from Micropython to C++.