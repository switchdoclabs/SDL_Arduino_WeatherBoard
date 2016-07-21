Weather Board (With the Grove Connectors) Libraries and Example

Supports SwitchDoc Labs WeatherRack / Argent Data / Sparkfun


Version 2.5
SwitchDocLabs
Documentation for Weather Board under products on:

http://www.switchdoc.com/

July 18, 2016


Support for all 7 I2C devices supported by Weather Board


Files include:
        SDL_Arduino_WeatherBoard.ino - main file


other support files and libraries


To Install:

MOVE all of the directories in the libraries folder into your Arduino libraries folder and then restart the Arduino IDE.

Additional library:

https://github.com/rambo/I2C  


------------------
Voltage Note
------------------
If you are running this board at another voltage rather than 5.0V, remember to change this in the SDL_Weather_80422.cpp file:

// For 5V , use 1.0.  For 3.3V (or for ADS1015 or ADS1116)  use 0.66, For Grove Base Shield at 3.3V (required for WeatherPiArdunino but not the Weather Board) use 0.462
#define ADJUST3OR5 1.0
#define PowerVoltage 5.0



