// Filename SDL_Arduino_WeatherBoard.ino
// Version 2.5 July 20, 2016
// SwitchDoc Labs, LLC
//
// Version 1.6 - Added support for ADS1015 and for MOD-1016 Lightning Detector
// Added support Weather Board
//





#include <SPI.h>




#define DEBUG


#include "MemoryFree.h"
#include "avr/pgmspace.h"

#include "elapsedMillis.h"




#include <Time.h>
#include <Wire.h>



long messageCount;


float windSpeedMin;
float windSpeedMax;
float windGustMin;
float windGustMax;
float windDirectionMin;
float windDirectionMax;


float currentWindSpeed;
float currentWindGust;

float rainTotal;

// weather

#define WpinLED     13   // LED connected to digital pin 13
#define WpinAnem    3  
#define WpinRain    2

   
#define WintAnem    1
#define WintRain    0

// for mega, have to use Port B - only Port B works.
/*
 Arduino Pins         PORT
 ------------         ----
 Digital 0-7          D
 Digital 8-13         B
 Analog  0-5          C
*/




#include "SDL_Weather_80422.h"


SDL_Weather_80422 weatherStation(WpinAnem, WpinRain, WintAnem, WintRain, A0, SDL_MODE_INTERNAL_AD);

// Mod-1016 - AS3935 Lightning Detector


#include "I2C.h"
#include <AS3935.h>

void printAS3935Registers();


// Interrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;

// Library object initialization First argument is interrupt pin, second is device I2C address
AS3935 AS3935(3, 0x03);



void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  Serial.print("Noise floor is: ");
  Serial.println(noiseFloor, DEC);
  Serial.print("Spike rejection is: ");
  Serial.println(spikeRejection, DEC);
  Serial.print("Watchdog threshold is: ");
  Serial.println(watchdogThreshold, DEC);
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935Irq()
{
  AS3935IrqTriggered = 1;
}

// RasPiConnect



// DS3231 Library functions

#include "SDL_ARDUINO_DS3231.h"

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void printDigits(int digits) {
  // utility function for digital clock display: prints an leading 0

  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);


}

void buildTimeString(char returnString[], char *buffer2, tmElements_t convertTime)
{



  char myBuffer[5];
  sprintf(myBuffer, "%i-", tmYearToCalendar(convertTime.Year));
  strcat(returnString, myBuffer);

  return2Digits(myBuffer, myBuffer, convertTime.Month);
  strcat(returnString, myBuffer);
  strcat(returnString, "-");

  return2Digits(myBuffer, myBuffer, convertTime.Day);
  strcat(returnString, myBuffer);
  strcat(returnString, " ");

  return2Digits(myBuffer, myBuffer, convertTime.Hour);
  strcat(returnString, myBuffer);
  strcat(returnString, ":");

  return2Digits(myBuffer, myBuffer, convertTime.Minute);
  strcat(returnString, myBuffer);
  strcat(returnString, ":");

  return2Digits(myBuffer, myBuffer, convertTime.Second);
  strcat(returnString, myBuffer);



}

// AT24C32 EEPROM

#include "AT24C32.h"

// Adafruit FRAM

#include <Adafruit_FRAM_I2C.h>

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

#include "FRAMLog.h"

// AM2316
// Connect RED of the AM2315 sensor to 5.0V
// Connect BLACK to Ground
// Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

#include "ESG_AM2315.h"

ESG_AM2315 am2315;

float dataAM2315[2];  //Array to hold data returned by sensor.  [0,1] => [Humidity, Temperature]
boolean AM2315OK;  // 1=successful read


// BMP280 Sensor



#define SENSORS_PRESSURE_SEALEVELHPA 1013.0
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;



// HTU21D-F Humidity Sensor
#include "Adafruit_HTU21DF.h"

Adafruit_HTU21DF htu = Adafruit_HTU21DF();




#include <stdarg.h>
void p(char *fmt, ... ) {
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}






elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs

bool BMP280Present;
bool HTU21DFPresent;
bool AS3935Present;
bool DS3231Present;
bool FRAMPresent;
bool AM2315Present;

void reportMyDevice(String myString, bool myStatus)
{

  Serial.print(myString + ":    \t");
  if (myStatus)
  {
    Serial.println(" Present");

  }
  else
  {

    Serial.println(" Not Present");
  }
}


void setup()
{

  Serial.begin(115200);           // set up Serial library at 115200 bps

  // set up Present variables

  BMP280Present = false;
  HTU21DFPresent = false;
  AS3935Present = false;
  DS3231Present = false;
  FRAMPresent = false;
  AM2315Present = false;

  Serial.println();
  Serial.println();
  Serial.print(F("Weather Board"));
  Serial.print(2.5);
  Serial.println(F(" 07/18/2016"));
  Serial.print(F("Compiled at:"));
  Serial.print (F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));




  randomSeed(analogRead(0));




  weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0);
  rainTotal = 0;

  timeElapsed = 0;

  // Change to set time and uncomment
  /*
  tmElements_t t_elements; // elements array to date of quitting smoking.

  time_t t;


  t_elements.Second = 0;
  t_elements.Minute = 42;
  t_elements.Hour = 14;
  t_elements.Wday = 5;
  t_elements.Day = 4;
  t_elements.Month = 5;
  t_elements.Year = 2016 - 1970;
  t = makeTime(t_elements); // Unix timestamp quit date/time

  RTC.set(t);

  */
  tmElements_t tm;
  RTC.read(tm);
  if (RTC.chipPresent())
  {
    DS3231Present = true;
  }
  else
  {
    DS3231Present = false;
  }

  // AM2315
  AM2315OK = am2315.readData(dataAM2315);

  if (AM2315OK) {
    AM2315Present = true;
  }
  else
    AM2315Present = false;


  // BMP280

  /* Initialise the sensor */
  if (!bmp.begin())
  {
    BMP280Present = false;
    /* There was a problem detecting the BMP280 ... check your connections */
    Serial.print(F("Ooops, no BMP280 detected ... Check your wiring or I2C ADDR!"));
  }
  else
  {
    BMP280Present = true;
  }

  if (!htu.begin()) {
    HTU21DFPresent = false;
    Serial.println(F("Couldn't find HTU21D-F sensor!"));
  }
  else
  {
    HTU21DFPresent = true;
  }



  if (fram.begin())
  { // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println(F("Found I2C FRAM"));
    FRAMPresent = true;
    initializeFRAMTable(104); // initialize with 104 size




  }
  else
  {
    FRAMPresent = false;
    Serial.println(F("No I2C FRAM found ... check your connections\r\n"));

  }


  // Lightning Detector

  //I2C library initialization
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); // 1 - 400kHz, 0 - 100kHz

  // optional control of power for AS3935 via a PNP transistor
  // very useful for lockup prevention and power saving
  //pinMode(4, OUTPUT);

  // reset all internal register values to defaults
  AS3935.reset();
  // and run calibration
  // if lightning detector can not tune tank circuit to required tolerance,
  // calibration function will return false


  /*
  Serial.println("AS3935 Calibration Started");
  // Mod-1016 comes pre-calibrated

  if (!AS3935.calibrate())
  {

    Serial.println("AS3935 Tuning out of range, check your wiring, your sensor.");
  }
  else
  {
    Serial.println("AS3935 Calibration Completed");

  }
  */
  int noiseFloor = AS3935.getNoiseFloor();

  Serial.print(F("noiseFloor="));
  Serial.println(noiseFloor);
  if (noiseFloor > 0)
    AS3935Present = true;
  else
    AS3935Present = false;



  // first let's turn on disturber indication and print some register values from AS3935
  // tell AS3935 we are indoors, for outdoors use setOutdoors() function
  //AS3935.setIndoors();
  AS3935.setOutdoors();
  // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()

  //AS3935.enableDisturbers();
  AS3935.disableDisturbers();

  AS3935.setNoiseFloor(0x02);
  printAS3935Registers();
  AS3935IrqTriggered = 0;
  // Using interrupts means you do not have to check for pin being set continiously, chip does that for you and
  // notifies your code
  // demo is written and tested on ChipKit MAX32, irq pin is connected to max32 pin 2, that corresponds to interrupt 1
  // look up what pins can be used as interrupts on your specific board and how pins map to int numbers

  // ChipKit Max32 - irq connected to pin 2
  // attachInterrupt(1,AS3935Irq,RISING);
  // uncomment line below and comment out line above for Arduino Mega 2560, irq still connected to pin 2, same for atmega328p

  // Pin 3 is interrupt 1 on mega 2560
  // comment out to use polls
  //attachInterrupt(1,AS3935Irq,RISING);


  Serial.println(F("----------------------"));
  Serial.println(F("   Device Status"));
  Serial.println(F("----------------------"));
  reportMyDevice(F("DS3231"), DS3231Present);
  reportMyDevice(F("AS3935"), AS3935Present);
  reportMyDevice(F("FRAM"), FRAMPresent);
  reportMyDevice(F("BMP280"), BMP280Present);
  reportMyDevice(F("AM2315"), AM2315Present);
  reportMyDevice(F("HTU21DF"), HTU21DFPresent);

  Serial.println(F("----------------------"));

}

void loop()
{

  //Serial.println(freeMemory());

  // update weather
  currentWindSpeed = weatherStation.current_wind_speed() / 1.6;


  if (timeElapsed > 5000)
  {

    Serial.print(F("time="));
    Serial.println(millis());
    Serial.print(F("micro time="));
    Serial.println(micros());
    currentWindSpeed = weatherStation.current_wind_speed() / 1.6;
    currentWindGust = weatherStation.get_wind_gust() / 1.6;

    float oldRain = rainTotal;
    rainTotal = rainTotal + weatherStation.get_current_rain_total() * 0.03937;
    if (oldRain < rainTotal)
    {
      Serial.println( F("It is Raining"));
    }


    timeElapsed = 0;			 // reset the counter to 0 so the counting starts over...

    /*
      windSpeedMin = windSpeedGraph.returnMinValue();
      windSpeedMax = windSpeedGraph.returnMaxValue();
      windGustMin = windGustGraph.returnMinValue();
      windGustMax = windGustGraph.returnMaxValue();
      windDirectionMin = windDirectionGraph.returnMinValue();
      windDirectionMax = windDirectionGraph.returnMaxValue();

      Serial.print("windSpeedMin =");
      Serial.print(windSpeedMin);
      Serial.print(" windSpeedMax =");
      Serial.println(windSpeedMax);

      Serial.print("windSpeedBuffer=");
      Serial.println(windSpeedBuffer);

      Serial.print("windGustMin =");
      Serial.print(windGustMin);
      Serial.print(" windGustMax =");
      Serial.println(windGustMax);

      Serial.print("windGustBuffer=");
      Serial.println(windGustBuffer);

      Serial.print("windDirectionMin =");
      Serial.print(windDirectionMin);
      Serial.print(" windDirectionMax =");
      Serial.println(windDirectionMax);


      Serial.print("windDirectionBuffer=");
      Serial.println(windDirectionBuffer);
      */

    Serial.print(F(" currentWindSpeed="));
    Serial.print(currentWindSpeed);

    Serial.print(F(" \tcurrentWindGust="));
    Serial.print (currentWindGust);

    Serial.print(F(" \tWind Direction="));
    Serial.print(weatherStation.current_wind_direction());


    Serial.print(F(" \t\tCumulative Rain = "));
    Serial.print(rainTotal);
    Serial.print(F("\t\tmemory free="));
    Serial.println(freeMemory());

    tmElements_t tm;
    Serial.println(F("---------------"));
    Serial.println(F("DS3231 Clock"));
    Serial.println(F("---------------"));
    RTC.read(tm);

    printDigits(tm.Hour);
    Serial.print(F(":"));
    printDigits(tm.Minute);
    Serial.print(F(":"));
    printDigits(tm.Second);
    Serial.print(F(", Date (D/M/Y) = "));
    printDigits(tm.Day);
    Serial.print('/');
    printDigits(tm.Month);
    Serial.print('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
    float DS3231_Temp;
    DS3231_Temp = RTC.get_treg();
    Serial.print(F("Temperature= "));
    Serial.print(DS3231_Temp);
    Serial.println(F(" C"));


    Serial.println(F("---------------"));
    Serial.println(F("AT24C32 EEPROM"));
    Serial.println(F("---------------"));
    Serial.println(F("Write 5 addresses with random numbers and read them back"));

    unsigned int i;
    byte  readValue;
    byte  writeValue;
    for (i = 0; i < 5; i++)
    {
      writeValue = random(0, 255);
      i2c_eeprom_write_byte( 0x57, i, writeValue );
      Serial.print(F("Address: "));
      Serial.print(i);
      Serial.print(F(" Write:"));
      Serial.println(writeValue);


    }
    // Now read
    for (i = 0; i < 5; i++)
    {
      readValue = i2c_eeprom_read_byte( 0x57, i );
      Serial.print(F("Address: "));
      Serial.print(i);
      Serial.print(F(" Read Value:"));
      Serial.println(readValue);


    }


    Serial.println(F("---------------"));
    if (BMP280Present)
      Serial.println(F("BMP280"));
    else
      Serial.println(F("BMP280 Not Present"));
    Serial.println(F("---------------"));
    float BMP280_Temperature;
    float BMP280_Pressure;
    BMP280_Temperature = 0.0;
    BMP280_Pressure = 0.0;
    if (BMP280Present)
    {


      /* Display the results (barometric pressure is measure in hPa) */
      BMP280_Pressure = bmp.readPressure();
      BMP280_Temperature = bmp.readTemperature();

      /* Display atmospheric pressue in hPa */
      Serial.print(F("Pressure:    "));
      Serial.print(BMP280_Pressure / 10.0);
      Serial.println(F(" hPa"));


      /* Calculating altitude with reasonable accuracy requires pressure    *
       * sea level pressure for your position at the moment the data is     *
       * converted, as well as the ambient temperature in degress           *
       * celcius.  If you don't have these values, a 'generic' value of     *
       * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
       * in sensors.h), but this isn't ideal and will give variable         *
       * results from one day to the next.                                  *
       *                                                                    *
       * You can usually find the current SLP value by looking at weather   *
       * websites or from environmental information centers near any major  *
       * airport.                                                           *
       *                                                                    *
       * For example, for Paris, France you can check the current mean      *
       * pressure and sea level at: http://bit.ly/16Au8ol                   */

      /* First we get the current temperature from the BMP280 */
      float temperature;
      temperature = bmp.readTemperature();
      Serial.print(F("Temperature: "));
      Serial.print(temperature);
      Serial.println(F(" C"));


      BMP280_Temperature = temperature;


      /* Then convert the atmospheric pressure, and SLP to altitude         */
      /* Update this next line with the current SLP for better results      */

      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      Serial.print(F("Altitude:    "));
      Serial.print(bmp.readAltitude());
      Serial.println(F(" m"));

    }


    Serial.println(F("---------------"));
    if (AM2315Present)
      Serial.println(F("AM2315"));
    else
      Serial.println(F("AM2315 Not Present"));
    Serial.println(F("---------------"));

    // AM2315
    AM2315OK = am2315.readData(dataAM2315);
    float AM2315_Temperature;
    float AM2315_Humidity;

    AM2315_Temperature = 0.0;
    AM2315_Humidity = 0.0;

    if (AM2315OK) {
      AM2315_Temperature = dataAM2315[1];
      AM2315_Humidity = dataAM2315[0];
      Serial.print(F("Temperature: ")); Serial.print(dataAM2315[1]); Serial.println(F(" C"));
      Serial.print(F("Humidity: ")); Serial.print(dataAM2315[0]); Serial.println(F(" %"));


    }
    else
    {
      Serial.println(F("Temperature: NOT FOUND"));
      Serial.println(F("Humidity:  NOT FOUND"));

    }



    Serial.println(F("---------------"));
    if (HTU21DFPresent)
      Serial.println(F("HTU21D-F"));
    else
      Serial.println(F("HTU21D-F Not Present"));
    Serial.println(F("---------------"));

    float HTU_Temp;
    HTU_Temp = 0.0;
    if (HTU21DFPresent)
    {
      HTU_Temp = htu.readTemperature();
      Serial.print(F("Temperature: ")); Serial.print(HTU_Temp); Serial.println(F(" C"));
      Serial.print(F("Humidity: ")); Serial.print(htu.readHumidity()); Serial.println(F(" %"));
      Serial.println(F("---------------"));
    }

    Serial.println(F("---------------"));
    Serial.println(F("Lightning Detection"));
    Serial.println(F("---------------"));

    // reset the flag
    AS3935IrqTriggered = 0;
    // first step is to find out what caused interrupt
    int strokeDistance = 0.0;
    int irqSource = 0;

    // comment out the next two lines if you want to just poll instead of interrupt
    //       if(AS3935IrqTriggered)
    //{
    irqSource = AS3935.interruptSource();
    Serial.print(F("irqSource: "));
    Serial.println(irqSource, HEX);



    // comment out the next two lines if you wish to use interrupts
    if (irqSource > 0)
    {

      // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!
      if (irqSource & 0b0001)
        Serial.println(F("INT_NH Interrupt: Noise level too high, try adjusting noise floor"));
      if (irqSource & 0b0100)
        Serial.println(F("INT_D Interrupt: Disturber detected"));
      if (irqSource & 0b1000)
      {
        // need to find how far that lightning stroke, function returns approximate distance in kilometers,
        // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
        // everything in between is just distance in kilometers
        strokeDistance = AS3935.lightningDistanceKm();

        Serial.print(F("INT_L Interrupt: Lightning Detected.  Stroke Distance:"));
        Serial.print(strokeDistance);
        Serial.println(F(" km"));
        if (strokeDistance == 1)
          Serial.println(F("Storm overhead"));
        if (strokeDistance == 63)
          Serial.println(F("Out of range lightning detected."));

      }
    }
    Serial.println(F("---------------"));

    Serial.println(F("---------------"));
    Serial.println(F("FRAM Log"));
    Serial.println("---------------");
    // now write the record to FRAM


    char returnString[100];
    returnString[0] = '\0';



    RTC.read(tm);


    char floatString[15];

    char timeNow[20];
    timeNow[0] = '\0';
    buildTimeString(timeNow, timeNow, tm);



    sprintf(returnString, "%s,", timeNow);

    floatString[0] = '\0';
    dtostrf(currentWindSpeed, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(currentWindGust, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(weatherStation.current_wind_direction(), 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(weatherStation.current_wind_direction_voltage(), 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(rainTotal, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");



    floatString[0] = '\0';
    dtostrf(DS3231_Temp, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(AM2315_Temperature, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(AM2315_Humidity, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(HTU_Temp, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ",");

    floatString[0] = '\0';
    dtostrf(BMP280_Temperature, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ", ");

    floatString[0] = '\0';
    dtostrf(BMP280_Pressure, 6, 2, floatString);
    strcat(returnString, floatString);
    strcat(returnString, ", ");

    floatString[0] = '\0';
    itoa(irqSource, floatString, 10);
    strcat(returnString, floatString);
    strcat(returnString, ", ");

    floatString[0] = '\0';
    dtostrf(strokeDistance, 6, 2, floatString);
    strcat(returnString, floatString);









    writeFramEntry(0,  returnString);

    Serial.println(returnString);

    // displayFram();


  }

  // if you wanted to do other work based on a connecton, it would go here
}
