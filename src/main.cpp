// Arduino script for running low power environmental logger, the design of the logger was adapted from the cave perl perl project
//I take no credit for any of the libarys used in this script and but did spend ages putting the scrip itself together, hope others find this code useful for their own projects.

//to use this code you will first need to install the following libaries with the latest versions at the time of writing (01/01/2021):
//http://librarymanager/All#SparkFun_MS5803-14BA - pressure sensor
//http://librarymanager/All#RTClib - real time clock
//http://librarymanager/All#SSD1306Ascii - display
//http://librarymanager/All#SPI - SD card code should already be included
//http://librarymanager/All#SD - also SD card code should also be included
//https://github.com/rocketscream/Low-Power - for powering down until interupt

//Edit code below for logger configuration-----------------------------------------------
int intial_runs = 10; //define the number of samples you want the logger to take in the intial rapid sampling phase.

//define the number of hours mins and seconds between readings.
int hrs = 0;
int mins = 1;
int secs = 0;
//---------------------------------------------------------------------------------------

//Pressure sensor code-------------------------------------------
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h> // Click here to get the library: http://librarymanager/All#SparkFun_MS5803-14BA
MS5803 sensor(ADDRESS_HIGH);
//Create variables to store results
float temperature_c;
double pressure_abs;
#ifdef DEPTH_SENSOR
double pressure_baseline, depth;
#endif
//------------------------------------------------------


//RTC code for clock reset and reading realtime----------------------------------------------
#include "RTClib.h" //libary containing real time clock code
RTC_DS3231 rtc;
// the pin that is connected to SQW for interupt
#define CLOCK_INTERRUPT_PIN 2
//------------------------------------------------------
#ifdef WITH_DISPLAY
////Display code------------------------------------------
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#endif
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
#ifdef WITH_DISPLAY
SSD1306AsciiAvrI2c oled;
#endif

#ifdef DEBUG
// Open serial communications and wait for port to open:
// wait for serial port to connect. Needed for native USB port only

#define DEBUG_INIT() Serial.begin(9600);while (!Serial) {}
#define DEBUG_PRINT(X) Serial.print(X)
#define DEBUG_PRINTLN(X) Serial.println(X)
#else
#define DEBUG_INIT()
#define DEBUG_PRINT(X)
#define DEBUG_PRINTLN(X)
#endif
//------------------------------------------------------

//SD card code------------------------------------------
#include <SPI.h>
#include <SD.h>
File CavedataLog;
//------------------------------------------------------

// Define Power variables
int ledpowerpin = 7; //pin for powering DISPLAY (T1)
#include "LowPower.h"


//The following are functions called during void loop--------------------------------------------------------------------------


//function for writing to SD card--------------------------------------------------------------------------
void cavedataLog(void) {
    DateTime now = rtc.now();
    // open the file. note that only one file can be open at a time,
    CavedataLog = SD.open("test.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (CavedataLog) {
        DEBUG_PRINT("Writing to test.txt...");

        CavedataLog.print(now.year(), DEC);
        CavedataLog.print('/');

        if (now.month() < 10) {
            CavedataLog.print("0");
        }
        CavedataLog.print(now.month(), DEC);

        CavedataLog.print('/');

        if (now.day() < 10) {
            CavedataLog.print("0");
        }
        CavedataLog.print(now.day(), DEC);

        CavedataLog.print(" ");

        if (now.hour() < 10) {
            CavedataLog.print("0");
        }
        CavedataLog.print(now.hour(), DEC);

        CavedataLog.print(':');

        if (now.minute() < 10) {
            CavedataLog.print("0");
        }
        CavedataLog.print(now.minute(), DEC);

        CavedataLog.print(':');

        if (now.second() < 10) {
            CavedataLog.print("0");
        }
        CavedataLog.print(now.second(), DEC);

        CavedataLog.print(", ");
    #ifdef DEPTH_SENSOR    
        CavedataLog.print(depth);
        CavedataLog.print(", ");
    #endif    
        CavedataLog.print(temperature_c);

        // read the input on analog pin 0:
        int sensorValue = analogRead(A0);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
        float voltage = sensorValue * (6.6 / 1023.0);
        // print out the value you read:
        CavedataLog.print(", ");
        CavedataLog.print(voltage);
        CavedataLog.println("");
        DEBUG_PRINTLN(" ");
        DEBUG_PRINT("Voltage: ");
        DEBUG_PRINT(voltage);


        // close the file:
        CavedataLog.close();
        DEBUG_PRINTLN(" ");
    } else {
        // if the file didn't open, print an error:
        DEBUG_PRINTLN("error opening test.txt");
    }
    //you have to close this one before opening another.
    CavedataLog.close();
}
//------------------------------------------------------------------------------------------------------------




//function for triggering sensor and returning distance-------------------------------------------------------
void detect_distance(void) {
    // Read temperature from the sensor in deg C. This operation takes about
    temperature_c = sensor.getTemperature(CELSIUS, ADC_512);

    // Read pressure from the sensor in mbar.
    pressure_abs = sensor.getPressure(ADC_4096);
#ifdef DEPTH_SENSOR
    // Taking our baseline pressure at the beginning we can find an approximate
    // change in altitude based on the differences in pressure.
    depth =  -(pressure_baseline - pressure_abs);
#endif
    // Report values via UART
    DEBUG_PRINT("Temperature C = ");
    DEBUG_PRINTLN(temperature_c);

    DEBUG_PRINT("Pressure abs (mbar)= ");
    DEBUG_PRINTLN(pressure_abs);
#ifdef DEPTH_SENSOR
    DEBUG_PRINT("Depth (cm) = ");
    DEBUG_PRINTLN(depth);
#endif    
    DEBUG_PRINTLN("done.");


    DEBUG_PRINTLN(" ");//padding between outputs
}
//------------------------------------------------------------------------------------------------------------




//function for shutting down the device if the battery gets too low-------------------------------------------------------
void check_battery(void) {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = sensorValue * (6.6 / 1023.0);
    // print out the value you read:
    if (voltage < 3.4) {
#ifdef WITH_DISPLAY
        oled.clear(); //clear the OLED display
        oled.println("Battery low");
        oled.println();
        oled.println("shutting down");
#endif
        CavedataLog.print("Battery low shutting down");
        DEBUG_PRINTLN("Battery low shutting down");//padding between outputs
        delay(500);
        while (voltage < 10) {
            DEBUG_PRINTLN("sleeping");
            DEBUG_PRINTLN("sleeping");
            DEBUG_PRINTLN("sleeping");
            digitalWrite(ledpowerpin, LOW); //turns off the display power pin
            delay(3000);
#ifdef WITH_DISPLAY
            oled.clear(); //clear the OLED display
#endif
            LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        }
    }
    DEBUG_PRINTLN("Battery okay");//padding between outputs
#ifdef WITH_DISPLAY
    oled.clear(); //clear the OLED display
#endif
}
//------------------------------------------------------------------------------------------------------------


//function for printing date time and distance to OLED--------------------------------------------------------
void display_oled(void) {
#ifdef WITH_DISPLAY
    //RTC code
    DateTime now = rtc.now();
    oled.clear();
    oled.set1X();

    oled.print(now.year(), DEC);
    oled.print('/');

    if (now.month() < 10) {
        oled.print('0');
    }
    oled.print(now.month(), DEC);

    oled.print('/');

    if (now.day() < 10) {
        oled.print('0');
    }
    oled.print(now.day(), DEC);

    oled.print(" ");

    if (now.hour() < 10) {
        oled.print('0');
    }
    oled.print(now.hour(), DEC);

    oled.print(':');

    if (now.minute() < 10) {
        oled.print('0');
    }
    oled.print(now.minute(), DEC);

    oled.print(':');

    if (now.second() < 10) {
        oled.print('0');
    }
    oled.print(now.second(), DEC);

    oled.println();
    DEBUG_PRINT("Temperature: ");
    DEBUG_PRINT(rtc.getTemperature());
    DEBUG_PRINTLN(" C");
    oled.println();
    //------------------------------

    //------------------------------
    //display code
    //oled.set2X();
#ifdef DEPTH_SENSOR
    oled.print(depth);
    oled.print(" cm");
    oled.print(" ");
#endif
    oled.print(temperature_c);
    oled.print((char)247);
    oled.print("C");

    delay(2000);
   
    //display battery voltage
  
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = sensorValue * (6.6 / 1023.0);
    oled.clear();
    oled.set1X();
    oled.println("Battery voltage: ");
    oled.set2X();
    oled.print(voltage);
    oled.print(" V");

    delay(1000);
    //------------------------------
#endif    

}

void LED_blink()
{
    digitalWrite(ledpowerpin, HIGH); //turns off the display power pin
    delay(10);
    digitalWrite(ledpowerpin, LOW); //turns off the LED
}

void onAlarm() {
    DEBUG_PRINTLN("Alarm occured!");
}
void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void sleep_until_interupt() {
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, LOW);

    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0);
}


void alarm_reset()
{
    // check to see if the alarm flag is set (also resets the flag if set)
    if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        DEBUG_PRINTLN("Alarm cleared");
    }
    // schedule an alarm 10 seconds in the future
    if (!rtc.setAlarm1(
                       rtc.now() + TimeSpan(0, hrs, mins, secs), //sets the delay time in the following format: day, hour, min, second
                       DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
                       )) {
        DEBUG_PRINTLN("Error, alarm wasn't set!");
    } else {
        DEBUG_PRINTLN("Alarm will happen in 10 seconds!");
    }
}


//---------------------------------------------------------------------------------------------------------------------------------
void setup() {
    DEBUG_INIT();
    //------------------------------------------------------------------------

    //Define power pins
    pinMode(ledpowerpin, OUTPUT);
    delay(100);

    // initalise pressure sensor
    Wire.begin();
    //Retrieve calibration constants for conversion math.
    sensor.reset();
    sensor.begin();
#ifdef DEPTH_SENSOR
    pressure_baseline = sensor.getPressure(ADC_4096);
    //------------------------------------------------------------------------
#endif
    //------------------------------
    //RTC
    //uncomment the line below and upload again after inital upload or it will reset the clock every time you reset the arduino
    // mrtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //to set clock time to time of compile

    //we don't need the 32K Pin, so disable it
    rtc.disable32K();

    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    // schedule an alarm a duration of time in the future from now, define at top of script
    if (!rtc.setAlarm1(
                       rtc.now() + TimeSpan(0, hrs, mins, secs), //sets the delay time in the following format: day, hour, min, second
                       DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
                       )) {
        DEBUG_PRINTLN("Error, alarm wasn't set!");
    } else {
        DEBUG_PRINTLN("Alarm will happen in 10 seconds!");
    }
    //------------------------------------------------------------------------

#ifdef WITH_DISPLAY

    //Display code------------------------------------------------------------
#if RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS);
#endif // RST_PIN >= 0
    // Call oled.setI2cClock(frequency) to change from the default frequency.
    oled.setFont(System5x7);
    //------------------------------------------------------------------------
    oled.println("Initializing SD card...");

#endif

    //  //SD card code------------------------------------------------------------
    DEBUG_PRINT("Initializing SD card...");

    delay(500);

    if (!SD.begin(4)) {
        DEBUG_PRINTLN("initialization failed!");
#ifdef WITH_DISPLAY
        oled.println("initialization failed!");
#endif
        while (1);
    }
    DEBUG_PRINTLN("initialization done.");
#ifdef WITH_DISPLAY
    oled.println("initialization done.");
#endif
    delay(1000);
    //-----------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------------------------
}





void loop() {
    while (intial_runs > 1) {
        check_battery();
        intial_runs = intial_runs - 1;
        DEBUG_PRINT(intial_runs); //prints the flag number over serial for debugging
        detect_distance(); //detect distance
        display_oled(); //print time/distance/voltage to display
        cavedataLog(); //save to time/distance/voltage to SD card
        LED_blink();
        //adjust sleep code as desired, add sucessive sleep periods for longer sleep times
        LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); //sleeps the logger for 2 seconds
        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //sleeps the logger for 8 seconds
        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //sleeps the logger for 8 seconds
#ifdef WITH_DISPLAY
        if (intial_runs < 2) {
            oled.clear(); //clear the OLED display
        }
#endif        
    }
    check_battery();
    detect_distance(); //detect distance
    cavedataLog();  //request time from RTC and save to time/distance to SD card
    LED_blink();
    sleep_until_interupt();
    alarm_reset();
}





