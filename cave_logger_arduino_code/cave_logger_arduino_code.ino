/* Arduino example sketch to control a JSN-SR04T ultrasonic distance sensor with Arduino. No library needed. More info: https://www.makerguides.com */
//Pressure sensor code-------------------------------------------
// Define variables for pressure sensor:
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h> // Click here to get the library: http://librarymanager/All#SparkFun_MS5803-14BA

MS5803 sensor(ADDRESS_HIGH);

//Create variables to store results
float temperature_c;
double pressure_abs, depth, pressure_baseline;
//------------------------------------------------------




//RTC code----------------------------------------------
#include <Wire.h>
#include "RTClib.h" //libary containing real time clock code
RTC_DS3231 rtc;
//------------------------------------------------------

////Display code------------------------------------------
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiAvrI2c oled;
//------------------------------------------------------

//SD card code------------------------------------------
#include <SPI.h>
#include <SD.h>
File CavedataLog;
//------------------------------------------------------

// Define Power variables
int flag; //for signaling module power on/off to make device more efficent when screen is no longer in use
int sensorpowerpin = 10; //pin for powering ulta sonic sensor
int ledpowerpin = 7; //pin for powering display
int rtcpowerpin = 9; //pin for powering all the peripheral devices sensors ect, but not the memory card slot
#include <Adafruit_SleepyDog.h>//import sleep code
int sleep_time;
//---------------------------------------------------------------------------------------------------------------------------------



void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //------------------------------------------------------------------------

  //Define power pins
  pinMode(sensorpowerpin, OUTPUT);
  pinMode(ledpowerpin, OUTPUT);
  pinMode(rtcpowerpin, OUTPUT);
  digitalWrite(ledpowerpin, HIGH);//turn on display power
  digitalWrite(rtcpowerpin, HIGH);//turn on display power
  delay(100);

  // Define pressure sensor code
  Wire.begin();
  //Retrieve calibration constants for conversion math.
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);
  //------------------------------------------------------------------------

  //------------------------------
  //RTC
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //to set clock time to time of compile uncomment this line
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //------------------------------------------------------------------------


//Display code------------------------------------------------------------
#if RST_PIN >= 0
oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
#endif // RST_PIN >= 0
// Call oled.setI2cClock(frequency) to change from the default frequency.
 oled.setFont(System5x7);
//------------------------------------------------------------------------


  //  //SD card code------------------------------------------------------------
    Serial.print("Initializing SD card...");
oled.println("Initializing SD card...");
    delay(500);
  
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
  oled.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");
oled.println("initialization done.");
    delay(1000);
//  //  //-----------------------------------------------------------------------
//
//  //flag signals when to clear and turn off the oled display,
//  //increase this number if you want the display to be on longer after the device is reset
  flag = 10;
  //-------------------------------------------------------------------------------------------------------------------------------
}







void loop() {
  while (flag > 1) {
    flag = flag - 1;
    Serial.print(flag); //prints the flag number over serial for debugging
    detect_distance(); //detect distance
    display_oled(); //request time from RTC and print time/distance to display
    cavedataLog(); //request time from RTC and save to time/distance to SD card
    digitalWrite(rtcpowerpin, LOW); //turn off rtc power before going to sleep
    sleep_time = 1000;
    int sleepMS = Watchdog.sleep(sleep_time); //sleep for x milliseconds
    digitalWrite(rtcpowerpin, HIGH); //turn on rtc power
    digitalWrite(ledpowerpin, HIGH); //turns off the display power pin
    delay(100);
    digitalWrite(ledpowerpin, LOW); //turns off the display power pin
    if (flag < 2) {
      oled.clear(); //clear the OLED display
    }
  }
  digitalWrite(rtcpowerpin, HIGH);  //turn off rtc power before going to sleep
  detect_distance(); //detect distance
  cavedataLog();  //request time from RTC and save to time/distance to SD card
  digitalWrite(ledpowerpin, HIGH); //turns off the display power pin
  delay(100);
  digitalWrite(ledpowerpin, LOW); //turns off the display power pin
  digitalWrite(rtcpowerpin, LOW);  //turn off rtc power before going to sleep
  sleep_time = 1000;
  int sleepMS = Watchdog.sleep(sleep_time); //sleep for x milliseconds
  //digitalWrite(rtcpowerpin, HIGH);  //turn on rtc power
}







//The following are functions called during the void loop
//function for writing to SD card--------------------------------------------------------------------------
void cavedataLog(void) {
  DateTime now = rtc.now();
  // open the file. note that only one file can be open at a time,
  CavedataLog = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (CavedataLog) {
    Serial.print("Writing to test.txt...");
    CavedataLog.print(now.day(), DEC);
    CavedataLog.print('/');
    CavedataLog.print(now.month(), DEC);
    CavedataLog.print('/');
    CavedataLog.print(now.year(), DEC);
    CavedataLog.print(" ");
    CavedataLog.print(now.hour(), DEC);
    CavedataLog.print(':');
    CavedataLog.print(now.minute(), DEC);
    CavedataLog.print(':');
    CavedataLog.print(now.second(), DEC);
    CavedataLog.print(", ");
    CavedataLog.print(depth);
    CavedataLog.print(", ");
    CavedataLog.print(temperature_c);  
    CavedataLog.println("");
    // close the file:
    CavedataLog.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
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

  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.
  depth =  -(pressure_baseline - pressure_abs);

  // Report values via UART
  Serial.print("Temperature C = ");
  Serial.println(temperature_c);

  Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_abs);

  Serial.print("Depth (cm) = ");
  Serial.println(depth);

  Serial.println(" ");//padding between outputs

  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(voltage);
}
//------------------------------------------------------------------------------------------------------------


//function for printing date time and distance to OLED--------------------------------------------------------
void display_oled(void) {
  //RTC code
  DateTime now = rtc.now();
  oled.clear();
  oled.set1X();
  oled.print(now.year(), DEC);
  oled.print('/');
  oled.print(now.month(), DEC);
  oled.print('/');
  oled.print(now.day(), DEC);
  oled.print(" ");
  oled.print(now.hour(), DEC);
  oled.print(':');
  oled.print(now.minute(), DEC);
  oled.print(':');
  oled.print(now.second(), DEC);
  oled.println();
  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
  oled.println();
  Serial.print("Temperature: ");
  //------------------------------

  //------------------------------
  //display code
  //oled.set2X();
  oled.print(depth);
  oled.print(" cm");
  oled.print(" ");
  oled.print(temperature_c);
  oled.print((char)247);
  oled.print("C");
  //------------------------------
}
