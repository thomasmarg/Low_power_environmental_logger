/* Arduino example sketch to control a JSN-SR04T ultrasonic distance sensor with Arduino. No library needed. More info: https://www.makerguides.com */
//Sensor code-------------------------------------------
// Define variables for ultra sonic sensor:
#define trigPin 5 //sends out ultrasonic call
#define echoPin 6 //recives ultra sonic call
long duration;
int distance;
//------------------------------------------------------

//RTC code----------------------------------------------
#include <Wire.h>
#include "RTClib.h" //libary containing real time clock code
RTC_DS3231 rtc;
//------------------------------------------------------

//Display code------------------------------------------
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
int sensorpowerpin = 8; //pin for powering sensor
int displaypowerpin = 7; //pin for powering display
int rtcpowerpin = 10; //pin for powering RTC
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
  pinMode(displaypowerpin, OUTPUT);
  pinMode(rtcpowerpin, OUTPUT);
  digitalWrite(displaypowerpin, HIGH);//turn on display power
  digitalWrite(rtcpowerpin, HIGH);//turn on display power
  delay(100);

  // Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //------------------------------------------------------------------------

  //------------------------------
  //RTC
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
    sleep_time = sleep_time_function(distance); //calculate amount of time the logger should stay asleep for
    int sleepMS = Watchdog.sleep(sleep_time); //sleep for x milliseconds
    digitalWrite(rtcpowerpin, HIGH); //turn on rtc power
    if (flag < 2) {
      oled.clear(); //clear the OLED display
      digitalWrite(displaypowerpin, LOW); //turns off the display power pin
    }
  }
  detect_distance(); //detect distance
  cavedataLog();  //request time from RTC and save to time/distance to SD card
  //digitalWrite(rtcpowerpin, LOW);  //turn off rtc power before going to sleep
  sleep_time = sleep_time_function(distance);  //calculate amount of time the logger should stay asleep for
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
    CavedataLog.print(distance);
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
  distance = 0;
  while (distance < 20) {
    //Sensor code

    // Turn on the sensor
    digitalWrite(sensorpowerpin, HIGH);
    delay(100); //wait a moment

    // Clear the trigPin by setting it LOW:
    digitalWrite(trigPin, LOW);

    delayMicroseconds(5);

    // Trigger the sensor by setting the trigPin high for 10 microseconds:
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance:
    distance = duration * 0.034 / 2;

    // Print the distance on the Serial Monitor (Ctrl+Shift+M):
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");

    digitalWrite(sensorpowerpin, LOW); //Turn off sensor
  }
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
  //------------------------------

  //------------------------------
  //display code
  oled.set2X();
  oled.print(distance);
  oled.print("cm");
  //------------------------------
}
//------------------------------------------------------------------------------------------------------------


//function setting the sample frequency--------------------------------------------------------
int sleep_time_function(int distance) {

sleep_time = 1000;
Serial.println(sleep_time);
return sleep_time;

}
//------------------------------------------------------------------------------------------------------------
