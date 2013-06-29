/**********************************************************/
//
// Name    : jD-IOBoard_PWMFlasher
// Version : v1.0 16-05-12
// Author  : Jani Hirvinen, jani@j....com
// 
//
//  Copyright (c) 2012 Jani Hirvinen.  All rights reserved.
//  An Open Source Arduino LED Driver.
//
//  This program is free software: you can redistribute it and/or modify 
//  it under the terms of the GNU General Public License as published by 
//  the Free Software Foundation, either version 3 of the License, or 
//  (at your option) any later version. 
//
//  This program is distributed in the hope that it will be useful, 
//  but WITHOUT ANY WARRANTY; without even the implied warranty of 
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
//  GNU General Public License for more details. 
//
//  You should have received a copy of the GNU General Public License 
//  along with this program. If not, see <http://www.gnu.org/licenses/>. 
// 
// Changelog:
// 16-05-12  Initial file
//
//
//////////////////////////////////////////////////////////////////////////
//  Description: 
// 
//  This is example Arduino sketch on how to use jD-IOBoard.
//  LED Driver board that listens I2C commands and changes patterns accordingly.
//
//  If you use, redistribute this please mention original source.
//
//  jD-IOBoard pinouts
//
//             S M M G       R T R
//         5 5 C O I N D D D X X S
//         V V K S S D 7 6 5 1 1 T
//         | | | | | | | | | | | |
//      +----------------------------+
//      |O O O O O O O O O O O O O   |
// O1 - |O O   | | |                O| _ DTS 
// O2 - |O O   3 2 1                O| - RX  F
// O3 - |O O   1 1 1                O| - TX  T
// O4 - |O O   D D D                O| - 5V  D
// O5 - |O O                        O| _ CTS I
// O6 - |O O O O O O O O   O O O O  O| - GND
//      +----------------------------+
//       |   | | | | | |   | | | |
//       C   G 5 A A A A   S S 5 G
//       O   N V 0 1 2 3   D C V N
//       M   D             A L   D
//
// More information, check http://www.jdrones.com/jDoc
//
/******************************************************************************/


///////////////////////////
// Global includes

///////////////////////////
// Global defines 

#define VER "v1.0"

#define O1 8      // High power Output 1
#define O2 9      // High power Output 2, PWM
#define O3 10     // High power Output 3, PWM
#define O4 4      // High power Output 4 
#define O5 3      // High power Output 5, PWM
#define O6 2      // High power Output 6

#define EN  1     // Enable value
#define DI  0     // Disable value

#define Circle_Dly 1000

#define ledPin 13     // Heartbeat LED if any
#define LOOPTIME  50  // Main loop time for heartbeat
#define BAUD 57600    // Serial speed

///////////////////////////
// Global variables
int counter = 0;     // General counter

byte ioOut = 0;      // Blinker output position
byte ioStat = LOW;   // Blinker last status
byte ioCounter = 0;  // Blinker count

int Out[] = {8,9,10,4,3,2};   // Output I/O pin array

long preMillis;
long curMillis;
long delMillis = 500;

// LED patterns
byte flight_patt[10][16] = {
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  },    // 0
  { 1,1,1,1,0,0,0,0 ,1,1,1,1,0,0,0,0  },    // 1
  { 1,1,1,1,1,0,0,0 ,0,0,0,0,0,1,0,0  },    // 2
  { 1,1,0,0,1,1,0,0 ,1,1,0,0,1,1,0,0  },    // 3
  { 1,0,0,0,1,0,0,0 ,1,0,0,0,1,0,0,0  },    // 4
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,0,0,0,0  },    // 5
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,1,0,1,0  },    // 6
  { 1,0,1,0,0,0,0,0 ,1,0,1,0,0,0,0,0  },    // 7
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  },    // 8
  { 1,0,0,0,0,0,0,0 ,1,0,0,0,0,0,0,0  }};   // 9

byte patt_pos;
byte patt;

//byte debug = 1;      // debug level

///////////////////////////
// Setup 
void setup() {

  Serial.begin(BAUD); 
  
  // Initializing output pins
  for(int looper = 0; looper <= 5; looper++) {
    pinMode(Out[looper],OUTPUT);
  }

  // Initializing general pattern
  patt_pos = 0;
  patt = 0;
  
  delay(1000);
}
 

///////////////////////////
// Main program
void loop() {
  
   Serial.println("Begin of MainLoop");

   // Filling outputs from left to right
   LEDFill(500,0);
   delay(500);
   
   // Flash all outputs slowly
   for(int looper = 0; looper <= 2; looper++) {
   AllOff();
   delay(250);
   AllOn();
   delay(250);
   }

   // Clearing outputs from left to right
   LEDClear(100,0);
   delay(1000);
   
   // NightRider effect and Chasers
   LEDRider(100,2);
   LEDChaser(50, 3);
   LEDChaser(25, 5);
   LEDChaser(50, 3);
   delay(1000);
   
   // NightRider effect
   LEDRider(50, 6);
   AllOff();
   delay(1000);

   // Speeding fill effect
   LEDWasp(50,0);
   LEDClear(100,0);
   delay(1000);

   // LED Fills from left to right and vise versa
   LEDFill(100,1);    // delay, direction 0 = from left to right, 1 = from right to left
   delay(1000);
   LEDClear(100,0);
   LEDFill(100,0);
   delay(1000);
   LEDClear(100,1);
   delay(1000);
  
   // Several PWM outputs with additional effects
   LEDPwm(4,0);  
   AllOff();
   delay(500);
   LEDPwm(3,1);  
   AllOff();
   delay(500);
   LEDPwm(3,2);  
   AllOff();
   delay(500);
   LEDPwm(4,3);  
   AllOff();

   delay(1000);
   
   // Final blinks after we start everything again.
   for(int loopy = 0; loopy <= 5; loopy++) {
    AllOn();
    delay(75);
    AllOff();
    delay(75);
   } 
    
    
delay(5000);
}



///////////////////////////
// Global functions


// Blink output per output with a speeding flash speed until fully lit.
// Delay X as time to wait until next output starts, filling direction Y
void LEDWasp(int dly, int dir) {
 Serial.println("LEDWasp");
 counter = 0; 
 int ledPos = 0;
 int looper;
 int looperdly = 35;
 if(dir == 1) ledPos = 5;

 while(counter <= 5) {
  for(looper = 255; looper >= 0; looper = looper - looperdly) {

   if(looper >= 201) looperdly = 35; // Checking and changing speed of delays
   if(looper <= 200) looperdly = 15; 
   if(looper <= 150) looperdly = 10; 
   if(looper <= 100) looperdly = 8; 
   if(looper <= 50) looperdly = 5; 
   if(looper <= 20) looperdly = 2; 
   
   digitalWrite(Out[ledPos],EN);
   delay(looper);
   digitalWrite(Out[ledPos],DI);
   delay(looper);  
  }
  digitalWrite(Out[ledPos],EN);
  
  if(dir) { 
    ledPos--;
  } else ledPos++;
  
  delay(dly); 
  looperdly = 35;
  counter++;  
 }  
}


// Clears all outputs with delay X from directioin Y
void LEDClear(int dly, int dir) { 
 Serial.println("LEDClear");
 if(dir == 0) 
   for(int looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper], DI);
    delay(dly);    
   } 
 if(dir == 1) 
   for(int looper = 5; looper >= 0; looper--) {
    digitalWrite(Out[looper], DI);
    delay(dly);    
   } 
}


// Fills all outputs with delay X from direction Y
void LEDFill(int dly, int dir) { 
  Serial.println("LEDFill");
  if(dir == 0) 
   for(int looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper], EN);
    delay(dly);    
   } 
 if(dir == 1) 
   for(int looper = 5; looper >= 0; looper--) {
    digitalWrite(Out[looper], EN);
    delay(dly);    
   } 
}

// Blinks all outputs with delay of X and repeast Y times
void LEDFullBlinker(int dly, int maxcount) {
  Serial.println("LEDFullBlinker");
  counter = 0;
  while(counter <= maxcount) {
   AllOn();
   delay(dly);
   AllOff();
   delay(dly);
   counter++;
  }   
}
  
  
// KnightRider effect with small delays
// One output is active at time, running up and down
void LEDRider(int dly, int maxcount) {
  Serial.print("LEDRider");
  counter = 0;
  int ledPos = -1;
  byte dir = 1;
  
  while(counter <= maxcount) {
   for(int looper = 0; looper <= 10; looper++) {
     if(dir == 0) { 
       ledPos --;
     } else ledPos ++;
 
     if(dir == 0 && ledPos == 0) dir = 1;
     if(dir == 1 && ledPos == 5) dir = 0;
     
     digitalWrite(Out[ledPos],EN);   
     delay(dly);
     digitalWrite(Out[ledPos],DI);
   } 
   counter++;
  }
}

// Up/Down LED Chaser effect. Start from O1 -> O6 and then back to O6 -> O1
void LEDChaser(int dly, int maxcount) {
  Serial.println("LEDChaser");
  counter = 0;
  int looper;
  
  while(counter <= maxcount) {
   for (looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper], EN); 
    delay(dly);
   }
   for (looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper], DI); 
    delay(dly);
   } 
   counter ++; 
  }
}


// PWM diving from 3 PWM output pins with optional extra effects
void LEDPwm(int maxcount, byte extraEffect) {
   Serial.println("LEDPwm");
   
   int pwmout = 31;
   byte dir = 1;
   counter = 0;
  
    while(counter <= maxcount - 1) {

     if(dir == 0) pwmout --;
     if(dir == 1) pwmout ++;

     if(pwmout <= 30 && dir == 0) counter ++; 
     if(pwmout >= 255) dir = 0;
     if(pwmout <= 30) dir = 1;

     analogWrite(O2, pwmout);
     analogWrite(O3, pwmout);
     analogWrite(O5, pwmout);
     
     if(extraEffect == 1) CheckBlink(15);
     if(extraEffect == 2) CheckBlink(30);
     if(extraEffect == 3) CheckBlink(10);
     
     delay(2);
   }   
}

// Extra blinks used during PWM outputs tests
void CheckBlink(int dly) {

   int OutPort;
   delMillis = dly;
   
   if(ioOut == 0) OutPort = Out[0];
   if(ioOut == 1) OutPort = Out[3];
   if(ioOut == 2) OutPort = Out[5];
  
   curMillis = millis();
   if(curMillis - preMillis > delMillis) {
     preMillis = curMillis;

     if(ioStat == LOW) {
      digitalWrite(OutPort, EN);
      ioStat = HIGH;
     } else {
      digitalWrite(OutPort, DI); 
      ioStat = LOW;
     }
     
     ioCounter++;
     
     if(ioCounter >= 10){
      ioCounter = 0;
      ioOut++; 
     }
     
     if(ioOut >= 3) ioOut = 0;
   }  
}

// Switch all outputs ON
void AllOn() {
  Serial.println("AllOn");
   for(int looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper],EN);
  }  
}


// Switch all outputs OFF
void AllOff() {
  Serial.println("AllOff");
   for(int looper = 0; looper <= 5; looper++) {
    digitalWrite(Out[looper],DI);
  }  
}

