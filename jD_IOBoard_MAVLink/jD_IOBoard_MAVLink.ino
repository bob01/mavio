/*

Copyright (c) 2012.  All rights reserved.
An Open Source Arduino based jD_IOBoard driver for MAVLink

Program  : jD_IOBoard
Version  : V1.0, June 06 2012
Author(s): Jani Hirvinen
Coauthor(s):
Sandro Beningo  (MAVLink routines)
Mike Smith      (BetterStream and Fast Serial libraries)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>

*/

/*
//////////////////////////////////////////////////////////////////////////
//  Description: 
// 
//  This is an Arduino sketch on how to use jD-IOBoard LED Driver board
//  that listens MAVLink commands and changes patterns accordingly.
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
/* **************************************************************************** */

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

#define MAVLINK10     // Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
//#define HEARTBEAT     // HeartBeat signal
#define SERDB         // Output debug information to SoftwareSerial 
//#define ONOFFSW       // Do we have OnOff switch connected in pins 

/* **********************************************/
/* ***************** INCLUDES *******************/

//#define membug 
//#define FORCEINIT  // You should never use this unless you know what you are doing 


// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <GCS_MAVLink.h>


#ifdef membug
#include <MemoryFree.h>
#endif

#include <SoftwareSerial.h>

// Configurations
#include "IOBoard.h"
#include "IOEEPROM.h"

#define CHKVER 80

//#define DUMPEEPROM            // Should not be activated in repository code, only for debug
//#define DUMPEEPROMTELEMETRY   // Should not be activated in repository code, only for debug


/* *************************************************/
/* ***************** DEFINITIONS *******************/

#define VER "v1.4_tb_plus"

// These are not in real use, just for reference
//#define O1 8      // High power Output 1
//#define O2 9      // High power Output 2, PWM
//#define O3 10     // High power Output 3, PWM
//#define O4 4      // High power Output 4 
//#define O5 3      // High power Output 5, PWM
//#define O6 2      // High power Output 6

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port

byte debug = 0;  // Shoud not be activated on repository code, only for debug


// Objects and Serial definitions
FastSerialPort0(Serial);

SimpleTimer  mavlinkTimer;

#ifdef SERDB
#define DPL if(debug) dbSerial.println 
#define DPN if(debug) dbSerial.print
SoftwareSerial dbSerial(6,5);
#endif


/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
#ifdef SERDB
    // Our software serial is connected on pins D6 and D5
    if(debug)
    {
        dbSerial.begin(57600);
        DPL("Debug Serial ready... ");
        DPL("No input from this serialport.  ");
    }
#endif  

    // Initialize Serial port, speed
    Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    mavlink_comm_0_port = &Serial;

    // reset EEPROM if pin 11 LOW on init
    if(digitalRead(11) == 0) {
        DPL("Force erase pin LOW, Eracing EEPROM");
        DPN("Writing EEPROM...");
        writeFactorySettings();
        DPL(" done.");
    }

    // Check that EEPROM has initial settings, if not write them
    if(readEEPROM(CHK1) + readEEPROM(CHK2) != CHKVER) {
        // Write factory settings on EEPROM
        DPN("Writing EEPROM...");
        writeFactorySettings();
        DPL(" done.");
    }

    // Initializing output pins
    for(int i = 0; i <= 5; i++) {
        pinMode(Out[i],OUTPUT);
    }

    // Init sequence
    // cycle segments...
    for(int i = 0; i <= 5; i++) {
        CycleOuputs(25); 
    }
    // ... then flash twice
    for(int i = 0; i <= 2 ; i++) {
        SetOutputs(HIGH);
        delay(100);
        SetOutputs(LOW);
        delay(100);
    }

    // Startup MAVLink timers, 50ms intervals
    // this affects pattern speeds too - 50hz timing important here
    mavlinkTimer.Set(&OnMavlinkTimer, 50);

    // enable timers
    mavlinkTimer.Enable();

} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// The thing that goes around and around and around for ethernity...
// MainLoop()
void loop() 
{
    if(enable_mav_request == 1) { //Request rate control
        for(int n = 0; n < 3; n++) {
            request_mavlink_rates();   //Three times to certify it will be readed
            delay(50);
        }
        enable_mav_request = 0;

        // wait 2 seconds...
        delay(2000);

        waitingMAVBeats = 0;
        lastMAVBeat = millis();    // Preventing error from delay sensing
    } 

    read_mavlink();
    mavlinkTimer.Run();
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */

// Function that is called every 120ms
void OnMavlinkTimer()
{
    if(millis() < lastMAVBeat + 3000)
    {
        // MAV alive - last MAVbeat less than 3s ago
        // TODO: run pattern
    }
    else
    {
        // MAV dead 
        // TODO: dead flash
    }
}



