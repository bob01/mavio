/*

 file     : frsky_telemetry_feeder
 version  : v1.0, 24.06.2012
 author   : Jani Hirvinen, jani@jd.....com
 
 Relies on: 
 - jD IOBoard
 - SoftwareSerial
 - FrSky modules rx/tx
 - jD IOBoard, MAVLink code
 
 Connection:
 Connect following cables/pins from IOBoard to FrSky D8R-II or similar telemetry receiver
 
 IOB   RX
 -----------
 GND - GND
 D5  - Rx
 
 Details:
 Program creates and populates FrSky HUB style protocol messages and feeds it out from SoftwareSerial pins on IOBoard.
 SoftwareSerial uses inverted signaling to output data correctly, if signal is non-inverted data will be corrupt due 
 XORing and shifthing one step to right process. 
 
 FrSky uses 3 frames on their HUB protocol
 
 Frame 1, every 200ms,  payload: accel-x, accel-y, accel-z, Altitude(Vario), Temp1, Temp2, Voltage (multiple), RPM
 Frame 2, every 1000ms, payload: course, lat, lon, speed, altitude (GPS), fuel level
 Frame 3, every 5000ms, payload: date, time
 
 */

void update_FrSky() {

  f_curMillis = millis();
  if(f_curMillis - f_preMillis > f_delMillis) {
    // save the last time you sent the messaga 
    f_preMillis = f_curMillis;   

    // 200ms payload, construct Frame 1 on every loop
    packetOpen = TRUE;
    payloadLen += addPayload(0x24); // accel-x
    payloadLen += addPayload(0x25); // accel-y
    payloadLen += addPayload(0x26); // accel-z

    payloadLen += addPayload(0x10); // alt(vario)
    
    payloadLen += addPayload(0x02); // temp1
    payloadLen += addPayload(0x05); // temp2
    
//    payloadLen += addPayload(0x06); // battery data, injection not ready

    payloadLen += addPayload(0x03); // rpm
    
    packetOpen = FALSE;
    payloadLen = sendPayload(payloadLen);



    // 1000ms (1s) payload, construct Frame 2 on every 5th loop
    if((msCounter % 5) == 0) {
      second++;
      updateTime();
      packetOpen = TRUE;

      payloadLen += addPayload(0x14);   // Course, degree
      payloadLen += addPayload(0x1c);   // Course, after "."
      payloadLen += addPayload(0x12);   // Longitude dddmmm
      payloadLen += addPayload(0x1a);   // Longitude .mmmm (after ".")
      payloadLen += addPayload(0x13);   // Latitude dddmmm 
      payloadLen += addPayload(0x1b);   // Latitude .mmmm (after ".")
      payloadLen += addPayload(0x22);   // E/W
      payloadLen += addPayload(0x11);   // GPS Speed Knots
      payloadLen += addPayload(0x19);   // GPS Speed after "."
      payloadLen += addPayload(0x23);   // N/S
      payloadLen += addPayload(0x04);   // Fuel level % 0,25,50,75,100
      payloadLen += addPayload(0x18);   // Second

      packetOpen = FALSE;
      payloadLen = sendPayload(payloadLen);
      
    }  


    // 5000ms (5s) payload, contruct Frame 3 on every 25th loop and reset counters
    if(msCounter >= 25) {
      packetOpen = TRUE;
      payloadLen += addPayload(0x15); // date/month      
      payloadLen += addPayload(0x16); // year      
      payloadLen += addPayload(0x17); // hour/min      
      payloadLen += addPayload(0x18); // secs     
      packetOpen = FALSE;
      payloadLen = sendPayload(payloadLen);
      msCounter = 0;
    }
    // Update loop counter
    msCounter ++;
  }
}


byte addPayload(byte DataID) {
  
//  int test = 0;
  
  byte addedLen;
  switch(DataID) {
    case 0x01:  // GPS Altitude
      outBuff[payloadLen + 0] = 0x01;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x02:  // Temperature 1
      outBuff[payloadLen + 0] = 0x02;
      outBuff[payloadLen + 1] = iob_temperature;
      outBuff[payloadLen + 2] = 0x00;
      
//      ShowPayload();
      addedLen = 3;      
      break;
    case 0x03:  // RPM
      outBuff[payloadLen + 0] = 0x03;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x04:  // Fuel Level
      outBuff[payloadLen + 0] = 0x04;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x01;
      addedLen = 3;      
      break;
    case 0x05:  // Temperature 2
      outBuff[payloadLen + 0] = 0x05;
      outBuff[payloadLen + 1] = 0x02;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x06:  // Voltage, first 4 bits are cell number, rest 12 are voltage in 1/500v steps, scale 0-4.2v
      outBuff[payloadLen + 0] = 0x06;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0xff;
      outBuff[payloadLen + 3] = 0x06;
      outBuff[payloadLen + 4] = 0x10;
      outBuff[payloadLen + 5] = 0xff;
      outBuff[payloadLen + 6] = 0x06;
      outBuff[payloadLen + 7] = 0x20;
      outBuff[payloadLen + 8] = 0xff;
      outBuff[payloadLen + 9] = 0x06;
      outBuff[payloadLen + 10] = 0x30;
      outBuff[payloadLen + 11] = 0xff;
      addedLen = 12;      
      break;
    case 0x10:
      outBuff[payloadLen + 0] = 0x10;
      outBuff[payloadLen + 1] = FixInt(iob_alt, 1);
      outBuff[payloadLen + 2] = FixInt(iob_alt, 2);
      addedLen = 3;      
      break;
      
    case 0x11:  // GPS Speed, before "."
      outBuff[payloadLen + 0] = 0x11;
      outBuff[payloadLen + 1] = FixInt(iob_groundspeed, 1);
      outBuff[payloadLen + 2] = FixInt(iob_groundspeed, 2);
      addedLen = 3;      
      break;
    case 0x11+8:  // GPS Speed, after "."
      outBuff[payloadLen + 0] = 0x11+8;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

    case 0x12:  // Longitude, before "."
      outBuff[payloadLen + 0] = 0x12;
      outBuff[payloadLen + 1] = int(iob_lat);
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x12+8:  // Longitude, after "."
      outBuff[payloadLen + 0] = 0x12+8;
      outBuff[payloadLen + 1] = FixInt(long((iob_lat - int(iob_lat)) * 10000), 1);  // Only allow .0000 4 digits
      outBuff[payloadLen + 2] = FixInt(long((iob_lat - int(iob_lat)) * 10000), 2);  // Only allow .0000 4 digits after .
      addedLen = 3;      
      break;
    case 0x13:  // Latitude, before "."
      outBuff[payloadLen + 0] = 0x13;
      outBuff[payloadLen + 1] = int(iob_lon);
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x13+8:  // Latitude, after "."
      outBuff[payloadLen + 0] = 0x13+8;
      outBuff[payloadLen + 1] = FixInt(long((iob_lon - int(iob_lon)) * 10000), 1);
      outBuff[payloadLen + 2] = FixInt(long((iob_lon - int(iob_lon)) * 10000), 2);      
      addedLen = 3;      
      break;

   
    case 0x14:  // course, before "."
      outBuff[payloadLen + 0] = 0x14;
      outBuff[payloadLen + 1] = FixInt(iob_heading, 1);
      outBuff[payloadLen + 2] = FixInt(iob_heading, 2);
      addedLen = 3;      
      break;
    case 0x14+8:  // course, after "."  .. check calculation
      outBuff[payloadLen + 0] = 0x14+8;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

      
    case 0x15: // date/month
      outBuff[payloadLen + 0] = 0x15;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x16: // year
      outBuff[payloadLen + 0] = 0x16;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x17: // hour/minute
      outBuff[payloadLen + 0] = 0x17;
      outBuff[payloadLen + 1] = hour, DEC;
      outBuff[payloadLen + 2] = minute, DEC;
      addedLen = 3;      
      break;
    case 0x18: // second
      outBuff[payloadLen + 0] = 0x18;
      outBuff[payloadLen + 1] = second, DEC;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

    case 0x24:
//      outBuff[payloadLen + 0] = 0x24;
//      outBuff[payloadLen + 1] = 0x00;
//      outBuff[payloadLen + 2] = 0x01;

      outBuff[payloadLen + 0] = 0x24;      
      outBuff[payloadLen + 1] = FixInt(iob_roll * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_roll * 100, 2);
      addedLen = 3;      
      break;
    case 0x25:
      outBuff[payloadLen + 0] = 0x25;
      outBuff[payloadLen + 1] = FixInt(iob_pitch * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_pitch * 100, 2);
      addedLen = 3;      
      break;
    case 0x26:
      outBuff[payloadLen + 0] = 0x26;
      outBuff[payloadLen + 1] = FixInt(iob_yaw * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_yaw * 100, 2);
      addedLen = 3;      
      break;

    default:
      addedLen = 0;
  }
  return addedLen; 

}

byte addEnd() {
 return 1; 
}

byte sendPayload(byte len) {
  
  byte pos2;
  // First fix outBuff and look if there are any 0x5E, if have 

/*
  for(byte pos = 0; pos <= len; pos++) { 
    if(outBuff[pos] != 0x5E || outBuff[pos] != 0x5D) {
      outBuffFixed[pos2] = outBuff[pos];      
    } else {
      if(outBuff[pos] == 0x5E) {
        outBuffFixed[pos2] = 0x5D;
        outBuffFixed[pos2 + 1] = 0x3E;
        pos2 ++;
      }
      if(outBuff[pos] == 0x5D) {
        outBuffFixed[pos2] = 0x5D;
        outBuffFixed[pos2 + 1] = 0x3D;
        pos2 ++;
      }
    }    
   pos2++;    
   }
*/
//   len = pos2;

  frSerial.write(0x5E);
//  if(deb2)  Serial.print(0x5E, HEX);
  for(byte pos = 0; pos <= len - 1; pos = pos + 3) {
    frSerial.write(byte(outBuff[pos + 0]));
    frSerial.write(byte(outBuff[pos + 1]));
    frSerial.write(byte(outBuff[pos + 2]));
    frSerial.write(0x5E);
   
//    if(deb2) Serial.print(byte(outBuff[pos + 0]), HEX); 
//    if(deb2) Serial.print(byte(outBuff[pos + 1]), HEX); 
//    if(deb2) Serial.print(byte(outBuff[pos + 2]), HEX); 
   
//    if(deb2) Serial.print(" "); 
  }
  //  frSerial.write(0x5E);
//  if(deb2)  Serial.println(); 
  return 0;
}

// FrSky int handling
long FixInt(long val, byte mp) {  
 if(mp == 2) return long(val / 256);
 if (val >= 256 && mp == 1) 
   return val % 256;  
}

void ShowPayload() {
      Serial.print("PL: ");
      Serial.print(outBuff[payloadLen + 1], DEC);
      Serial.print(", ");
      Serial.print(outBuff[payloadLen + 2], DEC);
      Serial.println(); 
}


void updateTime() {
  if(second >= 60) {
    second = 0;
    minute++;
   }
   if(minute >= 60) {
    second = 0;
    minute = 0;
    hour++;
   } 
   if(hour >= 24) {
     second = 0;
     minute = 0;
     hour = 0;
   }
}

