/*

jD-IOBoard general funcion calls


*/

// Roll all outputs one time one by one and have delay XX ms between status changes
void CycleOuputs(int pause) {
    for(int pos = 0; pos <= 5; pos ++ ) {
        digitalWrite(Out[pos], HIGH);
        delay(pause);

        digitalWrite(Out[pos], LOW);
        delay(pause);   
    }
}

// Switch all outputs
void SetOutputs(uint8_t state) {
    for(int i = 0; i <= 5; i++) {
        digitalWrite(Out[i], state);
    }  
}


// Our generic flight modes for ArduCopter & ArduPlane
void CheckFlightMode() {
    byte loopPos;

    if(apm_mav_type == 2) { // ArduCopter MultiRotor or ArduCopter Heli
        if(iob_mode == 0) flMode = STAB;   // Stabilize
        if(iob_mode == 1) flMode = ACRO;   // Acrobatic
        if(iob_mode == 2) flMode = ALTH;   // Alt Hold
        if(iob_mode == 3) flMode = AUTO;   // Auto
        if(iob_mode == 4) flMode = GUID;   // Guided
        if(iob_mode == 5) flMode = LOIT;   // Loiter
        if(iob_mode == 6) flMode = RETL;   // Return to Launch
        if(iob_mode == 7) flMode = CIRC;   // Circle
        if(iob_mode == 8) flMode = POSI;   // Position
        if(iob_mode == 9) flMode = LAND;   // Land
        if(iob_mode == 10) flMode = OFLO;  // OF_Loiter
    }
    else if(apm_mav_type == 1) { // ArduPlane
        if(iob_mode == 2 ) flMode = STAB;  // Stabilize
        if(iob_mode == 0) flMode = MANU;   // Manual
        if(iob_mode == 12) flMode = LOIT;  // Loiter
        if(iob_mode == 11 ) flMode = RETL; // Return to Launch
        if(iob_mode == 5 ) flMode = FBWA;  // FLY_BY_WIRE_A
        if(iob_mode == 6 ) flMode = FBWB;  // FLY_BY_WIRE_B
        if(iob_mode == 15) flMode = GUID;  // GUIDED
        if(iob_mode == 10 ) flMode = AUTO; // AUTO
        if(iob_mode == 1) flMode = CIRC;   // CIRCLE
    }

    DPL(mbind01_ADDR + flMode);
    while(flMode != readEEPROM(mbind01_ADDR + (loopPos))) {
        DPL(flMode);
        loopPos = loopPos + 2;
    }    
    DPL(loopPos);

    pattByteA = readEEPROM(pat01_ADDR + loopPos);
    pattByteB = readEEPROM(pat01_ADDR + (loopPos + 1));
    DPL(pattByteA, BIN);
    DPL(pattByteB, BIN); 
}

// Update main pattern
void RunPattern() {
    if((patt_pos & 0x08) == 0) {
        // pos [0 - 7], use byteA
        digitalWrite(REAR, getPatternState(pattByteA, patt_pos));
    } else {
        // pos [8 - 15], use byteB
        digitalWrite(REAR, getPatternState(pattByteB, patt_pos & 0x07));
    }
}

// Clear main pattern
void ClearPattern() {
    digitalWrite(REAR, LOW);
}


// general purpose BIT oprs borrowed from MinimOSD 
boolean getBit(byte Reg, byte whichBit) {
    boolean State;
    State = Reg & (1 << whichBit);
    return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat) {
    if (stat) {
        Reg = Reg | (1 << whichBit);
    } 
    else {
        Reg = Reg & ~(1 << whichBit);
    }
    return Reg;
}


// get pattern bit, bit '0' is really the MSB bit '7' etc
uint8_t getPatternState(byte Reg, byte whichBit) {
    return getBit(Reg, 7 - whichBit) ? HIGH : LOW;
}

