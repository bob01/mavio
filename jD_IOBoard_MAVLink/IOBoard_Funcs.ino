/*
general funcion calls

*/

// Roll all outputs one time one by one and have delay XX ms between status changes
void cycleOuputs(int pause) {
    // for all outputs...
    for(int pos = 0; pos <= 5; pos ++ ) {
        // ...power on, pause...
        digitalWrite(Out[pos], HIGH);
        delay(pause);

        // ...power off pause
        digitalWrite(Out[pos], LOW);
        delay(pause);   
    }
}

// Switch all outputs
void setOutputs(uint8_t state) {
    for(int i = 0; i <= 5; i++) {
        digitalWrite(Out[i], state);
    }  
}


// set pattern from ID
void setPattern(uint8_t pattern)
{
    // bail if no change
    if(pattern == patt)
        return;

    // select pattern bytes
    // note: pattern is option-base-1, convert to option-base-0 for EEPROM index
    uint8_t idx = pattern - 1; 
    pattByteA = readEEPROM(pat01_ADDR + (idx * 2));
    pattByteB = readEEPROM(pat01_ADDR + (idx * 2) + 1);

    // save pattern
    patt = pattern;

    // reset pattern position
    patt_pos = 0;
}

// update main pattern
void runPattern() {
    if((patt_pos & 0x08) == 0) {
        // pos [0 - 7], use byteA
        setOutputs(getPatternState(pattByteA, patt_pos));
    } else {
        // pos [8 - 15], use byteB
        setOutputs(getPatternState(pattByteB, patt_pos & 0x07));
    }

    // cycle pattern
    patt_pos = (patt_pos + 1) & 0x0F;
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

