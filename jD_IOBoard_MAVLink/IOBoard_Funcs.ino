/*
general funcion calls

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


// Update main pattern
void RunPattern() {
    if((patt_pos & 0x08) == 0) {
        // pos [0 - 7], use byteA
        SetOutputs(getPatternState(pattByteA, patt_pos));
    } else {
        // pos [8 - 15], use byteB
        SetOutputs(getPatternState(pattByteB, patt_pos & 0x07));
    }
}

// Clear main pattern
void ClearPattern() {
    SetOutputs(LOW);
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

