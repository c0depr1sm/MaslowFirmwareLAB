/*This file is part of the Maslow Control Software.

The Maslow Control Software is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Maslow Control Software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.

Copyright 2014-2017 Bar Smith*/

// This file contains all the functions used to receive and parse the gcode
// commands

#include "Maslow.h"

RingBuffer incSerialBuffer;
String readyCommandString = "";  //KRK why is this a global?
String gcodeLine          = "";  //Our use of this is a bit sloppy, at times,
                                 //we pass references to this global and then
                                 //name them the same thing.

void initGCode(){
    // Called on startup or after a stop command
    readyCommandString = "";
    incSerialBuffer.empty();
}

void readSerialCommands(){
    /*
    Check to see if a new character is available from the serial connection,
    if this is a necessary character write to the incSerialBuffer otherwise discard
    it.
    */

    static bool quickCommandFlag = false;

    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '!'){
                sys.stop = true;
                quickCommandFlag = true;
                bit_false(sys.pause, PAUSE_FLAG_USER_PAUSE);
                reportStatusMessage(STATUS_OK);
            }
            else if (c == '~'){
                quickCommandFlag = true;
                bit_false(sys.pause, PAUSE_FLAG_USER_PAUSE);
                reportStatusMessage(STATUS_OK);
            }
            else if (quickCommandFlag and c == '\n'){
              // Catch line ending and ignore after quick commands
              quickCommandFlag = false;
            }
            else{
                quickCommandFlag = false;
                int bufferOverflow = incSerialBuffer.write(c); //gets one byte from serial buffer, writes it to the internal ring buffer
                if (bufferOverflow != 0) {
                  sys.stop = true;
                }
            }
            if (sys.stop){return;}
        }
        #if defined (verboseDebug) && verboseDebug > 1
        // print ring buffer contents
        Serial.println(F("rSC added to ring buffer"));
        incSerialBuffer.print();
        #endif
    }
}

int   findEndOfNumber(const String& textString, const int& index){
    //Return the index of the last digit of the number beginning at the index passed in
    unsigned int i = index;

    while (i < textString.length()){

        if(isDigit(textString[i]) or isPunct(textString[i])){ //If we're still looking at a number, keep goin
            i++;
        }
        else{
            return i;                                         //If we've reached the end of the number, return the last index
        }
    }
    return i;                                                 //If we've reached the end of the string, return the last number
}

float extractGcodeValue(const String& readString, char target, const float& defaultReturn){

    /*Reads a string and returns the value of number following the target character.
    If no number is found, defaultReturn is returned*/

    int begin;
    int end;
    String numberAsString;
    float numberAsFloat;

    begin           =  readString.indexOf(target);
    end             =  findEndOfNumber(readString,begin+1);
    numberAsString  =  readString.substring(begin+1,end);

    numberAsFloat   =  numberAsString.toFloat();

    if (begin == -1){ //if the character was not found, return error
        return defaultReturn;
    }

    return numberAsFloat;
}

byte  executeBcodeLine(const String& gcodeLine){
    /*

    Executes a single line of gcode beginning with the character 'B'.

    */

    //Handle B-codes

    if(gcodeLine.substring(0, 3) == "B05"){
        Serial.print(F("Firmware Version "));
        Serial.println(VERSIONNUMBER);
        return STATUS_OK;
    }

    if(sys.state == STATE_OLD_SETTINGS){
      return STATUS_OLD_SETTINGS;
    }

    if(gcodeLine.substring(0, 3) == "B01"){

        Serial.println(F("Motor Calibration Not Needed"));

        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B02"){
        calibrateChainLengths(gcodeLine);
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B04"){
        //set flag to ignore position error limit during the tests
        sys.state = (sys.state | STATE_POS_ERR_IGNORE);
        //Test each of the axles
        maslowDelay(500);
        if(sys.stop){return STATUS_OK;}
        leftAxle.test();
        maslowDelay(500);
        if(sys.stop){return STATUS_OK;}
        rightAxle.test();
        maslowDelay(500);
        if(sys.stop){return STATUS_OK;}
        zAxle.test();
        Serial.println(F("Tests complete."));

        // update our position
        leftAxle.set(leftAxle.read());
        rightAxle.set(rightAxle.read());

        //clear the flag, re-enable position error limit
        sys.state = (sys.state & (!STATE_POS_ERR_IGNORE));
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B06"){
        Serial.println(F("Setting Chain Lengths To: "));
        float newL = extractGcodeValue(gcodeLine, 'L', 0);
        float newR = extractGcodeValue(gcodeLine, 'R', 0);

        leftAxle.set(newL);
        rightAxle.set(newR);

        Serial.print(F("Left: "));
        Serial.print(leftAxle.read());
        Serial.println(F("mm"));
        Serial.print(F("Right: "));
        Serial.print(rightAxle.read());
        Serial.println(F("mm"));

        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B08"){
        //Manually recalibrate chain lengths
        leftAxle.set(sysSettings.originalChainLength);
        rightAxle.set(sysSettings.originalChainLength);

        Serial.print(F("Left: "));
        Serial.print(leftAxle.read());
        Serial.println(F("mm"));
        Serial.print(F("Right: "));
        Serial.print(rightAxle.read());
        Serial.println(F("mm"));

        //Estimate the XY position based on the machine geometry and chain lenght extending beyond the sproket top.
        kinematics.forward(leftAxle.read(), rightAxle.read(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, 0, 0);
	    
        Serial.println(F("Message: The machine chains have been manually re-calibrated."));

        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B09"){
        //Directly command each axle to move to a given distance
        float lDist = extractGcodeValue(gcodeLine, 'L', 0);
        float rDist = extractGcodeValue(gcodeLine, 'R', 0);
        float speed = extractGcodeValue(gcodeLine, 'F', 800);

        if(sys.useRelativeUnits){
            if(abs(lDist) > 0){
                singleAxleMove(&leftAxle,  leftAxle.read()  + lDist, speed);
            }
            if(abs(rDist) > 0){
                singleAxleMove(&rightAxle, rightAxle.read() + rDist, speed);
            }
        }
        else{
            singleAxleMove(&leftAxle,  lDist, speed);
            singleAxleMove(&rightAxle, rDist, speed);
        }

        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B10"){
        //measure the left axle chain length
        Serial.print(F("[Measure: "));
        if (gcodeLine.indexOf('L') != -1){
            Serial.print(leftAxle.read());
        }
        else{
            Serial.print(rightAxle.read());
        }
        Serial.println(F("]"));
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B11"){
        //run right motor in the given direction at the given speed for the given time
        float  speed      = extractGcodeValue(gcodeLine, 'S', 100);
        float  time       = extractGcodeValue(gcodeLine, 'T', 1);

        double ms    = 1000*time;
        double begin = millis();

        int i = 0;
        sys.state = (sys.state | STATE_POS_ERR_IGNORE);
        while (millis() - begin < ms){
            if (gcodeLine.indexOf('L') != -1){
                leftAxle.motorGearboxEncoder.motor.directWrite(speed);
            }
            else{
                rightAxle.motorGearboxEncoder.motor.directWrite(speed);
            }
            
            i++;
            execSystemRealtime();
            if (sys.stop){return STATUS_OK;}
        }
        sys.state = (sys.state | (!STATE_POS_ERR_IGNORE));
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B13"){
        //PID Testing of Velocity
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);
        float  steps      = extractGcodeValue(gcodeLine, 'I', 1);
        float  version    = extractGcodeValue(gcodeLine, 'V', 1);

        Axle* axle = &rightAxle;
        if (left > 0) axle = &leftAxle;
        if (useZ > 0) axle = &zAxle;
        PIDTestVelocity(axle, start, stop, steps, version);
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B14"){
        //PID Testing of Position
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);
        float  steps      = extractGcodeValue(gcodeLine, 'I', 1);
        float  stepTime   = extractGcodeValue(gcodeLine, 'T', 2000);
        float  version    = extractGcodeValue(gcodeLine, 'V', 1);

        Axle* axle = &rightAxle;
        if (left > 0) axle = &leftAxle;
        if (useZ > 0) axle = &zAxle;
        PIDTestPosition(axle, start, stop, steps, stepTime, version);
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B16"){
        //Incrementally tests voltages to see what RPMs they produce
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);

        Axle* axle = &rightAxle;
        if (left > 0) axle = &leftAxle;
        if (useZ > 0) axle = &zAxle;
        voltageTest(axle, start, stop);
        return STATUS_OK;
    }

    if(gcodeLine.substring(0, 3) == "B15"){
        //The B15 command moves the chains to the length which will put the sled in the center of the sheet

        //Compute chain length for position 0,0
        float chainLengthAtMiddle;
        kinematics.inverse(0,0,&chainLengthAtMiddle,&chainLengthAtMiddle);

        //Adjust left chain length
        singleAxleMove(&leftAxle,  chainLengthAtMiddle, 800);

        //Adjust right chain length
        singleAxleMove(&rightAxle, chainLengthAtMiddle, 800);

        //Estimate the XY position based on the machine geometry and chain new lenght extending beyond the sproket top.
        kinematics.forward(leftAxle.read(), rightAxle.read(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, 0, 0);
                
        return STATUS_OK;
    }
    return STATUS_INVALID_STATEMENT;
}

void  executeGcodeLine(const String& gcodeLine){
    /*

    Executes a single line of gcode beginning with the character 'G'.  If the G code is
    not included on the front of the line, the code from the previous line will be added.

    */

    //Handle G-Codes

    int gNumber = extractGcodeValue(gcodeLine,'G', -1);

    if (gNumber == -1){               // If the line does not have a G command
        gNumber = sys.lastGCommand;        // apply the last one
    }

    switch(gNumber){
        case 0:   // Rapid positioning
        case 1:   // Linear interpolation
            G1(gcodeLine, gNumber);
            sys.lastGCommand = gNumber;    // remember G number for next time
            break;
        case 2:   // Circular interpolation, clockwise
        case 3:   // Circular interpolation, counterclockwise
            G2(gcodeLine, gNumber);
            sys.lastGCommand = gNumber;    // remember G number for next time
            break;
        case 4:
            G4(gcodeLine);
            break;
        case 10:
            G10(gcodeLine);
            break;
        case 20:
            setMeasurementUnitConversionFactor(INCHES_TO_MLLIMETERS);
            break;
        case 21:
            setMeasurementUnitConversionFactor(MILLIMETERS);
            break;
        case 40:
            break; //the G40 command turns off cutter compensation which is already off so it is safe to ignore
        case 38:
            G38(gcodeLine);
            break;
        case 90:
            sys.useRelativeUnits = false;
            break;
        case 91:
            sys.useRelativeUnits = true;
            break;
        default:
            Serial.print(F("Command G"));
            Serial.print(gNumber);
            Serial.println(F(" unsupported and ignored."));
    }

}

void  executeMcodeLine(const String& gcodeLine){
    /*

    Executes a single line of gcode beginning with the character 'M'.

    */

    //Handle M-Codes

    int mNumber = extractGcodeValue(gcodeLine,'M', -1);

    switch(mNumber){
        case 0:   // Program Pause / Unconditional Halt / Stop
        case 1:   // Optional Pause / Halt / Sleep
            pause();
            break;
        case 2:   // Program End
        case 30:  // Program End with return to program top
        case 5:   // Spindle Off
            setSpindlePower(false); // turn off spindle
            break;
        case 3:   // Spindle On - clockwise
        case 4:   // Spindle On - counterclockwise
            // Maslow spindle runs only one direction, but turn spindle on for either code
            setSpindlePower(true);  // turn on spindle
            break;
        case 6:   // Tool Change
            if (sys.nextTool != sys.lastTool) {
                setSpindlePower(false); // first, turn off spindle
                Serial.print(F("Tool Change: Please insert tool "));   // prompt user to change tool
                Serial.println(sys.nextTool);
                sys.lastTool = sys.nextTool;
                pause();
            }
            break;
        case 106:
            //Turn laser on
            laserOn();
            break;
        case 107:
            //Turn laser off
            laserOff();
            break;
        default:
            Serial.print(F("Command M"));
            Serial.print(mNumber);
            Serial.println(F(" unsupported and ignored."));
    }

}

void  executeOtherCodeLine(const String& gcodeLine){
    /*

    Executes a single line of gcode beginning with a character other than 'G', 'B', or 'M'.

    */

    if (gcodeLine.length() > 1) {
        if (gcodeLine[0] == 'T') {
            int tNumber = extractGcodeValue(gcodeLine,'T', 0);    // get tool number
            Serial.print(F("Tool change to tool "));
            Serial.println(tNumber);
            sys.nextTool = tNumber;                               // remember tool number to prompt user when G06 is received
        }
        else {  // try it as a 'G' command without the leading 'G' code
            executeGcodeLine(gcodeLine);
        }
    }
    else {
        Serial.print(F("Command "));
        Serial.print(gcodeLine);
        Serial.println(F(" too short - ignored."));
    }

}

int   findNextGM(const String& readString, const int& startingPoint){
    int nextGIndex = readString.indexOf('G', startingPoint);
    int nextMIndex = readString.indexOf('M', startingPoint);
    if (nextMIndex != -1) {           // if 'M' was found
        if ((nextGIndex == -1) || (nextMIndex < nextGIndex)) { // and 'G' was not found, or if 'M' is before 'G'
            nextGIndex = nextMIndex;  // then use 'M'
        }
    }
    if (nextGIndex == -1) {           // if 'G' was not found (and therefore 'M' was not found)
        nextGIndex = readString.length();   // then use the whole string
    }

    return nextGIndex;
}

void  sanitizeCommandString(String& cmdString){
    /*
    Primarily removes comments and some other non supported characters or functions.
    This is taken heavily from the GRBL project at https://github.com/gnea/grbl
    */

    byte line_flags = 0;
    size_t pos = 0;

    while (cmdString.length() > pos){
        if (line_flags) {
            // Throw away all (except EOL) comment characters and overflow characters.
            if (cmdString[pos] == ')') {
                // End of '()' comment. Resume line allowed.
                cmdString.remove(pos, 1);
                if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
            }
        }
        else {
            if (cmdString[pos] < ' ') {
                // Throw away control characters
                cmdString.remove(pos, 1);
            }
            else if (cmdString[pos] == '/') {
                // Block delete NOT SUPPORTED. Ignore character.
                // NOTE: If supported, would simply need to check the system if block delete is enabled.
                cmdString.remove(pos, 1);
            } else if (cmdString[pos] == '(') {
                // Enable comments flag and ignore all characters until ')' or EOL.
                // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
                // In the future, we could simply remove the items within the comments, but retain the
                // comment control characters, so that the g-code parser can error-check it.
                cmdString.remove(pos, 1);
                line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
            } else if (cmdString[pos] == ';') {
                // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
                cmdString.remove(pos, 1);
                line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
            } else if (cmdString[pos] == '%') {
              // Program start-end percent sign NOT SUPPORTED.
              cmdString.remove(pos, 1);
            } else {
                pos++;
            }
        }
    }
    #if defined (verboseDebug) && verboseDebug > 1
      // print results
      Serial.println(F("sCS execution complete"));
      Serial.println(cmdString);
    #endif
}

byte  interpretCommandString(String& cmdString){
    /*

    Splits a string into lines of gcode which begin with 'G' or 'M', executing each in order
    Also executes full lines for 'B' codes, and handles 'T' at beginning of line

    Assumptions:
        Leading and trailing white space has already been removed from cmdString
        cmdString has been converted to upper case

    */

    size_t firstG;
    size_t secondG;

    if (cmdString.length() > 0) {
        if (cmdString[0] == '$') {
            // Maslow '$' system command
            return(systemExecuteCmdstring(cmdString));
        }
        else if (cmdString[0] == 'B'){                   //If the command is a B command
            #if defined (verboseDebug) && verboseDebug > 0
            Serial.print(F("iCS executing B code line: "));
            #endif
            Serial.println(cmdString);
            return executeBcodeLine(cmdString);
        }
        else if (sys.state == STATE_OLD_SETTINGS){
          return STATUS_OLD_SETTINGS;
        }
        else {
            while(cmdString.length() > 0){          //Extract each line of gcode from the string
                firstG  = findNextGM(cmdString, 0);
                secondG = findNextGM(cmdString, firstG + 1);

                if(firstG == cmdString.length()){   //If the line contains no G or M letters
                    firstG = 0;                     //send the whole line
                }

                if (firstG > 0) {                   //If there is something before the first 'G' or 'M'
                    gcodeLine = cmdString.substring(0, firstG);
                    #if defined (verboseDebug) && verboseDebug > 0
                    Serial.print(F("iCS executing other code: "));
                    #endif
                    Serial.println(gcodeLine);
                    executeOtherCodeLine(gcodeLine);  // execute it first
                }

                gcodeLine = cmdString.substring(firstG, secondG);

                if (gcodeLine.length() > 0){
                    if (gcodeLine[0] == 'M') {
                        #if defined (verboseDebug) && verboseDebug > 0
                        Serial.print(F("iCS executing M code: "));
                        #endif
                        Serial.println(gcodeLine);
                        executeMcodeLine(gcodeLine);
                    }
                    else {
                        #if defined (verboseDebug) && verboseDebug > 0
                        Serial.print(F("iCS executing G code: "));
                        #endif
                        Serial.println(gcodeLine);
                        executeGcodeLine(gcodeLine);
                    }
                }

                cmdString = cmdString.substring(secondG, cmdString.length());

            }
            return STATUS_OK;
        }
        return STATUS_INVALID_STATEMENT;
    }
    return STATUS_OK;
}

void gcodeExecuteLoop(){
  byte status;
  if (incSerialBuffer.numberOfLines() > 0){
      incSerialBuffer.prettyReadLine(readyCommandString);
      sanitizeCommandString(readyCommandString);
      status = interpretCommandString(readyCommandString);
      readyCommandString = "";

      // Get next line of GCode
      if (!sys.stop){reportStatusMessage(status);}
  }
}

void G1(const String& readString, int G0orG1){

    /*G1() is the function which is called to process the string if it begins with
    'G01' or 'G00'*/

    float finalXPos;
    float finalYPos;
    float finalZPos;

    //identify the estimated starting coordinates of this straight path coordinated move
    float initialXPos = sys.estimatedBitTipXPosition;
    float initialYPos = sys.estimatedBitTipYPosition;
    float initialZPos = zAxle.read();
    
    float tempFeedRate; // make sure to not change the sys.targetFeedrate with a value until validated and constrained

    //extract and compute the target coordinates of this straight path move
    finalXPos      = sys.mmConversionFactor*extractGcodeValue(readString, 'X', initialXPos/sys.mmConversionFactor);
    finalYPos      = sys.mmConversionFactor*extractGcodeValue(readString, 'Y', initialYPos/sys.mmConversionFactor);
    finalZPos      = sys.mmConversionFactor*extractGcodeValue(readString, 'Z', initialZPos/sys.mmConversionFactor);

    if (sys.useRelativeUnits){ //if we are using a relative coordinate system

        if(readString.indexOf('X') >= 0){ //if there is an X command
            finalXPos = initialXPos + finalXPos;
        }
        if(readString.indexOf('Y') >= 0){ //if y has moved
            finalYPos = initialYPos + finalYPos;
        }
        if(readString.indexOf('Z') >= 0){ //if y has moved
            finalZPos = initialZPos + finalZPos;
        }
    }

    //keep the last used feedrate if none specified in the Gcode line 
    tempFeedRate = sys.mmConversionFactor*extractGcodeValue(readString, 'F', sys.targetFeedrate/sys.mmConversionFactor);
    //Top the target rate to the maximum XY feedrate,
    sys.targetFeedrate = constrain(tempFeedRate, 1, sysSettings.targetMaxXYFeedRate);   

    //if the zaxle is not attached, then get manual ajustment done before move.
    if(!sysSettings.zAxleMotorized){
        float threshold = .1; //units of mm
        if (abs(initialZPos - finalZPos) > threshold){
            Serial.print(F("Message: Please adjust Router Bit to a depth of "));
            if (finalZPos > 0){
                Serial.print(F("+"));
            }
            Serial.print(finalZPos/sys.mmConversionFactor);
            if (sys.mmConversionFactor == INCHES_TO_MLLIMETERS){
                Serial.println(F(" in"));
            }
            else{
                Serial.println(F(" mm"));
            }

            pause(); //Wait until the z depth is adjusted

            zAxle.set(finalZPos);

            maslowDelay(1000);
        }
    }


    if (G0orG1 == 1){
        //if this is a regular move
        coordinatedMove(finalXPos, finalYPos, finalZPos, sys.targetFeedrate); //The XY move is performed
    }
    else{
        //if this is a rapid move
        coordinatedMove(finalXPos, finalYPos, finalZPos, sysSettings.targetMaxXYFeedRate); //move the same as a regular move, but seek fastest feed rate
    }
}

void G2(const String& readString, int G2orG3){
    /*

    The G2 function handles the processing of the gcode line for both the command G2 and the
    command G3 which cut arcs.

    */

    //is it supposed to handle relative units? Apparently, unlike B09 G38 or G0 and G1,  it does not.

    //identify the estimated starting coordinates of this straight path coordinated move
    float X1 = sys.estimatedBitTipXPosition; 
    float Y1 = sys.estimatedBitTipYPosition;

    //extract and compute the target coordinates of this arc path move
    float X2      = sys.mmConversionFactor*extractGcodeValue(readString, 'X', X1/sys.mmConversionFactor);
    float Y2      = sys.mmConversionFactor*extractGcodeValue(readString, 'Y', Y1/sys.mmConversionFactor);
    float I       = sys.mmConversionFactor*extractGcodeValue(readString, 'I', 0.0);
    float J       = sys.mmConversionFactor*extractGcodeValue(readString, 'J', 0.0);
 
    //is it supposed to handle relative units? Apparently, unlike B09 G38 or G0 and G1,  it does not.

    //compute the circle center coordinates of this arc path move
    float centerX = X1 + I;
    float centerY = Y1 + J;

    //make sure to not change the sys.targetFeedrate with a value until validated and constrained
    float tempFeedRate; 
    //keep the last used feedrate if none specified in the Gcode line 
    tempFeedRate = sys.mmConversionFactor*extractGcodeValue(readString, 'F', sys.targetFeedrate/sys.mmConversionFactor);
    //Top the target rate to the maximum XY feedrate,
    sys.targetFeedrate = constrain(tempFeedRate, 1, sysSettings.targetMaxXYFeedRate);    

    if (G2orG3 == 2){
        arcMove(X1, Y1, X2, Y2, centerX, centerY, sys.targetFeedrate, CLOCKWISE);
    }
    else {
        arcMove(X1, Y1, X2, Y2, centerX, centerY, sys.targetFeedrate, COUNTERCLOCKWISE);
    }
}

void  G4(const String& readString){
    /*
      The G4() dwell function handles the G4 gcode which pauses for P milliseconds or S seconds.
      Only one of the two is accepted, the other ignored.
      Use maslowDelay() to remain responsive to GC during the dwell time.
      Because maslowDelay() operates in milliseconds, round to the nearest millisecond.
      Negative values are treated as positive (not a time machine).
    */
    float dwellMS = abs(extractGcodeValue(readString, 'P', 0));
    float dwellS  = abs(extractGcodeValue(readString, 'S', 0));

    if (dwellMS == 0) {
      /*
        ignore S parameter unless P is zero
      */
      dwellMS = dwellS * 1000;
    }
    dwellMS = long(dwellMS + .5);
    Serial.print(F("dwell time "));
    if (dwellS > 0) {
      Serial.print(dwellS);
      Serial.println(F(" seconds"));
    } else {
      Serial.print(dwellMS, 0);
      Serial.println(F(" ms."));
    }
    maslowDelay(dwellMS);
}

void  G10(const String& readString){
    /*The G10() function handles the G10 gcode which re-zeros one or all of the machine's axes.*/
    float initialZPos = zAxle.read();
    float finalZPos   = sys.mmConversionFactor*extractGcodeValue(readString, 'Z', initialZPos/sys.mmConversionFactor);

    zAxle.set(finalZPos);
    zAxle.endMove(finalZPos);
    zAxle.attach();
}

void  G38(const String& readString) {
  //if the zaxle is motorized
  if (sysSettings.zAxleMotorized) {
    /*
       The G38() function handles the G38 gcode which identify the zero depth Router Bit position.
       Currently ignores X and Y options
    */
    if (readString.substring(3, 5) == ".2") {
      // in a more complete implementation, the touche point is associated to a known position, not to zero.
      // This allows for probing apparatus thickness or deepness property to be taken into account.
      Serial.println(F("probing for Router Bit known depth position"));
      float finalZPos;
      float zMaxFeedRate = getZMaxFeedRate();

      float initialZPos = zAxle.read();

      float tempFeedRate; // make sure to not change the sys.targetFeedrate with a value until validated and constrained

      //extract and compute the target Z limit coordinate of this plunge move
      finalZPos = sys.mmConversionFactor * extractGcodeValue(readString, 'Z', initialZPos / sys.mmConversionFactor);
      if (sys.useRelativeUnits) { //if we are using a relative coordinate system
        if (readString.indexOf('Z') >= 0) { //if z has moved
          finalZPos = initialZPos + finalZPos;
        }
      }

      //keep the last used feedrate if none specified in the Gcode line 
      tempFeedRate = sys.mmConversionFactor*extractGcodeValue(readString, 'F', sys.targetFeedrate/sys.mmConversionFactor);
      //Top the target rate to the maximum Z feedrate,
      sys.targetFeedrate = constrain(tempFeedRate, 1, zMaxFeedRate);

      Serial.print(F("max depth "));
      Serial.print(finalZPos);
      Serial.println(F(" mm."));
      Serial.print(F("target feedrate "));
      Serial.print(sys.targetFeedrate);
      Serial.println(F(" mm per min."));

      //set Probe to input with pullup
      pinMode(ProbePin, INPUT_PULLUP);
      digitalWrite(ProbePin, HIGH);

      if (finalZPos != initialZPos) { // inapropriate unit conversion of currentZPos was removed.
        //        now move z to the Z destination;
        //        Currently ignores X and Y options
        //          we need a version of singleAxleMove that quits if the AUXn input changes (goes LOW)
        //          which will act the same as the stop found in singleAxleMove (need both?)
        //        singleAxleMove(&zAxle, finalZPos, zFeedrate);

        /*
           Takes a pointer to an axle object and rotates that axle to endPos distance at speed MMPerMin
        */

        Axle* axle = &zAxle;
        float startingPos          = axle->read();
        float endPos               = finalZPos;
        float moveDist             = endPos - initialZPos; //total distance to move

        float direction            = moveDist / abs(moveDist); //determine the direction of the move

        float stepSizeMMPerLoopInterval           = 0.01;                    //step size in mm for each PID Control LOOP INTERVAL

        //the argument to abs should only be a variable -- splitting calc into 2 lines
        long finalNumberOfSteps    = moveDist / stepSizeMMPerLoopInterval;    //number of steps taken in move
        finalNumberOfSteps = abs(finalNumberOfSteps);

        long numberOfStepsTaken    = 0;
        float whereAxleShouldBeAtThisStep = startingPos;

        axle->attach();
        //  zAxle->attach();

        while (numberOfStepsTaken < finalNumberOfSteps) {
          if (!movementUpdated){
              //find the target point for this step
              whereAxleShouldBeAtThisStep += stepSizeMMPerLoopInterval * direction;

              //write to each axle
              axle->write(whereAxleShouldBeAtThisStep);
              movementUpdate();

              // Run realtime commands
              execSystemRealtime();
              if (sys.stop){return;}

              //increment the number of steps taken
              numberOfStepsTaken++;
          }

          //check for Probe touchdown
          if (checkForProbeTouch(ProbePin)) {
            zAxle.set(0);
            zAxle.endMove(0);
            zAxle.attach(); // should it not be detach at the end of the move?
            Serial.println(F("Router Bit zero depth position is set"));
            return;
          }
        }

        /*
           If we get here, the probe failed to touch down
            - print error
            - STOP execution
        */
        axle->endMove(endPos);
        Serial.println(F("error: probe did not connect\nprogram stopped\nRouter Bit zero depth position not set\n"));
        sys.stop = true;
      } // end if finalZPos != initialZPos

    } else {
      Serial.print(F("G38"));
      Serial.print(readString.substring(3, 5));
      Serial.println(F(" is invalid. Only G38.2 recognized."));
    }
  } else {
    Serial.println(F("G38.2 gcode only available when z-axle is motorized"));
  }
}

void  setMeasurementUnitConversionFactor(float newConversionFactor){
    sys.mmConversionFactor = newConversionFactor;
}
