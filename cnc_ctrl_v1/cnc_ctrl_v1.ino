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
	
	
/* To the projects contributers:
 *
 * it is highly recommended to activate warning output of the arduino gcc compiler.
 * Compiler warnings are a great help to keep the codebase clean and can give clues
 * to potentally wrong code. Also, if a codebase produces too many warnings it gets 
 * more likely that possibly important warnings could be overlooked. 
 *
 * Since the Arduino IDE suppresses any compiler output by default we have to activate it.
 *
 * Therefore Arduino IDE users need to activate compiler output in the 
 * preferences dialog. Additionally Arduino IDE needs to tell the compiler to generate
 * warning  messages. This is done in the Arduino IDE's preferences.txt file - you can 
 * get there via the Preferences Dialog - there is a link to the file at the bottom. 
 * Edit the line "compiler.warning_level=none" to "compiler.warning_level=all"
 * and restart the IDE.
 */
    

// TLE5206 version

#include "Maslow.h"

// Define system global state structure
system_t sys;

// Define the global settings storage - treat as readonly
settings_t sysSettings;

// Global realtime executor bitflag variable for setting various alarms.
byte systemRtExecAlarm;  

// Define axles, it might be tighter to define these within the sys struct
Axle leftAxle;
Axle rightAxle;
Axle zAxle;

// Define kinematics, is it necessary for this to be a class?  Is this really
// going to be reused?
Kinematics kinematics;

void setup(){
    //hardware detect
    sys.shieldPcbVersion = getShieldPCBVersion();
    //initialize communication and report an initial wake-up sign
    Serial.begin(57600);
    Serial.print(F("PCB v1."));
    Serial.print(sys.shieldPcbVersion);
    if (TLE5206 == true) { Serial.print(F(" TLE5206 ")); }
    Serial.println(F(" Detected"));
    //load settings from non-volatile memory
    settingsLoadFromEEprom();
    //configure motion control
    setupAxles(); //connect according to the shield connected to the arduino controller
    settingsLoadStepsFromEEprom(); // load current position last memorized
    leftAxle.setTargetmmPosition(leftAxle.getCurrentmmPosition()); //for each axle reset target position to the current position
    rightAxle.setTargetmmPosition(rightAxle.getCurrentmmPosition());
    zAxle.setTargetmmPosition(zAxle.getCurrentmmPosition());
    //Gcode interpreter setup
    sys.mmConversionFactor = MILLIMETERS; // Default is to use milimeter: This is actually a Gcode value to milimeter conversion factor set to 1 (was inchesToMMConversion)
    readyCommandString.reserve(INCBUFFERLENGTH);           //Allocate memory so that this string doesn't fragment the heap as it grows and shrinks
    gcodeLine.reserve(INCBUFFERLENGTH);

    #ifndef SIMAVR // Using the timer will crash simavr, so we disable it.
                   // Instead, we'll run runsOnATimer periodically in loop().
    Timer1.initialize(LOOPINTERVAL);
    Timer1.attachInterrupt(runsOnATimer);
    #endif
    
    //do we need this since Ground Control is the Gcode sender for the Maslow firmware??? **C0depr1sm
    Serial.println(F("Grbl v1.00"));  // Why GRBL?  Apparently because some programs are silly and look for this as an initialization command
    
    Serial.println(F("ready"));
    reportStatusMessage(STATUS_OK);

}

void runsOnATimer(){
    #if misloopDebug > 0
    if (inMovementLoop && !movementUpdated){
        movementFail = true;
    }
    #endif
    leftAxle.computePID();
    rightAxle.computePID();
    zAxle.computePID();
    movementUpdated = false; // once each Axle was updated, then we can get the next position update
}

void loop(){
    // This section is called on startup and whenever a stop command is issued
    initGCode();
    if (sys.stop){               // only called on sys.stop to prevent stopping
        initMotion();            // on USB disconnect.  Might consider removing 
        setSpindlePower(false);  // this restriction for safety if we are 
    }                            // comfortable that USB disconnects are
                                 // not a common occurrence anymore
    kinematics.init();
    
    // Let's go!
    sys.stop = false;            // We should consider an abort option which
                                 // is not reset automatically such as a software
                                 // limit
    while (!sys.stop){
        gcodeExecuteLoop();
        #ifdef SIMAVR // Normally, runsOnATimer() will, well, run on a timer. See also setup().
        runsOnATimer();
        #endif
        execSystemRealtime();
    }
}
