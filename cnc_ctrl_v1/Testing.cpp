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

// This file contains various testing provisions

#include "Maslow.h"

void PIDTestVelocity(Axle* axle, const float start, const float stop, const float steps, const float version){
    // Moves the defined Axle at series of speed steps for PID tuning
    // Start Log
    Serial.println(F("--PID Velocity Test Start--"));
    Serial.println(axle->motorGearboxEncoder.getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output"));
    }

    double startTime;
    double print = micros();
    double current = micros();
    float error;
    float reportedSpeed;
    float span = stop - start;
    float speed;
   
    // Start the steps
    axle->disablePositionPID();
    axle->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        speed = start;
        if (i > 0){
            speed = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        axle->motorGearboxEncoder.write(speed);
        while (startTime + 2000000 > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              Serial.println(axle->motorGearboxEncoder.pidState());
            }
            else {
              reportedSpeed= axle->motorGearboxEncoder.cachedSpeed();
              error =  reportedSpeed - speed;
              print = current;
              Serial.println(error);
            }
          }
          current = micros();
        }
    }
    axle->motorGearboxEncoder.write(0);
    
    // Print end of log, and update axle for use again
    Serial.println(F("--PID Velocity Test Stop--\n"));
    axle->write(axle->read());
    axle->detach();
    axle->enablePositionPID();

    //Estimate the XY position based on the machine geometry and chain lenght extending beyond the sproket top.
    kinematics.forward(leftAxle.read(), rightAxle.read(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, 0, 0);

}

void positionPIDOutput (Axle* axle, float setpoint, float startingPoint){
  Serial.print((setpoint - startingPoint), 4);
  Serial.print(F(","));
  Serial.print((axle->pidInput() - startingPoint),4);
  Serial.print(F(","));  
  Serial.print(axle->pidOutput(),4);
  Serial.print(F(","));
  Serial.print(axle->motorGearboxEncoder.cachedSpeed(), 4);
  Serial.print(F(","));
  Serial.println(axle->motorGearboxEncoder.motor.lastSpeed());
}

void PIDTestPosition(Axle* axle, float start, float stop, const float steps, const float stepTime, const float version){
    // Moves the defined Axis at series of chain distance steps for PID tuning
    // Start Log
    Serial.println(F("--PID Position Test Start--"));
    Serial.println(axle->getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output,rpminput,voltage"));
    }

    unsigned long startTime;
    unsigned long print = micros();
    unsigned long current = micros();
    float error;
    float startingPoint = axle->read();
    start = startingPoint + start;
    stop  = startingPoint + stop;
    float span = stop - start;
    float location;
    
    // Start the steps
    axle->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        location = start;
        if (i > 0){
            location = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        current = micros();
        axle->write(location);
        while (startTime + (stepTime * 1000) > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              positionPIDOutput(axle, location, startingPoint);
            }
            else {
              error   =  axle->read() - location;
              Serial.println(error);
            }
            print = current;
          }
          current = micros();
        }
    }
    startTime = micros();
    current = micros();
    //Allow 1 seccond to settle out
    while (startTime + 1000000 > current){
      if (current - print > LOOPINTERVAL){
        if (version == 2) {
          positionPIDOutput(axle, location, startingPoint);
        }            
        else {
          error   =  axle->read() - location;
          Serial.println(error);
        }
        print = current;
      }
      current = micros();
    }
    // Print end of log, and update axle for use again
    Serial.println(F("--PID Position Test Stop--\n"));
    axle->write(axle->read());
    axle->detach();

    //Estimate the XY position based on the machine geometry and chain lenght extending beyond the sproket top.
    kinematics.forward(leftAxle.read(), rightAxle.read(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, 0, 0);

}

void voltageTest(Axle* axle, int start, int stop){
    // Moves the specified Axle object according to a series of motor voltages and reports the resulting
    // RPMs
    Serial.println(F("--Voltage Test Start--"));
    int direction = 1;
    if (stop < start){ direction = -1;}
    int steps = abs(start - stop);
    unsigned long startTime = millis() + 200;
    unsigned long currentTime = millis();
    unsigned long printTime = 0;
   
    for (int i = 0; i <= steps; i++){
        axle->motorGearboxEncoder.motor.directWrite((start + (i*direction)));
        while (startTime > currentTime - (i * 200)){
            currentTime = millis();
            if ((printTime + 50) <= currentTime){
                Serial.print((start + (i*direction)));
                Serial.print(F(","));
                Serial.print(axle->motorGearboxEncoder.computeSpeed(),4);
                Serial.print(F("\n"));
                printTime = millis();
            }
        }
    }
    
    // Print end of log, and update axle for use again
    axle->motorGearboxEncoder.motor.directWrite(0);
    Serial.println(F("--Voltage Test Stop--\n"));
    axle->write(axle->read());
    //Estimate the XY position based on the machine geometry and chain lenght extending beyond the sproket top.
    kinematics.forward(leftAxle.read(), rightAxle.read(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, 0, 0);

}
