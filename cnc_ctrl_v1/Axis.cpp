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


#include "Maslow.h"

// It is true that an Axis is a straitgh line about which a body rotates, but it must not be confused with either the x, y, or z coordinates directions of a coordinate system. 
// Hence, The rotation object is renamed to "Axle" who has the property of carying a load and move along a rotation axis.
// The axle also has a displacement pitch property related to a wheel/sprocket or screw or other. 
void Axle::setup(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const char& axleName, const unsigned long& loopInterval)
{
    // I don't really like this, but I don't know how else to initialize a pointer to a value
    float zero = 0.0;
    float one = 1.0;
    _Kp = _Ki = _Kd = &zero;
    
    motorGearboxEncoder.setup(pwmPin, directionPin1, directionPin2, encoderPin1, encoderPin2, loopInterval);
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, _Kp, _Ki, _Kd, &one, REVERSE);
    
    //initialize variables
    _axleName     = axleName;
    
    initializePID(loopInterval);
    
    motorGearboxEncoder.setName(&_axleName);
}

void   Axle::initializePID(const unsigned long& loopInterval){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-20, 20);
    _pidController.SetSampleTime( loopInterval / 1000);
}

void    Axle::write(const float& targetPosition){
    _timeLastMoved = millis();
    _pidSetpoint   =  targetPosition/ *_mmPerRotation;
    return;
}

float  Axle::read(){
    //returns the true axle position
    
    return (motorGearboxEncoder.encoder.read()/ *_encoderSteps) * *_mmPerRotation;
    
}

float  Axle::setpoint(){
    return _pidSetpoint * *_mmPerRotation;
}

void   Axle::set(const float& newAxlePosition){
    
    //reset everything to the new value
    _pidSetpoint  =  newAxlePosition/ *_mmPerRotation;
    motorGearboxEncoder.encoder.write((newAxlePosition * *_encoderSteps)/ *_mmPerRotation);
    
}

long Axle::steps(){
    /*
    Returns the number of steps reported by the encoder
    */
    return motorGearboxEncoder.encoder.read();
}

void   Axle::setSteps(const long& steps){
    
    //reset everything to the new value
    _pidSetpoint  =  steps/ *_encoderSteps;
    motorGearboxEncoder.encoder.write(steps);
    
}

void   Axle::computePID(){
    
    #ifdef FAKE_SERVO
      if (motorGearboxEncoder.motor.attachedPIDControl()){
        // Adds up to 10% error just to simulate servo noise
        double rpm = (-1 * _pidOutput) * random(90, 110) / 100;
        unsigned long steps = motorGearboxEncoder.encoder.read() + round( rpm * *_encoderSteps * LOOPINTERVAL)/(60 * 1000000);
        motorGearboxEncoder.encoder.write(steps);
      }
    #endif

    if (_disableAxleForTesting || !motorGearboxEncoder.motor.attachedPIDControl()){
        return;
    }
    
    _pidInput      =  motorGearboxEncoder.encoder.read()/ *_encoderSteps;
    
    if (_pidController.Compute()){
        // Only write output if the PID calculation was performed
        motorGearboxEncoder.write(_pidOutput);
    }
    
    motorGearboxEncoder.computePID();
    
}

void   Axle::disablePositionPID(){
    
    _pidController.SetMode(MANUAL);
    
}

void   Axle::enablePositionPID(){
    
    _pidController.SetMode(AUTOMATIC);
    
}

void   Axle::setPIDValues(float* KpPos, float* KiPos, float* KdPos, float* propWeight, float* KpV, float* KiV, float* KdV, float* propWeightV){
    /*
    
    Sets the positional PID values for the axle
    
    */
    _Kp = KpPos;
    _Ki = KiPos;
    _Kd = KdPos;
    
    _pidController.SetTunings(_Kp, _Ki, _Kd, propWeight);
    
    motorGearboxEncoder.setPIDValues(KpV, KiV, KdV, propWeightV);
}

String  Axle::getPIDString(){
    /*
    
    Get PID tuning values
    
    */
    String PIDString = "Kp=";
    return PIDString + *_Kp + ",Ki=" + *_Ki + ",Kd=" + *_Kd;
}

void   Axle::setPIDAggressiveness(float aggressiveness){
    /*
    
    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.
    
    */
    
    motorGearboxEncoder.setPIDAggressiveness(aggressiveness);
}

float  Axle::error(){

    float encoderErr = (motorGearboxEncoder.encoder.read()/ *_encoderSteps) - _pidSetpoint;

    return encoderErr * *_mmPerRotation;
}

void   Axle::changePitch(float *newPitch){
    /*
    Reassign the distance moved per-rotation for the axle.
    */
    _mmPerRotation = newPitch;
}

float  Axle::getPitch(){
    /*
    Returns the distance moved per-rotation for the axle.
    */  
    return *_mmPerRotation;
}

void   Axle::changeEncoderResolution(float *newResolution){
    /*
    Reassign the encoder resolution for the axle.
    */
    _encoderSteps = newResolution;
    
    //push to the gearbox for calculating RPM
    motorGearboxEncoder.setEncoderResolution(*newResolution);
    
}

int    Axle::detach(){
    
    motorGearboxEncoder.motor.detach();
    
    return 1;
}

int    Axle::attach(){
     motorGearboxEncoder.motor.attach();
     return 1;
}

bool   Axle::attachedPIDControl(){
    /*
    
    Returns true if the axle's PID Control is activated, false if it is not.
    
    */
    
    return motorGearboxEncoder.motor.attachedPIDControl();
}

void   Axle::detachIfIdle(){
    /*
    Detaches the axle, turning off the motor and PID control, if it has been
    stationary for more than axlePIDControlDetachTimeOutDelay
    */
    if (millis() - _timeLastMoved > sysSettings.axlePIDControlDetachTimeOutDelay){
        detach();
    }
    
}

void   Axle::endMove(const float& finalTarget){
    
    _timeLastMoved = millis();
    _pidSetpoint    = finalTarget/ *_mmPerRotation;
    
}

void   Axle::stop(){
    /*

    Immediately stop the axle where it is, not where it should be

    */

    _timeLastMoved = millis();
    _pidSetpoint   = read()/ *_mmPerRotation;

}

void   Axle::test(){
    /*
    Test the axle by directly commanding the motor and observing if the encoder moves
    */
    
    Serial.print(F("Testing "));
    Serial.print(_axleName);
    Serial.println(F(" motor:"));
    
    //print something to prevent the connection from timing out
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
    
    int i = 0;
    double encoderPos = motorGearboxEncoder.encoder.read(); //record the position now
    
    //move the motor
    while (i < 1000){
        motorGearboxEncoder.motor.directWrite(255);
        i++;
        maslowDelay(1);
        if (sys.stop){return;}
    }
    
    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() > 500){
        Serial.println(F("Direction 1 - Pass"));
    }
    else{
        Serial.println(F("Direction 1 - Fail"));
    }
    
    //record the position again
    encoderPos = motorGearboxEncoder.encoder.read();
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
    
    //move the motor in the other direction
    i = 0;
    while (i < 1000){
        motorGearboxEncoder.motor.directWrite(-255);
        i++;
        maslowDelay(1);
        if (sys.stop){return;}
    }
    
    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() < -500){
        Serial.println(F("Direction 2 - Pass"));
    }
    else{
        Serial.println(F("Direction 2 - Fail"));
    }
    
    //stop the motor
    motorGearboxEncoder.motor.directWrite(0);
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
}

double  Axle::pidInput(){ return _pidInput * *_mmPerRotation;}
double  Axle::pidOutput(){ return _pidOutput;}
