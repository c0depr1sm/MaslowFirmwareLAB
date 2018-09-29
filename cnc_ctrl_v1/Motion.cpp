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

// This contains all of the Motion commands

#include "Maslow.h"

// Flag for when to send movement commands. If false, then the PID control process is ready to ge a new target value. When true, the target position was updated but not yet captured by the PID control process loop as the next target.
volatile bool  movementUpdated  =  false;
// Global variables for misloop tracking
#if misloopDebug > 0
  volatile bool  inMovementLoop   =  false;
  volatile bool  movementFail     =  false;
#endif

void initMotion(){
    // Called on startup or after a stop command
    leftAxle.stop();
    rightAxle.stop();
    if(sysSettings.zAxleMotorized){
      zAxle.stop();
    }
}


float calculateFeedrate(const float& stepSizeMMPerLoopInterval, const float& usPerStep){
    /*
    Calculate the time delay between each step for a given feedrate
    */
    
    #define MINUTEINUS 60000000.0
    
    // derivation: ms / step = 1 min in ms / dist in one min
    
    float tempFeedrate = (stepSizeMMPerLoopInterval*MINUTEINUS)/usPerStep;
    
    return tempFeedrate;
}

float computeStepSize(const float& MMPerMin){
    /*
    
    Determines the minimum step size which can be taken for the given feed-rate
    based on the loop interval frequency.  Converts to MM per microsecond first,
    then mutiplies by the number of microseconds in each loop interval
    
    */
    return LOOPINTERVAL*(MMPerMin/(60 * 1000000));
}
 
void movementUpdate(){
  #if misloopDebug > 0
  if (movementFail){
    Serial.println("Movement loop failed to complete before interrupt.");
    movementFail = false;
  }
  #endif
  movementUpdated = true;
}


// why does this return anything
// moveSpeed (formerly MMPerMin) describes the demanded displacement speed in mm/min
int   coordinatedMove(const float& xEnd, const float& yEnd, const float& zEnd, float moveSpeed){
    
    /*The move() function moves the tool in a straight line to the position (xEnd, yEnd) at 
    the speed moveSpeed. 
    * The move is not necessarily occuring at a depth where cutting occurs. So the movespeed is not necessarily a feedrate.
    * Movements are correlated so that regardless of the distances moved in each 
    direction, the tool moves to the target in a straight line. 
    * Speed is scalled down as necessary to make sure no axis exceeds its max rate.
    * This function is used by the G00 and G01 commands. The units at this point should all 
    * be in mm or mm per minute*/
    
    float  xStartingLocation = sys.estimatedBitTipXPosition;
    float  yStartingLocation = sys.estimatedBitTipYPosition;
    float  zStartingLocation = zAxle.read();  // It turn out that the Z axle's length position = the Router Bit z axis position. 
    float  zMaxFeedRate      = getZMaxFeedRate();
    
    //find the total distances to move
    float  distanceToMoveInMM         = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  + sq(zEnd - zStartingLocation));
    float  xDistanceToMoveInMM        = xEnd - xStartingLocation;
    float  yDistanceToMoveInMM        = yEnd - yStartingLocation;
    float  zDistanceToMoveInMM        = zEnd - zStartingLocation;
    
    //compute feed details
    moveSpeed = constrain(moveSpeed, 1, sysSettings.targetMaxXYFeedRate);   //constrain the maximum feedrate, just in case the caller did not yet limit the rate 
    float  stepSizeMMPerLoopInterval  = computeStepSize(moveSpeed);
    float  finalNumberOfSteps   = abs(distanceToMoveInMM/stepSizeMMPerLoopInterval);
    float  delayTime            = LOOPINTERVAL;
    float  neededZFeedRate            = calculateFeedrate(abs(zDistanceToMoveInMM/finalNumberOfSteps), delayTime);
    
    //throttle back feedrate if it exceeds the z axis max feed rate
    if (neededZFeedRate > zMaxFeedRate){
      float  zStepSizeMMPerLoopInterval = computeStepSize(zMaxFeedRate);
      finalNumberOfSteps        = abs(zDistanceToMoveInMM/zStepSizeMMPerLoopInterval);
      stepSizeMMPerLoopInterval = (distanceToMoveInMM/finalNumberOfSteps);
      moveSpeed                  = calculateFeedrate(stepSizeMMPerLoopInterval, delayTime);
    }
    
    // (fraction of distance in x direction)* size of step toward target
    float  xStepSize            = (xDistanceToMoveInMM/finalNumberOfSteps);
    float  yStepSize            = (yDistanceToMoveInMM/finalNumberOfSteps);
    float  zStepSize            = (zDistanceToMoveInMM/finalNumberOfSteps);
    
    //attach the axes
    leftAxle.attach();
    rightAxle.attach();
    if(sysSettings.zAxleMotorized){
      zAxle.attach();
    }
    
    float aChainLength;
    float bChainLength;
    long  numberOfStepsTaken = 0;
    float nextXPosition = sys.estimatedBitTipXPosition;
    float nextYPosition = sys.estimatedBitTipYPosition;
    float nextZPosition = zStartingLocation;
    
    while(numberOfStepsTaken < finalNumberOfSteps){
      
        #if misloopDebug > 0
        inMovementLoop = true;
        #endif
        //if last movment was performed start the next
        if (!movementUpdated) {
            //find the target point for this step
            // This section ~20us
            nextXPosition += xStepSize;
            nextYPosition += yStepSize;
            nextZPosition += zStepSize;
            
            //find the chain lengths for this step
            // This section ~180us
            kinematics.inverse(nextXPosition,nextYPosition,&aChainLength,&bChainLength);
            
            //write to each axis
            // This section ~180us
            leftAxle.write(aChainLength);
            rightAxle.write(bChainLength);
            if(sysSettings.zAxleMotorized){
              zAxle.write(nextZPosition);
            }
            
            movementUpdate();
            
            //record new position estimations
            sys.estimatedBitTipXPosition = nextXPosition;
            sys.estimatedBitTipYPosition = nextYPosition;
            // = nextZPosition; // that is stored in the z axle when we move it.
            
            //increment the number of steps taken
            numberOfStepsTaken++;
            
            // Run realtime commands
            execSystemRealtime();
            if (sys.stop){return 1;}
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    leftAxle.endMove(aChainLength);
    rightAxle.endMove(bChainLength);
    if(sysSettings.zAxleMotorized){
      zAxle.endMove(nextZPosition);
    }
    
    //finalize new position estimations
    sys.estimatedBitTipXPosition = xEnd;
    sys.estimatedBitTipYPosition = yEnd;
    // = nextZPosition; // that is stored in the z axle when we move it.
    
    return 1;
    
}
// moveSpeed (formerly MMPerMin) describes the demanded displacement speed in mm/min
void  singleAxleMove(Axle* axle, const float& endPos, const float& moveSpeed){
    /*
    Takes a pointer to an axle object and rotates that axle up to endPos position at moveSpeed
    */
    
    float startingPos          = axle->read();
    float moveDist             = endPos - startingPos; //total distance to move
    
    float direction            = moveDist/abs(moveDist); //determine the direction of the move
    
    float stepSizeMMPerLoopInterval = computeStepSize(moveSpeed);                    //step size in mm

    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long finalNumberOfSteps    = abs(moveDist/stepSizeMMPerLoopInterval);      //number of steps taken in move
    finalNumberOfSteps = abs(finalNumberOfSteps);
    stepSizeMMPerLoopInterval = stepSizeMMPerLoopInterval*direction;
    
    long numberOfStepsTaken    = 0;
    
    //attach the axle we want to move
    axle->attach();
    
    float whereAxleShouldBeAtThisStep = startingPos;
    #if misloopDebug > 0
    inMovementLoop = true;
    #endif
    while(numberOfStepsTaken < finalNumberOfSteps){
        if (!movementUpdated) {
          //find the target point for this step
          whereAxleShouldBeAtThisStep += stepSizeMMPerLoopInterval;
          
          //write to axle
          axle->write(whereAxleShouldBeAtThisStep);
          movementUpdate();
          
          // Run realtime commands
          execSystemRealtime();
          if (sys.stop){return;}
          
          //increment the number of steps taken
          numberOfStepsTaken++;
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
    axle->endMove(endPos);
    
}

// return the sign of the parameter
int sign(double x) { return x<0 ? -1 : 1; }

// why does this return anything
// moveSpeed (formerly MMPerMin) describes the demanded displacement speed in mm/min
int   arcMove(const float& X1, const float& Y1, const float& X2, const float& Y2, const float& centerX, const float& centerY, const float& moveSpeed, const float& direction){
    /*
    
    Move the machine through an arc from point (X1, Y1) to point (X2, Y2) along the 
    arc defined by center (centerX, centerY) at speed up to moveSpeed
    
    Does not handle moves along the Z axis. So this can only implement a  subset of G2 and G3.
    because helix paths are not supported.
    */
    
    //compute geometry 
    float pi                     =  3.1415;
    float radius                 =  sqrt( sq(centerX - X1) + sq(centerY - Y1) ); 
    float circumference          =  2.0*pi*radius;
    
    float startingAngle          =  atan2(Y1 - centerY, X1 - centerX);
    float endingAngle            =  atan2(Y2 - centerY, X2 - centerX);
    
    // compute chord height of arc
    float chordSquared           = sqrt(sq(X2 - X1) + sq(Y2 - Y1));
    float tau                    = sqrt( sq(radius) - (chordSquared/4.0));
    float chordHeight            = radius - tau;

    //compute angle between lines
    float theta                  =  endingAngle - startingAngle;
    if (direction == COUNTERCLOCKWISE){
        if (theta <= 0){
            theta += 2*pi;
        }
    }
    else {
        //CLOCKWISE
        if (theta >= 0){
            theta -= 2*pi;
        }
    }
    if ((sign(theta) != sign(direction)) || ((abs(chordHeight) < .01) && (abs(theta) < 0.5)) || (radius > 25400)) {
      // There is a parameter error in this line of gcode, either in the size of the angle calculated
      //  or the chord height of the arc between the starting and ending points
      // In either case, the gcode cut was essentially a straight line, so 
      // Replace it with a G1 cut to the endpoint
      String gcodeSubstitution = "G1 X";
      gcodeSubstitution = gcodeSubstitution + String(X2 / sys.mmConversionFactor, 3) + " Y" + String(Y2 / sys.mmConversionFactor, 3) + " ";
      Serial.println("Large-radius arc replaced by straight line to improve accuracy: " + gcodeSubstitution);
      G1(gcodeSubstitution, 1);
      return 1;
    }

    float arcLengthMM            =  circumference * (theta / (2*pi) );
    
    //set up variables for movement
    long numberOfStepsTaken       =  0;
    
    float stepSizeMMPerLoopInterval  =  computeStepSize(moveSpeed);

    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long   finalNumberOfSteps     =  arcLengthMM/stepSizeMMPerLoopInterval;
    //finalNumberOfSteps = abs(finalNumberOfSteps);
    
    //Compute the starting position
    float angleNow = startingAngle;
    float degreeComplete = 0.0;
    
    float aChainLength;
    float bChainLength;
    
    float nextXPosition = sys.estimatedBitTipXPosition;
    float nextYPosition = sys.estimatedBitTipYPosition;
    // float nextZPosition = ...; // z move not yet implemented in arcMove()

    //attach the axes
    leftAxle.attach();
    rightAxle.attach();
    
    while(numberOfStepsTaken < abs(finalNumberOfSteps)){
        #if misloopDebug > 0
        inMovementLoop = true;
        #endif
        
        //if last movement was performed start the next one
        if (!movementUpdated){
            
            degreeComplete = float(numberOfStepsTaken)/float(finalNumberOfSteps);
            
            angleNow = startingAngle + theta*direction*degreeComplete;
            
            nextXPosition = radius * cos(angleNow) + centerX;
            nextYPosition = radius * sin(angleNow) + centerY;
            
            kinematics.inverse(nextXPosition,nextYPosition,&aChainLength,&bChainLength);
            
            leftAxle.write(aChainLength);
            rightAxle.write(bChainLength); 
            //if(sysSettings.zAxleMotorized){ // not yet  implemented in arcMove()
            //  zAxle.write(nextZPosition);
            //}

            //increment the number of steps taken
            movementUpdate();
            
            //record new position estimations
            sys.estimatedBitTipXPosition = nextXPosition;
            sys.estimatedBitTipYPosition = nextYPosition;
            // = nextZPosition; // that is stored in the Z axle object when we move it.

            // Run realtime commands
            execSystemRealtime();
            if (sys.stop){return 1;}

            numberOfStepsTaken++;
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
    kinematics.inverse(X2,Y2,&aChainLength,&bChainLength);
    leftAxle.endMove(aChainLength);
    rightAxle.endMove(bChainLength);
    //if(sysSettings.zAxleMotorized){ // z move not yet implemented in arcMove()
    //  zAxle.endMove(nextZPosition);
    //}
        
    //finalize new position estimations
    sys.estimatedBitTipXPosition = X2;
    sys.estimatedBitTipYPosition = Y2;
    // = nextZPosition; // that is stored in the Z axle object when we move it.
    
    return 1;
}

void  motionDetachIfIdle(){
    /*
    
    This function is called every time the main loop runs. When the machine is executing a move it is not called, but when the machine is
    not executing a line it is called regularly and causes the motors to hold their positions.
    
    */
    
    leftAxle.detachIfIdle();
    rightAxle.detachIfIdle();
    zAxle.detachIfIdle();
}
