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

/*
The Kinematics module relates the lengths of the chains to the position of the cutting head
in X-Y space.
*/

#include "Maslow.h"


Kinematics::Kinematics(){
     recomputeGeometry();
}

void Kinematics::init(){
    recomputeGeometry();
    if (sys.state != STATE_OLD_SETTINGS){
     //Estimate the XY position based on the machine geometry and chain new lenght extending beyond the sproket top.
     forward(leftAxle.getCurrentmmPosition(), rightAxle.getCurrentmmPosition(), &sys.estimatedBitTipXPosition, &sys.estimatedBitTipYPosition, sys.estimatedBitTipXPosition, sys.estimatedBitTipYPosition);
    }
}

void Kinematics::_constrainToWorkSurface(float* xTarget,float* yTarget){
    //If the target point is beyond one of the edges of the board, the machine stops at the edge

    *xTarget = (*xTarget < -halfWidth) ? -halfWidth : (*xTarget > halfWidth) ? halfWidth : *xTarget;
    *yTarget = (*yTarget < -halfHeight) ? -halfHeight : (*yTarget > halfHeight) ? halfHeight : *yTarget;

}

void Kinematics::recomputeGeometry(){
    /*
    Some variables are computed on class creation for the geometry of the machine to reduce overhead,
    calling this function regenerates those values.  These are all floats so they take up
    ~32bytes of RAM to keep them in memory.
    */
    Phi = -0.2;
    _h = sqrt((sysSettings.sledWidth/2)*(sysSettings.sledWidth/2) + sysSettings.sledHeight * sysSettings.sledHeight);
    Theta = atan(2*sysSettings.sledHeight/sysSettings.sledWidth);
    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;
  
    halfWidth = sysSettings.workSurfaceWidth / 2.0f;
    halfHeight = sysSettings.workSurfaceHeight / 2.0f;
    //add this sprocketEffectiveRadius initialisation value to make sure the sysSettings are used. C0depr1sm 2018-10-06
    sprocketEffectiveRadius = (sysSettings.lRDistPerRot)/(2.0f * M_PI); //replaced numerical with value defined in avr-lib.c (see http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html)
    
    // according to madgrizzle proposal to integrate into kynematics the chain tolerance and motor x,y coordinates
    // these are the idealised motor positions BEFORE any any adjustment accounting for deflection induced by sled weight
    leftMotorX = cos(sysSettings.topBeamTilt*DEG_TO_RAD)*sysSettings.distBetweenLRMotorsGearBoxShafts/-2.0;
    leftMotorY = (sysSettings.lRMotorsYOffsetAboveWorkSurface+halfHeight) - (sin(sysSettings.topBeamTilt*DEG_TO_RAD)*sysSettings.distBetweenLRMotorsGearBoxShafts/2.0);
    rightMotorX = cos(sysSettings.topBeamTilt*DEG_TO_RAD)*sysSettings.distBetweenLRMotorsGearBoxShafts/2.0;
    rightMotorY = (sysSettings.lRMotorsYOffsetAboveWorkSurface+halfHeight) + (sin(sysSettings.topBeamTilt*DEG_TO_RAD)*sysSettings.distBetweenLRMotorsGearBoxShafts/2.0);
    
}

void  Kinematics::inverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    /*
    
    This function works as a switch to call either the quadrilateralInverse kinematic function 
    or the triangularInverse kinematic function
    
    It also inserts the coordinates offset wsCorrections to seemlessly correct the remaining errors both
    in inverse and forward kinematics, both on triangular and quadrilateral...
    */
    
    //Confirm that the coordinates are on the wood
    _constrainToWorkSurface(&xTarget, &yTarget);

    float correctedXTarget = xTarget;
    float correctedYTarget = yTarget;
    kinematics.getCorrection(xTarget, yTarget, correctedXTarget, correctedYTarget);
    
    if(sysSettings.kinematicsType == 1){
        quadrilateralInverse(correctedXTarget, correctedYTarget, aChainLength, bChainLength);
    }
    else{
        triangularInverse(correctedXTarget, correctedYTarget, aChainLength, bChainLength);
    }
    
}

void  Kinematics::quadrilateralInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){

    //Confirm that the coordinates are on the wood
//    _constrainToWorkSurface(&xTarget, &yTarget); // moved to trhe inverse dispatch function

    //coordinate shift to put (0,0) in the center of the plywood from the left sprocket
    _y = (halfHeight) + sysSettings.lRMotorsYOffsetAboveWorkSurface  - yTarget;
    _x = (sysSettings.distBetweenLRMotorsGearBoxShafts/2.0) + xTarget;

    //Coordinates definition:
    //         x -->, y |
    //                  v
    // (0,0) at center of left sprocket
    // upper left corner of plywood (270, 270)

    byte Tries = 0;                                  //initialize
    if(_x > sysSettings.distBetweenLRMotorsGearBoxShafts/2.0){                              //the right half of the board mirrors the left half so all computations are done  using left half coordinates.
      _x = sysSettings.distBetweenLRMotorsGearBoxShafts-_x;                                  //Chain lengths are swapped at exit if the x,y is on the right half
      Mirror = true;
    }
    else{
        Mirror = false;
    }

    TanGamma = _y/_x;
    TanLambda = _y/(sysSettings.distBetweenLRMotorsGearBoxShafts-_x);
    Y1Plus = sprocketEffectiveRadius * sqrt(1 + TanGamma * TanGamma);
    Y2Plus = sprocketEffectiveRadius * sqrt(1 + TanLambda * TanLambda);


    while (Tries <= KINEMATICSMAXINVERSE) {

        _MyTrig();
                                             //These criteria will be zero when the correct values are reached
                                             //They are negated here as a numerical efficiency expedient

        Crit[0]=  - _moment(Y1Plus, Y2Plus, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2);
        Crit[1] = - _YOffsetEqn(Y1Plus, _x - _h * CosPsi1, SinPsi1);
        Crit[2] = - _YOffsetEqn(Y2Plus, sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + _h * CosPsi2), SinPsi2);

        if (abs(Crit[0]) < KINEMATICSMAXERROR) {
            if (abs(Crit[1]) < KINEMATICSMAXERROR) {
                if (abs(Crit[2]) < KINEMATICSMAXERROR){
                    break;
                }
            }
        }

                   //estimate the tilt angle that results in zero net _moment about the pen
                   //and refine the estimate until the error is acceptable or time runs out

                          //Estimate the Jacobian components

        Jac[0] = (_moment( Y1Plus, Y2Plus, MySinPhiDelta, SinPsi1D, CosPsi1D, SinPsi2D, CosPsi2D) + Crit[0])/DELTAPHI;
        Jac[1] = (_moment( Y1Plus + DELTAY, Y2Plus, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2) + Crit[0])/DELTAY;
        Jac[2] = (_moment(Y1Plus, Y2Plus + DELTAY, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2) + Crit[0])/DELTAY;
        Jac[3] = (_YOffsetEqn(Y1Plus, _x - _h * CosPsi1D, SinPsi1D) + Crit[1])/DELTAPHI;
        Jac[4] = (_YOffsetEqn(Y1Plus + DELTAY, _x - _h * CosPsi1,SinPsi1) + Crit[1])/DELTAY;
        Jac[5] = 0.0;
        Jac[6] = (_YOffsetEqn(Y2Plus, sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + _h * CosPsi2D), SinPsi2D) + Crit[2])/DELTAPHI;
        Jac[7] = 0.0;
        Jac[8] = (_YOffsetEqn(Y2Plus + DELTAY, sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + _h * CosPsi2D), SinPsi2) + Crit[2])/DELTAY;


        //solve for the next guess
        _MatSolv();     // solves the matrix equation Jx=-Criterion

        // update the variables with the new estimate

        Phi = Phi + Solution[0];
        Y1Plus = Y1Plus + Solution[1];                         //don't allow the anchor points to be inside a sprocket
        Y1Plus = (Y1Plus < sprocketEffectiveRadius) ? sprocketEffectiveRadius : Y1Plus;

        Y2Plus = Y2Plus + Solution[2];                         //don't allow the anchor points to be inside a sprocke
        Y2Plus = (Y2Plus < sprocketEffectiveRadius) ? sprocketEffectiveRadius : Y2Plus;

        Psi1 = Theta - Phi;
        Psi2 = Theta + Phi;

    Tries++;                                       // increment itteration count

    }

    //Variables are within accuracy limits
    //  perform output computation

    Offsetx1 = _h * CosPsi1;
    Offsetx2 = _h * CosPsi2;
    Offsety1 = _h *  SinPsi1;
    Offsety2 = _h * SinPsi2;
    TanGamma = (_y - Offsety1 + Y1Plus)/(_x - Offsetx1);
    TanLambda = (_y - Offsety2 + Y2Plus)/(sysSettings.distBetweenLRMotorsGearBoxShafts -(_x + Offsetx2));
    Gamma = atan(TanGamma);
    Lambda =atan(TanLambda);

    //compute the chain lengths

    if(Mirror){
        Chain2 = sqrt((_x - Offsetx1)*(_x - Offsetx1) + (_y + Y1Plus - Offsety1)*(_y + Y1Plus - Offsety1)) - sprocketEffectiveRadius * TanGamma + sprocketEffectiveRadius * Gamma;   //right chain length
        Chain1 = sqrt((sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + Offsetx2))*(sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + Offsetx2))+(_y + Y2Plus - Offsety2)*(_y + Y2Plus - Offsety2)) - sprocketEffectiveRadius * TanLambda + sprocketEffectiveRadius * Lambda;   //left chain length
    }
    else{
        Chain1 = sqrt((_x - Offsetx1)*(_x - Offsetx1) + (_y + Y1Plus - Offsety1)*(_y + Y1Plus - Offsety1)) - sprocketEffectiveRadius * TanGamma + sprocketEffectiveRadius * Gamma;   //left chain length
        Chain2 = sqrt((sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + Offsetx2))*(sysSettings.distBetweenLRMotorsGearBoxShafts - (_x + Offsetx2))+(_y + Y2Plus - Offsety2)*(_y + Y2Plus - Offsety2)) - sprocketEffectiveRadius * TanLambda + sprocketEffectiveRadius * Lambda;   //right chain length
    }

    *aChainLength = Chain1;
    *bChainLength = Chain2;

}

void  Kinematics::triangularInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    /*
    
    The inverse kinematics (relating an xy coordinate pair to the required chain lengths to hit that point)
    function for a triangular set up where the chains meet at a point, or are arranged so that they simulate 
    meeting at a point.
    
    */
    
    //Confirm that the coordinates are on the work surface
//    _constrainToWorkSurface(&xTarget, &yTarget); // moved to the inverse dispatch function

    //Set up variables
    float leftChainAngle = 0;
    float rightChainAngle = 0;
    float leftChainAroundSprocket = 0;
    float rightChainAroundSprocket = 0;
    float distanceRatio = 1.0f;
    float adjustedLeftMotorY = leftMotorY; 
    float adjustedRightMotorY = rightMotorY; 
    
    //Calculate vertical beam Tip deflection due to sled weight, according to sled horizontal position
    // Assumption: leftMotorX = -rightMotorX
    distanceRatio = (rightMotorX - xTarget)/ sysSettings.distBetweenLRMotorsGearBoxShafts; // note that distance ratio is 0 at right motor X position, and 1 at left motor X possition
    float topBeamRightTipFlexAndTwistVerticalCorrection = (pow(distanceRatio,2) * sysSettings.maxTopBeamTipFlexAndTwist);
    float topBeamLeftTipFlexAndTwistVerticalCorrection = (pow(1.0f-distanceRatio,2) * sysSettings.maxTopBeamTipFlexAndTwist);
    
    adjustedLeftMotorY = leftMotorY - topBeamLeftTipFlexAndTwistVerticalCorrection;
    adjustedRightMotorY = rightMotorY - topBeamRightTipFlexAndTwistVerticalCorrection;

    //Calculate motor axles chain length to the router bit
    float leftMotorDistance = sqrt(pow((leftMotorX - xTarget),2)+pow((adjustedLeftMotorY - yTarget),2)); // updated to reflect new madgrizzle proposal of using X,Y coordinates of motors
    float rightMotorDistance = sqrt(pow((rightMotorX - xTarget),2)+pow((adjustedRightMotorY - yTarget),2));

    //Calculate the chain angles from horizontal, based on if the chain connects to the sled from the top or bottom of the sprocket
    if(sysSettings.chainOverSprocket == 1){
		// Thanks to madgrizle pointing out that 
		//            this part is the chain not touching the sprocket + this part is the chain wrapped around the sprocket(held at tooth pitch distance).
        leftChainAngle  = asin((adjustedLeftMotorY  - yTarget)/leftMotorDistance)  + asin(sprocketEffectiveRadius/leftMotorDistance); // removing chain tolerance and , updated to reflect new way of using X,Y coordinates of motors
        rightChainAngle = asin((adjustedRightMotorY - yTarget)/rightMotorDistance) + asin(sprocketEffectiveRadius/rightMotorDistance);

        leftChainAroundSprocket  = sprocketEffectiveRadius * leftChainAngle; //replaced numerical with value defined in avr-lib.c (see http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html)
        rightChainAroundSprocket = sprocketEffectiveRadius * rightChainAngle; //replaced numerical with value defined in avr-lib.c (see http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html)

    }
    else{
        leftChainAngle  = asin((adjustedLeftMotorY  - yTarget)/leftMotorDistance)  - asin(sprocketEffectiveRadius/leftMotorDistance); // removing chain tolerance and , updated to reflect new way of using X,Y coordinates of motors
        rightChainAngle = asin((adjustedRightMotorY - yTarget)/rightMotorDistance) - asin(sprocketEffectiveRadius/rightMotorDistance);

        leftChainAroundSprocket  = sprocketEffectiveRadius * (M_PI - leftChainAngle); //replaced numerical with value defined in avr-lib.c (see http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html)
        rightChainAroundSprocket = sprocketEffectiveRadius * (M_PI - rightChainAngle); //replaced numerical with value defined in avr-lib.c (see http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html)

    }
    // reduce computation?
    float cos_leftAngle = cos(leftChainAngle);
    float sin_leftAngle = sin(leftChainAngle);
    float tan_leftAngle = sin(leftChainAngle)/cos(leftChainAngle);
    float cos_rightAngle = cos(rightChainAngle);
    float sin_rightAngle = sin(rightChainAngle);
    float tan_rightAngle = sin(rightChainAngle)/cos(rightChainAngle);
    
    //Calculate the straight chain length from the sprocket to the bit, including sprocket radius displacment of the triangle.
    float leftChainStraightSection = sqrt(pow(leftMotorDistance,2)-pow(sprocketEffectiveRadius,2)); // will apply chain tolerance after sag correction... because Ground control computes a correction without chain tolerance? (ask madgrizzle 
    float rightChainStraightSection = sqrt(pow(rightMotorDistance,2)-pow(sprocketEffectiveRadius,2));
    
    //Estimate chains tension, not considering chain weight
    float leftChainT = sysSettings.sledWeight/(cos_leftAngle*tan_rightAngle + sin_leftAngle);
    float rightChainT = sysSettings.sledWeight/(cos_rightAngle*tan_leftAngle + sin_rightAngle);

    //Corect the straight chain length to compensate stretch
    float leftChainElongationFactor = 1+ (leftChainT * sysSettings.chainElongationFactor);
    leftChainStraightSection = leftChainStraightSection / leftChainElongationFactor;
    float rightChainElongationFactor = 1+ (rightChainT * sysSettings.chainElongationFactor);
    rightChainStraightSection = rightChainStraightSection / rightChainElongationFactor;

    //Correct the straight chain lengths to account for chain sag
    if (sysSettings.chainSagCorrectionFactor>=0) {
      leftChainStraightSection  *= (1 + ((sysSettings.chainSagCorrectionFactor / 1000000000000) * pow(cos_leftAngle,2)  * pow(leftChainStraightSection,2)  * pow(tan_rightAngle * cos_leftAngle  + sin_leftAngle,2)));
      rightChainStraightSection *= (1 + ((sysSettings.chainSagCorrectionFactor / 1000000000000) * pow(cos_rightAngle,2) * pow(rightChainStraightSection,2) * pow(tan_leftAngle  * cos_rightAngle + sin_rightAngle,2)));
    }
    //Calculate total chain lengths accounting for sprocket geometry and chain sag
    // 2018-12-02 test: inverse compensation: divide instead of multiply.
    float leftChainReachBeyondSprocketTop = leftChainAroundSprocket + leftChainStraightSection / sysSettings.leftChainLengthCorrection;  // madgrizzle point out: "added the chain tolerance here.. "
    float rightChainReachBeyondSprocketTop = rightChainAroundSprocket + rightChainStraightSection / sysSettings.rightChainLengthCorrection;

    //Subtract of the virtual length which is added to the chain by the rotation mechanism
    leftChainReachBeyondSprocketTop = leftChainReachBeyondSprocketTop - sysSettings.sledRotationDiskRadius;
    rightChainReachBeyondSprocketTop = rightChainReachBeyondSprocketTop - sysSettings.sledRotationDiskRadius;
    
    *aChainLength = leftChainReachBeyondSprocketTop;
    *bChainLength = rightChainReachBeyondSprocketTop;
}

void  Kinematics::forward(const float& chainALength, const float& chainBLength, float* xPos, float* yPos, float xGuess, float yGuess){
  
    Serial.println(F("[Forward Calculating Position]"));
    

    float guessLengthA;
    float guessLengthB;

    int guessCount = 0;

    while(1){


        //check our guess
        inverse(xGuess, yGuess, &guessLengthA, &guessLengthB);

        float aChainError = chainALength - guessLengthA;
        float bChainError = chainBLength - guessLengthB;


        //adjust the guess based on the result
        xGuess = xGuess + .1*aChainError - .1*bChainError;
        yGuess = yGuess - .1*aChainError - .1*bChainError;
        
        guessCount++;

        #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0 
          Serial.print(F("[PEk:"));
          Serial.print(aChainError);
          Serial.print(',');
          Serial.print(bChainError);
          Serial.print(',');
          Serial.print('0');
          Serial.println(F("]"));
        #endif

        execSystemRealtime();
        // No need for sys.stop check here

        //if we've converged on the point...or it's time to give up, exit the loop
        if((abs(aChainError) < .1 && abs(bChainError) < .1) or guessCount > KINEMATICSMAXGUESS or guessLengthA > sysSettings.maxChainReachBeyondSprocketTop  or guessLengthB > sysSettings.maxChainReachBeyondSprocketTop){
            if((guessCount > KINEMATICSMAXGUESS) or guessLengthA > sysSettings.maxChainReachBeyondSprocketTop or guessLengthB > sysSettings.maxChainReachBeyondSprocketTop){
                Serial.print(F("Message: Unable to find valid machine position for chain lengths "));
                Serial.print(chainALength);
                Serial.print(", ");
                Serial.print(chainBLength);
                Serial.println(F(" . Please set the chains to a known length (Actions -> Set Chain Lengths)"));
                *xPos = 0;
                *yPos = 0;
            }
            else{
                Serial.println("position loaded at:");
                Serial.println(xGuess);
                Serial.println(yGuess);
                *xPos = xGuess;
                *yPos = yGuess;
            }
            break;
        }
    }
}

void  Kinematics::_MatSolv(){
    float Sum;
    int NN;
    int i;
    int ii;
    int J;
    int JJ;
    int K;
    int KK;
    int L;
    int M;
    int N;

    float fact;

    // gaus elimination, no pivot

    N = 3;
    NN = N-1;
    for (i=1;i<=NN;i++){
        J = (N+1-i);
        JJ = (J-1) * N-1;
        L = J-1;
        KK = -1;
        for (K=0;K<L;K++){
            fact = Jac[KK+J]/Jac[JJ+J];
            for (M=1;M<=J;M++){
                Jac[KK + M]= Jac[KK + M] -fact * Jac[JJ+M];
            }
        KK = KK + N;
        Crit[K] = Crit[K] - fact * Crit[J-1];
        }
    }

//Lower triangular matrix solver

    Solution[0] =  Crit[0]/Jac[0];
    ii = N-1;
    for (i=2;i<=N;i++){
        M = i -1;
        Sum = Crit[i-1];
        for (J=1;J<=M;J++){
            Sum = Sum-Jac[ii+J]*Solution[J-1];
        }
    Solution[i-1] = Sum/Jac[ii+i];
    ii = ii + N;
    }
}

float Kinematics::_moment(const float& Y1Plus, const float& Y2Plus, const float& MSinPhi, const float& MSinPsi1, const float& MCosPsi1, const float& MSinPsi2, const float& MCosPsi2){   //computes net moment about center of mass
    float Offsetx1;
    float Offsetx2;
    float Offsety1;
    float Offsety2;
    float TanGamma;
    float TanLambda;

    Offsetx1 = _h * MCosPsi1;
    Offsetx2 = _h * MCosPsi2;
    Offsety1 = _h * MSinPsi1;
    Offsety2 = _h * MSinPsi2;
    TanGamma = (_y - Offsety1 + Y1Plus)/(_x - Offsetx1);
    TanLambda = (_y - Offsety2 + Y2Plus)/(sysSettings.distBetweenLRMotorsGearBoxShafts -(_x + Offsetx2));

    return sysSettings.sledCG*MSinPhi + (_h/(TanLambda+TanGamma))*(MSinPsi2 - MSinPsi1 + (TanGamma*MCosPsi1 - TanLambda * MCosPsi2));
}

void Kinematics::_MyTrig(){
    float Phisq = Phi * Phi;
    float Phicu = Phi * Phisq;
    float Phidel = Phi + DELTAPHI;
    float Phidelsq = Phidel * Phidel;
    float Phidelcu = Phidel * Phidelsq;
    float Psi1sq = Psi1 * Psi1;
    float Psi1cu = Psi1sq * Psi1;
    float Psi2sq = Psi2 * Psi2;
    float Psi2cu = Psi2 * Psi2sq;
    float Psi1del = Psi1 - DELTAPHI;
    float Psi1delsq = Psi1del * Psi1del;
    float Psi1delcu = Psi1del * Psi1delsq;
    float Psi2del = Psi2 + DELTAPHI;
    float Psi2delsq = Psi2del * Psi2del;
    float Psi2delcu = Psi2del * Psi2delsq;

    // Phirange is 0 to -27 degrees
    // sin -0.1616   -0.0021    1.0002   -0.0000 (error < 6e-6)
    // cos(phi): 0.0388   -0.5117    0.0012    1.0000 (error < 3e-5)
    // Psi1 range is 42 to  69 degrees,
    // sin(Psi1):  -0.0942   -0.1368    1.0965   -0.0241 (error < 2.5 e-5)
    // cos(Psi1):  0.1369   -0.6799    0.1077    0.9756  (error < 1.75e-5)
    // Psi2 range is 15 to 42 degrees
    // sin(Psi2): -0.1460   -0.0197    1.0068   -0.0008 (error < 1.5e-5)
    // cos(Psi2):  0.0792   -0.5559    0.0171    0.9981 (error < 2.5e-5)

    MySinPhi = -0.1616*Phicu - 0.0021*Phisq + 1.0002*Phi;
    MySinPhiDelta = -0.1616*Phidelcu - 0.0021*Phidelsq + 1.0002*Phidel;

    SinPsi1 = -0.0942*Psi1cu - 0.1368*Psi1sq + 1.0965*Psi1 - 0.0241;//sinPsi1
    CosPsi1 = 0.1369*Psi1cu - 0.6799*Psi1sq + 0.1077*Psi1 + 0.9756;//cosPsi1
    SinPsi2 = -0.1460*Psi2cu - 0.0197*Psi2sq + 1.0068*Psi2 - 0.0008;//sinPsi2
    CosPsi2 = 0.0792*Psi2cu - 0.5559*Psi2sq + 0.0171*Psi2 + 0.9981;//cosPsi2

    SinPsi1D = -0.0942*Psi1delcu - 0.1368*Psi1delsq + 1.0965*Psi1del - 0.0241;//sinPsi1
    CosPsi1D = 0.1369*Psi1delcu - 0.6799*Psi1delsq + 0.1077*Psi1del + 0.9756;//cosPsi1
    SinPsi2D = -0.1460*Psi2delcu - 0.0197*Psi2delsq + 1.0068*Psi2del - 0.0008;//sinPsi2
    CosPsi2D = 0.0792*Psi2delcu - 0.5559*Psi2delsq + 0.0171*Psi2del +0.9981;//cosPsi2

}

float Kinematics::_YOffsetEqn(const float& YPlus, const float& Denominator, const float& Psi){
    float Temp;
    Temp = ((sqrt(YPlus * YPlus - sprocketEffectiveRadius * sprocketEffectiveRadius)/sprocketEffectiveRadius) - (_y + YPlus - _h * sin(Psi))/Denominator);
    return Temp;
}

/* Decode a string of the form 
 *  B18DX591DY591MX1.1X1.2X1.3X1.4X1.5LX2.1X2.2X2.3X2.4X2.5LX3.1X3.2X3.3X3.4X3.5EY4.1Y4.2Y4.3Y4.4Y4.5LY5.1Y5.2Y5.3Y5.4Y5.5LY6.2Y6.2Y6.3Y6.4Y6.5E
 * X1.1X1.2X1.3X1.4X1.5L
 * X2.1X2.2X2.3X2.4X2.5L
 * X3.1X3.2X3.3X3.4X3.5
 * 
 * Y4.1Y4.2Y4.3Y4.4Y4.5L
 * Y5.1Y5.2Y5.3Y5.4Y5.5L
 * Y6.2Y6.2Y6.3Y6.4Y6.5E
 * 
 *  B18DX591DY591MX-1.1X-1.2X-1.3X-1.4X-1.5LX-2.1X-2.2X-2.3X-2.4X-2.5LX-3.1X-3.2X-3.3X-3.4X-3.5EY-4.1Y-4.2Y-4.3Y-4.4Y-4.5LY-5.1Y-5.2Y-5.3Y-5.4Y-5.5LY-6.2Y-6.2Y-6.3Y-6.4Y-6.5E
 *  Blank (but initialised) table
 *  B18DX591DY591MX0X0X0X0X0LX0X0X0X0X0LX0X0X0X0X0EY0Y0Y0Y0Y0LY0Y0Y0Y0Y0LY0Y0Y0Y0Y0E
 *  Real MAslow errors 2019-01-03
 *  B18DX585DY585MX-0.6X-0.3X0X0.2X0.0LX-1.0X-1.0X0X0.3X1.3LX-0.1X-0.6X0X0.6X1.2EY-3.0Y-1.3Y-1.3Y-2.8Y-2.8YLY-2.8Y-2.1Y-2.1Y-2.1Y-2.8YLY-3.0Y-3.0Y-3.8Y-3.1Y-3.0YE
 *  Real Maslow errors 2019-01-04
 *  B18DX585DY585MX-0.6X-0.3X0X0.2X0.0LX-0.5X-0.5X0X0.3X1.3LX-0.6X-0.6X0X0.6X1.2EY-3.0Y-1.3Y-1.3Y-2.8Y-2.8YLY-2.8Y-2.1Y-2.1Y-2.6Y-2.8YLY-3.5Y-3.0Y-4.3Y-3.6Y-3.0YE
 * where DX, DY, MX, X and LX, LY, Y EY are simply arbitrary non digit delimiters for individual values.
 * Only the order of values is important
 * The two first values are int, the following ones are floats values and any precision higher than one digit will be rounded off.  
 * first table line is most negative Y, first line item is most negative X.
 * In the example, DX and DY are preceeding the horizontal and vertical distance between correction values
 * The center of the matrix is always designating the 0,0 in the workspace
 * X and Y values of these tables are offsets to be removed on the x or Y coordinates before executing inverse kinematics calculation. 
 * Another function interpolates table values for coordinates that lie between mutiples of deltaX and deltaY.
 * 
 * The following deals with Gcode line decoding responses but also handles kinnematics private data.
 * Would need to be split.
 * 
 * */
byte Kinematics::setCorrectionGrid(const String& gcodeLine) {

  unsigned int begin = 3; // skip B18 code
  unsigned int end;
  String numberAsString;
  float numberAsFloat;
  int numberAsInt;

  // check if string is not exhausted
  if (begin >= gcodeLine.length()) {
    return(STATUS_BAD_NUMBER_FORMAT); // string too short, bad format
  }
  //locate value, skipping newline delimiter if needed
  end = findEndOfNumber(gcodeLine,begin);
  while (end == begin and begin < gcodeLine.length()) {
    begin = begin +1;
    end = findEndOfNumber(gcodeLine,begin);
  }
  // get deltaX value if exists
  if (end>begin) {// some digits were found
    // extract number
    numberAsString  =  gcodeLine.substring(begin,end);
    numberAsInt   =  numberAsString.toInt();
    begin           =  end+1; // skip next delimiter
    //add value to table X
    wsCorrections.deltaX =  numberAsInt;
    wsCorrections.xRange[0] =  -2*numberAsInt;
    wsCorrections.xRange[1] =  -numberAsInt;
    wsCorrections.xRange[2] =  0;
    wsCorrections.xRange[3] =  numberAsInt;
    wsCorrections.xRange[4] =  2*numberAsInt;
  }
  else {
   return(STATUS_BAD_NUMBER_FORMAT);
  }
  // check if string remains
  if (begin >= gcodeLine.length()) {
    return(STATUS_BAD_NUMBER_FORMAT); // string too short, bad format
  }
  //locate value, skipping newline delimiter if needed
  end = findEndOfNumber(gcodeLine,begin);
  while (end == begin and begin < gcodeLine.length()) {
    begin = begin +1;
    end = findEndOfNumber(gcodeLine,begin);
  }
  // get deltaY value if exists
  if (end>begin) {// some digits were found
    // extract number
    numberAsString  =  gcodeLine.substring(begin,end);
    numberAsInt   =  numberAsString.toInt();
    begin           =  end+1; // skip next delimiter
    //add value to table Y
    wsCorrections.deltaY =  numberAsInt;
    wsCorrections.yRange[0] =  -numberAsInt;
    wsCorrections.yRange[1] =  0;
    wsCorrections.yRange[2] =  numberAsInt;
  }
  else {
   return(STATUS_BAD_NUMBER_FORMAT);
  }
  
  unsigned int i = 0 ;
  unsigned int j = 0 ;
  //table X
  for (j=0;j<CORR_NBLINES;j++){
    for (i=0;i<CORR_NBCOLUMNS;i++){
      // check if string remains
      if (begin >= gcodeLine.length()) {
        return(STATUS_BAD_NUMBER_FORMAT); // string too short, bad format
      }
      //locate value, skipping newline delimiter if needed
      end = findEndOfNumber(gcodeLine,begin);
      while (end == begin and begin < gcodeLine.length()) {
        begin = begin +1;
        end = findEndOfNumber(gcodeLine,begin);
      }
      // get value if exists
      if (end>begin) {// some digits were found
        // extract number
        numberAsString  =  gcodeLine.substring(begin,end);
        numberAsFloat   =  numberAsString.toFloat();
        begin           =  end+1; // skip next delimiter
        //add value to table X
        wsCorrections.xCorrections[j][i]=  (char)(numberAsFloat/CORR_STEPS_SIZE);
      }
      else {
       return(STATUS_BAD_NUMBER_FORMAT);
      }
    }
  } 
  i = 0 ;
  j = 0 ;
  //table Y
  for (j=0;j<CORR_NBLINES;j++){
    for (i=0;i<CORR_NBCOLUMNS;i++){
      // check if string remains
      if (begin >= gcodeLine.length()) {
        return(STATUS_BAD_NUMBER_FORMAT); // string too short, bad format
      }
      //locate value, skipping newline delimiter if needed
      end = findEndOfNumber(gcodeLine,begin);
      while (end == begin and begin < gcodeLine.length()) {
        begin = begin +1;
        end = findEndOfNumber(gcodeLine,begin);
      }
      // get value if exists
      if (end>begin) {// some digits were found
        // extract number
        numberAsString  =  gcodeLine.substring(begin,end);
        numberAsFloat   =  numberAsString.toFloat();
        begin           =  end+1; // skip next delimiter
        //add value to table Y
        wsCorrections.yCorrections[j][i]=  (char)(numberAsFloat/CORR_STEPS_SIZE);
      }
      else {
       return(STATUS_BAD_NUMBER_FORMAT);
      }
    }
  } 
  return(STATUS_OK);
}

/* The following function could be a Repport.cpp item, but it deals with private Kinamatics class data.
 * Would need to split the string generation and string serial port communication. 
*/
void  Kinematics::reportCorrectionGrid(){
    #ifndef REPORT_GUI_MODE
    unsigned int i = 0 ;
    unsigned int j = 0 ;
    Serial.print(F("deltas to be used for X and Y value corrections\r\n"));
    Serial.print(F(" deltaX=")); Serial.print(wsCorrections.deltaX,0);
    Serial.print(F(" deltaY=")); Serial.print(wsCorrections.deltaY,0);
    Serial.print(F("\r\n Ranges to be used for X and Y value corrections\r\n"));
    Serial.print(F(" xRange=")); Serial.print(wsCorrections.xRange[0],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.xRange[1],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.xRange[2],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.xRange[3],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.xRange[4],0);
    Serial.print(F("\r\n yRange=")); Serial.print(wsCorrections.yRange[0],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.yRange[1],0);
    Serial.print(F(" ,")); Serial.print(wsCorrections.yRange[2],0);
    
    Serial.print(F("\r\n Corrections to be added to X values before computing chain lengths\r\n"));
    for (j=0;j<CORR_NBLINES;j++){
      for (i=0;i<CORR_NBCOLUMNS;i++){
        Serial.print(F(" X"));
        Serial.print(j+1,DEC); Serial.print(i+1,DEC); Serial.print('=');
        Serial.print(wsCorrections.xCorrections[j][i]*CORR_STEPS_SIZE ,1);
      }
      Serial.println(F("\r\n"));
    } 
    Serial.print(F("Corrections to be added to Y values before computing chain lengths\r\n"));
    for (j=0;j<CORR_NBLINES;j++){
      for (i=0;i<CORR_NBCOLUMNS;i++){
        Serial.print(F(" Y"));
        Serial.print(j+1,DEC); Serial.print(i+1,DEC); Serial.print('=');
        Serial.print(wsCorrections.yCorrections[j][i]*CORR_STEPS_SIZE ,1);
      }
      Serial.println(F("\r\n"));
    } 
    #endif
}

/*
 * This function should yield an error if the correction table was not initialised. 
 * 
*/
void Kinematics::getCorrection(const float xposraw, const float yposraw, float& xPosCorrected, float& yPosCorrected) {
  // analyse the coordinates to find the correction matrix zone to use
  int i = 0;
  int j = 0;
  bool inXRange = false;
  bool inYRange = false;
  float dx =0.0;
  float dy =0.0;
  float px11 =0.0;
  float px12 =0.0;
  float px21 =0.0;
  float px22 =0.0;
  float py11 =0.0;
  float py12 =0.0;
  float py21 =0.0;
  float py22 =0.0;
  float dE1 = 0.0;
  float dE2 = 0.0;
  float xC = 0.0;
  float yC = 0.0;
  float A = 0.0;
  float B = 0.0;
  float Cx = 0.0;
  float Cy = 0.0;

  if (wsCorrections.deltaX==0) { // this is a sign the correction table was not initialised
    Serial.print(F("error: correction table not initialised"));
    xPosCorrected = xposraw;
    yPosCorrected = yposraw;
    return;
  }        
  
  #if defined (verboseDebug) && verboseDebug > 0
  Serial.print(F(" xraw,yraw = ")); Serial.print(xposraw,1); Serial.print(yposraw,1);
  #endif  

        
  for (i=0;i<CORR_NBCOLUMNS;i++){
    if (xposraw <= wsCorrections.xRange[i]) {
      inXRange = true;
      break;
    }
  }
  //fix index if value is above table range
  if (inXRange == false) i=CORR_NBCOLUMNS-1;
  
  for (j=0;j<CORR_NBLINES;j++){
    if (yposraw <= wsCorrections.yRange[j]) {
      inYRange = true;
      break;
    }
  }
  //fix index if value is above table range
  if (inYRange == false) j=CORR_NBLINES-1;

  #if defined (verboseDebug) && verboseDebug > 0
  Serial.print(F(" j,i = ")); Serial.print(j,DEC); Serial.print(','); Serial.print(i,DEC);
  #endif  

  // i and j now are indicating the correction table interval to be used.
  // lets collect data to compute the interpolation.  
  if (i>0 && j>0 && inXRange==true && inYRange==true) {
    // far from table edges, we can use freely the indices
    dx = xposraw-wsCorrections.xRange[i-1];
    dy = yposraw-wsCorrections.yRange[j-1];
    px11 = wsCorrections.xCorrections[j-1][i-1];
    px12 = wsCorrections.xCorrections[j-1][i];
    px21 = wsCorrections.xCorrections[j][i-1];
    px22 = wsCorrections.xCorrections[j][i];
    py11 = wsCorrections.yCorrections[j-1][i-1];
    py12 = wsCorrections.yCorrections[j-1][i];
    py21 = wsCorrections.yCorrections[j][i-1];
    py22 = wsCorrections.yCorrections[j][i];
  }
  else {
    if (i==0 || inXRange==false) { // leftmost or right most column edge 
      dx = wsCorrections.deltaX;
      if (j==0 || inYRange ==false) { // corner
        dy = wsCorrections.deltaY;
        px11 = wsCorrections.xCorrections[j][i];
        px12 = wsCorrections.xCorrections[j][i];
        px21 = wsCorrections.xCorrections[j][i];
        px22 = wsCorrections.xCorrections[j][i];
        py11 = wsCorrections.yCorrections[j][i];
        py12 = wsCorrections.yCorrections[j][i];
        py21 = wsCorrections.yCorrections[j][i];
        py22 = wsCorrections.yCorrections[j][i];        
      }
      else { // mid left or right edge column
        dy = yposraw-wsCorrections.yRange[j-1];
        px11 = wsCorrections.xCorrections[j-1][i];
        px12 = wsCorrections.xCorrections[j-1][i];
        px21 = wsCorrections.xCorrections[j][i];
        px22 = wsCorrections.xCorrections[j][i];
        py11 = wsCorrections.yCorrections[j-1][i];
        py12 = wsCorrections.yCorrections[j-1][i];
        py21 = wsCorrections.yCorrections[j][i];
        py22 = wsCorrections.yCorrections[j][i];
      }
    }
    else {
      if (j==0 || inYRange==false) { // top or bottom rows excluding corners
        dy = wsCorrections.deltaY;
        dx = xposraw-wsCorrections.xRange[i-1];
        px11 = wsCorrections.xCorrections[j][i-1];
        px12 = wsCorrections.xCorrections[j][i];
        px21 = wsCorrections.xCorrections[j][i-1];
        px22 = wsCorrections.xCorrections[j][i];
        py11 = wsCorrections.yCorrections[j][i-1];
        py12 = wsCorrections.yCorrections[j][i];
        py21 = wsCorrections.yCorrections[j][i-1];
        py22 = wsCorrections.yCorrections[j][i];
      }        
      else {
        Serial.print(F("error: correction out of bound"));
        dy = wsCorrections.deltaY; 
        dx = wsCorrections.deltaX;
        px11 = 1;
        px12 = 1;
        px21 = 1;
        px22 = 1;
        py11 = 1;
        py12 = 1;
        py21 = 1;
        py22 = 1;
      }
    }
  } 
  // calculs de Cx et Cy.
  dE1 = px12-px11;
  dE2 = px22-px21;
  xC = dx/wsCorrections.deltaX;
  yC = dy/wsCorrections.deltaY;
  A = px11+dE1*xC;
  B = px21+dE2*xC;
  Cx = (A+(B-A)*yC)*CORR_STEPS_SIZE;
  #if defined (verboseDebug) && verboseDebug > 0
  Serial.print(F(" Cx Correction = ")); Serial.print(Cx,1);
  #endif  
  //offset is to be removed from target x value
  xPosCorrected = xposraw-Cx;

  dE1 = py12-py11;
  dE2 = py22-py21;
  xC = dx/wsCorrections.deltaX;
  yC = dy/wsCorrections.deltaY;
  A = py11+dE1*xC;
  B = py21+dE2*xC;
  Cy = (A+(B-A)*yC)*CORR_STEPS_SIZE;
  #if defined (verboseDebug) && verboseDebug > 0
  Serial.print(F(" Cy Correction = ")); Serial.print(Cy,1);
  #endif  
  //offset is to be removed from target y value
  yPosCorrected = yposraw-Cy;
 
}
