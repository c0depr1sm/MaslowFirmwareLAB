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
    
    #ifndef Kinematics_h
    #define Kinematics_h


    //Calculation tolerances
    #define DELTAPHI 0.001
    #define DELTAY 0.01
    #define KINEMATICSMAXERROR 0.001
    #define KINEMATICSMAXINVERSE 10
    #define KINEMATICSMAXGUESS 200

    class Kinematics{
        public:
            //setup functions
            Kinematics();
            void init();
            void recomputeGeometry();
            //control functions
            void inverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void forward(const float& chainALength, const float& chainBLength, float* xPos, float* yPos, float xGuess, float yGuess);
            //set geometry
            float sprocketEffectiveRadius = 10.1;                                //sprocket radius
            // ref: madgrizzle proposal to let Kynematics handle chain tolerance and motor positions.
            // here some default values upon initialization
            float leftMotorX = -1800.0;
            float leftMotorY = 1200.0;
            float topBeamLeftTipFlexAndTwistVerticalCorrection = 0; // latest calculated left motor vertical position correction 
            float rightMotorX = 1800.0;
            float rightMotorY = 1200.0;
            float topBeamRightTipFlexAndTwistVerticalCorrection = 0; // latest calculated right motor vertical position correction 
            float leftChainTolerance = 0;
            float rightChainTolerance = 0;

            float halfWidth;                      //Half the machine width
            float halfHeight;                    //Half the machine height
            //float RleftChainTolerance     = 10.1;    // Left sprocket radius including chain tolerance --unused
            //float RrightChainTolerance    = 10.1;    // Right sprocket radius including chain tolerance --unused
         private:
            void  quadrilateralInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  triangularInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            // common items
            void _constrainToWorkSurface(float* xTarget,float* yTarget);
             // quadrilateral specific
            float _h; //distance between sled attach point and bit
            float _moment(const float& Y1Plus, const float& Y2Plus, const float& MSinPhi, const float& MSinPsi1, const float& MCosPsi1, const float& MSinPsi2, const float& MCosPsi2);
            float _YOffsetEqn(const float& YPlus, const float& Denominator, const float& Psi);
            void  _MatSolv();
            void  _MyTrig();
            float _x = 0; //target router bit coordinates.
            float _y = 0;
             //Criterion Computation Variables
            float Phi = -0.2;
            float TanGamma; 
            float TanLambda;
            float Y1Plus ;
            float Y2Plus;
            float Theta;
            float Psi1 = Theta - Phi;
            float Psi2 = Theta + Phi;
            float Jac[9];
            float Solution[3];
            float Crit[3];
            float Offsetx1;
            float Offsetx2;
            float Offsety1;
            float Offsety2;
            float SinPsi1;
            float CosPsi1;
            float SinPsi2;
            float CosPsi2;
            float SinPsi1D;
            float CosPsi1D;
            float SinPsi2D;
            float CosPsi2D;
            float MySinPhi;
            float MySinPhiDelta;
            boolean Mirror;
            //intermediate output
            float Lambda;
            float Gamma;
            // output = chain lengths measured from 12 o'clock
            float Chain1; //left chain length 
            float Chain2; //right chain length

    };

    #endif
