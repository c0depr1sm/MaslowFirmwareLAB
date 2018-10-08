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
    
    #ifndef Axle_h
    #define Axle_h

    class Axle{
        public:
            MotorGearboxEncoder    motorGearboxEncoder;
            // setup functions
            void   setup(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const char& axlesName, const unsigned long& loopInterval);
            void   setPIDValues(float* Kp, float* Ki, float* Kd, float* propWeight, float* KpV, float* KiV, float* KdV, float* propWeightV);
            void   initializePID(const unsigned long& loopInterval);
            void   setPIDAggressiveness(float aggressiveness);
            void   setEncoderResolution(float* newResolution);
            void   setCurrentEncoderCount(const long& stepsCount);
            void   setmmPitch(float* newPitch);
            void   setCurrentmmPosition(const float& newAxlePosition);
            int    updatePositionFromEncoder();
            // control functions
            void   setTargetmmPosition(const float& targetPosition);
            float  getCurrentmmPosition();
            void   endMoveAtmmPosition(const float& finalTarget);
            void   stop();
            int    attachPWMControl();
            int    detachPWMControl();
            void   detachPWMControlIfIdle();
            void   computePID();
            void   disablePositionPID();
            void   enablePositionPID();
            //inspection functions
            float  getPIDmmPositionError();
            float  getPIDmmPositionsetpoint();
            String getPIDString();
            double getPIDmmPositionInput();
            double getPIDOutput();
            long   getCurrentEncoderCount();
            float  getmmPitch();
            bool   attachedPWMControl();
            void   test();
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(const unsigned int& addr, const float& x);
            float      _readFloat(const unsigned int& addr);
            unsigned long   _timeLastMoved;
            volatile double _pidRotationCountSetPoint;
            volatile double _pidRotationCountInput; 
            volatile double _pidOutput;
            float      *_Kp, *_Ki, *_Kd;
            PID        _pidController;
            float      *_mmPerRotation;
            float      *_encoderStepsCountPerAxleRotation;
            bool       _disableAxleForTesting = false;
            char       _axleName;
    };

    #endif
