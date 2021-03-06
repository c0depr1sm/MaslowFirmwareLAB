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

// This file contains the machine settings that are saved to eeprom

// EEPROM addresses 300 and up can be used by Maslow.  Under 300 was used
// previously by pre v1.00 Firmware.

#include "Maslow.h"
#include <EEPROM.h>

void settingsLoadFromEEprom(){
    /*
    Loads data from EEPROM if EEPROM data is valid, only called on startup

    Settings are stored starting at address 340 all the way up.
    */
    settingsVersion_t settingsVersionStruct;
    settings_t tempSettings;

    settingsReset(); // Load default values first
    EEPROM.get(300, settingsVersionStruct);
    EEPROM.get(340, tempSettings);
    if (settingsVersionStruct.settingsVersion == SETTINGSVERSION &&
        settingsVersionStruct.eepromValidData == EEPROMVALIDDATA &&
        tempSettings.eepromValidData == EEPROMVALIDDATA){
          sysSettings = tempSettings;
    }
    else {
      reportStatusMessage(STATUS_SETTING_READ_FAIL);
    }

    // Apply settings
    setPWMPrescalers(int(sysSettings.fPWM));
    kinematics.recomputeGeometry();
    leftAxle.setEncoderResolution(&sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn);
    rightAxle.setEncoderResolution(&sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn);
    // chain pitch tolerances are handled by the Kynematics calculation function. Not anymore recorded into Axle pitch. 
    //The Kinematics also gets lRDistPerRot information through sprocketEffectiveRadius. These are linked and should be tracked together.
    leftAxle.setmmPitch(&sysSettings.lRDistPerRot);
    rightAxle.setmmPitch(&sysSettings.lRDistPerRot);
    zAxle.setmmPitch(&sysSettings.zDistPerRot);
    zAxle.setEncoderResolution(&sysSettings.encoderZScrewStepsCountPerTurn);
}

void settingsReset() {
    /*
    Loads default data into settings, many of these values are approximations
    from the an ideal stock frame.  Other values are just the recommended
    value.  Ideally we want these defaults to match the defaults in GroundControl
    so that if a value is not changed by a user or is not used, it doesn't
    need to be updated here.
    */
    sysSettings.workSurfaceWidth = 2446; // float workSurfaceWidth
    sysSettings.workSurfaceHeight = 1222; // float workSurfaceHeight;
    sysSettings.distBetweenLRMotorsGearBoxShafts = 3501.2; // float distBetweenLRMotors;
    sysSettings.lRMotorsYOffsetAboveWorkSurface = 618;  // float LRMotorsYOffsetAboveWorkSurface;
    sysSettings.sledWidth = 310.0;  // float sledWidth;
    sysSettings.sledHeight = 139.0;  // float sledHeight;
    sysSettings.sledCG = 79.0;   // float sledCG;
    sysSettings.kinematicsType = 1;      // byte kinematicsType;
    sysSettings.sledRotationDiskRadius = 138.14;  // float sledRotationDiskRadius;
    sysSettings.axlePIDControlDetachTimeOutDelay = 2000;   // int axlePIDControlDetachTimeOutDelay;
/*
 blurfl commit da17048 on october 1st 2018 
 *	add missing setting to defaults
 *  settingsReset() does not include a default value for chainLength. Without a value for chainLength,  kinematics fails with:
 * ```
 *  "Message: Unable to find valid machine position for chain lengths "
 * ```
 *  GC responds by sending it's own setting value ($10), but the kinematics error requires the user to recalibrate the chains.
 */ 
    sysSettings.maxChainReachBeyondSprocketTop = 3450;   // int maximum length of chain;
    sysSettings.originalChainLength = 2000;   // int originalChainLength;
    sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn = 8113.73; // float encoderLRMotorStepsCountPerOutputShaftTurn -- Updated by madgrizzle on sept 10 2018
    sysSettings.lRDistPerRot = 63.5;   // float distPerRot;
    sysSettings.maxXYFeedRate = 800;   // int targetMaxXYFeedRate
    sysSettings.zAxleMotorized = true;   // zAxleMotorized;
    sysSettings.spindleAutomateType = NONE;  // bool spindleAutomate;
    sysSettings.zScrewMaxRPM = 12.60;  // float zScrewMaxRPM;
    sysSettings.zDistPerRot = 2.5;   // float zDistPerRot;
    sysSettings.encoderZScrewStepsCountPerTurn = 7560.0; // float encoderZScrewStepsCountPerTurn;
    sysSettings.KpPos = 1300.0; // float KpPos;
    sysSettings.KiPos = 0.0;    // float KiPos;
    sysSettings.KdPos = 34.0;   // float KdPos;
    sysSettings.propWeightPos = 1.0;    // float propWeightPos;
    sysSettings.KpV = 5.0;    // float KpV;
    sysSettings.KiV = 0.0;    // float KiV;
    sysSettings.KdV = 0.28;   // float KdV;
    sysSettings.propWeightV = 1.0;    // float propWeightV;
    sysSettings.zKdPos = 1300.0; // float zKpPos;
    sysSettings.zKiPos = 0.0;    // float zKiPos;
    sysSettings.zKdPos = 34.0;   // float zKdPos;
    sysSettings.zPropWeightPos = 1.0;    // float zPropWeightPos;
    sysSettings.zKpV = 5.0;    // float zKpV;
    sysSettings.zKiV = 0.0;    // float zKiV;
    sysSettings.zKdV = 0.28;   // float zKdV;
    sysSettings.zPropWeightV = 1.0;    // float zPropWeightV;
    sysSettings.chainSagCorrectionFactor = 12;  // float chainSagCorrectionFactor;
    sysSettings.chainOverSprocket = 1;   // byte chainOverSprocket;
    sysSettings.fPWM = 3;   // byte fPWM;
    sysSettings.leftChainLengthCorrection = 1.0+0.00271;    // float leftChainLengthCorrection;
    sysSettings.rightChainLengthCorrection = 1.0-0.0008;    // float rightChainLengthCorrection;
    sysSettings.positionErrorLimit = 2.0;  // float positionErrorLimit;
    sysSettings.topBeamTilt = 0.0; // degree, measured relative to horizontal, counter clockwise is positive 
    sysSettings.maxTopBeamTipFlexAndTwist = 0.0; // mm beam tip vertical shift under sled weight 
    sysSettings.chainElongationFactor = 8.1E-6; // m/m/N
    sysSettings.sledWeight = 11.6*9.8; // Newtons. My sled has one ring kit, one Rigid 2200 router and two 2.35kg bricks on a 5/8" thick mdf 18" diameter base.
    sysSettings.eepromValidData = EEPROMVALIDDATA; // byte eepromValidData;
}

void settingsWipe(byte resetType){
  /*
  Wipes certain bytes in the EEPROM, you probably want to reset after calling
  this
  */
  if (bit_istrue(resetType, SETTINGS_RESTORE_SETTINGS)){
    for (size_t i = 340 ; i < sizeof(sysSettings) + 340 ; i++) {
      EEPROM.write(i, 0);
    }
  }
  else if (bit_istrue(resetType, SETTINGS_RESTORE_MASLOW)){
    for (size_t i = 300 ; i < sizeof(sysSettings) + 340; i++) {
      EEPROM.write(i, 0);
    }
  }
  else if (bit_istrue(resetType, SETTINGS_RESTORE_ALL)){
    for (size_t i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
  }
}

void settingsSaveToEEprom(){
    /*
    Saves settings to EEPROM, only called when settings change

    Settings are stored starting at address 340 all the way up.
    */
    settingsVersion_t settingsVersionStruct = {SETTINGSVERSION, EEPROMVALIDDATA};
    EEPROM.put(300, settingsVersionStruct);
    EEPROM.put(340, sysSettings);
}

void settingsSaveStepstoEEprom(){
    /*
    Saves position to EEPROM, is called frequently by execSystemRealtime

    Steps are saved in address 310 -> 339.  Room for expansion for additional
    axles in the future.
    */
    // don't run if old position data has not been incorporated yet
    if (!sys.oldSettingsFlag){
      settingsStepsV1_t sysSteps = {
        leftAxle.getCurrentEncoderCount(),
        rightAxle.getCurrentEncoderCount(),
        zAxle.getCurrentEncoderCount(),
        EEPROMVALIDDATA
      };
      EEPROM.put(310, sysSteps);
    }
}

void settingsLoadStepsFromEEprom(){
    /*
    Loads position to EEPROM, is called on startup.

    Steps are saved in address 310 -> 339.  Room for expansion for additional
    axles in the future.
    */
    settingsStepsV1_t tempStepsV1;

    EEPROM.get(310, tempStepsV1);
    if (tempStepsV1.eepromValidData == EEPROMVALIDDATA){
            leftAxle.setCurrentEncoderCount(tempStepsV1.lSteps);
            rightAxle.setCurrentEncoderCount(tempStepsV1.rSteps);
            zAxle.setCurrentEncoderCount(tempStepsV1.zSteps);
    }
    else if (EEPROM.read(5) == EEPROMVALIDDATA &&
        EEPROM.read(105) == EEPROMVALIDDATA &&
        EEPROM.read(205) == EEPROMVALIDDATA){
        bit_true(sys.oldSettingsFlag, NEED_ENCODER_STEPS);
        bit_true(sys.oldSettingsFlag, NEED_DIST_PER_ROT);
        bit_true(sys.oldSettingsFlag, NEED_Z_ENCODER_STEPS);
        bit_true(sys.oldSettingsFlag, NEED_Z_DIST_PER_ROT);
        sys.state = STATE_OLD_SETTINGS;
        Serial.println(F("Old position data detected."));
        Serial.println(F("Please set $12, $13, $19, and $20 to load position."));
    }
    else {
        systemRtExecAlarm |= ALARM_POSITION_LOST;  // if this same global is touched by ISR then need to make atomic somehow
                                                   // also need to consider if need difference between flag with bits and
                                                   // error message as a byte.
    }
}

void settingsLoadOldSteps(){
    /*
    Loads the old version of step settings, only called once encoder steps
    and distance per rotation have been loaded.  Wipes the old data once
    incorporated to prevent oddities in the future
    */
    if (sys.state == STATE_OLD_SETTINGS){
      float l, r , z;
      EEPROM.get(9, l);
      EEPROM.get(109, r);
      EEPROM.get(209, z);
      leftAxle.setCurrentmmPosition(l);
      rightAxle.setCurrentmmPosition(r);
      zAxle.setCurrentmmPosition(z);
      for (int i = 0; i <= 200; i = i +100){
        for (int j = 5; j <= 13; j++){
          EEPROM.write(i + j, 0);
        }
      }
      sys.state = STATE_IDLE;
    }
  }

byte settingsStoreGlobalSetting(const byte& parameter,const float& value){
    /*
    Alters individual settings which are then stored to EEPROM.  Returns a
    status message byte value
    */

    // We can add whatever sanity checks we want here and error out if we like
    switch(parameter) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
            switch(parameter) {
                case 0:
                      sysSettings.workSurfaceWidth = value;
                      break;
                case 1:
                      sysSettings.workSurfaceHeight = value;
                      kinematics.recomputeGeometry();
                      break;
                case 2:
                      sysSettings.distBetweenLRMotorsGearBoxShafts = value;
                      kinematics.recomputeGeometry();
                      break;
                case 3:
                      sysSettings.lRMotorsYOffsetAboveWorkSurface = value;
                      kinematics.recomputeGeometry();
                      break;
                case 4:
                      sysSettings.sledWidth = value;
                      break;
                case 5:
                      sysSettings.sledHeight = value;
                      break;
                case 6:
                      sysSettings.sledCG = value;
                      break;
                case 7:
                      sysSettings.kinematicsType = value;
                      break;
                case 8:
                      sysSettings.sledRotationDiskRadius = value;
                      break;
            }
            kinematics.init();
            break;
        case 9:
              sysSettings.axlePIDControlDetachTimeOutDelay = value;
              break;
        case 10:
              sysSettings.maxChainReachBeyondSprocketTop = value;
              break;
        case 11:
              sysSettings.originalChainLength = value;
              break;
        case 12:
              sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn = value;
              leftAxle.setEncoderResolution(&sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn);
              rightAxle.setEncoderResolution(&sysSettings.encoderLRMotorStepsCountPerOutputShaftTurn);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_ENCODER_STEPS);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              kinematics.init();
              break;
        case 13:
              sysSettings.lRDistPerRot = value;
              // now merged in to kinematics.recomputeGeometry() 
              // kinematics.sprocketEffectiveRadius = (sysSettings.lRDistPerRot)/(2.0 * 3.14159);
              kinematics.recomputeGeometry();
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_DIST_PER_ROT);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              kinematics.init();
              break;
        case 15:
              sysSettings.maxXYFeedRate = value;
              break;
        case 16:
              sysSettings.zAxleMotorized = value;
              break;
        case 17: 
              sysSettings.spindleAutomateType = static_cast<SpindleAutomationType>(value);
              break;
        case 18:
              sysSettings.zScrewMaxRPM = value;
              break;
        case 19:
              sysSettings.zDistPerRot = value;
              zAxle.setmmPitch(&sysSettings.zDistPerRot);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_Z_DIST_PER_ROT);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              break;
        case 20:
              sysSettings.encoderZScrewStepsCountPerTurn = value;
              zAxle.setEncoderResolution(&sysSettings.encoderZScrewStepsCountPerTurn);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_Z_ENCODER_STEPS);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              break;
        case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28:
            switch(parameter) {
                case 21:
                      sysSettings.KpPos = value;
                      break;
                case 22:
                      sysSettings.KiPos = value;
                      break;
                case 23:
                      sysSettings.KdPos = value;
                      break;
                case 24:
                      sysSettings.propWeightPos = value;
                      break;
                case 25:
                      sysSettings.KpV = value;
                      break;
                case 26:
                      sysSettings.KiV = value;
                      break;
                case 27:
                      sysSettings.KdV = value;
                      break;
                case 28:
                      sysSettings.propWeightV = value;
                      break;
                }
                leftAxle.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
                rightAxle.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
                break;
        case 29: case 30: case 31: case 32: case 33: case 34: case 35: case 36:
            switch(parameter) {
                case 29:
                      sysSettings.zKpPos = value;
                      break;
                case 30:
                      sysSettings.zKiPos = value;
                      break;
                case 31:
                      sysSettings.zKdPos = value;
                      break;
                case 32:
                      sysSettings.zPropWeightPos = value;
                      break;
                case 33:
                      sysSettings.zKpV = value;
                      break;
                case 34:
                      sysSettings.zKiV = value;
                      break;
                case 35:
                      sysSettings.zKdV = value;
                      break;
                case 36:
                      sysSettings.zPropWeightV = value;
                      break;
            }
            zAxle.setPIDValues(&sysSettings.zKpPos, &sysSettings.zKiPos, &sysSettings.zKdPos, &sysSettings.zPropWeightPos, &sysSettings.zKpV, &sysSettings.zKiV, &sysSettings.zKdV, &sysSettings.zPropWeightV);
            break;
        case 37:
              sysSettings.chainSagCorrectionFactor = value;
              break;
        case 38:
              settingsSaveStepstoEEprom();
              sysSettings.chainOverSprocket = value;
              setupAxles();
              settingsLoadStepsFromEEprom();
              // Set initial desired position of the machine to its current position
              leftAxle.setTargetmmPosition(leftAxle.getCurrentmmPosition());
              rightAxle.setTargetmmPosition(rightAxle.getCurrentmmPosition());
              zAxle.setTargetmmPosition(zAxle.getCurrentmmPosition());
              kinematics.init();
              break;
        case 39:
              sysSettings.fPWM = value;
              setPWMPrescalers(value);
              break;
        case 40:
              // waiting for ground control to change for new meaning. Meanwhile, we convert it here.
              // distPerRotLeftChainTolerance = (1 + (float(self.config.get('Advanced Settings', 'leftChainTolerance')) / 100)) * float(self.config.get('Advanced Settings', 'gearTeeth')) * float(self.config.get('Advanced Settings', 'chainPitch'))
              sysSettings.leftChainLengthCorrection = (value / sysSettings.lRDistPerRot);
              kinematics.recomputeGeometry();
              break;
        case 41:
              // waiting for ground control to change for new meaning. Meanwhile, we convert it here.
              // distPerRotLeftChainTolerance = (1 + (float(self.config.get('Advanced Settings', 'leftChainTolerance')) / 100)) * float(self.config.get('Advanced Settings', 'gearTeeth')) * float(self.config.get('Advanced Settings', 'chainPitch'))
              sysSettings.rightChainLengthCorrection = (value / sysSettings.lRDistPerRot);
              kinematics.recomputeGeometry();
              break;
        case 42:
              sysSettings.positionErrorLimit = value;
              break;
        case 43:
              sysSettings.topBeamTilt = value;
              kinematics.recomputeGeometry();
              break;
        case 44:
              sysSettings.maxTopBeamTipFlexAndTwist = value;
              kinematics.recomputeGeometry();
              break;
        case 45:
              sysSettings.chainElongationFactor = value;
              kinematics.recomputeGeometry();
              break;
        case 46:
              sysSettings.sledWeight = value;
              kinematics.recomputeGeometry();
              break;
        default:
              return(STATUS_INVALID_STATEMENT);
    }
    settingsSaveToEEprom();
    return(STATUS_OK);
}
