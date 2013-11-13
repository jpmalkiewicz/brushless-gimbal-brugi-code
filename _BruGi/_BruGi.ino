
/*
Brushless Gimbal Controller Software by Christian Winkler and Alois Hahn (C) 2013

Brushless Gimbal Controller Hardware and Software support 
by Ludwig Fäerber, Alexander Rehfeld and Martin Eckart

Special Contributions:
  Michael Schätzel

Project homepage: http://brushlessgimbal.de/
Discussions:
http://fpv-community.de/showthread.php?20795-Brushless-Gimbal-Controller-SOFTWARE
http://fpv-community.de/showthread.php?22617-Gimbal-Brushless-Controller-V3-0-50x50mm-by-Martinez
http://fpv-community.de/showthread.php?19252-Brushless-Gimbal-Controller

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version. see <http://www.gnu.org/licenses/>

Anyhow, if you start to commercialize our work, please read on http://code.google.com/p/brushless-gimbal/ on how to contribute

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
*/

#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 49
#define REVISION "r185"
#define VERSION_EEPROM 10 // change this number when eeprom data structure has changed


/*************************/
/* Include Header Files  */
/*************************/
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "definitions.h"
#include "MPU6050.h"
#include "SerialCommand.h"
#include "EEPROMAnything.h"
#include "PinChangeInt.h"
#include "Timer1.h"
#include "Trace.h"
#include "variables.h"
MPU6050 mpu;            // Create MPU object
SerialCommand sCmd;     // Create SerialCommand object

#include "fastMathRoutines.h"     // fast Math functions required by orientationRoutines.h
#include "orientationRoutines.h"  // get Orientation from ACC
#include "RCdecode.h"             // RC Decoder to move camera by servo signal input
#include "BLcontroller.h"         // Motor Movement Functions and Timer Config
#include "SerialCom.h"            // Serial Protocol for Configuration and Communication


/**********************************************/
/* Initialization                             */
/**********************************************/
void setup() 
{

  // just for debugging
#ifdef STACKHEAPCHECK_ENABLE
  stackCheck();
  heapCheck();
#endif

  LEDPIN_PINMODE
  
  CH2_PINMODE
  CH3_PINMODE
  
  // Init BL Controller
  initBlController();
  // Init Sinus Arrays
  initMotorStuff();
  
  // switch off PWM Power
  MoveMotorPosSpeed(config.motorNumberPitch, 0, 0); 
  MoveMotorPosSpeed(config.motorNumberRoll, 0, 0);
  
  // Start Serial Port
  Serial.begin(115200);

  // Set Serial Protocol Commands
  setSerialProtocol();
  
  // Read Config, initialize if versions do not match or CRC fails
  readEEPROM();
  if ((config.vers != VERSION) || (config.versEEPROM != VERSION_EEPROM))
  {
    Serial.print(F("EEPROM version mismatch, initialize EEPROM"));
    setDefaultParameters();
    writeEEPROM();
  }
   
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 
  // Initialize MPU 
  initResolutionDevider();
    
  // Auto detect MPU address
  mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
  mpu.initialize();
  if(mpu.testConnection()) {
    Serial.println(F("MPU6050 ok (HIGH)"));  
  } else {
    mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
    mpu.initialize();
    if(mpu.testConnection()) {
      Serial.println(F("MPU6050 ok (LOW)"));  
    } else {
      Serial.println(F("MPU6050 falied"));  
    }
  }

  // Init MPU mode
  initMPU();
  
  // Gyro Offset calibration
  Serial.println(F("Gyro calibration: do not move"));
  gyroOffsetCalibration();
  Serial.println(F("Gyro calibration: done"));
 
  // Init IMU variables
  initIMU();
  // set sensor orientation
  initSensorOrientation();
  
  // Init PIDs parameters
  initPIDs();

  // init RC variables
  initRC();

  // Init RC-Input
  initRCPins();
  
  Serial.println(F("BruGi ready."));

  LEDPIN_OFF
  CH2_OFF
  CH3_OFF

}

/************************/
/* PID Controller       */
/************************/
// PID integer inplementation
//   DTms  ... sample period (ms)
//   DTinv ... sample frequency (Hz), inverse of DT (just to avoid division)
inline int32_t ComputePID(int32_t DTms, int32_t DTinv, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd)
{
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTinv;
  *errorOld = error;

  out = out / 4096;
  
  return out;
  
}


/*****************************************************************

main loop execution time budget


exected each iteration (main tick)
  time(us)   function
  --------------------------------
   330        readGyros
   175        updateGyroAttitude
    21        updateACCAttitude
   372        getAttiduteAngles
    92        pid pitch
    92        pid roll
     6        motor update
    84        RC low pass
  --------------------------------
  1172        sum
  1250        measured, real, no RC

executed each 10th iteration (sub tick)
  time(us)   function
  --------------------------------
   330        readACC
   120        updateACC
   210        voltage compensation
    56        gimbal state
    26        RC roll
    26        RC pitch
   286        evaluate RC
  --------------------------------
   330        maximum of all
   --        measured, real, no RC
  --------------------------------


total
  time(us)    function
  --------------------------------
   1172       main tick execution time
    330       sub tick maximum
  --------------------------------
   1502       sum 
  ================================


motor ISR duration
  version r161
  10 us every 31 us
  18 us every 2000 us
  
  --> improved
   6.3 us every 31 us
   
RC interrupt duration
   43 us, every 1000 ms (one RC channel)
   54 us, every 1000 ms, worst case, two RC channel

*******************************************************************/


/**********************************************/
/* Main Loop                                  */
/**********************************************/
void loop() 
{ 
  int32_t pitchPIDVal;
  int32_t rollPIDVal;
  static int32_t pitchErrorSum;
  static int32_t rollErrorSum;
  static int32_t pitchErrorOld;
  static int32_t rollErrorOld;
  
  static char pOutCnt = 0;
  static char tOutCnt = 0;
  static char tOutCntSub = 0;
  static int stateCount = 0;
  static uint8_t ledBlinkCnt = 0;
  static uint8_t ledBlinkOnTime = 10;
  static uint8_t ledBlinkPeriod = 20;

  if (motorUpdate) // loop runs with motor ISR update rate (500 Hz)
  {
   
    motorUpdate = false;

    CH2_ON
    
    // loop period
    //     2.053/2.035 ms max/min, error = +5/-13 us (w/o rc)
    //     2.098/2.003 ms max/min, error = +50/-45 us (1 x PPM16 1 x PWM)
    
    // update IMU data            
    readGyros();   // td = 330us

    if (config.enableGyro) updateGyroAttitude(); // td = 176 us
    if (config.enableACC) updateACCAttitude(); // td = 21 us
    getAttiduteAngles(); // td = 372 us
   
    //****************************
    // pitch PID
    //****************************
    // td = 92 us
    pitchPIDVal = ComputePID(DT_INT_MS, DT_INT_INV, angle[PITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
    // motor control
    pitchMotorDrive = pitchPIDVal * config.dirMotorPitch;
 
    //****************************
    // roll PID
    //****************************
    // td = 92 us
    rollPIDVal = ComputePID(DT_INT_MS, DT_INT_INV, angle[ROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);
    // motor control
    rollMotorDrive = rollPIDVal * config.dirMotorRoll;

    // motor update t=6us (*)
    if (enableMotorUpdates)
    {
      // set pitch motor pwm
      MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, maxPWMmotorPitchScaled); 
      // set roll motor pwm
      MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, maxPWMmotorRollScaled);
    }

    // Evaluate RC-Signals, td = 84 us
    if (fpvModePitch) {
      utilLP_float(&pitchAngleSet, PitchPhiSet, rcLPFPitchFpv_tc);
    } else if(config.rcAbsolutePitch==1) {
      utilLP_float(&pitchAngleSet, PitchPhiSet, rcLPFPitch_tc); // t=16us
    } else {
      utilLP_float(&pitchAngleSet, PitchPhiSet, 0.01);
    }
    if (fpvModeRoll) {
      utilLP_float(&rollAngleSet, RollPhiSet, rcLPFRollFpv_tc);
    } else if(config.rcAbsoluteRoll==1) {
      utilLP_float(&rollAngleSet, RollPhiSet, rcLPFRoll_tc); // t=28us
    } else {
      utilLP_float(&rollAngleSet, RollPhiSet, 0.01);
    }
    
    // tElapsed = 1.250 ms

    //****************************
    // slow rate actions
    //****************************
    switch (count) {
    case 1:
      readACCs(); // td = 330us
      break;
    case 2:
      updateACC(); // td = 120us
      break;
    case 3:
      // td = 210us, total
      voltageCompensation();
      break;
    case 4:
      
      // gimbal state transitions, td=56us
      switch (gimState)
      {
        case GIM_IDLE :
          // wait 2 sec to settle ACC, before PID controlerbecomes active 
          stateCount++;
          if (stateCount >= LOOPUPDATE_FREQ/10*1) // 1 sec 
          {
            gimState = GIM_UNLOCKED;
            stateCount = 0;
          }
          break;
        case GIM_UNLOCKED :
          // allow PID controller to settle on ACC position
          stateCount++;
          if (stateCount >= LOOPUPDATE_FREQ/10*LOCK_TIME_SEC) 
          {
            gimState = GIM_LOCKED;
            stateCount = 0;
          }
          break;
        case GIM_LOCKED :
          // normal operation
          break;
      }
      // gimbal state actions 
      switch (gimState) {
        case GIM_IDLE : // allow settling IMU
          enableMotorUpdates = false;
          setACCtc(0.2);
          disableAccGtest = true;
          break;
        case GIM_UNLOCKED : // fast settling of desired position
          enableMotorUpdates = true;
          disableAccGtest = true;
          setACCtc(2.0);
          disableAccGtest = true;
          break;
        case GIM_LOCKED : // normal operation
          enableMotorUpdates = true;
          disableAccGtest = false;
          if (altModeAccTime) { // alternate time constant mode switch
            setACCtc(config.accTimeConstant2);
          } else {
            setACCtc(config.accTimeConstant);
          }
          break;
      }
      // handle mode switches
      decodeModeSwitches();  // td = 4 us
      
      // lpf avoids jerking during offset config
      updateLPFangleOffset(); // td = 65 us
           
      break;
    case 5:
      // td = 26 us, total
      // RC Pitch function
      if (fpvModePitch) {
        if (rcData[RC_DATA_FPV_PITCH].valid) {
          PitchPhiSet = rcData[RC_DATA_FPV_PITCH].setpoint;
        } else {
          PitchPhiSet = 0;
        } 
      } else if (rcData[RC_DATA_PITCH].valid) {
        if(config.rcAbsolutePitch==1) {
          PitchPhiSet = rcData[RC_DATA_PITCH].setpoint;
        } else {
          if (abs(rcData[RC_DATA_PITCH].rcSpeed)>0.01) {
            PitchPhiSet += rcData[RC_DATA_PITCH].rcSpeed * 0.01;
          }
        } 
      } else {
        PitchPhiSet = 0;
      }
      if (config.minRCPitch < config.maxRCPitch) {
        PitchPhiSet = constrain(PitchPhiSet, config.minRCPitch, config.maxRCPitch);
      } else {
        PitchPhiSet = constrain(PitchPhiSet, config.maxRCPitch, config.minRCPitch);
      }
      break;
    case 6:
      // td = 26us, total
      // RC roll function
      if (fpvModeRoll) {
        if (rcData[RC_DATA_FPV_ROLL].valid) {
          RollPhiSet = rcData[RC_DATA_FPV_ROLL].setpoint;
        } else {
          RollPhiSet = 0;
        } 
      } else if (rcData[RC_DATA_ROLL].valid){
        if(config.rcAbsoluteRoll==1){
          RollPhiSet = rcData[RC_DATA_ROLL].setpoint;
        } else {
          if(abs(rcData[RC_DATA_ROLL].rcSpeed)>0.01) {
            RollPhiSet += rcData[RC_DATA_ROLL].rcSpeed * 0.01;
          }
        }
      } else {
        RollPhiSet = 0;
      }
      if (config.minRCRoll < config.maxRCRoll) {
        RollPhiSet = constrain(RollPhiSet, config.minRCRoll, config.maxRCRoll);
      } else {
        RollPhiSet = constrain(RollPhiSet, config.maxRCRoll, config.minRCRoll);
      }
      break;
    case 7:
      // evaluate RC-Signals. td = 286 us
      evaluateRCPitch();
      evaluateRCRoll();
      evaluateRCAux();
      
      // check RC channel timeouts
      checkRcTimeouts();  // td = 15 us

      break;
    case 8:
      // unused slot
      break;
    case 9:
      // unused slot
      break;
    case 10:    
      // regular ACC output
      pOutCnt++;
      if (pOutCnt == (LOOPUPDATE_FREQ/10/POUT_FREQ))
      {
        if (config.fTrace != TRC_OFF) {
          printTrace(config.fTrace);
        }
        pOutCnt = 0;
      }
      
      // print regular trace output
      tOutCnt++;
      if (tOutCnt == (LOOPUPDATE_FREQ/10/TRACE_OUT_FREQ))
      {
        tOutCntSub++;
        if (tOutCntSub > STRACE_IDX) {
          tOutCntSub = 1;
        }

        if (config.sTrace == TRC_ALL) {
            // cycle all trace types
            printTrace((traceModeType)tOutCntSub);      
        } else if (config.sTrace != TRC_OFF) {
            // use specific trace type
            printTrace(config.sTrace);
        }
        
        tOutCnt = 0;
      }

      ledBlinkCnt++;
      if (ledBlinkCnt <= ledBlinkOnTime) {
          LEDPIN_ON
      } else if (ledBlinkCnt <= ledBlinkPeriod) {
          LEDPIN_OFF
      } else {
        ledBlinkCnt = 0;
      }
        
#ifdef STACKHEAPCHECK_ENABLE
      stackHeapEval(false);
#endif
      count=0;
      break;
    default:
      break;
    }
    count++;
       
    //****************************
    // Evaluate Serial inputs 
    //****************************
    sCmd.readSerial();

    // worst-case finalize after
    //    1.67 ms (w/o RC)
    //    1.76 ms (with 1 x PPM)
    //    1.9 ms (with 2 RC channels + 1 x PPM )

    CH2_OFF
  }

}


