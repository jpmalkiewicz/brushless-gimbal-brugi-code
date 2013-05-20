
/*
Brushless Gimbal Controller Software by Christian Winkler and Alois Hahn (C) 2013

Brushless Gimbal Controller Hardware and Software support 
by Ludwig Fäerber, Alexander Rehfeld and Martin Eckart

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


// FOR CHANGES PLEASE READ: ReleaseHistory.txt

// Serial Programming for Settings!!!
/* HOWTO:
- edit setDefaultParameters() in variables.h if you want to.
- Upload Firmware.
- Open Arduino Terminal and enable NL in the lower right corner of the window.
- Type in HE 
-... enjoy
*/


#define VERSION_STATUS A // A = Alpha; B = Beta , N = Normal Release
#define VERSION 49


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
  LEDPIN_PINMODE
  
  CH2_PINMODE
  CH3_PINMODE
    
  // Start Serial Port
  Serial.begin(115200);

  // Set Serial Protocol Commands
  setSerialProtocol();
  
  // Read Config or fill with default settings
  if(EEPROM.read(0)==VERSION)
  {
    EEPROM_readAnything(0, config);
  }
  else
  {
    setDefaultParameters();
    EEPROM_writeAnything(0, config);
  }
  
  // Init Sinus Arrays and Motor Stuff 
  recalcMotorStuff();
  
  // Init PIDs to reduce floating point operations.
  initPIDs();
  
  // Init RC-Input
  initRCPins();
  
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 
   // Init BL Controller
  initBlController();
  // Initialize MPU 
  initResolutionDevider();
  
  // Init IMU varibles
  initIMU();
  
  
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
  
  // set sensor orientation (from config)
  initSensorOrientation();
  
  // Init MPU Stuff
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G
  mpu.setDLPFMode(MPU6050_DLPF_BW);                     // Set Gyro Low Pass Filter to config.h
  mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  mpu.setSleepEnabled(false); 
  
  // Gyro Offset calibration
  gyroOffsetCalibration();
  Serial.println(F("Gyro calibration: done"));
  
  LEDPIN_ON
  
   // Init BL Controller
  initBlController();
  // motorTest();
  
 // Initialize timer
  timer=micros();

  enableMotorUpdates = true;
  Serial.println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));

  CH2_OFF
  CH3_OFF
 
}

/************************/
/* PID Controller       */
/************************/
int32_t ComputePID(int32_t DTms, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int16_t Kp, int16_t Ki, int32_t Kd)
{
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTms;
  *errorOld = error;

  return out / 4096;
}



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
  int32_t pitchAngleSet;
  int32_t rollAngleSet;
  
  //actual perfomance
  //  loop time = wc 1600us (OAC off)
  //  loop time = wc 2000us (OAC on)
  
  if (motorUpdate) // loop runs with motor ISR update rate (1000Hz)
  {
    motorUpdate = false;
    CH2_ON
    // Evaluate RC-Signals
    // 22us
    if(config.rcAbsolute==1) {
      evaluateRCSignalAbsolute();  // Gives rollRCSetPoint, pitchRCSetpoint
    } else {
      evaluateRCSignalProportional(); // Gives rollRCSpeed, pitchRCSpeed
    }
    
    // update IMU data            
    readGyros();
    
    if (config.enableGyro) updateGyroAttitude();
    if (config.enableACC) updateACCAttitude(); 

    getAttiduteAngles();
   
    //****************************
    // pitch PID
    //****************************
    pitchAngleSet = PitchPhiSet * 100;
    
    pitchPIDVal = ComputePID(DT_INT_MS, angle[PITCH], pitchAngleSet, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
    // motor control
    pitchMotorDrive = pitchPIDVal * config.dirMotorPitch;

    //****************************
    // roll PID
    //****************************
    rollAngleSet = RollPhiSet * 100;
    
    rollPIDVal = ComputePID(DT_INT_MS, angle[ROLL], rollAngleSet, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);

    // motor control
    rollMotorDrive = rollPIDVal * config.dirMotorRoll;
 
    //****************************
    // slow rate actions
    //****************************
    switch (count) {
    case 1:
      readACC(ROLL); break;
    case 2:
      readACC(PITCH); break;
    case 3:
      readACC(YAW); break;
    case 4:
      updateACC(); break;
    case 5:
      break;
    case 7:
      if (validRCPitch) {
        if(config.rcAbsolute==1) {
          PitchPhiSet = PitchPhiSet*0.95 + pitchRCSetpoint*0.05;
        }
        else {
          if(abs(pitchRCSpeed)>0.01) {
            PitchPhiSet += pitchRCSpeed;
          }
        }
      } else {
        PitchPhiSet = 0;
      }
      break;
    case 8:
      if (validRCRoll){
        if(config.rcAbsolute==1){
          RollPhiSet = RollPhiSet*0.95 + rollRCSetpoint*0.05;
        } else {
          if(abs(rollRCSpeed)>0.01) {
            RollPhiSet += rollRCSpeed;
          }
        }
      } else {
        RollPhiSet = 0;
      }
      break;
    case 9:
      // 600 us
      if(config.accOutput==1){ Serial.print(angle[PITCH]); Serial.print(" ACC ");Serial.println(angle[ROLL]);}     
      //if(config.accOutput==1){ Serial.print(pitchAngleSet); Serial.print(" ACC ");Serial.println(rollAngleSet);}     
      //if(config.accOutput==1){ Serial.print(accMag); Serial.print(" ACC ");Serial.println(angle[ROLL]);}     
      // 1360 us
      //if(config.accOutput==1){ Serial.print((float)(angle[PITCH]/100.0),2); Serial.print(" ACC ");Serial.println((float)(angle[ROLL]/100.0),2);}     
      // 490 us
      //if(config.accOutput==1){ Serial.print(11); Serial.print(" ACC ");Serial.println(12);}     
      break;
    case 10:
      count=0;
      break;
    default:
      break;
    }
    count++;
       
    //****************************
    // check PPM timeouts
    //****************************
    checkPWMRollTimeout();
    checkPWMPitchTimeout();

    //****************************
    // Evaluate Serial inputs 
    //****************************
    sCmd.readSerial();

    CH2_OFF    

  }

}


