
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


#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 48


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
  motorTest();
  
 // Initialize timer
  timer=micros();

  enableMotorUpdates = true;
  Serial.println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
void loop() 
{ 
  if((micros()-timer)/CC_FACTOR>=1000) // Fixed loop length at 1000Hz
  {
    // Serial.println((micros()-timer)/CC_FACTOR);
    timer = micros();
   
    // Evaluate RC-Signal
    if(config.rcAbsolute==1)
      evaluateRCSignalAbsolute();  // Gives rollRCSetPoint, pitchRCSetpoint
    else
      evaluateRCSignalProportional(); // Gives rollRCSpeed, pitchRCSpeed
      
      // Update raw Gyro, switch axes for roll gyro to allow for 90 deg downward view 
    updateRawGyroDataDecoupled(&gyroRoll,&gyroPitch,pitchAngleACC);

    // Get Angles to compensate Drift from raw ACC, also included: I-Term
    getOrientationAndDriftCompensation();

    // Controller Loops
    // Calculation of pitchAnglePID and rollAnglePID MOVED TO getOrientationAndDriftCompensation();, see Orientation Routines.h

    // gyro PID, just P
    pitchPID = pitchPIDpar.Kp * gyroPitch;
    rollPID = rollPIDpar.Kp * gyroRoll;

    // gyro PID, just D, will be processed further in ISR, see BLcontroller.h
    pitchGyroDamp = pitchPIDpar.Kd * gyroPitch;
    rollGyroDamp = rollPIDpar.Kd * gyroRoll;

    // avoid drift offset and reduce noise sensitivity, avoid highfreq oscillations
    pitchPID = (abs(pitchPID) < 1.0) ? 0.0 : pitchPID; // 1.0 ist just a good guess
    rollPID = (abs(rollPID) < 1.0) ? 0.0 : rollPID;

    // add PIDs and set motor speed
    pitchGyroDamp *= config.dirMotorPitch;
    pitchPIDVal = pitchPID + pitchAnglePID + pitchRCSpeed * config.rcGain * 200;
    pitchPIDVal = pitchPIDVal * config.dirMotorPitch;
    cli();
    pitchMotorDamp = pitchGyroDamp;
    pitchMotorSpeed  = pitchPIDVal;
    sei();
  
    rollGyroDamp *= config.dirMotorRoll;
    rollPIDVal = rollPID + rollAnglePID + rollRCSpeed * config.rcGain * 200;
    rollPIDVal = rollPIDVal * config.dirMotorRoll;
    cli();
    rollMotorDamp  = rollGyroDamp;
    rollMotorSpeed  = rollPIDVal;
    sei();
  
    // Evaluate Serial inputs 
    sCmd.readSerial(); 
    #if 0
      Serial.println((micros()-timer)/CC_FACTOR);
    #endif
  }
}


