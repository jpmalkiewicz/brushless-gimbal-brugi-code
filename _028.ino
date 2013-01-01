/*
Brushless Gimbal Controller by Ludwig Färber and Alexander Rehfeld 

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version. see <http://www.gnu.org/licenses/>

Thanks To : 

-Miniolli 
-brettbeauregard
-jrowberg
-rgsteele

*/

byte VersionStatus = 'B'; // B = Beta , N = Normal Release
byte VersionNumber = 001; 

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"
#include <EEPROM.h>

/* MPU 6050 DMP */

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int angle[3];
int rangle[3]; 
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
   
/* PID 1 */
unsigned long lastTime_pid1;
int Input_pid1, Output_pid1, Setpoint_pid1;
float ITerm_pid1, lastInput_pid1;
float kp_pid1, ki_pid1, kd_pid1;
int SampleTime_pid1 = 1000; //1 sec
float outMin_pid1, outMax_pid1;
bool inAuto_pid1 = false;
byte kp1 = 0,ki1 = 0,kd1 = 0;
float kp1rf = 0,ki1rf = 0,kd1rf = 0,kp2rf = 0,ki2rf = 0,kd2rf = 0;

/* PID 2 */
unsigned long lastTime_pid2;
int Input_pid2, Output_pid2, Setpoint_pid2;
float ITerm_pid2, lastInput_pid2;
float kp_pid2, ki_pid2, kd_pid2;
int SampleTime_pid2 = 1000; //1 sec
float outMin_pid2, outMax_pid2;
bool inAuto_pid2 = false;
byte kp2 = 0,ki2 = 0,kd2 = 0;
float kp1r = 0,ki1r = 0,kd1r = 0,kp2r = 0,ki2r = 0,kd2r = 0; 
   
/* BL Controller */   
const int SinusValues[256]={0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,3,3,3,4,4,5,5,6,6,6,7,7,8,9,9,10,10,11,12,12,13,14,14,15,16,17,17,18,19,20,21,22,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42,43,44,46,47,48,49,50,52,53,54,56,57,58,59,61,62,64,65,66,68,69,70,72,73,75,76,77,79,80,82,83,85,86,88,89,91,92,94,95,97,98,100,101,103,104,106,108,109,111,112,114,115,117,118,120,122,123,125,126,128,129,131,132,134,136,137,139,140,142,143,145,146,148,150,151,153,154,156,157,159,160,162,163,165,166,168,169,171,172,174,175,177,178,179,181,182,184,185,186,188,189,191,192,193,195,196,197,198,200,201,202,204,205,206,207,208,210,211,212,213,214,215,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,232,233,234,235,236,237,237,238,239,240,240,241,242,242,243,244,244,245,245,246,247,247,248,248,248,249,249,250,250,251,251,251,252,252,252,252,253,253,253,253,253,254,254,254,254,254,254,254,254};

// Motor One
static uint32_t position_a_one = 0;
static uint32_t position_b_one = 170;
static uint32_t position_c_one = 170;

int direction_a_one = 1;
int direction_b_one = 1;
int direction_c_one = -1;
int direction_one = Direction_Motor_Nick;

// Motor Two
static uint32_t position_a_two = 0;
static uint32_t position_b_two = 170;
static uint32_t position_c_two = 170;

int direction_a_two = 1;
int direction_b_two = 1;
int direction_c_two = -1;
int direction_two = Direction_Motor_Roll;

// MPU 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    
    Wire.begin();
    
    Serial.begin(115200);

    initBlController();
    Init_pid1();
    Init_pid2();
    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {/* ERROR */}

}

void loop() {

    if (!dmpReady) return; // Only Start the Programm if MPU are Ready ! 
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // Calculate Angle in 1/100° 
            angle[0] = (ypr[0] * 180/M_PI) * 100;
            angle[1] = (ypr[1] * 180/M_PI) * 100;
            angle[2] = (ypr[2] * 180/M_PI) * 100;
            
            // Calculate Angle in °
            rangle[0] = angle[0] / 100; 
            rangle[1] = angle[1] / 100;
            rangle[2] = angle[2] / 100;
    }
    
    Setpoint_pid1 = 0; 
    Input_pid1 = rangle[2];
    
    Setpoint_pid2 = 0; 
    Input_pid2 = rangle[1];
    
    Compute_pid1();
    Compute_pid2();
    
    if (Output_pid1 < 0) { makestep_motortwo(-1, abs(Output_pid1));} else {makestep_motortwo(+1, abs(Output_pid1));}
    if (Output_pid2 < 0) { makestep_motorone(-1, abs(Output_pid2));} else {makestep_motorone(+1, abs(Output_pid2));}
    
    serial();  
    
}


