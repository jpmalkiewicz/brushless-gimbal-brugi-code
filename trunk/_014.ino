/*
 Thanks to : 
 - MultiWiiCopter by Alexandre Dubus
 - Brett Beauregard
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "define.h"

const int SinusValues[256]={0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,3,3,3,4,4,5,5,6,6,6,7,7,8,9,9,10,10,11,12,12,13,14,14,15,16,17,17,18,19,20,21,22,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42,43,44,46,47,48,49,50,52,53,54,56,57,58,59,61,62,64,65,66,68,69,70,72,73,75,76,77,79,80,82,83,85,86,88,89,91,92,94,95,97,98,100,101,103,104,106,108,109,111,112,114,115,117,118,120,122,123,125,126,128,129,131,132,134,136,137,139,140,142,143,145,146,148,150,151,153,154,156,157,159,160,162,163,165,166,168,169,171,172,174,175,177,178,179,181,182,184,185,186,188,189,191,192,193,195,196,197,198,200,201,202,204,205,206,207,208,210,211,212,213,214,215,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,232,233,234,235,236,237,237,238,239,240,240,241,242,242,243,244,244,245,245,246,247,247,248,248,248,249,249,250,250,251,251,251,252,252,252,252,253,253,253,253,253,254,254,254,254,254,254,254,254};

// Motor One
static uint32_t position_a_one = 0;
static uint32_t position_b_one = 170;
static uint32_t position_c_one = 170;

int direction_a_one = 1;
int direction_b_one = 1;
int direction_c_one = -1;
int direction_one = 1;

// Motor Two
static uint32_t position_a_two = 0;
static uint32_t position_b_two = 170;
static uint32_t position_c_two = 170;

int direction_a_two = 1;
int direction_b_two = 1;
int direction_c_two = -1;
int direction_two = 1;


static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     
static int16_t  gyroADC[3],accADC[3],accSmooth[3];
static int16_t  annex650_overrun_count = 0;
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static uint16_t acc_1G;             
static int16_t  acc_25deg;
static int16_t angle[2] = {0,0};  
static int16_t databus_errors_count = 0;
static uint16_t calibrating_Gyro;
static uint16_t calibrating_Acc = 0;  
static uint16_t Calibrate_accZero[3];

/* PID 1 */
unsigned long lastTime_pid1;
int Input_pid1, Output_pid1, Setpoint_pid1;
float ITerm_pid1, lastInput_pid1;
float kp_pid1, ki_pid1, kd_pid1;
int SampleTime_pid1 = 1000; //1 sec
float outMin_pid1, outMax_pid1;
bool inAuto_pid1 = false;

/* PID 1 */
unsigned long lastTime_pid2;
int Input_pid2, Output_pid2, Setpoint_pid2;
float ITerm_pid2, lastInput_pid2;
float kp_pid2, ki_pid2, kd_pid2;
int SampleTime_pid2 = 1000; //1 sec
float outMin_pid2, outMax_pid2;
bool inAuto_pid2 = false;
 
 
float P_Poti = 0; 
float I_Poti = 0; 
float D_Poti = 0; 

boolean button = false; 

int offset_gyro = 0; 

void annexCode() {  
  
}

void setup() {
  
  I2C_PULLUPS_DISABLE
  initSensors();
  calibrating_Gyro = 400;
 
  pinMode(A1, INPUT); //D 
  pinMode(A2, INPUT); //I
  pinMode(A3, INPUT); //P
  pinMode(12, INPUT); // Button
  
  initBlController();
  previousTime = micros() / micros_divider; // micros_divider = 64 , to get the real micros funktion back , because Timer0 is now speed up
    
  Init_pid1();
  Init_pid2();
    
  Serial.begin(115200); // for the moment , the Arduino Serial funktion is OK for us ;) 
  
  offset_gyro = angle[0]; 
  
}

void loop() {
    
    computeIMU();
    
    /*
    Setpoint = 0; 
    Input = angle[0] /10; 
    Compute(); 
    */
    
    /*
    if (Output != 0) {
    if (Output < 0) {makestep_motorone(-1,abs(Output / 5)); } else { makestep_motorone(1,abs(Output / 5)); }
    }
    */
    
    Setpoint_pid1 = 0; 
    Input_pid1 = angle[0] + offset_gyro;
    Compute_pid1();
    
    Setpoint_pid2 = 0; 
    Input_pid2 = angle[0];
    Compute_pid2();
     
     /*
    if (digitalRead(12) == 0 && button == false) {   
    button = true; 
    
    P_Poti = map(analogRead(A3),1023,0,0,500); 
    I_Poti = map(analogRead(A2),1023,0,0,500); 
    D_Poti = map(analogRead(A1),1023,0,0,500); 
    
    P_Poti = P_Poti / 100; 
    I_Poti = I_Poti / 100; 
    D_Poti = D_Poti / 100; 
    
    SetTunings_pid1(P_Poti,D_Poti,I_Poti);
      
    Serial.print(P_Poti);
    Serial.print("   ");
    
    Serial.print(I_Poti);
    Serial.print("   ");
    
    Serial.print(D_Poti);
    Serial.println("");
    } 
    */
    
    if (Output_pid1 != 0) {
    if (Output_pid1 < 0) {makestep_motortwo(-1,abs(Output_pid1)); } else { makestep_motortwo(1,abs(Output_pid1)); }
    }
    
    if (Output_pid2 != 0) {
    if (Output_pid2 < 0) {makestep_motorone(-1,abs(Output_pid2)); } else { makestep_motorone(1,abs(Output_pid2)); }
    }
    
    if (digitalRead(12) == 1) { button = false; }
    
    
    currentTime = micros() / micros_divider; 
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;
   
    Serial.println(angle[0]);
}
