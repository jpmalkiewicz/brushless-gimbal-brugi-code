#include <limits.h>

/*************************/
/* MPU6050 Routines      */
/*************************/
/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 */
/*
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
*/

//#define GRAVITY 16384.0f
#define GRAVITY 15500.0f

void initResolutionDevider()
{
    if(MPU6050_GYRO_FS == 0x00) resolutionDevider = 131.0;
    if(MPU6050_GYRO_FS == 0x01) resolutionDevider = 65.5;
    if(MPU6050_GYRO_FS == 0x02) resolutionDevider = 32.8;
    if(MPU6050_GYRO_FS == 0x03) resolutionDevider = 16.4;
}


// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds
void gyroOffsetCalibration()
{
  int i;
  #define TOL 64
  #define GYRO_INTERATIONS 4000
  int16_t prevGyro[3],gyro[3];
  float fp_gyroOffset[3];
  uint8_t tiltDetected = 0;
  int calibGCounter = GYRO_INTERATIONS;
  
  // set to slow mode during calibration
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  
  // wait 2 seconds
  delayT1(2000);
  
  while(calibGCounter>0)
  {

    if(calibGCounter==GYRO_INTERATIONS)
    {
      delayT1(700);
      mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]); 
      for (i=0; i<3; i++) {
        fp_gyroOffset[i] = 0;
        prevGyro[i]=gyro[i];
      }
    }
    
    mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);  

    for (i=0; i<3; i++) {
      if(abs(prevGyro[i] - gyro[i]) > TOL) {
        tiltDetected++;
        //Serial.print(F(" i="));Serial.print(i);
        //Serial.print(F(" calibGCounter="));Serial.print(calibGCounter);
        //Serial.print(F(" diff="));Serial.print(prevGyro[i] - gyro[i]);
        //Serial.print(F(" gyroi="));Serial.print(gyro[i]);
        //Serial.print(F(" prevgyroi="));Serial.println(prevGyro[i]);
        break;
      }
    } 
     
    for (i=0; i<3; i++) {
        fp_gyroOffset[i] += (float)gyro[i]/GYRO_INTERATIONS;
        prevGyro[i]=gyro[i];
    }
      
    calibGCounter--;
    if(tiltDetected>=1)
    {
      Serial.println(F("Motion detected during Gyro calibration. Starting over!"));
      calibGCounter=GYRO_INTERATIONS;
      tiltDetected=0;
    }
  }

  // put result into integer
  config.gyrOffsetX = fp_gyroOffset[0];
  config.gyrOffsetY = fp_gyroOffset[1];
  config.gyrOffsetZ = fp_gyroOffset[2];

  // restore MPU mode
  initMPU();

}




//***********************************************************
//  ACC calibration
//***********************************************************
//  compensate for zero point offset
//  run acc compensation at least for two directions.
//  e.g.
//      1st run: 90 deg vertical position (pitch down)
//      2nd run   0 deg horizontal position
//
#define ACC_ITERATIONS 500
#define ACC_THRESH_FAIL 1000
#define ACC_THRESH_GMIN 3000
char accCalibration() {
  
  int16_t devVal[3];
  int16_t minAcc[3] = {INT_MAX, INT_MAX, INT_MAX};
  int16_t maxAcc[3] = {INT_MIN, INT_MIN, INT_MIN};
  
  float fp_accOffset[3] = {0,};

  // wait 0.5 seconds
  delayT1(500);

  // read acc values, determine average/min/max
  for (int i=0; i<ACC_ITERATIONS; i++) {
    mpu.getAcceleration(
      &devVal[0],
      &devVal[1],
      &devVal[2]
      );
    for (char j=0; j<3; j++) {
      fp_accOffset[j] += (float)devVal[j]/ACC_ITERATIONS;
      if (devVal[j] > maxAcc[j]) {
        maxAcc[j] = devVal[j];
      }
      if (devVal[j] < minAcc[j]) {
        minAcc[j] = devVal[j];
      }
    }
    delayT1(2);  // 2ms
  }

#if 0 
  for (char j=0; j<3; j++) {
    Serial.print(F("avg/max/min["));
    Serial.print((int)j);
    Serial.print(F("] "));
    Serial.print(fp_accOffset[j], 3);
    Serial.print(F(" / "));
    Serial.print(maxAcc[j]);
    Serial.print(F(" / "));
    Serial.print(minAcc[j]);
    Serial.println("");
  }
#endif
  
  // plausibility check
  for (char j=0; j<3; j++) {
    if ((maxAcc[j] - minAcc[j]) > ACC_THRESH_FAIL) {
      return -1; // failed
    }
  }

  // store calibration values
  if (abs(fp_accOffset[0]) < ACC_THRESH_GMIN) {
     config.accOffsetX = fp_accOffset[0];
  }
  if (abs(fp_accOffset[1]) < ACC_THRESH_GMIN) {
     config.accOffsetY = fp_accOffset[1];
  }
  if (abs(fp_accOffset[2]) < ACC_THRESH_GMIN) {
     config.accOffsetZ = fp_accOffset[2];
  }
  
  return 0;
  
}

       
