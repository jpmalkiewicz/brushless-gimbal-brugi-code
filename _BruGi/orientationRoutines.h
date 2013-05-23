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
  #define TOL 256
  int16_t prevGyro[3],gyro[3];
  float fp_gyroOffset[3];
  uint8_t tiltDetected = 0;
  int calibGCounter = 2000;
  
  // Set voltage on all motor phases to zero 
  enableMotorUpdates = false;
  
  // TODO: doublecheck delay values .... 5s sec ?
  // allow motor to settle
  delay(5000);
    
  while(calibGCounter>0)
  {
    OCR2A = 0; OCR2B = 0; OCR1A = 0; OCR1B = 0; OCR0A = 0; OCR0B = 0; 
    mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);  
    if(calibGCounter==2000)
    {
      for (char i=0; i<3; i++) {
        fp_gyroOffset[i] = 0;
        prevGyro[i]=gyro[i];
      }
    }
    
    if(calibGCounter % 10 == 0)
    { 
      if((abs(prevGyro[0]-gyro[0])>TOL)||(abs(prevGyro[1]-gyro[1])>TOL)||(abs(prevGyro[2]-gyro[2])>TOL)) tiltDetected++;
      prevGyro[0] = gyro[0]; prevGyro[1] = gyro[1]; prevGyro[2] = gyro[2];
    }
  
    for (char i=0; i<3; i++) {
        fp_gyroOffset[i] += gyro[i]/2000.0;
        prevGyro[i]=gyro[i];
    }
      
    calibGCounter--;
    if(tiltDetected>=1)
    {
      Serial.println(F("Motion detected during Gyro calibration. Starting over!"));
      calibGCounter=2000;
      tiltDetected=0;
    }
  }

  // put result into integer
  for (char i=0; i<3; i++) {
    gyroOffset[i] = fp_gyroOffset[i];
//    Serial.print(F("gyroOffset="));Serial.println(fp_gyroOffset[i], 3);
}



  enableMotorUpdates = true;
}
       
