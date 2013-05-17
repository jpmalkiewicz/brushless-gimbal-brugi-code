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
  int16_t prevGyroX,gyroX;
  int16_t prevGyroY,gyroY;
  int16_t prevGyroZ,gyroZ; 
  uint8_t tiltDetected = 0;
  int calibGCounter = 2000;
  
  // Set voltage on all motor phases to zero 
  enableMotorUpdates = false;
    
  while(calibGCounter>0)
  {
    OCR2A = 0; OCR2B = 0; OCR1A = 0; OCR1B = 0; OCR0A = 0; OCR0B = 0; 
    gyroX = mpu.getRotationX();  
    gyroY = mpu.getRotationY();  
    gyroZ = mpu.getRotationZ();  
    if(calibGCounter==2000)
    {
      xGyroOffset=0; yGyroOffset=0; zGyroOffset=0;
      prevGyroX = gyroX; prevGyroY = gyroY; prevGyroZ = gyroZ;
    }
    
    if(calibGCounter % 10 == 0)
    { 
      if((abs(prevGyroX-gyroX)>TOL)||(abs(prevGyroY-gyroY)>TOL)||(abs(prevGyroZ-gyroZ)>TOL)) tiltDetected++;
      prevGyroX = gyroX; prevGyroY = gyroY; prevGyroZ = gyroZ;
    }
    
    xGyroOffset += gyroX/2000.0;
    yGyroOffset += gyroY/2000.0;
    zGyroOffset += gyroZ/2000.0;

    calibGCounter--;
    if(tiltDetected>=1)
    {
      Serial.println(F("Motion detected during Gyro calibration. Starting over!"));
      calibGCounter=2000;
      tiltDetected=0;
    }
  }
  enableMotorUpdates = true;
}
       
