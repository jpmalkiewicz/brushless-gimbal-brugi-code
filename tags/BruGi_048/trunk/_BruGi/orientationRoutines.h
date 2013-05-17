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

#define GRAVITY 16384.0f

void initResolutionDevider()
{
    if(MPU6050_GYRO_FS == 0x00) resolutionDevider = 131.0;
    if(MPU6050_GYRO_FS == 0x01) resolutionDevider = 65.5;
    if(MPU6050_GYRO_FS == 0x02) resolutionDevider = 32.8;
    if(MPU6050_GYRO_FS == 0x03) resolutionDevider = 16.4;
}

// Read Raw Gyro signal
void updateRawGyroDataDecoupled(float* gyroRoll, float* gyroPitch, float pitchAngle)
{
  int16_t x,y,z;
  mpu.getRotation(&x,&y,&z);

  *gyroPitch = (y-yGyroOffset)/resolutionDevider;
  int16_t absPitchAngle = fabs(pitchAngle);
  
  if((absPitchAngle<=45)||(absPitchAngle>=135)) 
    *gyroRoll = (x-xGyroOffset)/resolutionDevider;
  else
    *gyroRoll = (z-zGyroOffset)/resolutionDevider;
//  if(absPitchAngle>=90) *gyroRoll = *gyroRoll * -1;
}



// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds 
void gyroOffsetCalibration()
{
  #define TOL 96
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

       

// Calculate Orientation Angles from RAW ACC, Drift compensation...
// Calculation steps are interleaved into the main loop to keep time for loop execution even and low
void getOrientationAndDriftCompensation()
{
    count++;
    // Update ACC data approximately at 50Hz to save calculation time.
    if(count == 10) x_val = mpu.getAccelerationX();
    if(count == 11) y_val = mpu.getAccelerationY();
    if(count == 12) z_val = mpu.getAccelerationZ();
    if(count == 13) rollAngleACC = 0.9 * rollAngleACC + 0.1 * ultraFastAtan2(-y_val,-z_val); 
    if(count == 14) pitchAngleACC = 0.9 * pitchAngleACC + 0.1 * -ultraFastAtan2(-x_val,-z_val);
    if(count == 15)
    {
      // Calculate the magnitude of the accelerometer vector and scale to gravity.
      accelMagnitude = sqrt((float)(x_val/ GRAVITY * x_val/ GRAVITY) + (float)(y_val/ GRAVITY * y_val/ GRAVITY) + (float)(z_val/ GRAVITY * z_val/ GRAVITY));// 
    }
    if(count == 16)
    {
      // Dynamic weighting of accelerometer info (reliability filter)
      // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
      accelWeight = constrain(1.0 - 2.0*abs(1.0 - accelMagnitude),0.0,1.0); 
    }
    if(count == 17)
    {
      if(config.rcAbsolute==1)
      {
        if(pitchRCSetpoint!=0)
        {
          float delta = pitchAngleACC - pitchRCSetpoint; 
          pitchAnglePID = delta * ((abs(delta)<1.0) ? 1 : abs(delta)) * config.rcGain * accelWeight;
        }
        else
          pitchAnglePID = (pitchAngleACC - pitchSetpoint)  * abs(pitchAngleACC - pitchSetpoint) * pitchPIDpar.Ki * accelWeight;
      }
      else
      {
        if(abs(pitchRCSpeed)>0.01)
        {
          pitchAnglePID =0.0;
          pitchSetpoint = pitchAngleACC;
        }
        else
        {
          pitchAnglePID = (pitchAngleACC - pitchSetpoint)  * abs(pitchAngleACC - pitchSetpoint) * pitchPIDpar.Ki * accelWeight;
        }
      }
    }
    if(count == 18)
    {
      if(config.rcAbsolute==1)
      {
        if(rollRCSetpoint!=rollSetpoint)
        {
          float delta = rollAngleACC - rollRCSetpoint;
          rollAnglePID = delta * ((abs(delta)<1.0) ? 1 : abs(delta)) * config.rcGain * accelWeight;
        }
        else
          rollAnglePID = (rollAngleACC - rollSetpoint) * abs(rollAngleACC - rollSetpoint) * rollPIDpar.Ki * accelWeight;
      }
      else
      {
        if(abs(rollRCSpeed)>0.01)
        {
          rollAnglePID =0.0;
          rollSetpoint = rollAngleACC;
        }
        else
        {
          rollAnglePID = (rollAngleACC - rollSetpoint) * abs(rollAngleACC - rollSetpoint) * rollPIDpar.Ki * accelWeight;
        }
      }
    }
    if(count == 19)
    {
      count=0;
      if(config.accOutput==1){Serial.print(pitchAngleACC);Serial.print(" ACC ");Serial.println(rollAngleACC);}
    }
}
