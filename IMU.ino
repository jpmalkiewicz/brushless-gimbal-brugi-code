// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC Magnitude */
#define ACC_LPF_FACTOR 40

#define ACC_1G 16384.0f

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f


// init and check I2C connection
bool initI2C() {
 
  bool success = true;
  
  // Auto detect MPU address
  mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
  mpu.initialize();
  if(mpu.testConnection()) {
//    printMessage(MSG_INFO, F("MPU6050 (HIGH)"));
  } else {
    mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
    mpu.initialize();
    if(mpu.testConnection()) {
//      printMessage(MSG_INFO, F("MPU6050 (LOW)"));
    } else {
      printMessage(MSG_ERROR, F("MPU6050 not found on I2C")); 
      success = false;
    }
  }
  return success;
}


// init MPU modes
void initMPU() {
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       // +- 2G
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);                 // Set Gyro Low Pass Filter
  mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  mpu.setSleepEnabled(false);
  mpu.i2cErrors = 0;
}


// set default sensor orientation (sensor upside)
void initSensorOrientationDefault() {
  
  // channel assignment
  sensorDef.Gyro[ROLL].idx = 0;
  sensorDef.Gyro[PITCH].idx = 1;
  sensorDef.Gyro[YAW].idx = 2;

  sensorDef.Acc[ROLL].idx = 1;     // y
  sensorDef.Acc[PITCH].idx = 0;    // x
  sensorDef.Acc[YAW].idx = 2;      // z

  // direction
  sensorDef.Gyro[ROLL].dir = 1;
  sensorDef.Gyro[PITCH].dir = -1;
  sensorDef.Gyro[YAW].dir = 1;

  sensorDef.Acc[ROLL].dir = 1;
  sensorDef.Acc[PITCH].dir = 1;
  sensorDef.Acc[YAW].dir = 1;
  
}

// swap two char items
void swap_char(char * a, char * b) {
  char tmp = *a;
  *a = *b;
  *b = tmp;
}
// swap two int items
void swap_int(int * a, int * b) {
  int tmp = *a;
  *a = *b;
  *b = tmp;
}

// set sensor orientation according config
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void initSensorOrientation() {
  
  initSensorOrientationDefault();
  
  if (config.axisReverseZ) {
    // flip over roll
    sensorDef.Acc[YAW].dir *= -1;
    sensorDef.Acc[ROLL].dir *= -1;
    sensorDef.Gyro[PITCH].dir *= -1;
    sensorDef.Gyro[YAW].dir *= -1;
  }
  if (config.axisSwapXY) {
    // swap gyro axis
    swap_char(&sensorDef.Gyro[ROLL].idx, &sensorDef.Gyro[PITCH].idx); 
    swap_int(&sensorDef.Gyro[ROLL].dir, &sensorDef.Gyro[PITCH].dir);
    sensorDef.Gyro[YAW].dir *= -1;
    // swap acc axis
    swap_char(&sensorDef.Acc[ROLL].idx, &sensorDef.Acc[PITCH].idx);
    swap_int(&sensorDef.Acc[ROLL].dir, &sensorDef.Acc[PITCH].dir);
  }
}

_NO_INLINE_ void setACCtc (int16_t accTimeConstant) {
  AccComplFilterConst = (float)DT_FLOAT/(accTimeConstant + DT_FLOAT);
}

void initIMUtc() {
  setACCtc(config.accTimeConstant);
}

// update angle offest
_NO_INLINE_ void updateAngleOffset() {
   angleOffsetPitch = (int32_t)config.angleOffsetPitch * 10; //angleOffsetPitch_f;
   angleOffsetRoll = (int32_t)config.angleOffsetRoll * 10; //angleOffsetRoll_f;
 }

void initIMU() {
 
  // resolutionDevider=131, scale = 0.000133
  // 102us
  gyroScale =  1.0 / resolutionDevider / 180.0 * 3.14159265359 * DT_FLOAT;  // convert to radians
  
  // initialize complementary filter timw constant
  setACCtc(config.accTimeConstant);
 
  accMag = ACC_1G*ACC_1G; // magnitude = 1G initially
 
  // initialize coordinate system in EstG
  EstG.V.X = 0;
  EstG.V.Y = 0;
  EstG.V.Z = ACC_1G;

}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
inline void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void readGyros() {
  int16_t axisRot[3];
  char idx;
  // 414 us

  // read gyros
  mpu.getRotation(&axisRot[0], &axisRot[1], &axisRot[2]);
  axisRot[0] -= config.gyrOffsetX;
  axisRot[1] -= config.gyrOffsetY;
  axisRot[2] -= config.gyrOffsetZ;
  
  idx = sensorDef.Gyro[0].idx;
  gyroADC[ROLL] = axisRot[idx];
  gyroADC[ROLL] *= sensorDef.Gyro[0].dir;

  idx = sensorDef.Gyro[1].idx;
  gyroADC[PITCH] = axisRot[idx];
  gyroADC[PITCH] *= sensorDef.Gyro[1].dir;

  idx = sensorDef.Gyro[2].idx;
  gyroADC[YAW] = axisRot[idx];  
  gyroADC[YAW] *= sensorDef.Gyro[2].dir;
  
}

// get acceleration for 3-axis
void readACCs()
{
  int16_t rawVal[3];
  int16_t devVal[3];
  
  mpu.getAcceleration(
    &rawVal[0],
    &rawVal[1],
    &rawVal[2]
    );
    
  devVal[sensorDef.Acc[ROLL].idx]  = rawVal[0] - config.accOffsetX;
  devVal[sensorDef.Acc[PITCH].idx] = rawVal[1] - config.accOffsetY;
  devVal[sensorDef.Acc[YAW].idx]   = rawVal[2] - config.accOffsetZ;
  
  for (int8_t axis = 0; axis < 3; axis++) {
    accADC[axis] = devVal[axis]*sensorDef.Acc[axis].dir;
  }
}


void updateGyroAttitude(){
  uint8_t axis;
  
  float deltaGyroAngle[3];

  // 43 us
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * gyroScale;
  }
  
  // 111 us
  rotateV(&EstG.V,deltaGyroAngle);
 
}

void updateACC(){
  uint8_t axis;
  float accMagSum = 0;

  for (axis = 0; axis < 3; axis++) {
    accLPF[axis] = accADC[axis];
    accMagSum += accLPF[axis]*accLPF[axis];
  }

  // 24 us
  accMagSum = accMagSum*100.0/(ACC_1G*ACC_1G);
  utilLP_float(&accMag, accMagSum, (1.0f/ACC_LPF_FACTOR)); 
}


void updateACCAttitude(){
  uint8_t axis;
  
  // 80 us
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if (( 36 < accMag && accMag < 196 ) || disableAccGtest) {
    for (axis = 0; axis < 3; axis++) {
      //utilLP_float(&EstG.A[axis], accLPF[axis], AccComplFilterConst);
      EstG.A[axis] = EstG.A[axis] * (1.0 - AccComplFilterConst) + accLPF[axis] * AccComplFilterConst; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
    } 
  }
}


void getAttiduteAngles() {

  // attitude of the estimated vector  
  // 200us
  angle[ROLL]  = angleOffsetRoll +  Rajan_FastArcTan2_deg1000(EstG.V.X , sqrt(EstG.V.Z*EstG.V.Z+EstG.V.Y*EstG.V.Y));
  // 142 us
  angle[PITCH] = angleOffsetPitch + Rajan_FastArcTan2_deg1000(EstG.V.Y , EstG.V.Z);

}

