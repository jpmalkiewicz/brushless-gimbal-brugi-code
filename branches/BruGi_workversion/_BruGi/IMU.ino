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
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 40
#endif


/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR (1.0f/2000.0f)
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

#define ACC_1G 15500.0f

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f



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
  sensorDef.Gyro[PITCH].dir = 1;
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
  }
  if (config.axisSwapXY) {
    // swap gyro axis
    swap_char(&sensorDef.Gyro[ROLL].idx, &sensorDef.Gyro[PITCH].idx); 
    swap_int(&sensorDef.Gyro[ROLL].dir, &sensorDef.Gyro[PITCH].dir); sensorDef.Gyro[PITCH].dir *= -1;   // try and error ;-)
    // swap acc axis
    swap_char(&sensorDef.Acc[ROLL].idx, &sensorDef.Acc[PITCH].idx);
    swap_int(&sensorDef.Acc[ROLL].dir, &sensorDef.Acc[PITCH].dir); sensorDef.Acc[ROLL].dir *= -1;
  }
}

void initIMU() {
 
  // resolutionDevider=131, scale = 0.000133
  // 102us
  gyroScale =  1.0 / resolutionDevider / 180.0 * 3.14159265359;  // convert to radians
  
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
void rotateV(struct fp_vector *v,float* delta) {
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
  idx = sensorDef.Gyro[0].idx;
  gyroADC[ROLL] = axisRot[idx]-gyroOffset[idx];
  gyroADC[ROLL] *= sensorDef.Gyro[0].dir;

  idx = sensorDef.Gyro[1].idx;
  gyroADC[PITCH] = -(axisRot[idx]-gyroOffset[idx]);
  gyroADC[PITCH] *= sensorDef.Gyro[1].dir;

  idx = sensorDef.Gyro[2].idx;
  gyroADC[YAW] = axisRot[idx]-gyroOffset[idx];  
  gyroADC[YAW] *= sensorDef.Gyro[2].dir;
  
}

void readACC(axisDef axis) {
  // get acceleration
  // 382 us
  char idx;
  int16_t val;
  idx = sensorDef.Acc[axis].idx;
  val = mpu.getAccelerationN(idx);  // TODO: 370us 
  val *= sensorDef.Acc[axis].dir;
  accADC[axis] = val;
}

void updateGyroAttitude(){
  uint8_t axis;
  
  float deltaGyroAngle[3];

  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * gyroScale * DT_FLOAT;
  }
  // 168 us
  rotateV(&EstG.V,deltaGyroAngle);
}

void updateACC(){
  uint8_t axis;

  // 179 us
  for (axis = 0; axis < 3; axis++) {
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    accSmooth[axis] = accLPF[axis];
    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }

  //  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G); 
  // 130 us
  // split operation to avoid 32-bit overflow, TODO: no division may happen !!!
  accMag = accMag/(int32_t)ACC_1G;
  accMag = accMag*100;
  accMag = accMag/(int32_t)ACC_1G;

  // 11 us
  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

}


void updateACCAttitude(){
  uint8_t axis;
  
  // 255 us
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 ) {
    for (axis = 0; axis < 3; axis++) {
      int32_t acc = accSmooth[axis];
      EstG.A[axis] = EstG.A[axis] * (1.0 - GYR_CMPF_FACTOR) + acc * GYR_CMPF_FACTOR; // note: this is different from MultiWii ...
    } 
  }
  
}


void getAttiduteAngles() {

  // attitude of the estimated vector
  // 280 us
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z);
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z);

}



void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  static t_fp_vector EstG;
  
  static float accLPF[3];

  float deltaGyroAngle[3];

  // Initialization
  // 258 us
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * gyroScale * DT_FLOAT;
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    accSmooth[axis] = accLPF[axis];
    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }


  //  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G); 
  // 17 us
  // split operation to avoid 32-bit overflow, TODO: no division may happen !!!
  accMag = accMag/(int32_t)ACC_1G;
  accMag = accMag*100;
  accMag = accMag/(int32_t)ACC_1G;

  // 168 us
  rotateV(&EstG.V,deltaGyroAngle);
  // 15 us
  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // 255 us
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 ) {
    for (axis = 0; axis < 3; axis++) {
      int32_t acc = accSmooth[axis];
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
    } 
  }
  
  // Attitude of the estimated vector  TODO: true angle calculation 
  // 280 us
  angle[ROLL]  =  _atan2x(EstG.V.X , EstG.V.Z);
  angle[PITCH] =  _atan2x(EstG.V.Y , EstG.V.Z);

}

