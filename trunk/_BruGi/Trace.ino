#include "Trace.h"

void printTrace(traceModeType traceMode)
{
 
  Serial.print(F("traceData"));

  switch (traceMode) {
    case TRC_RC:
      // *******  RC  *********
      Serial.print(F(" RC"));
      for (char id = 0; id < RC_DATA_SIZE; id++)
      {
        Serial.print(F(" "));
        Serial.print(rcData[id].rx);
      }
      
      for (char id = 0; id < RC_DATA_SIZE; id++)
      {
        Serial.print(F(" "));
        Serial.print(rcData[id].valid);
      }

      for (char id = 0; id < RC_DATA_SIZE; id++)
      {
        Serial.print(F(" "));
        Serial.print(rcData[id].rcAuxSwitch1);
        Serial.print(rcData[id].rcAuxSwitch2);
      }
      break;
      
    case TRC_AUX:
      // *******  AUX  *********
      Serial.print(F(" AUX "));
      Serial.print(fpvModePitch);
      
      Serial.print(F(" "));
      Serial.print(fpvModeRoll);
      
      Serial.print(F(" "));
      Serial.print(altModeAccTime);
      
      break;
      
    case TRC_IMU:
      // *******  IMU  *********
      Serial.print(F(" IMU "));
      Serial.print(EstG.V.X);
      Serial.print(F(" "));
      Serial.print(EstG.V.Y);
      Serial.print(F(" "));
      Serial.print(EstG.V.Z);
    
      Serial.print(F(" "));
      Serial.print(angle[ROLL]);
      Serial.print(F(" "));
      Serial.print(angle[PITCH]);
    
      break;

    case TRC_GYRO:
      // *******  Gyro Sensor  *********
      Serial.print(F(" GYRO"));

      for (char axis = 0; axis < 3; axis++)
      {
        Serial.print(F(" "));
        Serial.print(gyroOffset[axis]);
      }

      for (char axis = 0; axis < 3; axis++)
      {
        Serial.print(F(" "));
        Serial.print(gyroADC[axis]);
      }
      break;
     
    case TRC_ACC:
      // *******  ACC Sensor  *********
      Serial.print(F(" SENSOR_ACC"));

      for (char axis = 0; axis < 3; axis++)
      {
        Serial.print(F(" "));
        Serial.print(accADC[axis]);
      }
      
      Serial.print(F(" "));
      Serial.print(accMag);
      
      break;

    case TRC_PID_PITCH:
      // *******  PID Pitch  *********
      Serial.print(F(" PID_PITCH "));
      
      Serial.print(pitchAngleSet);
      
      Serial.print(F(" "));
      Serial.print(angle[PITCH]);
      
      Serial.print(F(" "));
      Serial.print(pitchMotorDrive);
      
      break;

    case TRC_PID_ROLL:
      // *******  PID Roll  *********
      Serial.print(F(" PID_ROLL "));
      
      Serial.print(rollAngleSet);
      
      Serial.print(F(" "));
      Serial.print(angle[ROLL]);
      
      Serial.print(F(" "));
      Serial.print(rollMotorDrive);
      
      break;

    case TRC_OAC:
      // *******  OAC mode 2 (replaces oac/acc mode)  *********
      Serial.print(F(" ACC2 "));
      Serial.print(angle[PITCH]); 
      Serial.print(F(" "));
      Serial.print(angle[ROLL]);       
      break;
            
    default:
      break;
  }
  
  Serial.println(F(""));

}
