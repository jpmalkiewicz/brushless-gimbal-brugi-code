#ifndef _TRACE_H_
#define _TRACE_H_

// trace modes
enum traceModeType {
 TRC_OFF=0, 
 TRC_RC,
 TRC_AUX,
 TRC_MPU,
#ifdef TRACE_EXTRA 
 TRC_IMU,
 TRC_ACC,
 TRC_GYRO,
 TRC_PID_PITCH,
 TRC_PID_ROLL,
#endif
 TRC_LAST_IDX,       // just to mark the last strace mode index
 TRC_OAC=254,        // replaces accOutput for the moment
 TRC_ALL=255
};

enum msgSeverity_t {
  MSG_INFO=0, 
  MSG_WARNING,
  MSG_ERROR,
  MSG_VERSION
};


void printTrace(traceModeType traceMode);
void printTrace_int(int32_t value);
void printTrace_float(float value);

void printMessage(msgSeverity_t msgSeverity, const __FlashStringHelper * msg);

#endif /*  _TRACE_H_ */
