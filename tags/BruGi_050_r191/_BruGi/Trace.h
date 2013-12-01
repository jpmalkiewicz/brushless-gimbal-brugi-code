#ifndef _TRACE_H_
#define _TRACE_H_

// trace modes
enum traceModeType {
 TRC_OFF=0, 
 TRC_RC,
 TRC_AUX, 
 TRC_IMU,
 TRC_ACC,
 TRC_GYRO,
 TRC_PID_PITCH,
 TRC_PID_ROLL,   // last, sTrace
 TRC_OAC,        // replaces accOutput for the moment
 TRC_ALL
};

enum msgSeverity_t {
  MSG_INFO=0, 
  MSG_WARNING,
  MSG_ERROR,
  MSG_VERSION
};

// index of last "slow Trace" element in traceModeType, others are fast types, just by convention 
#define STRACE_IDX 7

void printTrace(traceModeType traceMode);
void printTrace_int(int32_t value);
void printTrace_float(float value);

void printMessage(msgSeverity_t msgSeverity, const __FlashStringHelper * msg);

#endif /*  _TRACE_H_ */
