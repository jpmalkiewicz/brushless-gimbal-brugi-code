#ifndef _TRACE_H_
#define _TRACE_H_

// Gimbal State
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

// index of last "slow Trace" element in traceModeType, others are fast types, just by convention 
#define STRACE_IDX 7

void printTrace(traceModeType traceMode);

#endif /*  _TRACE_H_ */
