#ifndef _TRACE_H_
#define _TRACE_H_

// Gimbal State
enum traceModeType {
 TRC_OFF=0, 
 TRC_RC,
 TRC_IMU,
 TRC_ACC,
 TRC_GYRO,
 TRC_PID_PITCH,
 TRC_PID_ROLL
};

void printTrace(traceModeType traceMode);

#endif /*  _TRACE_H_ */
