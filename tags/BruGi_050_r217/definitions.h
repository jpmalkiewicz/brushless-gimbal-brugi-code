#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

#include <Arduino.h>

#define _INLINE_		__attribute__( ( always_inline ) ) inline
#define _NO_INLINE_		__attribute__( ( noinline ) )

/*************************/
/* user definitions      */
/*************************/

// max number of configuration sets in EEPROM
#define NUM_EEPROM_CONFIG_SETS 3

// add code to support manual control panel with push button and leds (see ControlPanel.ino)
//#define USE_CONTROL_PANEL

// do no generate help text to reduce code space
#define MINIMIZE_TEXT

// trace minimum to save code space
//#define TRACE_EXTRA
// maximum code size depends on bootloader 
//   Arduino Uno: 32256 bytes (0x7e00)
//   Arduino Pro or Pro Mini: 30720 bytes (0x7800)
//   Arduino Mini w/ ATmega328: 28672 bytes (0x7000)
// for further reduction USE_CONTROL_PANEL can be diabled (~550 bytes)

/*************************/
/* internal definitions  */
/*************************/

// MPU Address Settings
#define MPU6050_ADDRESS_AD0_LOW     0x68 // default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Drotek MPU breakout board
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH


// Define Brushless PWM Mode, uncomment ONE setting
#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
//#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM
//#define NO_PWM_LOOP

#define MOTORUPDATE_FREQ 500                 // in Hz, 1000 is default
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ*1.024) // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)


#define POUT_FREQ 25      // rate of ACC print output in Hz, 25 Hz is default
#define TRACE_OUT_FREQ 10 // rate of Trace Outoput in Hz, 10Hz is default 
#define LOCK_TIME_SEC 5   // gimbal fast lock time at startup 

// LP filter coefficient
#define LOWPASS_K_FLOAT(TAU) (DT_FLOAT/(TAU+DT_FLOAT))

// Do not change for now
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250  // +-250,500,1000,2000 deg/s
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_256  // 5,10,20,42,98,188,256 Hz

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

#define SCALE_ACC 10000.0
#define SCALE_PID_PARAMS 1000.0f

// specifie input for VCC/Ubat measurement
#define ADC_VCC_PIN A3 

#define UBAT_ADC_SCALE (5.0 / 1023.0)
// voltage divider 
#define UBAT_R1 10000.0
#define UBAT_R2 2200.0
#define UBAT_SCALE ( (UBAT_R1 + UBAT_R2) / UBAT_R2 )

// RC data size and channel assigment
#define RC_DATA_SIZE  5
#define RC_DATA_PITCH 0
#define RC_DATA_ROLL  1
#define RC_DATA_AUX   2
#define RC_DATA_FPV_PITCH 3
#define RC_DATA_FPV_ROLL 4

// RC PPM pin A0, A1 or A2
#define RC_PIN_PPM_A2
//#define RC_PIN_PPM_A1
//#define RC_PIN_PPM_A0

#define RC_PIN_CH0 A2 
#define RC_PIN_CH1 A1 
#define RC_PIN_CH2 A0 

#define MIN_RC 1000
#define MID_RC 1500
#define MAX_RC 2000
#define MIN_RC_VALID 900
#define MAX_RC_VALID 2100
#define RC_DEADBAND 50
#define RC_TIMEOUT 100000

// rc switch on/off threshold (as offset from MID_RC)
#define RC_SW_THRESH 150

// PPM Decoder
#define RC_PPM_GUARD_TIME 4000
#define RC_PPM_RX_MAX_CHANNELS 32

// I2C Frequency
//#define I2C_SPEED 100000L     //100kHz normal mode
//#define I2C_SPEED 400000L   //400kHz fast mode
#define I2C_SPEED 800000L   //800kHz ultra fast mode

// Hardware Abstraction for Motor connectors,
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B


#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif


#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 32
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 4
#endif
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 8
#endif
#ifdef NO_PWM_LOOP
  #define CC_FACTOR 1
#endif




/*************************/
/* user pin definitions  */
/*************************/

// standard LED (not on all boards)
#define LEDPIN                     8
#define LEDPIN_PINMODE             pinMode (LEDPIN, OUTPUT);
#define LEDPIN_OFF                 digitalWrite(LEDPIN, LOW);
#define LEDPIN_ON                  digitalWrite(LEDPIN, HIGH);

#ifdef USE_CONTROL_PANEL
  // manual control panel elements
  // push button
  #define PB_PIN               12   // CH4/MISO
  // status leds
  #define ST_LED0              4    // CH2
  #define ST_LED1              7    // CH3
  
  // set pin output
  #define SET_LED( pin, x )    ((x==0) ? digitalWrite(pin, LOW) : digitalWrite(pin, HIGH))

  // note: execution time for CH2_ON/CH2_OFF = 4 us
  #define CH2_PINMODE
  #define CH2_OFF
  #define CH2_ON
  
  #define CH3_PINMODE
  #define CH3_OFF
  #define CH3_ON

#else
  // this is used just for debugging and profiling
  // note: execution time for CH2_ON/CH2_OFF = 4 us
  #define CH2_PINMODE                pinMode (4, OUTPUT);
  #define CH2_OFF                    digitalWrite(4, LOW);
  #define CH2_ON                     digitalWrite(4, HIGH);
  
  #define CH3_PINMODE                pinMode (7, OUTPUT);
  #define CH3_OFF                    digitalWrite(7, LOW);
  #define CH3_ON                     digitalWrite(7, HIGH);
#endif


//
// Fast Digital Output
//
// set port=1
inline void FastPort_Set_1(uint8_t pin)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));
  uint8_t val = *pport;
  *pport = val | digitalPinToBitMask(pin);
}

// port=0
inline void FastPort_Set_0(uint8_t pin)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));
  uint8_t val = *pport;
  *pport = val & ~digitalPinToBitMask(pin);
}


// just for debugging 


// enable stack and heapsize check (use just for debugging)
//#define STACKHEAPCHECK_ENABLE


#endif // _DEFINITIONS_H
