/*************************/
/* Config Structure      */
/*************************/

struct config_t
{
uint8_t vers;
int32_t gyroPitchKp; 
int32_t gyroPitchKi;   
int32_t gyroPitchKd;
int32_t gyroRollKp;
int32_t gyroRollKi;
int32_t gyroRollKd;
int16_t accelWeight;
uint16_t gyroPitchLP;
uint16_t gyroRollLP;
uint8_t nPolesMotorPitch;
uint8_t nPolesMotorRoll;
int8_t dirMotorPitch;
int8_t dirMotorRoll;
uint8_t motorNumberPitch;
uint8_t motorNumberRoll;
uint8_t maxPWMmotorPitch;
uint8_t maxPWMmotorRoll;
int8_t minRCPitch;
int8_t maxRCPitch;
int8_t minRCRoll;
int8_t maxRCRoll;
int16_t rcGain;
bool rcAbsolute;
bool useACC;
bool accOutput;
bool dmpOutput;
} config;

void recalcMotorStuff();
void initPIDs();

void setDefaultParameters()
{
  config.vers = VERSION;
  config.gyroPitchKp = 10000;
  config.gyroPitchKi = 500;
  config.gyroPitchKd = 2000;
  config.gyroRollKp = 20000;
  config.gyroRollKi = 500;
  config.gyroRollKd = 6000;
  config.accelWeight = 0;
  config.gyroPitchLP = 1;
  config.gyroRollLP = 1;
  config.nPolesMotorPitch = 14;
  config.nPolesMotorRoll = 14;
  config.dirMotorPitch = 1;
  config.dirMotorRoll = -1;
  config.motorNumberPitch = 0;
  config.motorNumberRoll = 1;
  config.maxPWMmotorPitch = 80;
  config.maxPWMmotorRoll = 80;
  config.minRCPitch = -45;
  config.maxRCPitch = 45;
  config.minRCRoll = -45;
  config.maxRCRoll = 45;
  config.rcGain = 1;
  config.rcAbsolute = true;
  config.useACC = true;
  config.accOutput=false;
  config.dmpOutput=false;
  recalcMotorStuff();
  initPIDs();
}


typedef struct PIDdata {
  float   Kp, Ki, Kd;
} PIDdata_t;

PIDdata_t pitchPIDpar,rollPIDpar;

void initPIDs(void)
{
  rollPIDpar.Kp = config.gyroRollKp/SCALE_PID_PARAMS;
  rollPIDpar.Ki = config.gyroRollKi/SCALE_PID_PARAMS;
  rollPIDpar.Kd = config.gyroRollKd/SCALE_PID_PARAMS;

  pitchPIDpar.Kp = config.gyroPitchKp/SCALE_PID_PARAMS;
  pitchPIDpar.Ki = config.gyroPitchKi/SCALE_PID_PARAMS;
  pitchPIDpar.Kd = config.gyroPitchKd/SCALE_PID_PARAMS;
}

/*************************/
/* Variables             */
/*************************/


uint8_t pwmSinMotorPitch[256];
uint8_t pwmSinMotorRoll[256];

int currentStepMotor0 = 0;
int currentStepMotor1 = 0;

int subCountMotor0 = 0;
int subCountMotor1 = 0;

int8_t pitchDirection = 1;
int8_t rollDirection = 1;

int freqCounter=0;

uint32_t pitchMotorPos = 0;
uint32_t rollMotorPos = 0;
uint32_t pitchMotorDamp = 0;
uint32_t rollMotorDamp = 0;
uint32_t pitchGyroDamp = 0;
int32_t rollGyroDamp = 0;

int pitchMotorSpeed = 0;
int rollMotorSpeed = 0;

bool enableMotorUpdates = false;



// Variables for MPU6050
float gyroPitch;
float gyroRoll; //in deg/s
float xGyroOffset;
float yGyroOffset;
float zGyroOffset;
float resolutionDevider;
int16_t x_val;
int16_t y_val;
int16_t z_val;


float pitchSetpoint = 0;
float pitchAngle = 0;
float pitchAngleACC = 0;
float pitchPID = 0;

float pitchAnglePID = 0;

float rollSetpoint = 0;
float rollAngle = 0;
float rollAngleACC = 0;
float rollPID = 0;

float rollAnglePID = 0;

float pitchPIDVal = 0;
float rollPIDVal = 0;

float accelMagnitude;
float accelWeight;
  
//general purpuse timer
unsigned long timer=0;   
unsigned long timerACC=0;
int count=0;

// Variables for RC Decoder
uint32_t microsRisingEdgeRoll = 0;
uint32_t microsRisingEdgePitch = 0;
uint16_t pulseInPWMRoll = MID_RC;
uint16_t pulseInPWMPitch = MID_RC;
float pitchRCSpeed=0.0;
float rollRCSpeed=0.0;
bool updateRCRoll=false;
bool updateRCPitch=false;
float pitchRCSetpoint = 0.0;
float rollRCSetpoint = 0.0;









// UtilFilter: first order filter
typedef struct utilFilter {
    float tc;
    float z1;
} utilFilter_t;

// gyro LP filter
utilFilter_t pitchGyroFilter;
utilFilter_t rollGyroFilter;






