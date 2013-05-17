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
  int32_t   Kp, Ki, Kd;
} PIDdata_t;

PIDdata_t pitchPIDpar,rollPIDpar;

void initPIDs(void)
{
  rollPIDpar.Kp = config.gyroRollKp;
  rollPIDpar.Ki = config.gyroRollKi/1000;
  rollPIDpar.Kd = config.gyroRollKd;

  pitchPIDpar.Kp = config.gyroPitchKp;
  pitchPIDpar.Ki = config.gyroPitchKi/1000;
  pitchPIDpar.Kd = config.gyroPitchKd;
  
}



/*************************/
/* Variables             */
/*************************/



// motor drive

uint8_t pwmSinMotorPitch[256];
uint8_t pwmSinMotorRoll[256];

int currentStepMotor0 = 0;
int currentStepMotor1 = 0;
bool motorUpdate = false; 

int8_t pitchDirection = 1;
int8_t rollDirection = 1;

int freqCounter=0; // TODO: back to char later ...

int pitchMotorDrive = 0;
int rollMotorDrive = 0;

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

#if 0
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
#endif

float PitchPhiSet = 0;
float RollPhiSet = 0;

  
//general purpuse timer
unsigned long timer=0;   
unsigned long timerACC=0;
int count=0;

// Variables for RC Decoder
uint32_t microsRisingEdgeRoll = 0;
uint32_t microsRisingEdgePitch = 0;
uint16_t pulseInPWMRoll = MID_RC;
uint16_t pulseInPWMPitch = MID_RC;

uint32_t microsLastPWMRollUpdate = 0;
uint32_t microsLastPWMPitchUpdate = 0;

float pitchRCSpeed=0.0;
float rollRCSpeed=0.0;
bool updateRCRoll=false;        // RC channel value got updated
bool updateRCPitch=false;
bool validRCRoll=false;         // RC inputs valid
bool validRCPitch=false;
float pitchRCSetpoint = 0.0;
float rollRCSetpoint = 0.0;

//*************************************
//  IMU
//*************************************
struct flags_struct {
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
} f;

enum axisDef {
  ROLL,
  PITCH,
  YAW
};

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

static float gyroScale=0;

static int32_t accSmooth[3];
static int16_t gyroADC[3];
static int16_t accADC[3];

static t_fp_vector EstG;

static float accLPF[3];
static int32_t accMag = 0;

static int16_t acc_25deg = 25;      //** TODO: check

static int32_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

