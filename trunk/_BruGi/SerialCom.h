//************************************************************************************
// general config parameter access routines
//************************************************************************************

// types of config parameters
enum confType {
  BOOL,
  INT8,
  INT16,
  INT32,
  UINT8,
  UINT16,
  UINT32
};

#define CONFIGNAME_MAX_LEN 17
typedef struct configDef {
  char name[CONFIGNAME_MAX_LEN];  // name of config parameter
  confType type;                  // type of config parameters
  void * address;                 // address of config parameter
  void (* updateFunction)(void);  // function is called when parameter update happens
} t_configDef;

t_configDef configDef;

// access decriptor as arry of bytes as well
typedef union {
  t_configDef   c;
  char          bytes[sizeof(t_configDef)];
} t_configUnion;

t_configUnion configUnion;

//
// list of all config parameters
// to be accessed by par command
//
// descriptor is stored in PROGMEN to preserve RAM space
// see http://www.arduino.cc/en/Reference/PROGMEM
// and http://jeelabs.org/2011/05/23/saving-ram-space/
const t_configDef PROGMEM configListPGM[] = {
  {"vers",             UINT8, &config.vers,             NULL},

  {"gyroPitchKp",      INT32, &config.gyroPitchKp,      &initPIDs},
  {"gyroPitchKi",      INT32, &config.gyroPitchKi,      &initPIDs},
  {"gyroPitchKd",      INT32, &config.gyroPitchKd,      &initPIDs},
  {"gyroRollKp",       INT32, &config.gyroRollKp,       &initPIDs},
  {"gyroRollKi",       INT32, &config.gyroRollKi,       &initPIDs},
  {"gyroRollKd",       INT32, &config.gyroRollKd,       &initPIDs},
  {"accTimeConstant",  INT16, &config.accTimeConstant,  &initIMUtc},
  {"mpuLPF",           INT8,  &config.mpuLPF,           &initMPUlpf},
  
  {"angleOffsetPitch", INT16, &config.angleOffsetPitch, NULL},
  {"angleOffsetRoll",  INT16, &config.angleOffsetRoll,  NULL},
  
  {"dirMotorPitch",    INT8,  &config.dirMotorPitch,    NULL},
  {"dirMotorRoll",     INT8,  &config.dirMotorRoll,     NULL},
  {"motorNumberPitch", UINT8, &config.motorNumberPitch, NULL},
  {"motorNumberRoll",  UINT8, &config.motorNumberRoll,  NULL},
  {"maxPWMmotorPitch", UINT8, &config.maxPWMmotorPitch, NULL},
  {"maxPWMmotorRoll",  UINT8, &config.maxPWMmotorRoll,  NULL},
  {"refVoltageBat",    UINT16, &config.refVoltageBat,   NULL},
  {"cutoffVoltage",    UINT16, &config.cutoffVoltage,   NULL},
  {"motorPowerScale",  BOOL,  &config.motorPowerScale,  NULL},
  
  {"rcAbsolutePitch",  BOOL,  &config.rcAbsolutePitch,  NULL},
  {"rcAbsoluteRoll",   BOOL,  &config.rcAbsoluteRoll,   NULL},

  {"maxRCPitch",       INT8,  &config.maxRCPitch,       NULL},
  {"maxRCRoll",        INT8,  &config.maxRCRoll,        NULL},
  {"minRCPitch",       INT8,  &config.minRCPitch,       NULL},
  {"minRCRoll",        INT8,  &config.minRCRoll,        NULL},
  {"rcGainPitch",      INT16, &config.rcGainPitch,      NULL},
  {"rcGainRoll",       INT16, &config.rcGainRoll,       NULL},
  {"rcLPFPitch",       INT16, &config.rcLPFPitch,       &initRC},
  {"rcLPFRoll",        INT16, &config.rcLPFRoll,        &initRC},

  {"rcModePPMPitch",   BOOL,  &config.rcModePPMPitch,   &initRCPins},
  {"rcModePPMRoll",    BOOL,  &config.rcModePPMRoll,    &initRCPins},
  {"rcModePPMAux",     BOOL,  &config.rcModePPMAux,     &initRCPins},
  {"rcChannelPitch",   INT8,  &config.rcChannelPitch,   NULL},
  {"rcChannelRoll",    INT8,  &config.rcChannelRoll,    NULL},
  {"rcChannelAux",     INT8,  &config.rcChannelAux,     NULL},
  {"rcMid",            INT16, &config.rcMid,            NULL},
  
  {"accOutput",        BOOL,  &config.accOutput,        NULL},
  {"trace",            UINT8, &config.trace,            NULL},

  {"enableGyro",       BOOL,  &config.enableGyro,       NULL},
  {"enableACC",        BOOL,  &config.enableACC,        NULL},

  {"axisReverseZ",     BOOL,  &config.axisReverseZ,     &initSensorOrientation},
  {"axisSwapXY",       BOOL,  &config.axisSwapXY,       &initSensorOrientation},
  
  {"fpvSwPitch",       INT8, &config.fpvSwPitch,        NULL},
  {"fpvSwRoll",        INT8, &config.fpvSwRoll,         NULL},

  {"altSwAccTime",     INT8, &config.altSwAccTime,      NULL},
  {"accTimeConstant2", INT16, &config.accTimeConstant2, &initIMU},
    
  {"", BOOL, NULL, NULL} // terminating NULL required !!
};

// read bytes from program memory
void getPGMstring (PGM_P s, char * d, int numBytes) {
  char c;
  for (int i=0; i<numBytes; i++) {
    *d++ = pgm_read_byte(s++);
  }
}

// find Config Definition for named parameter
t_configDef * getConfigDef(char * name) {

  void * addr = NULL;
  bool found = false;  
  t_configDef * p = (t_configDef *)configListPGM;

  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    if (strncmp(configUnion.c.name, name, CONFIGNAME_MAX_LEN) == 0) {
      addr = configUnion.c.address;
      found = true;
      break;
   }
   p++; 
  }
  if (found) 
      return &configUnion.c;
  else 
      return NULL;
}


// print single parameter value
void printConfig(t_configDef * def) {
  if (def != NULL) {
    Serial.print(def->name);
    Serial.print(F(" "));
    switch (def->type) {
      case BOOL   : Serial.print(*(bool *)(def->address)); break;
      case UINT8  : Serial.print(*(uint8_t *)(def->address)); break;
      case UINT16 : Serial.print(*(uint16_t *)(def->address)); break;
      case UINT32 : Serial.print(*(uint32_t *)(def->address)); break;
      case INT8   : Serial.print(*(int8_t *)(def->address)); break;
      case INT16  : Serial.print(*(int16_t *)(def->address)); break;
      case INT32  : Serial.print(*(int32_t *)(def->address)); break;
    }
    Serial.println("");
  } else {
    Serial.println(F("ERROR: printConfig: illegal parameter"));    
  }
}

// write single parameter with value
void writeConfig(t_configDef * def, int32_t val) {
  if (def != NULL) {
    switch (def->type) {
      case BOOL   : *(bool *)(def->address)     = val; break;
      case UINT8  : *(uint8_t *)(def->address)  = val; break;
      case UINT16 : *(uint16_t *)(def->address) = val; break;
      case UINT32 : *(uint32_t *)(def->address) = val; break;
      case INT8   : *(int8_t *)(def->address)   = val; break;
      case INT16  : *(int16_t *)(def->address)  = val; break;
      case INT32  : *(int32_t *)(def->address)  = val; break;
    }
    // call update function
    if (def->updateFunction != NULL) def->updateFunction();
  } else {
    Serial.println(F("ERROR: writeConfig: illegal parameter"));    
  }
}


// print all parameters
void printConfigAll(t_configDef * p) {
  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    printConfig(&configUnion.c);
    p++; 
  }
  Serial.println(F("done."));
  // show firmware version after parameter output
  Serial.print(F("BruGi version "));
  Serial.print(VERSION);
  Serial.print(F(" "));
  Serial.println(F(REVISION));
}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod() {

  char * paraName = NULL;
  char * paraValue = NULL;
  
  int32_t val = 0;

  if ((paraName = sCmd.next()) == NULL) {
    // no command parameter, print all config parameters
    printConfigAll((t_configDef *)configListPGM);
  } else if ((paraValue = sCmd.next()) == NULL) {
    // one parameter, print single parameter
    printConfig(getConfigDef(paraName));
  } else {
    // two parameters, set specified parameter
    val = atol(paraValue);
    writeConfig(getConfigDef(paraName), val);
  }
}
//************************************************************************************


void updateAllParameters() {
  recalcMotorStuff();
  initPIDs();
  initIMU();
  initMPUlpf();
  initSensorOrientation();
  initRCPins();
  initRC();
}

void setDefaultParametersAndUpdate() {
  setDefaultParameters();
  updateAllParameters();
}

void writeEEPROM()
{
  bool oldAcc = config.accOutput;
  traceModeType oldTrace = config.trace;
  
  config.accOutput = false; // do not save enabled OAC output mode
  config.trace = TRC_OFF;    
  
  config.crc8 = crcSlow((crc *)&config, sizeof(config)-1); // set proper CRC 
  EEPROM_writeAnything(0, config);
  
  config.accOutput = oldAcc;
  config.trace = oldTrace;
}

void readEEPROM()
{
  EEPROM_readAnything(0, config); 
  if (config.crc8 == crcSlow((crc *)&config, sizeof(config)-1))
  { 
    updateAllParameters();
  } else {
    // crc failed intialize directly here, as readEEPROM is void
    Serial.print(F("EEPROM CRC failed, initialize EEPROM"));
    setDefaultParameters();
    writeEEPROM();
  }
}

void gyroRecalibrate()
{
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
  gyroOffsetCalibration();
  initMPUlpf();
  Serial.println(F("recalibration: done"));
}

void saveBatteryRefVoltage()
{
  // save the actual battery voltage to configuration
  config.refVoltageBat = voltageBat * 100;
}

void printHelpUsage()
{
  Serial.println(F("List of available commands:"));
  Serial.println(F("Explanations are in brackets(), use integer values only !"));
  Serial.println(F(""));
  Serial.println(F("SD    (Set Defaults)"));
  Serial.println(F("WE    (Writes active config to eeprom)"));
  Serial.println(F("RE    (Restores values from eeprom to active config)"));  
  Serial.println(F("GC    (Recalibrates the Gyro Offsets)"));
  Serial.println(F("SBV   (save battery voltage to config)"));
  Serial.println(F("par <parName> <parValue>   (general parameter read/set command)"));
  Serial.println(F("    example usage:"));
  Serial.println(F("       par                     ... list all config parameters"));
  Serial.println(F("       par gyroPitchKi         ... list gyroPitchKi"));
  Serial.println(F("       par gyroPitchKi 12000   ... set gyroPitchKi to 12000"));
  Serial.println(F(""));
  Serial.println(F("VER    (show version string)"));  
  Serial.println(F("HE     (print this output)"));
  Serial.println(F(""));
  Serial.println(F("Note: command input is case-insensitive, commands are accepted in both upper/lower case"));
}

void unrecognized(const char *command) 
{
  Serial.print(F("'"));
  Serial.print(command);
  Serial.println(F("' unrecognized command, type HE for help ..."));
}


void setSerialProtocol()
{
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("sd", setDefaultParametersAndUpdate);   
  sCmd.addCommand("we", writeEEPROM);   
  sCmd.addCommand("re", readEEPROM); 
  sCmd.addCommand("par", parameterMod);
  sCmd.addCommand("gc", gyroRecalibrate);
  sCmd.addCommand("sbv", saveBatteryRefVoltage);

  sCmd.addCommand("he", printHelpUsage);
  
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}

