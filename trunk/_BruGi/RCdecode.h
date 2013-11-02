/*************************/
/* RC-Decoder            */
/*************************/

// init RC config variables
void initRC() 
{
  rcLPFPitch_tc = LOWPASS_K_FLOAT(config.rcLPFPitch*0.1);
  rcLPFRoll_tc = LOWPASS_K_FLOAT(config.rcLPFRoll*0.1);
}


//******************************************
// PWM Decoder
//******************************************

// pinChange Int driven Functions

inline void decodePWM(rcData_t* rcData)
{
  uint16_t pulseInPWMtmp;

  if (PCintPort::pinState==HIGH)
  {
    rcData->microsRisingEdge = microsNow;
  }
  else
  {
    rcData->microsLastUpdate = microsNow;
    pulseInPWMtmp = microsNow - rcData->microsRisingEdge;
    if ((pulseInPWMtmp >= MIN_RC_VALID) && (pulseInPWMtmp <= MAX_RC_VALID)) 
    {
      // update if within expected RC range
      rcData->rx = pulseInPWMtmp;
      rcData->valid=true;
      rcData->update=true;
    } else {
      rcData->rx     = config.rcMid;
      rcData->valid  = false;
      rcData->update = true;
    }
  }
}


//******************************************
// PPM Decoder
//******************************************

inline void intDecodePPM()
{ 

  static int32_t microsPPMLastEdge = 0;
  uint16_t pulseInPPM;

  static char channel_idx = 0;

  pulseInPPM = microsNow - microsPPMLastEdge;
  microsPPMLastEdge = microsNow;

  if (pulseInPPM > RC_PPM_GUARD_TIME) 
  {
    channel_idx = 0;
  }
  else if (channel_idx < RC_PPM_RX_MAX_CHANNELS)
  {
    rcData_t* data = 0; 
    if ((channel_idx == config.rcChannelPitch) && (config.rcModePPMPitch)) 
      data = &rcData[RC_DATA_PITCH];
    else if ((channel_idx == config.rcChannelRoll) && (config.rcModePPMRoll))
      data = &rcData[RC_DATA_ROLL];
    else if ((channel_idx == config.rcChannelAux) && (config.rcModePPMAux))
      data = &rcData[RC_DATA_AUX];
    if (data)
    {
      data->microsLastUpdate = microsNow;    
      if ((pulseInPPM >= MIN_RC_VALID) && (pulseInPPM <= MAX_RC_VALID)) 
      {
        data->rx     = pulseInPPM;
        data->valid  = true;
        data->update = true;
      } else {
        rcData->rx     = config.rcMid;
        rcData->valid  = false;
        rcData->update = true;
      }
    }
    channel_idx++;
  }

}

//******************************************
// Interrupts
//******************************************

// Connector Channel 1 (A2)
void intDecodePWM_Ch0()
{ 
  microsNow = microsT1();
  sei(); // re-enable interrupts
  
  // PWM: 6 / 10 us (min/max)
  // PPM: 0.5 / 12 us (min/max)
  if (config.rcModePPMPitch || config.rcModePPMRoll || config.rcModePPMAux)
  {
#ifdef RC_PIN_PPM_A2
    if (PCintPort::pinState==HIGH) intDecodePPM();
#endif  
  }
  if ((config.rcChannelRoll == 0) && (config.rcModePPMRoll == false))
    decodePWM(&rcData[RC_DATA_ROLL]);
  if ((config.rcChannelPitch == 0) && (config.rcModePPMPitch == false))
    decodePWM(&rcData[RC_DATA_PITCH]);
  if ((config.rcChannelAux == 0) && (config.rcModePPMAux == false))
    decodePWM(&rcData[RC_DATA_AUX]);
}

// Connector Channel 2 (A1)
void intDecodePWM_Ch1()
{ 
  microsNow = microsT1();
  sei(); // re-enable interrupts

  if (config.rcModePPMPitch || config.rcModePPMRoll || config.rcModePPMAux)
  {
#ifdef RC_PIN_PPM_A1
    if (PCintPort::pinState==HIGH) intDecodePPM();
#endif  
  }
  if ((config.rcChannelRoll == 1) && (config.rcModePPMRoll == false))
    decodePWM(&rcData[RC_DATA_ROLL]);
  if ((config.rcChannelPitch == 1) && (config.rcModePPMPitch == false))
    decodePWM(&rcData[RC_DATA_PITCH]);
  if ((config.rcChannelAux == 1) && (config.rcModePPMAux == false))
    decodePWM(&rcData[RC_DATA_AUX]);
}

// Connector Channel 3 (A0)
void intDecodePWM_Ch2()
{ 
  microsNow = microsT1();
  sei(); // re-enable interrupts
  
  if (config.rcModePPMPitch || config.rcModePPMRoll || config.rcModePPMAux)
  {
#ifdef RC_PIN_PPM_A0
    if (PCintPort::pinState==HIGH) intDecodePPM();
#endif  
  }
  if ((config.rcChannelRoll == 2) && (config.rcModePPMRoll == false))
    decodePWM(&rcData[RC_DATA_ROLL]);
  if ((config.rcChannelPitch == 2) && (config.rcModePPMPitch == false))
    decodePWM(&rcData[RC_DATA_PITCH]);
  if ((config.rcChannelAux == 2) && (config.rcModePPMAux == false))
    decodePWM(&rcData[RC_DATA_AUX]);
}



//******************************************
// PPM & PWM Decoder
//******************************************

// check for RC timout

void checkRcTimeouts()
{
  int32_t microsNow = microsT1();
  int32_t microsLastUpdate;
  for (char id = 0; id < RC_DATA_SIZE; id++)
  {
    cli();
    microsLastUpdate = rcData[id].microsLastUpdate;
    sei();
    if (rcData[id].valid && (microsNow - microsLastUpdate) > RC_TIMEOUT) 
    {
      rcData[id].rx     = config.rcMid;
      rcData[id].valid  = false;
      rcData[id].update = true;
    }
  }
}

// initialize RC Pin mode

void initRCPins()
{  
  static bool first = true;
  if (first)
  {
    pinMode(A2, INPUT); digitalWrite(A2, HIGH);
    PCintPort::attachInterrupt(A2, &intDecodePWM_Ch0, CHANGE);
    pinMode(A1, INPUT); digitalWrite(A1, HIGH);
    PCintPort::attachInterrupt(A1, &intDecodePWM_Ch1, CHANGE);
    pinMode(A0, INPUT); digitalWrite(A0, HIGH);
    PCintPort::attachInterrupt(A0, &intDecodePWM_Ch2, CHANGE);
    first = false;
  }
  for (char id = 0; id < RC_DATA_SIZE; id++)
  {
    cli();
    rcData[id].microsRisingEdge = 0;
    rcData[id].microsLastUpdate = 0;
    rcData[id].rx               = 1500;
    rcData[id].update           = true;
    rcData[id].valid            = true;
    rcData[id].rcSpeed          = 0.0;
    rcData[id].setpoint         = 0.0;
    rcData[id].rcAuxSwitch1     = false;
    rcData[id].rcAuxSwitch2     = false;
    sei();
  }
  
}

//******************************************
// Proportional
//******************************************

void evalRCChannelProportional(rcData_t* rcData, int16_t rcGain, int16_t rcMid)
{
  if(rcData->update == true)
  {
    if(rcData->rx >= rcMid + RC_DEADBAND)
    {
      rcData->rcSpeed = rcGain * (float)(rcData->rx - (rcMid + RC_DEADBAND))/ (float)(MAX_RC - (rcMid + RC_DEADBAND)) + 0.9 * rcData->rcSpeed;
    }
    else if(rcData->rx <= rcMid-RC_DEADBAND)
    {
      rcData->rcSpeed = -rcGain * (float)((rcMid - RC_DEADBAND) - rcData->rx)/ (float)((rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * rcData->rcSpeed;
    }
    else
    {
      rcData->rcSpeed = 0.0;
    }
    rcData->rcSpeed = constrain(rcData->rcSpeed, -200, +200);  // constrain for max speed
    rcData->update = false;
  }
}

// Proportional RC control

void evaluateRCProportionalPitch()
{
  evalRCChannelProportional(&rcData[RC_DATA_PITCH], config.rcGainPitch, config.rcMid);
}
void evaluateRCProportionalRoll()
{
  evalRCChannelProportional(&rcData[RC_DATA_ROLL ], config.rcGainRoll, config.rcMid);
}


//******************************************
// Absolute
//******************************************

inline void evalRCChannelAbsolute(rcData_t* rcData, int16_t rcMin, int16_t rcMax, int16_t rcMid)
{
  float k;
  float y0;
  int16_t rx;
  
  if(rcData->update == true)
  {
    k = (float)(rcMax - rcMin)/(MAX_RC - MIN_RC);
    y0 = rcMin + k * (MID_RC - MIN_RC);
    rx = rcData->rx - rcMid;
    utilLP_float(&rcData->setpoint, y0 + k * rx, 0.05);
    rcData->update = false;
  }
}

// Absolute RC control

void evaluateRCAbsolutePitch()
{
  evalRCChannelAbsolute(&rcData[RC_DATA_PITCH], config.minRCPitch, config.maxRCPitch, config.rcMid);
}
void evaluateRCAbsoluteRoll()
{
  evalRCChannelAbsolute(&rcData[RC_DATA_ROLL ], config.minRCRoll , config.maxRCRoll,  config.rcMid);
}


// auxiliary channel, decode switches
inline void evalRCChannelAux(rcData_t* rcData, int16_t rcSwThresh, int16_t rcMid)
{
  int16_t rx;
  int8_t hyst;
  
 
  if(rcData->valid == true)
  {
    if(rcData->update == true)
    {
      rx = rcData->rx - rcMid;
      utilLP_float(&rcData->setpoint, rx, 0.05);
     
      hyst = rcData->rcAuxSwitch1 ? 0 : 50;
      rcData->rcAuxSwitch1 = (rcData->setpoint > (rcSwThresh+hyst)) ? true : false;
      hyst = rcData->rcAuxSwitch2 ? 0 : 50;
      rcData->rcAuxSwitch2 = (rcData->setpoint < -(rcSwThresh+hyst)) ? true : false;
      rcData->update = false;
    }
  } else {
    rcData->rcAuxSwitch1 = false;
    rcData->rcAuxSwitch2 = false;
  }
}

// auxiliary 
void evaluateRCAux()
{
  evalRCChannelAux(&rcData[RC_DATA_AUX], RC_SW_THRESH, config.rcMid);
}

// decode mode switches
void decodeModeSwitches() {
  // fpv mode
  switch (config.fpvSwPitch) {
    case -1:
      fpvModePitch = true;
      break;
    case 0:
      fpvModePitch = false;
      break;
    case 1: // aux Switch 1
      fpvModePitch = (rcData[RC_DATA_AUX].rcAuxSwitch1) ? true : false;
      break;
    case 2:
      fpvModePitch = (rcData[RC_DATA_AUX].rcAuxSwitch2) ? true : false;
      break;
  }
  switch (config.fpvSwRoll) {
    case -1:
      fpvModeRoll = true;
      break;
    case 0:
      fpvModeRoll = false;
      break;
    case 1: // aux Switch 1
      fpvModeRoll = (rcData[RC_DATA_AUX].rcAuxSwitch1) ? true : false;
      break;
    case 2:
      fpvModeRoll = (rcData[RC_DATA_AUX].rcAuxSwitch2) ? true : false;
      break;
  }

  switch (config.altSwAccTime) {
    case -1:
      altModeAccTime = true;
      break;
    case 0:
      altModeAccTime = false;
      break;
    case 1: // aux Switch 1
      altModeAccTime = (rcData[RC_DATA_AUX].rcAuxSwitch1) ? true : false;
      break;
    case 2:
      altModeAccTime = (rcData[RC_DATA_AUX].rcAuxSwitch2) ? true : false;
      break;
  }


}












