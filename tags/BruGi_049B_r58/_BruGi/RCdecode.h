/*************************/
/* RC-Decoder            */
/*************************/

// init RC config variables
void initRC() {
  rcLPF_tc = LOWPASS_K_FLOAT(config.rcLPF*0.1);
}

// pinChange Int driven Functions

// RC Channel 0
void intDecodePWM_Ch0()
{ 
  uint32_t microsNow = micros();
  uint16_t pulseInPWMtmp;

  char chIdx=0;

  if (PCintPort::pinState==HIGH)
  {
    microsRisingEdge[chIdx]= microsNow;
  }
  else
  {
    pulseInPWMtmp = (microsNow - microsRisingEdge[chIdx])/CC_FACTOR;
    if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) 
    {
      // update if within expected RC range
      rcRxChannel[chIdx] = pulseInPWMtmp;
      microsLastPWMUpdate[chIdx] = microsNow;
      validRC[chIdx]=true;
      updateRC[chIdx]=true;
    }
  }
}

// RC Channel 1
void intDecodePWM_Ch1()
{ 
  uint32_t microsNow = micros();
  uint16_t pulseInPWMtmp;

  char chIdx=1;

  if (PCintPort::pinState==HIGH)
  {
    microsRisingEdge[chIdx]= microsNow;
  }
  else
  {
    pulseInPWMtmp = (microsNow - microsRisingEdge[chIdx])/CC_FACTOR;
    if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) 
    {
      // update if within expected RC range
      rcRxChannel[chIdx] = pulseInPWMtmp;
      microsLastPWMUpdate[chIdx] = microsNow;
      validRC[chIdx]=true;
      updateRC[chIdx]=true;
    }
  }
}

// check for RC timout
void checkPWMTimeout(char channelNum)
{
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPWMUpdate[channelNum];
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_TIMEOUT) 
  {
    microsLastPWMUpdate[channelNum] = microsNow;
    rcRxChannel[channelNum] = config.rcMid;
    validRC[channelNum]=false;
    updateRC[channelNum]=true;
  }
}


//******************************************
// PPM Decoder
//******************************************
void intDecodePPM()
{ 
  int32_t microsNow = micros();
  
  static int32_t microsPPMLastEdge = 0;
  uint16_t pulseInPPM;

  static char channel_idx = 0;
  static bool rxPPMvalid = false;

  // PinChangeInt prolog = pin till here = 17 us 
  
  // 45us
  pulseInPPM = (microsNow - microsPPMLastEdge)/CC_FACTOR;
  microsPPMLastEdge = microsNow;

  if (pulseInPPM > RC_PPM_GUARD_TIME) 
  {
    // sync detected 
    channel_idx = 0;
    if (rxPPMvalid)
    {
      microsLastPPMupdate = microsNow;
    }
    rxPPMvalid = false;
  }
  else if (channel_idx < RC_PPM_RX_MAX_CHANNELS-1)
  {      
    if ((pulseInPPM >= MIN_RC) && (pulseInPPM <= MAX_RC)) 
    {
      rcRxChannel[channel_idx] = pulseInPPM;
      validRC[channel_idx] = true;
      updateRC[channel_idx] = true;
      channel_idx++;
    }  
    else
    {
      rxPPMvalid = false;
    }
  }
}

void checkPPMTimeout()
{
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPPMupdate;
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_TIMEOUT) 
  {
    for (char i=0; i<RC_PPM_RX_MAX_CHANNELS; i++)
    {
      rcRxChannel[i] = config.rcMid;
      validRC[i] = false;
      updateRC[i] = true;
    }
    microsLastPPMupdate = microsNow;
  }
}


// initialize RC Pin mode
void initRCPins()
{  
  if (config.rcModePPM)
  {
    pinMode(RC_PIN_ROLL, INPUT); digitalWrite(RC_PIN_ROLL, HIGH);
    PCintPort::attachInterrupt(RC_PIN_ROLL, &intDecodePPM, RISING);
  }
  else
  {
    pinMode(RC_PIN_ROLL, INPUT); digitalWrite(RC_PIN_ROLL, HIGH);
    PCintPort::attachInterrupt(RC_PIN_ROLL, &intDecodePWM_Ch0, CHANGE);
    pinMode(RC_PIN_PITCH, INPUT); digitalWrite(RC_PIN_PITCH, HIGH);
    PCintPort::attachInterrupt(RC_PIN_PITCH, &intDecodePWM_Ch1, CHANGE);
  }
}

// Proportional RC control
void evaluateRCSignalProportional()
{
  int16_t rxData;
  
  #define RCSTOP_ANGLE 5.0

  rxData = rcRxChannel[config.rcChannelPitch];
  if(updateRC[config.rcChannelPitch]==true)
  {
    if(rxData >= config.rcMid+RC_DEADBAND)
    {
      pitchRCSpeed = config.rcGain * (float)(rxData - (config.rcMid + RC_DEADBAND))/ (float)(MAX_RC - (config.rcMid + RC_DEADBAND)) + 0.9 * pitchRCSpeed;
    }
    else if(rxData <= config.rcMid-RC_DEADBAND)
    {
      pitchRCSpeed = -config.rcGain * (float)((config.rcMid - RC_DEADBAND) - rxData)/ (float)((config.rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * pitchRCSpeed;
    }
    else
    {
      pitchRCSpeed = 0.0;
    }
    pitchRCSpeed = constrain(pitchRCSpeed, -200, +200);  // constrain for max speed
    updateRC[config.rcChannelPitch] = false;
  }
  
  rxData = rcRxChannel[config.rcChannelRoll];
  if(updateRC[config.rcChannelRoll]==true)  {
    if(rxData >= config.rcMid+RC_DEADBAND)
    {
      rollRCSpeed = config.rcGain * (float)(rxData - (config.rcMid + RC_DEADBAND))/ (float)(MAX_RC - (config.rcMid + RC_DEADBAND)) + 0.9 * rollRCSpeed;
    }
    else if(rxData <= config.rcMid-RC_DEADBAND)
    {
      rollRCSpeed = -config.rcGain * (float)((config.rcMid - RC_DEADBAND) - rxData)/ (float)((config.rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * rollRCSpeed;
    }
    else
    {
      rollRCSpeed = 0.0;
    }
    rollRCSpeed = constrain(rollRCSpeed, -200, +200);  // constrain for max speed
    updateRC[config.rcChannelRoll] = false;
  }
}

// Absolute RC control
void evaluateRCSignalAbsolute()
{
    int16_t rxData;
    
  // Get Setpoint from RC-Channel if available.
  // LPF on pitchSetpoint
  rxData = rcRxChannel[config.rcChannelPitch];
  if(updateRC[config.rcChannelPitch]==true)
  {
    utilLP_float(&pitchRCSetpoint, (config.minRCPitch + (float)(rxData - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCPitch - config.minRCPitch)), 0.05);
    updateRC[config.rcChannelPitch] = false;
  }

  // LPF on rollSetpoint
  rxData = rcRxChannel[config.rcChannelRoll];
  if(updateRC[config.rcChannelRoll]==true)
  {
    utilLP_float(&rollRCSetpoint, (float)(config.minRCRoll + (float)(rxData - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCRoll - config.minRCRoll)), 0.05);
    updateRC[config.rcChannelRoll] = false;
  }
}

















