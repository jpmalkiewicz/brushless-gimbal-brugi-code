/*************************/
/* RC-Decoder            */
/*************************/

// pinChange Int driven Functions
void intDecodePWMRoll()
{ 
  uint32_t microsNow = micros();
  uint16_t pulseInPWMtmp;
  
  stackCheck(); // debugging purpose

  if (PCintPort::pinState==HIGH)
  {
    microsRisingEdgeRoll = microsNow;
  }
  else
  {
    pulseInPWMtmp = (microsNow - microsRisingEdgeRoll)/CC_FACTOR;
    if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) 
    {
      // update if within expected RC range
      pulseInPWMRoll = pulseInPWMtmp;
      microsLastPWMRollUpdate = microsNow;
      validRCRoll=true;
      updateRCRoll=true;
    }
  }
}

void intDecodePWMPitch()
{ 
  uint32_t microsNow = micros();
  uint16_t pulseInPWMtmp;

  stackCheck(); // debugging purpose

  if (PCintPort::pinState==HIGH)
  {
    microsRisingEdgePitch = microsNow;
  }
  else
  {
    pulseInPWMtmp = (microsNow - microsRisingEdgePitch)/CC_FACTOR;
    if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) 
    {
      // update if within expected RC range
      pulseInPWMPitch = pulseInPWMtmp;
      microsLastPWMPitchUpdate = microsNow;
      validRCPitch=true;
      updateRCPitch=true;
    }
  }
}

void checkPWMRollTimeout()
{
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPWMRollUpdate;
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_PPM_TIMEOUT) 
  {
    pulseInPWMRoll = MID_RC;
    microsLastPWMRollUpdate = microsNow;
    validRCRoll=false;
    updateRCRoll=true;
  }
}

void checkPWMPitchTimeout()
{
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPWMPitchUpdate;
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_PPM_TIMEOUT) 
  {
    pulseInPWMPitch = MID_RC;
    microsLastPWMPitchUpdate = microsNow;
    validRCPitch=false;
    updateRCPitch=true;
  }
}

void initRCPins()
{  
  pinMode(RC_PIN_ROLL, INPUT); digitalWrite(RC_PIN_ROLL, HIGH);
  PCintPort::attachInterrupt(RC_PIN_ROLL, &intDecodePWMRoll, CHANGE);
  pinMode(RC_PIN_PITCH, INPUT); digitalWrite(RC_PIN_PITCH, HIGH);
  PCintPort::attachInterrupt(RC_PIN_PITCH, &intDecodePWMPitch, CHANGE);
}

// Proportional RC control
void evaluateRCSignalProportional()
{
  #define RCSTOP_ANGLE 5.0

  if(updateRCPitch==true)
  {
    pulseInPWMPitch = constrain(pulseInPWMPitch,MIN_RC,MAX_RC);
    if(pulseInPWMPitch>=MID_RC+RC_DEADBAND)
    {
      pitchRCSpeed = config.rcGain * (float)(pulseInPWMPitch - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * pitchRCSpeed;
    }
    else if(pulseInPWMPitch<=MID_RC-RC_DEADBAND)
    {
      pitchRCSpeed = -config.rcGain * (float)((MID_RC - RC_DEADBAND) - pulseInPWMPitch)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * pitchRCSpeed;
    }
    else
    {
      pitchRCSpeed = 0.0;
    }
    pitchRCSpeed = constrain(pitchRCSpeed, -200, +200);
    updateRCPitch=false;
  }
  if(updateRCRoll==true)
  {
    pulseInPWMRoll = constrain(pulseInPWMRoll,MIN_RC,MAX_RC);
    if(pulseInPWMRoll>=MID_RC+RC_DEADBAND)
    {
      rollRCSpeed = config.rcGain * (float)(pulseInPWMRoll - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * rollRCSpeed;
    }
    else if(pulseInPWMRoll<=MID_RC-RC_DEADBAND)
    {
      rollRCSpeed = -config.rcGain * (float)((MID_RC - RC_DEADBAND) - pulseInPWMRoll)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * rollRCSpeed;
    }
    else
    {
      rollRCSpeed = 0.0;
    }
    rollRCSpeed = constrain(rollRCSpeed, -200, +200);
    updateRCRoll=false;
  }
}

// Absolute RC control
void evaluateRCSignalAbsolute()
{
  // Get Setpoint from RC-Channel if available.
  // LPF on pitchSetpoint
  if(updateRCPitch==true)
  {
    pulseInPWMPitch = constrain(pulseInPWMPitch,MIN_RC,MAX_RC);
    pitchRCSetpoint = 0.1 * (config.minRCPitch + (float)(pulseInPWMPitch - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCPitch - config.minRCPitch)) + 0.9 * pitchRCSetpoint;
    updateRCPitch=false;
  }
  if(updateRCRoll==true)
  {
    pulseInPWMRoll = constrain(pulseInPWMRoll,MIN_RC,MAX_RC);
    rollRCSetpoint = 0.1 * (config.minRCRoll + (float)(pulseInPWMRoll - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCRoll - config.minRCRoll)) + 0.9 * rollRCSetpoint;
    updateRCRoll=false;
  }
}

















