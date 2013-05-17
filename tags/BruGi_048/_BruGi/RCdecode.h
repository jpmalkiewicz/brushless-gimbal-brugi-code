/*************************/
/* RC-Decoder            */
/*************************/

// pinChange Int driven Functions
void intDecodePWMRoll()
{ 
  if (PCintPort::pinState==HIGH)
    microsRisingEdgeRoll = micros();
  else
  {
    pulseInPWMRoll = (micros() - microsRisingEdgeRoll)/CC_FACTOR;
    updateRCRoll=true;
  }
}

void intDecodePWMPitch()
{ 
  if (PCintPort::pinState==HIGH)
    microsRisingEdgePitch = micros();
  else
  {
    pulseInPWMPitch = (micros() - microsRisingEdgePitch)/CC_FACTOR;
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
      pitchRCSpeed = 0.1 * (float)(pulseInPWMPitch - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * pitchRCSpeed;
    }
    else if(pulseInPWMPitch<=MID_RC-RC_DEADBAND)
    {
      pitchRCSpeed = -0.1 * (float)((MID_RC - RC_DEADBAND) - pulseInPWMPitch)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * pitchRCSpeed;
    }
    else pitchRCSpeed = 0.0;
    // if((pitchAngleACC <= (config.minRCPitch+RCSTOP_ANGLE))||(rollAngleACC>=(config.maxRCPitch-RCSTOP_ANGLE))) pitchRCSpeed = 0.0;
    if((pitchAngleACC <= (config.minRCPitch+RCSTOP_ANGLE))&&(pitchRCSpeed > 0.0))pitchRCSpeed = 0.0;
    if((pitchAngleACC >= (config.maxRCPitch-RCSTOP_ANGLE))&&(pitchRCSpeed < 0.0))pitchRCSpeed = 0.0;
    updateRCPitch=false;
  }
  if(updateRCRoll==true)
  {
    pulseInPWMRoll = constrain(pulseInPWMRoll,MIN_RC,MAX_RC);
    if(pulseInPWMRoll>=MID_RC+RC_DEADBAND)
    {
      rollRCSpeed = 0.1 * (float)(pulseInPWMRoll - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * rollRCSpeed;
    }
    else if(pulseInPWMRoll<=MID_RC-RC_DEADBAND)
    {
      rollRCSpeed = -0.1 * (float)((MID_RC - RC_DEADBAND) - pulseInPWMRoll)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * rollRCSpeed;
    }
    else rollRCSpeed = 0.0;
    //if((rollAngleACC <= (config.minRCRoll+RCSTOP_ANGLE))||(rollAngleACC>=(config.maxRCRoll-RCSTOP_ANGLE))) rollRCSpeed = 0.0;
    if((rollAngleACC <= (config.minRCRoll+RCSTOP_ANGLE))&&(rollRCSpeed > 0.0))rollRCSpeed = 0.0;
    if((rollAngleACC >= (config.maxRCRoll-RCSTOP_ANGLE))&&(rollRCSpeed < 0.0))rollRCSpeed = 0.0;
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

















