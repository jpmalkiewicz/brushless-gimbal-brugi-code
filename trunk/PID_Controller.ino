void Init_pid1() {
  
  /* Init PID 1 */
  lastInput_pid1 = Input_pid1;
  ITerm_pid1 = Output_pid1;
  if(ITerm_pid1 > outMax_pid1) ITerm_pid1= outMax_pid1;
  else if(ITerm_pid1 < outMin_pid1) ITerm_pid1= outMin_pid1;
   
  SetSampleTime_pid1(100);
  SetParameter_pid1(P_Roll,I_Roll,D_Roll);
  
}

void Init_pid2() {
  
  /* Init PID 1 */
  lastInput_pid2 = Input_pid2;
  ITerm_pid2 = Output_pid2;
  if(ITerm_pid2 > outMax_pid2) ITerm_pid2= outMax_pid2;
  else if(ITerm_pid2 < outMin_pid2) ITerm_pid2= outMin_pid2;
   
  SetSampleTime_pid2(100);
  SetParameter_pid2(P_Nick,I_Nick,D_Nick);
  
}

void Compute_pid1()
{
   unsigned long now_pid1 = millis();
   int timeChange_pid1 = (now_pid1 - lastTime_pid1);
   if(timeChange_pid1>=SampleTime_pid1)
   {
      /*Compute all the working error variables*/
      float error_pid1 = Setpoint_pid1 - Input_pid1;
      ITerm_pid1+= (ki_pid1 * error_pid1);
      if(ITerm_pid1 > outMax_pid1) ITerm_pid1= outMax_pid1;
      else if(ITerm_pid1 < outMin_pid1) ITerm_pid1= outMin_pid1;
      float dInput_pid1 = (Input_pid1 - lastInput_pid1);
 
      /*Compute PID Output*/
      Output_pid1 = kp_pid1 * error_pid1 + ITerm_pid1- kd_pid1 * dInput_pid1;
  
 
      /*Remember some variables for next time*/
      lastInput_pid1 = Input_pid1;
      lastTime_pid1 = now_pid1;    
    
   }
}

void Compute_pid2()
{
   unsigned long now_pid2 = millis();
   int timeChange_pid2 = (now_pid2 - lastTime_pid2);
   if(timeChange_pid2>=SampleTime_pid2)
   {
      /*Compute all the working error variables*/
      float error_pid2 = Setpoint_pid2 - Input_pid2;
      ITerm_pid2+= (ki_pid2 * error_pid2);
      if(ITerm_pid2 > outMax_pid2) ITerm_pid2= outMax_pid2;
      else if(ITerm_pid2 < outMin_pid2) ITerm_pid2= outMin_pid2;
      float dInput_pid2 = (Input_pid2 - lastInput_pid2);
 
      /*Compute PID Output*/
      Output_pid2 = kp_pid2 * error_pid2 + ITerm_pid2- kd_pid2 * dInput_pid2;
 
      /*Remember some variables for next time*/
      lastInput_pid2 = Input_pid2;
      lastTime_pid2 = now_pid2;
   }
}

void SetParameter_pid1(float Kp_pid1, float Ki_pid1, float Kd_pid1)
{
  float SampleTimeInSec_pid1 = ((float)SampleTime_pid1)/1000;
   kp_pid1 = Kp_pid1;
   ki_pid1 = Ki_pid1 * SampleTimeInSec_pid1;
   kd_pid1 = Kd_pid1 / SampleTimeInSec_pid1;
}

void SetParameter_pid2(float Kp_pid2, float Ki_pid2, float Kd_pid2)
{
  float SampleTimeInSec_pid2 = ((float)SampleTime_pid2)/1000;
   kp_pid2 = Kp_pid2;
   ki_pid2 = Ki_pid2 * SampleTimeInSec_pid2;
   kd_pid2 = Kd_pid2 / SampleTimeInSec_pid2;
}
  
void SetSampleTime_pid1(int NewSampleTime_pid1)
{
   if (NewSampleTime_pid1 > 0)
   {
      float ratio_pid1  = (float)NewSampleTime_pid1
                      / (float)SampleTime_pid1;
      ki_pid1 *= ratio_pid1;
      kd_pid1 /= ratio_pid1;
      SampleTime_pid1 = (unsigned long)NewSampleTime_pid1;
   }
}

void SetSampleTime_pid2(int NewSampleTime_pid2)
{
   if (NewSampleTime_pid2 > 0)
   {
      float ratio_pid2  = (float)NewSampleTime_pid2
                      / (float)SampleTime_pid2;
      ki_pid2 *= ratio_pid2;
      kd_pid2 /= ratio_pid2;
      SampleTime_pid2 = (unsigned long)NewSampleTime_pid2;
   }
}


