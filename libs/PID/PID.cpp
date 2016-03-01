/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Modified by rtlopez <rtlopezdev@gmail.com>
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
	inAuto = false;
	
	SetOutputLimits(0, 255, 0.6); //default output limit corresponds to the arduino pwm limits

  SampleTime = 100;			   //default Controller Sample Time is 0.1 seconds

  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd);

  lastTime = millis() - SampleTime;
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = now - lastTime;
   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/
      error = *mySetpoint - *myInput;
      double dInput = (*myInput - lastInput);
      double dSetpoint = (*mySetpoint - lastSetpoint);

      PTerm = kp * error;
      LimitPTerm();

      ITerm += (ki * error);
      LimitITerm();

      //DTerm = - kd * dInput;
      DTerm = - kd * dInput + 0.01 * kd * dSetpoint;
      LimitDTerm();
 
      /*Compute PID Output*/
      //*myOutput = kp * error + ITerm - kd * dInput;
      //*myOutput = kp * error + ITerm - kd * dInput + 0.005 * kd * dSetpoint;
      *myOutput = PTerm + ITerm + DTerm;
      LimitOutput();
	  
      /*Remember some variables for next time*/
      lastSetpoint = *mySetpoint;
      lastInput = *myInput;
      lastTime = now;

	    return true;
   }
   return false;
}

void PID::LimitITerm()
{
  //ITerm = ApplyLimit(ITerm, outMin, outMax);
  ITerm = ApplyLimit(ITerm, outMin * outRate, outMax * outRate);
}

void PID::LimitPTerm()
{
  PTerm = ApplyLimit(PTerm, outMin, outMax);
  //PTerm = ApplyLimit(PTerm, outMin * outRate, outMax * outRate);
}

void PID::LimitDTerm()
{
  DTerm = ApplyLimit(DTerm, outMin, outMax);
  //DTerm = ApplyLimit(DTerm, outMin * outRate, outMax * outRate);
}

void PID::LimitOutput()
{
  *myOutput = ApplyLimit(*myOutput, outMin, outMax);
}

double PID::ApplyLimit(double v, double min, double max)
{
  if(v > max) return max;
  else if(v < min) return min;
  else return v;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
    
   kp = Kp;
   ki = Ki * (SampleTime / 1000.0);
   kd = Kd / (SampleTime / 1000.0);
 
   if(controllerDirection == REVERSE)
   {
      kp = -kp;
      ki = -ki;
      kd = -kd;
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max, double Rate)
{
    if(Min >= Max) return;
    outMin = Min;
    outMax = Max;
    outRate = Rate;
    if(inAuto)
    {
      LimitPTerm();
      LimitITerm();
      LimitDTerm();
      LimitOutput();
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
    LimitOutput();
    ITerm = *myOutput;
    lastInput = *myInput;
    lastSetpoint = *mySetpoint;
    LimitPTerm();
    LimitITerm();
    LimitDTerm();
    LimitOutput();
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = -kp;
      ki = -ki;
      kd = -kd;
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() { return  dispKp; }
double PID::GetKi() { return  dispKi; }
double PID::GetKd() { return  dispKd; }
int PID::GetMode() { return  inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() { return controllerDirection; }
