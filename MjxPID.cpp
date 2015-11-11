/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Modified by rtlopez <rtlopezdev@gmail.com>
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "Arduino.h"
#include "MjxPID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
MjxPID::MjxPID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
	inAuto = false;
	
	SetOutputLimits(0, 255); //default output limit corresponds to the arduino pwm limits

  SampleTime = 100;			   //default Controller Sample Time is 0.1 seconds

  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd);

  lastTime = millis() - SampleTime;
  debugTime = millis();
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool MjxPID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = now - lastTime;
   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/
      double error = *mySetpoint - *myInput;

      ITerm += (ki * error);
      FixITerm();
      
      double dInput = (*myInput - lastInput);
      double dSetpoint = (*mySetpoint - lastSetpoint);
 
      /*Compute PID Output*/
      //double output = kp * error + ITerm - kd * dInput;
      *myOutput = kp * error + ITerm - kd * dInput + 0.005 * kd * dSetpoint;
      FixOutput();
	  
      /*Remember some variables for next time*/
      lastSetpoint = *mySetpoint;
      lastInput = *myInput;
      lastTime = now;

      if(0 && now - debugTime > 50)
      {
        const char * sep = reinterpret_cast<const char *>(F(" "));
        Serial.print(F("#"));  Serial.print(now / 1000.0, 2); // 0
        Serial.print(sep); Serial.print(*mySetpoint, 1);      // 1
        Serial.print(sep); Serial.print(*myInput, 1);         // 2
        Serial.print(sep); Serial.print(error, 1);            // 3
        Serial.print(sep); Serial.print(*myOutput, 1);        // 4
        Serial.print(sep); Serial.print(ITerm, 1);            // 5
        Serial.print(sep); Serial.print(dInput, 1);           // 6
        Serial.print(sep); Serial.print(dSetpoint, 1);        // 7
        Serial.println();
        debugTime = now;
      }
	    return true;
   }
   return false;
}

void MjxPID::FixITerm()
{
    const double rate = 0.5;
    if(ITerm > outMax * rate) ITerm = outMax * rate;
    else if(ITerm < outMin * rate) ITerm = outMin * rate;
}

void MjxPID::FixOutput()
{
    if(*myOutput > outMax) *myOutput = outMax;
    else if(*myOutput < outMin) *myOutput = outMin;
}
/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void MjxPID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = SampleTime / 1000.0;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
   if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void MjxPID::SetSampleTime(int NewSampleTime)
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
void MjxPID::SetOutputLimits(double Min, double Max)
{
    if(Min >= Max) return;
    outMin = Min;
    outMax = Max;
    if(inAuto)
    {
      FixOutput();
      FixITerm();
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void MjxPID::SetMode(int Mode)
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
void MjxPID::Initialize()
{
    FixOutput();
    ITerm = *myOutput;
    lastInput = *myInput;
    lastSetpoint = *mySetpoint;
    FixITerm();
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void MjxPID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double MjxPID::GetKp() { return  dispKp; }
double MjxPID::GetKi() { return  dispKi; }
double MjxPID::GetKd() { return  dispKd; }
int MjxPID::GetMode() { return  inAuto ? AUTOMATIC : MANUAL; }
int MjxPID::GetDirection() { return controllerDirection; }

