/**********************************************************************************************
* Arduino PID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v1.h"

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up 
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
		 double Kp, double Ki, double Kd,bool Direction_DirectTrue_ReverseFalse)
{

	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	PID::SetOutputLimits(-131072,131071);				//default output limit corresponds to 
	//the arduino pwm limits

	mSampleTimeIn_us = 1000;							//default Controller Sample Time is 1 us

	//PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd);
	mDirection_DirectTrue_ReverseFalse=Direction_DirectTrue_ReverseFalse;
	//   lastTime = millis()-mSampleTimeIn_us;				
}


/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
double PID::Compute()
{
	// if(!inAuto) return false;
	/*Compute all the working error variables*/
	double input = *myInput;
	double dInput = (input - lastInput);

	error = *mySetpoint - input;
	ITerm+= (ki * error);
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;

	/*Compute PID Output*/
	double output=0;
	if (mDirection_DirectTrue_ReverseFalse==false)
	{//output = kp * error + ITerm- kd * dInput;
		output+= kp * error;
		output+= ITerm;
		output-= kd * dInput;
	}
	else
	{//output =-( kp * error + ITerm- kd * dInput;)
		output-= kp * error;
		output-= ITerm;
		output+= kd * dInput;
	}		

	if(output > outMax) output = outMax;
	else if(output < outMin) output = outMin;
	*myOutput = output;

	/*Remember some variables for next time*/
	lastInput = input;
	// return true;
	//   lastTime = now;
	//return true;
	//}
	//else return false;
	return output;
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
	ki = Ki * mSampleTimeIn_us/1000000;// convert second
	kd = Kd / mSampleTimeIn_us/1000000;

	/*	if(controllerDirection ==REVERSE)
	{
	kp = (0 - kp);
	ki = (0 - ki);
	kd = (0 - kd);
	}
	*/
}

/* SetSampleTime(...) *********************************************************
* sets the period, in micro seconds, at which the calculation is performed	
******************************************************************************/
void PID::SetSampleTime(double NewSampleTimeIn_us)
{
	if (NewSampleTimeIn_us > 0)
	{
		double ratio  = NewSampleTimeIn_us/mSampleTimeIn_us;
		ki *= ratio;
		kd /= ratio;
		mSampleTimeIn_us = NewSampleTimeIn_us;
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
void PID::SetOutputLimits(double Min, double Max)
{
	if(Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if(inAuto)
	{
		if(*myOutput > outMax) *myOutput = outMax;
		else if(*myOutput < outMin) *myOutput = outMin;

		if(ITerm > outMax) ITerm= outMax;
		else if(ITerm < outMin) ITerm= outMin;
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/ 
//void PID::SetMode(int Mode)
//{
//    bool newAuto = (Mode == AUTOMATIC);
//    if(newAuto == !inAuto)
//    {  /*we just went from manual to auto*/
//        PID::Initialize();
//    }
//    inAuto = newAuto;
//}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void PID::Initialize()
{
	ITerm = *myOutput;
	lastInput = *myInput;
	if(ITerm > outMax) ITerm = outMax;
	else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads 
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
/*void PID::SetControllerDirection(int Direction)
{
if(inAuto && Direction !=controllerDirection)
{
kp = (0 - kp);
ki = (0 - ki);
kd = (0 - kd);
}   
controllerDirection = Direction;
}
*/
/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display 
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}


//void PID::FastCompute()
//{
//
//      /*Compute all the working error variables*/
//	  //double (*myInput) = *myInput;
//      double error = *mySetpoint - (*myInput);
//      ITerm+= (ki * error);
//      if(ITerm > outMax) ITerm= outMax;
//      else if(ITerm < outMin) ITerm= outMin;
//      double dInput = ((*myInput) - lastInput);
// 
//      /*Compute PID Output*/
//      double output = kp * error + ITerm- kd * dInput;
//      
//	  if(output > outMax) output = outMax;
//      else if(output < outMin) output = outMin;
//	  *myOutput = output;	  
//      /*Remember some variables for next time*/
//      lastInput = (*myInput);
//}
