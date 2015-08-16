#ifndef _PIEZO_FEED_FORWARD_
#define _PIEZO_FEED_FORWARD_
#include "constant_define.h"


double data_polyFit(int axis, double position_input )
{
	double dOutputVoltage=0; 
	//const 	double p1 = 0.388672361415601;
	//const 	double p2 = -0.302985646005988;
	//const    double p3 = 0.907347050211084;
	//const    double p4 = 0.003683583151077; 

	const 	double p1[]={0.382364269054315 ,  0.414344751801367 ,  0.379784211770158};
	const 	double p2[]={  -0.322245171965016 , -0.315853467579674 , -0.305993313910179};
	const 	double p3[]={   0.942222481094926 ,  0.914194677705042  , 0.927866407877878};
	const 	double p4[]={   0.000512055566326 , -0.007787467649116  , 0.000795511360525};

	///	cf(x) = p1*x^3 + p2*x^2 + p3*x + p4; 
	dOutputVoltage+=p4[axis];
	dOutputVoltage+=p3[axis]*position_input;
	position_input*=position_input;	
	dOutputVoltage+=p2[axis]*position_input;
	position_input*=position_input;	
	dOutputVoltage+=p1[axis]*position_input;

	//dOutputVoltage = p1*pow( position_input, 3.0) + p2*pow( position_input,2.0) + p3*position_input + p4;
	return dOutputVoltage; 
}
/// Description: determinate the voltage output based on feedforward model 

//#define MAX_STEPS_PIEZO_MODEL (3) //(1000.0/85/3)// each time use=85us, 3 axes, max steps=3
//uint32_t piezo_predict_Position2Voltage_DAC_sysmetric(int axis,double PositionInput,int steps)
//	// input output range 0~1
//	// this function is non-re_enter_able
//{
//	//alpha: 0.4000
//	// beta: 9.1500
//	//gamma: 1.5000
//	//   dp: 1.0030
//	//    n: 1
//
//	const	double alpha=0.4,beta=9.15,gamma=1.5,dp=1.003;
//
//	static double H_now[NUM_OF_PIEZO_MODEL]={0};
//	static double PositionNow[NUM_OF_PIEZO_MODEL]={0};
//
//	double deltaPosition = PositionInput - PositionNow[axis];
//	double temp_PositionInput=PositionInput;
//	double model_outputV=0;
//	uint32_t value_dac=0;
//	//double max_step_size=max(MAX_STEP_SIZE_PIEZO_MODEL,(deltaPosition/MAX_STEPS_PIEZO_MODEL+0.001));
//	double max_step_size=max(MAX_STEP_SIZE_PIEZO_MODEL,(deltaPosition/steps+0.001));
//
//	do
//	{
//		//fastDigitalWrite(23,true);
//		if (abs(deltaPosition)>max_step_size)
//			if (deltaPosition>0)
//			{
//				temp_PositionInput=PositionNow[axis]+max_step_size;
//				deltaPosition=max_step_size;
//			}
//			else
//			{
//				temp_PositionInput=PositionNow[axis]-max_step_size;
//				deltaPosition=-max_step_size;
//			}
//			//double H_now = H_now;
//			//H_now = H_now.^n;
//			double dH = (alpha/dp*deltaPosition - beta*H_now[axis]*deltaPosition*SIGN(deltaPosition) - gamma*abs(H_now[axis])*deltaPosition) /
//				(1-alpha/dp+beta*H_now[axis]*SIGN(deltaPosition)+gamma*abs(H_now[axis]) );
//			//	% dH = alpha*dV-beta*abs(dV)*H(k)-gamma*dV*abs(H(k));   //   % dV * (alpha - (gamma + beta * SIGN(dV * H)) * abs(H)^n);
//			H_now[axis] += dH;
//			PositionNow[axis] = temp_PositionInput;
//			//	% y(k) = dp.*ExcitationVoltage(k) -H_now;
//			//ExcitationVoltage=(temp_PositionInput+H_now)/dp;
//			model_outputV=(temp_PositionInput+H_now[axis])/dp;
//
//			//Serial.print("dH ");
//			//Serial.println(dH,DEC);
//			//Serial.print("H_now[axis] ");
//			//Serial.println(H_now[axis],DEC);
//
//			model_outputV+= data_polyFit(axis, temp_PositionInput)-temp_PositionInput;
//
//			//Serial.print("model_outputV after");
//			//Serial.println(model_outputV,DEC);
//
//			model_outputV=Max(model_outputV,0);
//			model_outputV=Min(model_outputV,1);
//
//			deltaPosition = PositionInput - PositionNow[axis];
//			//if (axis>0)
//			value_dac=(uint32_t)(model_outputV*(double)BIT18MAX);
//			DAC_write(axis, value_dac);
//
//
//			//Serial.print("value_dac ");
//			//Serial.println(value_dac,DEC);
//			//
//			//Serial.print("deltaPosition ");
//			//Serial.println(deltaPosition,DEC);
//			//fastDigitalWrite(23,false);
//	}
//	while (abs(deltaPosition)>max_step_size);
//
//	return value_dac;
//}
//uint32_t piezo_predict_Position2Voltage_DAC_variant_step_number(int axis,double PositionInput,int steps)
//	// input output range 0~1
//	// this function is non-re_enter_able
//{
//	//alpha: 0.4000
//	// beta: 9.1500
//	//gamma: 1.5000
//	//   dp: 1.0030
//	//    n: 1
////	pmdl.alpha = 0.35;
////pmdl.beta = 7;
////pmdl.gamma = 4;
////pmdl.dp = 1.09;
////pmdl.n = 1.02;
//	const	double alpha=0.35,beta=7,gamma=4,dp=1.09;
//
//	static double H_now[NUM_OF_PIEZO_MODEL]={0};
//	static double PositionNow[NUM_OF_PIEZO_MODEL]={0};
//
//	double deltaPosition = PositionInput - PositionNow[axis];
//	double temp_PositionInput=PositionInput;
//	double model_outputV=0;
//	uint32_t value_dac=0;
//	//double max_step_size=max(MAX_STEP_SIZE_PIEZO_MODEL,(deltaPosition/MAX_STEPS_PIEZO_MODEL+0.001));
//	double max_step_size=max(MAX_STEP_SIZE_PIEZO_MODEL,(deltaPosition/steps+0.001));
//
//	do
//	{
//		//fastDigitalWrite(23,true);
//		if (abs(deltaPosition)>max_step_size)
//			if (deltaPosition>0)
//			{
//				temp_PositionInput=PositionNow[axis]+max_step_size;
//				deltaPosition=max_step_size;
//			}
//			else
//			{
//				temp_PositionInput=PositionNow[axis]-max_step_size;
//				deltaPosition=-max_step_size;
//			}
//		//double H_now = H_now;
//		//H_now = H_now.^n;
//		double dH = (alpha/dp*deltaPosition - beta*H_now[axis]*deltaPosition*SIGN(deltaPosition) - gamma*abs(H_now[axis])*deltaPosition) /
//			(1-alpha/dp+beta*H_now[axis]*SIGN(deltaPosition)+gamma*abs(H_now[axis]) );
//		//	% dH = alpha*dV-beta*abs(dV)*H(k)-gamma*dV*abs(H(k));   //   % dV * (alpha - (gamma + beta * SIGN(dV * H)) * abs(H)^n);
//		H_now[axis] += dH;
//		PositionNow[axis] = temp_PositionInput;
//		//	% y(k) = dp.*ExcitationVoltage(k) -H_now;
//		//ExcitationVoltage=(temp_PositionInput+H_now)/dp;
//		model_outputV=(temp_PositionInput+H_now[axis])/dp;
//
//		//Serial.print("dH ");
//		//Serial.println(dH,DEC);
//		//Serial.print("H_now[axis] ");
//		//Serial.println(H_now[axis],DEC);
//
//		//model_outputV+= data_polyFit(axis, temp_PositionInput)-temp_PositionInput;
//
//		//Serial.print("model_outputV after");
//		//Serial.println(model_outputV,DEC);
//
//		model_outputV=Max(model_outputV,0);
//		model_outputV=Min(model_outputV,1);
//
//		deltaPosition = PositionInput - PositionNow[axis];
//		//if (axis>0)
//		value_dac=(uint32_t)(model_outputV*(double)BIT18MAX);
//		DAC_write(axis, value_dac);
//
//
//		//Serial.print("value_dac ");
//		//Serial.println(value_dac,DEC);
//		//
//		//Serial.print("deltaPosition ");
//		//Serial.println(deltaPosition,DEC);
//		//fastDigitalWrite(23,false);
//	}
//	while (abs(deltaPosition)>max_step_size);
//
//	return value_dac;
//}

uint32_t piezo_predict_Position01_To_Voltage_DAC18(int axis,double PositionInput,uint32_t* pV18_Adc_value)//,int steps
	// input output range 0~1
	// this function is non-re_enter_able
{	//alpha: 0.4000
	// beta: 9.1500
	//gamma: 1.5000
	//   dp: 1.0030
	//    n: 1
	//	pmdl.alpha = 0.35;
	//pmdl.beta = 7;
	//pmdl.gamma = 4;
	//pmdl.dp = 1.09;
	//pmdl.n = 1.02;
	const	double alpha=0.35,beta=7,gamma=4,dp=1.09;

	static double H_now[NUM_OF_PIEZO_MODEL]={0};
	static double PositionNow[NUM_OF_PIEZO_MODEL]={0};

	double deltaPosition = PositionInput - PositionNow[axis];
	double temp_PositionInput=PositionInput;
	double model_outputV=0;
	uint32_t value_dac=0;
	double temp_deltaPosition=0;
	double dH=0;

	while (abs(deltaPosition)>EPS)// 85us each round
	{
		//fastDigitalWrite(23,true);
		if (abs(deltaPosition)<MAX_STEP_SIZE_PIEZO_MODEL)
			temp_deltaPosition=deltaPosition;// last step
		else// equal steps
		{
			if (deltaPosition>0)// move direction
				temp_deltaPosition=MAX_STEP_SIZE_PIEZO_MODEL;
			else
				temp_deltaPosition=-MAX_STEP_SIZE_PIEZO_MODEL;
		}
		temp_PositionInput=PositionNow[axis]+temp_deltaPosition;
		deltaPosition-=temp_deltaPosition;
		//double H_now = H_now;
		//H_now = H_now.^n;
		dH = (alpha/dp*temp_deltaPosition - beta*H_now[axis]*temp_deltaPosition*SIGN(temp_deltaPosition) - gamma*abs(H_now[axis])*temp_deltaPosition) /
			(1-alpha/dp+beta*H_now[axis]*SIGN(temp_deltaPosition)+gamma*abs(H_now[axis]) );
		//	% dH = alpha*dV-beta*abs(dV)*H(k)-gamma*dV*abs(H(k));   //   % dV * (alpha - (gamma + beta * SIGN(dV * H)) * abs(H)^n);
		H_now[axis] += dH;
		PositionNow[axis] = temp_PositionInput;
		//	% y(k) = dp.*ExcitationVoltage(k) -H_now;
		//ExcitationVoltage=(temp_PositionInput+H_now)/dp;
		model_outputV=(temp_PositionInput+H_now[axis])/dp;

		//Serial.print("dH ");
		//Serial.println(dH,DEC);
		//Serial.print("H_now[axis] ");
		//Serial.println(H_now[axis],DEC);

		//model_outputV+= data_polyFit(axis, temp_PositionInput)-temp_PositionInput;

		//Serial.print("model_outputV after");
		//Serial.println(model_outputV,DEC);

		model_outputV=Max(model_outputV,0);
		model_outputV=Min(model_outputV,1);

		//deltaPosition = PositionInput - PositionNow[axis];
		//if (axis>0)
		value_dac=(uint32_t)(model_outputV*(double)BIT18MAX);
		//DAC_write(axis, value_dac);		
		ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac);// speed up for both read and write
		//fastDigitalWrite(23,false);
		//delay(1);
	}

	//Serial.print("value_dac ");
	//Serial.println(value_dac,DEC);
	//
	//Serial.print("deltaPosition ");
	//Serial.println(deltaPosition,DEC);
	//fastDigitalWrite(23,false);

	//		while (abs(deltaPosition)>max_step_size);
	
	// model reset
	if (PositionInput<=EPS)
	{
		H_now[axis]=0;
		PositionNow[axis]=0;
		value_dac=0;
		//DAC_write(axis, value_dac);
		ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac);// speed up for both read and write
	}
	return value_dac;
}

uint32_t piezo_predict_Position01_To_Voltage_DAC18_compatible(int axis,double PositionInput)//,int steps
	// input output range 0~1
	// this function is non-re_enter_able
{
	uint32_t V18_Adc_value=0;// not used
	return piezo_predict_Position01_To_Voltage_DAC18(axis,PositionInput,&V18_Adc_value);
}

uint32_t piezo_predict_Position01_To_Voltage_DAC18(int axis,double PositionInput)//,int steps
{	// input output range 0~1
	// this function is non-re_enter_able
	
	//alpha: 0.4000
	// beta: 9.1500
	//gamma: 1.5000
	//   dp: 1.0030
	//    n: 1
	//	pmdl.alpha = 0.35;
	//pmdl.beta = 7;
	//pmdl.gamma = 4;
	//pmdl.dp = 1.09;
	//pmdl.n = 1.02;
	const	double alpha=0.35,beta=7,gamma=4,dp=1.09;

	static double H_now[NUM_OF_PIEZO_MODEL]={0};
	static double PositionNow[NUM_OF_PIEZO_MODEL]={0};
	
	double deltaPosition = PositionInput - PositionNow[axis];
	double temp_PositionInput=PositionInput;
	double model_outputV=0;
	uint32_t value_dac=0;
	double temp_deltaPosition=0;
	double dH=0;	
	
	//////////////////////////////////////////////////////////// reset model
	// model reset
	if (PositionInput<=EPS)
	{
		H_now[axis]=0;
		PositionNow[axis]=0;
		value_dac=0;
		DAC_write(axis, value_dac);
		//ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac);// speed up for both read and write
		return value_dac;
	}

	////////////////////////////////////////////////////////////// normail feedforward

	while (abs(deltaPosition)>EPS)// 85us each round
	{
		//fastDigitalWrite(23,true);
		if (abs(deltaPosition)<MAX_STEP_SIZE_PIEZO_MODEL)
			temp_deltaPosition=deltaPosition;// last step
		else// equal steps
		{
			if (deltaPosition>0)// move direction
				temp_deltaPosition=MAX_STEP_SIZE_PIEZO_MODEL;
			else
				temp_deltaPosition=-MAX_STEP_SIZE_PIEZO_MODEL;
		}
		temp_PositionInput=PositionNow[axis]+temp_deltaPosition;
		deltaPosition-=temp_deltaPosition;
		//double H_now = H_now;
		//H_now = H_now.^n;
		dH = (alpha/dp*temp_deltaPosition - beta*H_now[axis]*temp_deltaPosition*SIGN(temp_deltaPosition) - gamma*abs(H_now[axis])*temp_deltaPosition) /
			(1-alpha/dp+beta*H_now[axis]*SIGN(temp_deltaPosition)+gamma*abs(H_now[axis]) );
		//	% dH = alpha*dV-beta*abs(dV)*H(k)-gamma*dV*abs(H(k));   //   % dV * (alpha - (gamma + beta * SIGN(dV * H)) * abs(H)^n);
		H_now[axis] += dH;
		PositionNow[axis] = temp_PositionInput;
		//	% y(k) = dp.*ExcitationVoltage(k) -H_now;
		//ExcitationVoltage=(temp_PositionInput+H_now)/dp;
		model_outputV=(temp_PositionInput+H_now[axis])/dp;

		//Serial.print("dH ");
		//Serial.println(dH,DEC);
		//Serial.print("H_now[axis] ");
		//Serial.println(H_now[axis],DEC);

		//model_outputV+= data_polyFit(axis, temp_PositionInput)-temp_PositionInput;

		//Serial.print("model_outputV after");
		//Serial.println(model_outputV,DEC);

		model_outputV=Max(model_outputV,0);
		model_outputV=Min(model_outputV,1);

		//deltaPosition = PositionInput - PositionNow[axis];
		//if (axis>0)
		value_dac=(uint32_t)(model_outputV*(double)BIT18MAX);
		DAC_write(axis, value_dac);		
		//ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac);// speed up for both read and write
		//fastDigitalWrite(23,false);
		//delay(1);
	}

	//Serial.print("value_dac ");
	//Serial.println(value_dac,DEC);
	//
	//Serial.print("deltaPosition ");
	//Serial.println(deltaPosition,DEC);
	//fastDigitalWrite(23,false);

	//		while (abs(deltaPosition)>max_step_size);
	

}
#endif