#ifndef _CONSTATN_DEFINE_
#define _CONSTATN_DEFINE_

#define SIGN(x) ((x>0)-(x<0)) 
#define NUM_OF_PIEZO_MODEL (3)
#define PIEZO_Z (0)
#define PIEZO_X (1)
#define PIEZO_Y (2)
#define PIEZO_T (3)

#define SCANNER_Z_ONLY (0)
#define SCANNER_Z_LPF (1)
#define SCANNER_T_ONLY (2)
#define SCANNER_ZT (3)
//"Zonly",
//"Z_LPF",
//"Tonly",
//"ZT"});



// calibrated by afm grid 5um on optical microscope
//#define MAX_RANGE_Z_NM (21.387973775678940*1000.0)
//#define MAX_RANGE_X_NM (27.067266247186911*1000.0)
//#define MAX_RANGE_Y_NM (27.209844045785250*1000.0)
// devin's new piezo actuator
#define MAX_RANGE_Z_NM ( 7.299788679537674*1000.0)
#define MAX_RANGE_X_NM (27.067266247186911*1000.0)
#define MAX_RANGE_Y_NM (27.209844045785250*1000.0)


//#define MAX_RANGE_Z_NM (25000)
//#define MAX_RANGE_X_NM (22000)
//#define MAX_RANGE_Y_NM (21000)

#define MAX_RANGE_Z_PM (MAX_RANGE_Z_NM*1000)
#define MAX_RANGE_X_PM (MAX_RANGE_X_NM*1000)
#define MAX_RANGE_Y_PM (MAX_RANGE_Y_NM*1000)
#define CONV_PM2DAC(x)  ((x)*((double)BIT18MAX/(double)MAX_RANGE_Z_PM)) //*0.01048576


//#define MAX_STEP_NUMBER (1000000.0/85.0)
#define EPS (0.000001)
#define MAX_STEP_SIZE_PIEZO_MODEL_NM (20)// tuning fork(2.0)//(4.0)
#define MAX_STEP_SIZE_PIEZO_MODEL (MAX_STEP_SIZE_PIEZO_MODEL_NM/MAX_RANGE_Z_NM)//(0.001)//step size=20~27 nm (0.05)




#define LENGTH_I2C_DATA_SG (11)
#define MCU_AD0 (54)// PIN 54 for analog adc0 on MCU


//PID mZ_Loop_PID(&DInput_01, &DOutput_01, &DSet_01,2,5,0, DIRECT);
#define BIT18MAX (262143.0)
#define BIT18MAX_RECIPROCAL  (3.814711817595740e-06)
#define BIT18MAX_HALF (BIT18MAX/2.0)
#define BIT18MAX_0D75 (BIT18MAX*3.0/4.0)
#define BIT32MAX (4294967295.0)
#define BIT24MAX (16777215.0)





# define DAC_PER_NM_Z (BIT18MAX/MAX_RANGE_Z_NM) //10.48572

# define DAC_PER_NM_X (BIT18MAX/MAX_RANGE_X_NM) //
# define DAC_PER_NM_Y (BIT18MAX/MAX_RANGE_Y_NM) //

#define LIMIT_MAX_MIN(x,up,down) (Max(Min(x,up),down))

//#define PERIOD_TIME_CHECK_EXIT(dt) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt))return;	else time_store=time_now;}while(0);}
//#define PERIOD_TIME_CHECK_EXIT(dt) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt)) 	{		time_store=time_now;		return;	}}while(0);}
//#define PERIOD_TIME_CHECK_EXIT_WITH_VALUE(dt,value) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt)) 	{	return (value);	}		time_store=time_now;	}while(0);}
#define DIGITAL_PIN_TOGGLE(pin)	{do{static bool x=true; x=!x;fastDigitalWrite(pin,x);} while(0);}
#endif