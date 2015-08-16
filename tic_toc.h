#ifndef _TIC_TOC_INCLUDED_
#define _TIC_TOC_INCLUDED_

#define _MICRO_TIMER_ENABLE_


#ifndef _MICRO_TIMER_ENABLE_
#define TIC()
#define TOC()
#define TIC_P()
#define TOC_P()

#define TICX()
#define TOCX()
#define TICX_P()
#define TOCX_P()

#else
#define TIC() tic_toc(0)
#define TOC() tic_toc(1)
#define TIC_P()	{Serial.println("TIC");TIC();}
//#define TIC_P	do{Serial.println("TIC");TIC;}while(0)
#define TOC_P()	{long t=TOC();Serial.print("TOC");Serial.println(t,DEC);} 
long tic_toc(int start0_end1)
{
	static long t=0;
	if (start0_end1==0)
		t=micros();
	else
		t= micros()-t;
	return t;
}

////////////
#define TICX(n) tic_tocX(0,(n))
#define TOCX(n) tic_tocX(1,(n))
#define TICX_P(n)	{Serial.println("TIC");TICX((n));}
//#define TIC_P	do{Serial.println("TIC");TIC;}while(0)
#define TOCX_P(n)	{long t=TOCX((n));Serial.print("TOC");Serial.println(t,DEC);} 
#define MAX_NUMBER_OF_MICRO_TIMER (10)
long tic_tocX(int start0_end1, int n)
{
	static long t[MAX_NUMBER_OF_MICRO_TIMER]={0};
	if (start0_end1==0)
		t[n]=micros();
	else
		t[n]= micros()-t[n];
	return t[n];
}
//long evaluate_function_time(void (*pfun)())
//{
//	TIC;
//	pfun();
//	long t= TOC;
//	Serial.println(t,DEC);
//}

#define PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(dt) period_check_time_us_due(4,dt)

#define PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(dt) period_check_time_us_due(3,dt)

#define PERIOD_CHECK_TIME_US_DUE_Z_REAL_TIME_SERVO_LOOP(dt) period_check_time_us_due(2,dt)
#define PERIOD_CHECK_TIME_US_DUE_ZSCANNERENGAGE(dt) period_check_time_us_due(1,dt)
#define PERIOD_CHECK_TIME_US_DUE_APPROACH(dt) period_check_time_us_due(0,dt)
bool period_check_time_us_due(int index,  unsigned long dt)
	// do not use this function directly, to avoid multi use at different places
{
	static unsigned long time_store[5] ={0};	
	unsigned long time_now=micros();	
	if((time_now - time_store[index])<(dt))
		return false;
	else 
	{
		time_store[index]=time_now;
		return true;
	}
}

void Software_Reset() 
{
	const int RSTC_KEY = 0xA5;
	RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
	while (true);
}

#endif
#endif