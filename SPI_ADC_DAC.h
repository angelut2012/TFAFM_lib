// pass test 20150227 by JC
//speed max=57.4 kHz, min =6 kHz
#ifndef _SPI_DAC_INCLUDED
#define _SPI_DAC_INCLUDED
#include <DigitalIO.h>
#include <SPI.h>
#include "constant_define.h"

#define _ALWAYS_INLINE_ __attribute__((always_inline))
//#define Z_PIEZO_REVERSE	(true)// z axis move direction is up when voltage increases.
////#define Z_PIEZO_REVERSE	fales
////
//////#define Z_PIEZO_REVERSE_POSITIONING(value)	{do{if(Z_PIEZO_REVERSE==true) value=BIT18MAX-value;}while(0);}
////uint32_t Z_PIEZO_REVERSE_POSITIONING(byte axis, uint32_t  value) _ALWAYS_INLINE_;
////uint32_t Z_PIEZO_REVERSE_POSITIONING(byte axis, uint32_t  value) 
////{
////#if(Z_PIEZO_REVERSE==true) 
////	if(axis==PIEZO_Z)
////		return BIT18MAX-value;
////	else
////		return value;
////#else
////	return value;
////#endif
////}

//#ifndef DigitalIO_h
//	#define DAC_PORT_INITIALIZE(pin_number) pinMode((pin_number), OUTPUT)
//	#define SET_PIN_H(x) digitalWrite(x, HIGH)
//	#define SET_PIN_L(x) digitalWrite(x, LOW)
//#else
//	#define PORT_INITIALIZE(pin_number) fastPinMode((pin_number), OUTPUT)
//	#define SET_PIN_H(x) fastDigitalWrite(x, HIGH)
//	#define SET_PIN_L(x) fastDigitalWrite(x, LOW)
//#endif

#define PORT_INITIALIZE(pin_number) fastPinMode((pin_number), OUTPUT)
#define SET_PIN_H(x) fastDigitalWrite(x, HIGH)
#define SET_PIN_L(x) fastDigitalWrite(x, LOW)




#define PDAC0   (28) //DAC SS on pin 28
#define PDAC1   (29)
#define PDAC2   (30)
#define PDAC3   (31)
#define NUM_OF_DAC (4)
const int pdac[4] = {PDAC0,PDAC1, PDAC2, PDAC3};

void DAC_initialize(void);
void DAC_initialize(byte clock_devider);
void DAC_write(byte channel, uint32_t value); // long is 4 bytes
////////////////////////////////
#define PADC0   (24) //DAC SS on pin 28
#define PADC1   (25)
#define PADC2   (26)
#define PADC3   (27)
#define PADC_16   (55)

#define ADC_PORT_PRC (4)
#define ADC_PORT_TUNING_FORK (0)
// select z sensor
//#define ADC_PORT_ZlOOP_SENSOR ADC_PORT_PRC
//#define ADC_PORT_ZlOOP_SENSOR ADC_PORT_TUNING_FORK

#define NUM_OF_ADC (5)
const int padc[NUM_OF_ADC] = {PADC0,PADC1, PADC2, PADC3,PADC_16};




////////////////////////////////
void DAC_initialize(void)
{
	DAC_initialize(1);
}
void DAC_initialize(byte clock_devider)
{
	for (int p = 0; p < NUM_OF_DAC; p++)
		PORT_INITIALIZE(pdac[p]);
	SPI.begin(4);  // initialize SPI:
	SPI.setClockDivider(4, clock_devider);
	SPI.setDataMode(4, SPI_MODE1);
	//delay(50);
	//default is 4MHz
	SPI.setBitOrder(4, MSBFIRST); //To set MSB or LSB first.
	//MAX: CPOL0, CPHA1 = SPI_MODE1
	//To change clock phase [MAX: Change on rising edge] and polarity [MAX: Idles low] (see SPI Lib reference)
}
//////////////////////////////////
void DAC_write(byte channel, uint32_t value) // long is 4 bytes
{
	//value=Z_PIEZO_REVERSE_POSITIONING(channel,value);

	if (channel > NUM_OF_DAC)
		Serial.println("DAC channel number error!");
	SET_PIN_L(pdac[channel]);
	value <<= 2;
	byte v3 = (byte)(value & 0x000000ff);
	value >>= 8;
	byte v2 = (byte)(value & 0x000000ff);
	value >>= 8;
	byte v1 = (byte)(value & 0x0000000f);
	v1 |= 0x10;
	SPI.transfer(4, v1, SPI_CONTINUE);
	SPI.transfer(4, v2, SPI_CONTINUE);
	SPI.transfer(4, v3, SPI_LAST);
	SET_PIN_H(pdac[channel]);
	//Serial.print(v1,BIN);
	//Serial.print(v2,BIN);
	//Serial.print(v3,BIN);
	//    data format
	//    B0001****
	//    B********
	//    B******00;
	//    SPI.transfer(4,B00011100,SPI_CONTINUE);
	//    SPI.transfer(4,B10101010,SPI_CONTINUE);
	//    SPI.transfer(4,B10101010,SPI_CONTINUE);
	//    SPI.transfer(4,B10101010,SPI_LAST);

}
/////////////////////////////AD-DA
void ADC_DAC_initialize(void);
void ADC_DAC_initialize(byte clock_devider);
void ADC_read_DAC_write(byte ADC_channel, uint32_t * pADC_value,byte DAC_channel, uint32_t DAC_value);
////////
void ADC_DAC_initialize(void)
{
	ADC_DAC_initialize(1);
}
void ADC_DAC_initialize(byte clock_devider)
{
	for (int p = 0; p < NUM_OF_DAC; p++)
		PORT_INITIALIZE(pdac[p]);
	for (int p = 0; p < NUM_OF_ADC; p++)
	{
		PORT_INITIALIZE(padc[p]);
		SET_PIN_L(padc[p]);
	}
	SPI.begin(4);  // initialize SPI:
	SPI.setClockDivider(4, clock_devider);
	SPI.setDataMode(4, SPI_MODE1);
	//delay(50);
	//default is 4MHz
	SPI.setBitOrder(4, MSBFIRST); //To set MSB or LSB first.
	//MAX: CPOL0, CPHA1 = SPI_MODE1
	//To change clock phase [MAX: Change on rising edge] and polarity [MAX: Idles low] (see SPI Lib reference)
}
//void ADC_StartConvert(byte ADC_channel)
//{
//	//SET_PIN_H(padc[ADC_channel]);
//	//delayMicroseconds(2);
//	SET_PIN_L(padc[ADC_channel]);
//}
void ADC_read_DAC_write(byte ADC_channel, uint32_t * pADC_value,byte DAC_channel, uint32_t DAC_value) // long, unsigned int is 4 bytes
{
	if ((DAC_channel > NUM_OF_DAC-1) | (ADC_channel > NUM_OF_ADC-1) )
		Serial.println("channel number error!");

	//DAC_value=Z_PIEZO_REVERSE_POSITIONING(DAC_channel,DAC_value);

	SET_PIN_L(padc[ADC_channel]);

	SET_PIN_L(pdac[DAC_channel]);

	DAC_value <<= 2;
	byte v3 = (byte)(DAC_value & 0x000000ff);
	DAC_value >>= 8;
	byte v2 = (byte)(DAC_value & 0x000000ff);
	DAC_value >>= 8;
	byte v1 = (byte)(DAC_value & 0x0000000f);
	v1 |= 0x10;

	uint32_t adc_value_temp[3] = {0};

	//SPI.transfer(4, v1, SPI_CONTINUE);
	//SPI.transfer(4, v2, SPI_CONTINUE);
	//SPI.transfer(4, v3, SPI_LAST);
	adc_value_temp[0] =  SPI.transfer(4, v1,SPI_CONTINUE);
	adc_value_temp[1] =  SPI.transfer(4, v2,SPI_CONTINUE);
	adc_value_temp[2] =  SPI.transfer(4, v3,SPI_LAST);

	SET_PIN_H(padc[ADC_channel]);// start next convert
	SET_PIN_H(pdac[DAC_channel]);

	uint32_t outADC_value=0;//CLEAR
	//outADC_value += adc_value_temp[0] <<10;
	//outADC_value += adc_value_temp[1] << 2;
	//outADC_value += adc_value_temp[2]>>6;
	outADC_value += adc_value_temp[0] <<16;
	outADC_value += adc_value_temp[1] <<8;
	outADC_value += adc_value_temp[2];
	outADC_value>>=6;//(24-18=6)

	if(ADC_channel==ADC_PORT_PRC)// for PRC adc16, set last 2 zero
		outADC_value&=0x3FFFC;//0xFFFFFF00;

	*pADC_value=outADC_value;
}

uint32_t ADC_read_old(byte ADC_channel) // long, unsigned int is 4 bytes
{
	if ((ADC_channel > NUM_OF_ADC-1) )
		Serial.println("channel number error!");

	SET_PIN_L(padc[ADC_channel]);
	uint32_t adc_value_temp[3] = {0};

	adc_value_temp[0] =  SPI.transfer(4, 0,SPI_CONTINUE);
	adc_value_temp[1] =  SPI.transfer(4, 0,SPI_CONTINUE);
	adc_value_temp[2] =  SPI.transfer(4, 0,SPI_LAST);

	uint32_t outADC_value=0;//CLEAR
	//outADC_value += adc_value_temp[0] <<10;
	//outADC_value += adc_value_temp[1] << 2;
	//outADC_value += adc_value_temp[2]>>6;
	outADC_value += adc_value_temp[0] <<16;
	outADC_value += adc_value_temp[1] <<8;
	outADC_value += adc_value_temp[2];
	outADC_value>>=6;//(24-18=6)

	if(ADC_channel==ADC_PORT_PRC)// for PRC adc16, set last 2 zero
		outADC_value&=0x3FFFC;//0xFFFFFF00;


	SET_PIN_H(padc[ADC_channel]);// start next convert
	return outADC_value;
}


//void	ADC_read_start_convert(byte ADC_channel)_ALWAYS_INLINE_;
#define	ADC_read_start_convert(ADC_channel) SET_PIN_H(padc[ADC_channel]) // start next convert

uint32_t ADC_read_data_back(byte ADC_channel) // long, unsigned int is 4 bytes
{
	if ((ADC_channel > NUM_OF_ADC-1) )
		Serial.println("channel number error!");

	SET_PIN_L(padc[ADC_channel]);
	uint32_t adc_value_temp[3] = {0};

	adc_value_temp[0] =  SPI.transfer(4, 0,SPI_CONTINUE);
	adc_value_temp[1] =  SPI.transfer(4, 0,SPI_CONTINUE);
	adc_value_temp[2] =  SPI.transfer(4, 0,SPI_LAST);

	uint32_t outADC_value=0;//CLEAR
	//outADC_value += adc_value_temp[0] <<10;
	//outADC_value += adc_value_temp[1] << 2;
	//outADC_value += adc_value_temp[2]>>6;
	outADC_value += adc_value_temp[0] <<16;
	outADC_value += adc_value_temp[1] <<8;
	outADC_value += adc_value_temp[2];
	outADC_value>>=6;//(24-18=6)

	if(ADC_channel==ADC_PORT_PRC)// for PRC adc16, set last 2 zero
		outADC_value&=0x3FFFC;//0xFFFFFF00;
	return outADC_value;
}
uint32_t ADC_read(byte ADC_channel)// read current and start next convertion
{
	uint32_t data=ADC_read_data_back(ADC_channel);
	ADC_read_start_convert(ADC_channel);// start next
	return data;
}
uint32_t ADC_start_convert_delay_read(byte ADC_channel, int delay_time_us)
{
	ADC_read_start_convert(ADC_channel);
	delayMicroseconds(delay_time_us);
	return ADC_read_data_back(ADC_channel);
}

//uint32_t ADC_read_16(byte ADC_channel=4) // long, unsigned int is 4 bytes
//{
//	if ((ADC_channel > NUM_OF_ADC-1) )
//		Serial.println("channel number error!");
//
//	SET_PIN_L(padc[ADC_channel]);
//	uint32_t adc_value_temp[3] = {0};
//
//	adc_value_temp[0] =  SPI.transfer(4, 0);
//	adc_value_temp[1] =  SPI.transfer(4, 0);
//	//adc_value_temp[2] =  SPI.transfer(4, 0);
//
//
//
//	//unsigned int ADC_value = 0;
//	uint32_t outADC_value=0;//CLEAR
//	//lost the lowest bit when spi communication
//	outADC_value += adc_value_temp[0] <<8;
//	outADC_value += adc_value_temp[1] << 3;
//	//outADC_value += adc_value_temp[2]>>5;
//	//*pADC_value += adc_value_temp[0] << 10;
//	//*pADC_value += adc_value_temp[1] << 2;
//	//*pADC_value += adc_value_temp[2]>>6;
//	SET_PIN_H(padc[ADC_channel]);// start next convert
//	return outADC_value;
//}


#endif



//  //Write data [R3 R2 R1 R0 D17 D16 D15 ... D1 D0 X X]
//  //DIN register write 0001...
////  SPI.transfer(4,B00010100);
////  SPI.transfer(4,B00000000);
////  SPI.transfer(4,B00000000);
////      delay(1);
////  //OFFSET register write 0010...
//  //SPI.transfer(4,B00100000);
//  //SPI.transfer(4,B00000000);
//  //SPI.transfer(4,B00000000);
//
//  //Gain register write 0011...
//  //SPI.transfer(4,00111111);
//  //SPI.transfer(4,11111111);
//  //SPI.transfer(4,11111100);
//
//
//  //Config register write 0100...
//  SPI.transfer(4,01000111);//Normal mode,bus hold disabled,normal operation,Busy input disabled
//  SPI.transfer(4,10000000);//[output disabled
//  SPI.transfer(4,00000000);
//      delay(1);
// digitalWrite(PDAC1, HIGH);