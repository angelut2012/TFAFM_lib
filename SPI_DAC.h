// pass test 20150227 by JC
//speed max=57.4 kHz, min =6 kHz
#ifndef _SPI_DAC_INCLUDED
#define _SPI_DAC_INCLUDED
#include <DigitalIO.h>
#include <SPI.h>

#define PDAC0   (28) //DAC SS on pin 28
#define PDAC1   (29)
#define PDAC2   (30)
#define PDAC3   (31)
#define NUM_OF_DAC (4)

#ifndef DigitalIO_h
	#define DAC_PORT_INITIALIZE(pin_number) pinMode((pin_number), OUTPUT)
	#define SET_PIN_H(x) digitalWrite(x, HIGH)
	#define SET_PIN_L(x) digitalWrite(x, LOW)
#else
	#define DAC_PORT_INITIALIZE(pin_number) fastPinMode((pin_number), OUTPUT)
	#define SET_PIN_H(x) fastDigitalWrite(x, HIGH)
	#define SET_PIN_L(x) fastDigitalWrite(x, LOW)
#endif
void DAC_initialize(void);
void DAC_initialize(byte clock_devider);
void DAC_write(byte channel, long value); // long is 4 bytes

const int pdac[4] = {PDAC0,PDAC1, PDAC2, PDAC3};
void DAC_initialize(void)
{
	DAC_initialize(1);
}
void DAC_initialize(byte clock_devider)
{
  for (int p = 0; p < NUM_OF_DAC; p++)
    DAC_PORT_INITIALIZE(pdac[p]);
  SPI.begin(4);  // initialize SPI:
  SPI.setClockDivider(4, clock_devider);
  SPI.setDataMode(4, SPI_MODE1);
  //delay(50);
  //default is 4MHz
  SPI.setBitOrder(4, MSBFIRST); //To set MSB or LSB first.
  //MAX: CPOL0, CPHA1 = SPI_MODE1
  //To change clock phase [MAX: Change on rising edge] and polarity [MAX: Idles low] (see SPI Lib reference)
}
void DAC_write(byte channel, long value) // long is 4 bytes
{
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