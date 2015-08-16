#ifndef _I2C_DRPOT_INCLUDED_
#define _I2C_DRPOT_INCLUDED_
#include <Wire.h>
#define FREQUENCY_MAX (400000)//max 400000
#define FREQUENCY_DEFAULT (FREQUENCY_MAX/2)
void DRpot_initialize( int clock_frequency)
{
  Wire1.setClock(clock_frequency);
  Wire1.begin(); // join i2c bus (address optional for master)
}
void DRpot_initialize()
{
	DRpot_initialize(FREQUENCY_DEFAULT);
}
byte write_digital_potentiometer(byte address_IC, byte address_channel, byte R_value)
{ 
	address_IC = address_IC&0x03; //value=0~3, get the last two bits for chip address

	static byte channel_values[4][6] = {0};
	channel_values[address_IC][address_channel] = R_value; // store the values
// digital output
  bool DO1 = channel_values[address_IC][4] ==1;
  bool DO2 = channel_values[address_IC][5] ==1;
  //Serial.write(DO2); 

  address_IC = address_IC |  B00101100; //add the header;
  Wire1.beginTransmission(address_IC); //94 transmit to device #44 (0x2c)
  address_channel = address_channel&0x03;
  address_channel <<= 5;

  address_channel = address_channel |  ((byte)DO1 << 2);
  address_channel = address_channel |  ((byte)DO2 << 1);
// bit operation test
//byte x=0xf1&0x1f;
//Serial.write(x);
//  Serial.write(address_IC) ;                
//  Serial.write(address_channel) ;
//  Serial.write(R_value) ;

  Wire1.write(address_channel);//command B01100100
  Wire1.write(R_value);
  return Wire1.endTransmission();   // send out data
}

#endif
//new 
//AA 55 52 00 05 01 00 00 55 AA  turn on DO2
//AA 55 52 00 03 F1 00 00 55 AA  DR4 value F1


// old
//AA 55 52 03 f0 55 AA
//AA 55 52 05 01 55 AA turn on IC0_DO2
//AA 55 52 05 00 55 AA turn off IC0_DO2
