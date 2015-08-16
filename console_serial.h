#ifndef _CONSOLE_SERIAL_INCLUDED_
#define _CONSOLE_SERIAL_INCLUDED_

#define LENGTH_COM_BUFFER_PC2MCU (2+6+2)//10
#define LENGTH_COM_BUFFER_FRAME (LENGTH_COM_BUFFER_PC2MCU-4)
// send back to PC
#define LENGTH_COM_BUFFER_MCU2PC (2+12+2)//16
#define LENGTH_IMAGE_FRAME_BUFFER (LENGTH_COM_BUFFER_MCU2PC*3)

#define COM_HEADER1 (0xAA)//170
#define COM_HEADER2 (0x55)//85
#define COM_TAIL1 (0x55)
#define COM_TAIL2 (0xAA)

#define _ALWAYS_INLINE_ __attribute__((always_inline))


//////////// send image package back tp PC///////////////////
///////send one byte each time
class CPackageToPC
{

private:
	byte * com_image_frame_buffer;
	int pointer_out_frame_buffer;
	int Length_Of_Buffer;
	int rtos_buffer_in_index;
public:
	int frame_number;
public:
	CPackageToPC(int input_frame_number)
	{
		frame_number=input_frame_number;
		Length_Of_Buffer=LENGTH_COM_BUFFER_MCU2PC*frame_number;	
		rtos_buffer_in_index=0;
		pointer_out_frame_buffer=Length_Of_Buffer;
		com_image_frame_buffer=new byte[Length_Of_Buffer];
	};
private:
	~CPackageToPC(void)
	{
		delete com_image_frame_buffer;
	};

	void Serial_write_reset_input_output()
	{rtos_buffer_in_index=0;pointer_out_frame_buffer=0;};

	void Serial_write(byte *d,int L)
	{
		for (int k=0;k<L;k++)
			Serial_write(d[k]);
	};
	//inline void Serial_write(byte d) 
	inline void Serial_write(byte d)
	{
		com_image_frame_buffer[rtos_buffer_in_index]=d;
		rtos_buffer_in_index++;
	} _ALWAYS_INLINE_;
	void prepare_package_to_PC_sub(byte *BufferByte12)
	{
		Serial_write(COM_HEADER1);
		Serial_write(COM_HEADER2);
		Serial_write(BufferByte12,12);	
		Serial_write(COM_TAIL1);
		Serial_write(COM_TAIL2);
	};
public:
	int rtos_send_image_frame_to_PC()
	{
		if (pointer_out_frame_buffer<Length_Of_Buffer)
		{
			Serial.write(com_image_frame_buffer[pointer_out_frame_buffer]);
			pointer_out_frame_buffer++;
		}
		return pointer_out_frame_buffer;
	};
	void prepare_package_Byte12_to_PC(byte *BufferByte12)
	{
		Serial_write_reset_input_output();
		for (int k=0;k<frame_number;k++)
			prepare_package_to_PC_sub(BufferByte12);
	};


};
#endif