#ifndef _UNIVERSIAL_FUNCTION_
#define _UNIVERSIAL_FUNCTION_

#define _ALWAYS_INLINE_ __attribute__((always_inline))


int MOD_range(int value, int range_max) _ALWAYS_INLINE_;
int MOD_range(int value, int range_max)
{
	// speed up of "index=k%mI_MaxStep;", and % operator has problem with int
	while(value>=range_max)
		value-=range_max;
	while(value<0)
		value+=range_max;
	return value;
}

#endif