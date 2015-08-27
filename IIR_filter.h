#ifndef _IIR_FILTER_
#define  _IIR_FILTER_
//LPF
//define number of 2nd-order stages
#define N_IIR (3)
double IIR_filter(double input,int order,
				  double* buf0,double* buf1, 
				  const double* a0,const double* a1,
				  const double *b0,const double * b1, const double *b2)
{
	double wn=0;
	double result = 0; //initialize the accumulator
	//Biquad section filtering stage-by-stage
	//using a double accumulator.
	for (int i = 0; i < order; i++)
	{
		//2nd-order LCCDE code
		wn = input - a0[i] * buf0[i] - a1[i] * buf1[i];
		result = b0[i]*wn + b1[i]*buf0[i] + b2[i]*buf1[i];
		//Update filter buffers for stage i
		buf1[i]=buf0[i];
		buf0[i] = wn;
		input = result; /*in case we have to loop again*/
	}
	//result *= scalevalue; //Apply cascade final stage scale factor
	return result;
}

double IIR_filter_HPF(double input)
{

	const double Ha0[N_IIR]=
	{0.226395357685043,
	0.0187037918273417,
	-0.0404326893752033,
	};
	const double Ha1[N_IIR]=
	{0.606633166651684,
	0.153377180801087,
	0
	};

	const double Hb0[N_IIR]=
	{0.509810680512777,
	0.335401664868460,
	0.520216344687602
	};
	const double Hb1[N_IIR]=
	{-0.360616447941088,
	-0.463870059236825,
	-0.520216344687602
	};
	const double Hb2[N_IIR]=
	{
		0.509810680512777,
		0.335401664868460,
		0
	};

	static double Hbuf0[N_IIR],Hbuf1[N_IIR]; //buffer for delay samples LPF
	return IIR_filter(input,N_IIR,Hbuf0,Hbuf1,Ha0,Ha1,Hb0,Hb1,Hb2);
}
double IIR_filter_LPF(double input)
{
	const double La0[N_IIR]=
	{-0.428276325883684,
	-0.168010775478271,
	-0.0244586047360019
	};
	const double La1[N_IIR]=
	{0.614919704567223,
	0.158497913712364,
	0
	};

	const double Lb0[N_IIR]=
	{0.226039837317933,
	0.378144670742543,
	0.487770697631999
	};
	const double Lb1[N_IIR]=
	{
		0.226039837317933,
		0.378144670742543,
		0.487770697631999
	};
	const double Lb2[N_IIR]=
	{
		0.480301770682803,
		0.306171233745775,
		0
	};
	static double Lbuf0[N_IIR],Lbuf1[N_IIR]; //buffer for delay samples LPF
	return IIR_filter(input,N_IIR,Lbuf0,Lbuf1,La0,La1,Lb0,Lb1,Lb2);
}


#endif