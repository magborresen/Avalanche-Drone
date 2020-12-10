#include "Kontrol.hpp"



double calc_Angle(double FFT_output1, double FFT_output2, double FFT_phase1, double FFT_phase2){ // input from the FFT
	double H = sqrt(pow(FFT_output1,2)+pow(FFT_output2,2)); // calculating the length of the H-field
	std::cout << H;
	std::cout << "\n";
	double angle = FFT_output1/H; // calculating the input for acos func.
	std::cout << "acos input: ";
	std::cout << angle;
	double alpha = acos(angle)*180.0/PI; // the angle we want to find
	if(FFT_phase1 - FFT_phase2 ==0)
		{
			alpha = alpha*-1;
		}
	std::cout << " alpha: ";
	std::cout << alpha;
	return alpha;
}


void cast2complex (double iir_output[], fftw_complex* FFT_input)
{
	for(int i = 0; i<L; i++)
	{
		FFT_input[i][REAL] = iir_output[i];
		FFT_input[i][IMAG] = 0;
	}
}



void do_FFT (fftw_plan* plan, fftw_complex* FFToutput, double *mag , double *phase)
{
	fftw_execute(*plan);
	*mag = sqrt(FFToutput[457][REAL]*FFToutput[457][REAL] + FFToutput[457][IMAG]*FFToutput[457][IMAG]);
	*phase = atan2(FFToutput[457][IMAG],FFToutput[457][REAL]);
}

void input_sim (double *out1, double *out2)
{
	for(int i = 0; i < L; ++i) {
		out1[i] = 0.5*SIGVOL*TOT_G*sin(2*M_PI*CFREQ/FS*i);
		out2[i] = 0.5*SIGVOL*TOT_G*sin(2*M_PI*CFREQ/FS*i);
	}
}