#include "fft_test.hpp"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <vector>
#include <fftw3>


using namespace std;

#define N 4096
#define REAL 0
#define IMAG 1
#define FS 3000000
#define CFREQ 457000
#define SIGVOL 0.0000000112
#define TOT_G 108500
#define V_N_RMS 0.000052747
#define M_PI 3.14159265359


void acquire_from_somewhere(fftw_complex* signal) {
	/* Generate two sine waves of different
	frequencies and amplitudes. Also generate noise */
	
	for(int i = 0; i < N; i++) {
		srand(time(NULL));
		
		/* Real part of the signal
		multiplied by the total gain of the system
		the signal is normalized with the sampling frequency */
		signal[i][REAL] = sin(2*M_PI*CFREQ/FS*i)+sin(M_PI*CFREQ/FS*i)+sin(M_PI*CFREQ/FS*i/2)+sin(M_PI*CFREQ/FS*i/4);
		
		// Imaginary part of the signal. Set to 0
		signal[i][IMAG] = 0;
	}
}

void do_something_with(fftw_complex* result) {
	/* Calculate magnitude of FFT result and save each sample
	to a file */
	
	std::ofstream myFile;
	
	myFile.open("result_10.txt", std::ofstream::trunc);
	
	if(!myFile.is_open())
		std::cout << "File is not open \n";
	else
		std::cout << "Result file created \n";
	
	for(int i = 0; i < N; ++i) {
		// Magnitude calculation
		double mag = sqrt(result[i][REAL] * result[i][REAL] + result[i][IMAG] * result[i][IMAG]);
		double phase = atan2(result[i][IMAG], result[i][REAL])* 180 / M_PI;
		myFile << mag << "," << phase << std::endl;
	}
	myFile.close();
}


int fftTest() {
	
	// Create vectors for filling signal and result with length L
	fftw_complex *signal;
	fftw_complex *result;

	signal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	result = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
	
	/* Create FFT plan that will do a 1 dimensional
	N point FFT. It will take the values from signal
	and save into result.
	It is a forward FFT and uses the FFTW_ESTIMATE algorithm */
	fftw_plan plan = fftw_plan_dft_1d(N,
									  signal,
									  result,
									  FFTW_FORWARD,
									  FFTW_ESTIMATE);



	// Fill up the signal with values
	acquire_from_somewhere(signal);
	// Execute the plan
	fftw_execute(plan);
	do_something_with(result);
	// Destroy the plan and free up memory
	fftw_destroy_plan(plan);
	
	return 0;
}
