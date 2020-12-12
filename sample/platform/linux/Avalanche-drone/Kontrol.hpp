#include <iostream>
#include <math.h>
#include <fftw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <vector>

using namespace std;

#define PI 3.14159265
#define N 4096
#define L 4096 //Tager 0,5ms samplingtid at fylde array
#define REAL 0
#define IMAG 1
#define FS 1950000
#define CFREQ 457000
#define SIMANGLE 30*180/M_PI
#define SIGVOL 0.0000000112
#define TOT_G 108500
#define V_N_RMS 0.000052747


double calc_Angle(double FFT_output1, double FFT_output2, double FFT_phase1, double FFT_phase2); // for calculating the angle

void cast2complex (double *iir_output, fftw_complex* FFT_input);

void do_FFT (fftw_plan* plan, fftw_complex* FFToutput, double *mag , double *phase);

void input_sim (double *out1, double *out2);