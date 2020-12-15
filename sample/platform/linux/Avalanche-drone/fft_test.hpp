#ifndef FFT_TEST_HPP
#define FFT_TEST_HPP
#include <fftw3.h>

void acquire_from_somewhere(fftw_complex* signal);
void do_something_with(fftw_complex* result);
int fftTest();

#endif