#ifndef ADC_HPP
#define ADC_HPP

// Using BCM2835 library to establish SPI connection
#include "bcm2835.h"
#include "stdio.h"
#include <iostream>
#include <vector>

using namespace std;

/* Send SPI command to ADC to start reading and send back
result. Results are saved as unsigned 16-bit integers.
Return result */

int startADCSPI();
int stopADCSPI();
vector<uint16_t> readADC();

#endif