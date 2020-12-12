#ifndef ADC_HPP
#define ADC_HPP

//Clk - 23
//data - 21
//select - overfor 23
//5VV - 5V
//GND - GND
//clk = gul
//data = lilla
//sort select
// Using BCM2835 library to establish SPI connection
#include "bcm2835.h"
#include "stdio.h"
#include <iostream>
#include <vector>

using namespace std;

/* Start the SPI interface */
int startADCSPI();

/* Stop the SPI interface */
int stopADCSPI();

/* Reads data from both adc's.
Returns a vector containing results from both adc's
CS0 is saved on the first position
CS1 is saved on the second position
*/
vector<uint16_t> readADC();

#endif