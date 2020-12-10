/* Compile with sudo g++ -o adc ADC.cpp -l bcm2835 */

#include "ADC.hpp"

using namespace std;

int startADCSPI() {
		// Initialize the hardware
	if (!bcm2835_init())
	{
		//printf("bcm2835_init_failed. Are you running as root? \n");
		return 1;
	}
	
	// Start the SPI hardware
	if (!bcm2835_spi_begin())
	{
		//printf("bcm2835_spi_begin failed. Are your running as root? \n");
		return 1;
	}
	
	// Set bit order to MSB first
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	
	// Set SPI mode
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE2);
	
	// Set clock divider
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);
	
	return 0;
}

vector<uint16_t> readADC() {
	
	vector<uint16_t> result;
	char buf[2] = {0};
	
	// read from first ADC
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_transfern(buf, 2);
	result.push_back( ((buf[0] << 8) + buf[1]) >> 2 );
	//cout << "Read from SPI 0: " << result[0] << endl;
	
	// Read from second ADC
	bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
	bcm2835_spi_transfern(buf, 2);
	result.push_back( ((buf[0] << 8) + buf[1]) >> 2 );
	//cout << "Read from SPI 1: " << result[0] << endl;
	return result;
}

int closeADCSPI() {
	bcm2835_spi_end();
	bcm2835_close();
	return 0;
}

