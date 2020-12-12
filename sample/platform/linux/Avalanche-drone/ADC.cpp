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
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	return 0;
}

//readADC takes in number of samples to perform in a row and return the result in a uint16 vector
vector<uint16_t> readADC(int numberOfSamples) {
	vector<uint16_t> result;
	char buf[numberOfSamples*2] = {0};

	uint32_t RXCnt=0;

	volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;

	char txEmpty = 0;
 

	/* Clear TX and RX fifos */
	bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

	while(true){
		/* Set TA = 1 */
		bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);
		while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < 2 ))
		{
			rbuf[RXCnt] = bcm2835_peri_read_nb(fifo);
			RXCnt++;
		}
		/* Set TA = 0 */
		bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
		//while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE));
	}
    

	/*
	// read from first ADC
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	while(true){
		bcm2835_spi_transfern(buf, 2);
	}
	

	for (int i = 0; i < numberOfSamples/2; i++)
	{
		result.push_back( ((buf[i] << 8) + buf[i+1]) >> 2 );
	}
	*/
	return result;
	
}

int closeADCSPI() {
	bcm2835_spi_end();
	bcm2835_close();
	return 0;
}

