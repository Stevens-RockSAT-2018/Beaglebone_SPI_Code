/**
 * Zach Shoop
 * 3/28/2018
 * spi.cpp
 * Uses SPI to communicate 6 adc lines
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SETUP_FLAG	10000000	//setup flag is a 1 on the MSBit
#define SCAN_MODE_NONE	00000110	//scan mode for no scanning through
#define SCAN_MODE_0_N	00000000	//scan mode for scanning 0 to channel N
#define SCAN_MODE_N_4	00000100	//scan mode for scanning N to 4

#define GPIOPIN0	"/sys/class/gpio/gpio66"
#define GPIOPIN1	"/sys/class/gpio/gpio67"
#define GPIOPIN2	"/sys/class/gpio/gpio68"
#define GPIOPIN3	"/sys/class/gpio/gpio69"
#define GPIOPIN4	"/sys/class/gpio/gpio44"
#define GPIOPIN5	"/sys/class/gpio/gpio45"


#define MAX11_BUS	1	//connected to /dev/spidev1.x bus
#define	MAX11_FREQ	10	//SPI clock frequency in Hz
#define MAX11_BITS	16	//SPI bits per word
#define MAX11_CLOCKMODE	3	//SPI clock mode
#define MAX11_CS	1	//SPI Chip Select, useless but required
#define MAX_TRANSFER_SIZE	4096
#define SPIDEV_PATH_LEN	20
using namespace std;
static uint8_t running;
int SPI_getMode(int spidev_fd){
 uint8_t mode;
 if(ioctl(spidev_fd,SPI_IOC_RD_MODE, &mode) < 0) return -1;
 return 0;
}
int SPI_setMode(int spidev_fd,uint8_t mode){
 if(ioctl(spidev_fd, SPI_IOC_WR_MODE, &mode) < 0) return -1;
 return 0;
}
int SPI_setBitsPerWord(int spidev_fd, uint8_t bits_per_word){
 if(ioctl(spidev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0){
   return -1;
 }
 return 0;
}
int SPI_setMaxFrequency(int spidev_fd, uint32_t frequency){
 if(ioctl(spidev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequency) < 0) return -1;
 return 0;
}
int SPI_setClockMode(int spidev_fd, uint8_t clock_mode){
 uint8_t mode;
 mode = SPI_getMode(spidev_fd);
 if(mode < 0) return mode;
 mode &= ~0x3;
 mode |= clock_mode & 0x3;
 return SPI_setMode(spidev_fd, mode);
}
int SPI_setCSActiveLow(int spidev_fd){
 int mode = SPI_getMode(spidev_fd);
 if(mode < 0) return -1;
 return SPI_setMode(spidev_fd, mode & ~SPI_CS_HIGH);
}
typedef enum {
 SPI_MSBFIRST,
 SPI_LSBFIRST
} SPI_bit_order;
int SPI_setBitOrder(int spidev_fd, SPI_bit_order bit_order){
 uint8_t order = (uint8_t) bit_order; //Just to be safe
 if (ioctl(spidev_fd, SPI_IOC_WR_LSB_FIRST, &order) < 0) return -1;
 return 0;
}
int SPI_getBitsPerWord(int spidev_fd){
 uint8_t bits_per_word;
 if(ioctl(spidev_fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
  return -1;
 }
 return bits_per_word == 0 ? 8 : bits_per_word;
}

int SPI_write(int spidev_fd, void *tx_buffer, int n_words) {
 uint8_t bits_per_word;
 uint32_t n_bytes;
 struct spi_ioc_transfer transfer;
 bits_per_word = SPI_getBitsPerWord(spidev_fd);
 if(bits_per_word < 0) return bits_per_word;
 n_bytes = (uint32_t) (((float) (bits_per_word * n_words)) / 8.0 + 0.5);
 if(!n_bytes) return 0;
 if(n_bytes > MAX_TRANSFER_SIZE) n_bytes = MAX_TRANSFER_SIZE;

 memset((void *) &transfer, 0, sizeof(struct spi_ioc_transfer));
 transfer.tx_buf = (uintptr_t) tx_buffer;
 transfer.rx_buf = 0;
 transfer.len = n_bytes;
 transfer.speed_hz = 0;
 transfer.delay_usecs = 0;
 transfer.bits_per_word = bits_per_word;
 transfer.cs_change = 0;
 if (ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) return -1;
 return (n_bytes << 3) / bits_per_word;
}

int SPI_read(int spidev_fd, void *tx_buffer, int n_words) {
 uint8_t bits_per_word;
 uint32_t n_bytes;
 struct spi_ioc_transfer transfer;
 bits_per_word = SPI_getBitsPerWord(spidev_fd);
 if(bits_per_word < 0) return bits_per_word;
 n_bytes = (uint32_t) (((float) (bits_per_word * n_words)) / 8.0 + 0.5);
 if(!n_bytes) return 0;
 if(n_bytes > MAX_TRANSFER_SIZE) n_bytes = MAX_TRANSFER_SIZE;

 memset((void *) &transfer, 0, sizeof(struct spi_ioc_transfer));
 transfer.rx_buf = (uintptr_t) tx_buffer;
 transfer.tx_buf = 0;
 transfer.len = n_bytes;
 transfer.speed_hz = 0;
 transfer.delay_usecs = 0;
 transfer.bits_per_word = bits_per_word;
 transfer.cs_change = 0;
 if (ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) return -1;
 return (n_bytes << 3) / bits_per_word;
}
int SPI_open(uint8_t bus, uint8_t cs) {
  char device[SPIDEV_PATH_LEN];
  sprintf(device, "/dev/spidev%d.%d", bus, bus);
  return open(device, O_RDWR, 0);
}
void SPI_close(int spidev_fd){
 close(spidev_fd);
}
/**
 * @brief Sets the SPI bus per the ADC's required configuration
 *
 * @param spi_fd SPI bus file descriptor
 */
void MAX11_SPIConfig(int spi_fd){
	SPI_setMaxFrequency(spi_fd, MAX11_FREQ);
	SPI_setBitsPerWord(spi_fd, MAX11_BITS);
	SPI_setClockMode(spi_fd, MAX11_CLOCKMODE);
	SPI_setCSActiveLow(spi_fd);
	SPI_setBitOrder(spi_fd, SPI_MSBFIRST);
}

/*
 * @brief reads off ADC value from MAX11
 *
 * @param spi_fd SPI bus file descriptor
 * @param channel selects channel(s) to read from
 * @param scanMode selects scanning mode for the arduino
 * @param cs gpio pin for chip select
 */
uint8_t MAX11_setup(int spi_fd, uint8_t channel, uint8_t scanMode, fstream* cs){
	uint8_t regData = SETUP_FLAG | channel << 3 | scanMode;
	*cs << "0";
	uint16_t msgbuf[1] = {regData}; 
	uint16_t message = SPI_write(spi_fd, (void*) msgbuf, 1);
	*cs << "1";
	return message;
}
/*
 * @brief returns message read from SPI device 
 *
 * @param spi_fd SPI bus file descriptor
 * @param cs gpio pin for chip select in file system
 * @param numBytes number of bytes to read off of MAX11
 */
uint16_t MAX11_readmsg(int spi_fd, fstream* cs, int numBytes){
	uint16_t msgbuf[16]; //16 for buffer size = ADC buffer
	uint16_t message = SPI_read(spi_fd, (void*) msgbuf, numBytes);
	return message;
}

/*
 * @brief sets up the chip select pins in the linux file system
 */
void setup_cs_pins(){
	char csPinArray[6][23];
	strcpy(csPinArray[0], GPIOPIN0 "/direction");
	strcpy(csPinArray[1], GPIOPIN1 "/direction");
	strcpy(csPinArray[2], GPIOPIN2 "/direction");
	strcpy(csPinArray[3], GPIOPIN3 "/direction");
	strcpy(csPinArray[4], GPIOPIN4 "/direction");
	strcpy(csPinArray[5], GPIOPIN5 "/direction");
	fstream cspins;
	for(uint8_t i = 0; i < 6; i++){
		cspins.open(csPinArray[i], ofstream::out | ofstream::trunc);
		const char n[] = {'o', 'u', 't'};
		cspins.write(n, sizeof(n));
		//cspins << "out";
		cspins.close();
	}
	/*
	fstream cspins;
	cspins.open(GPIOPIN0 "/direction");
	cspins < "out";
	cspins.close();
	cspins.open(GPIOPIN1 "/directon");
	cspins < "out";
	cspins.close();
	cspins.open(GPIOPIN2 "/directon");
	cspins < "out";
	cspins.close();
	cspins.open(GPIOPIN3 "/directon");
	cspins < "out";
	cspins.close();
	cspins.open(GPIOPIN4 "/directon");
	cspins < "out";
	cspins.close();
	cspins.open(GPIOPIN5 "/directon");
	cspins < "out";
	cspins.close();
	*/
}
/*
void stopHandler(int sig){
	running = 0;
}
*/
/*
 *
 */
int main(int argc, char** argv){
	int spi_fd;
	spi_fd = SPI_open(MAX11_BUS, MAX11_CS);
	if(spi_fd < 0){
		printf("*Could not open SPI bus %d\n", MAX11_BUS);
		return 0;
	}
	//Config SPI bus
	MAX11_SPIConfig(spi_fd);

	//Config GPIO CS pins
	setup_cs_pins();
	char csPinArray[6][23];
	strcpy(csPinArray[0], GPIOPIN0 "/value");
	strcpy(csPinArray[1], GPIOPIN1 "/value");
	strcpy(csPinArray[2], GPIOPIN2 "/value");
	strcpy(csPinArray[3], GPIOPIN3 "/value");
	strcpy(csPinArray[4], GPIOPIN4 "/value");
	strcpy(csPinArray[5], GPIOPIN5 "/value");
	//Setup MAX11 control register
	fstream cspins;
	for(uint8_t i =0; i < 6; i++){
		cspins.open(csPinArray[i]);
		cout << csPinArray[i] << endl;
		MAX11_setup(spi_fd, 0, SCAN_MODE_NONE,  &cspins);
		cspins.close();
	}
	//Loop until close
	running = 1;
//	signal(SIGINT, stopHandler);
	cspins.open(csPinArray[0]);
	while(running){
		cout << MAX11_readmsg(spi_fd, &cspins, 1) << endl;
	}
	SPI_close(spi_fd);
	return 0;
}
