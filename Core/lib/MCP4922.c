/*
 * MCP4922.c
 *
 *  Created on: Jul 25, 2024
 *      Author: ADMIN-HPZ2
 */
#include "MCP4922.h"

#define ADC0_5V
mcp4922_pin spiPinDAC ;

void DAC_begin (SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	spiPinDAC._hspi = hspi ;
	spiPinDAC.GPIO_p = GPIOx;
	spiPinDAC.GPIO_pin = GPIO_Pin;
}


//************************************************************************
/*
Bitmasking for setting options in dac:

The four MSB in the Mask 0b0111000000000000 and 0b1111000000000000 is for
setting different options of the DAC setup.

0bX111000000000000 where X is What DAC channel the SPI is writing to.
bit15			   X=0 is writing to channel A.
				   X=1 is writing to channel B.

0b0X11000000000000 where X is Buffered or UnBuffered mode. Buffered uses LDAC
bit14			   pin to simuttaneous update both channels.
				   UnBuffered I guess is writing outputs directly to DAC
				   outputs and ignoring LDAC pin.
				   X=0 is UnBuffered.
				   X=1 is Buffered.

0b01x1000000000000 where X is GAIN selector.
bit13			   X=0 is 2X GAIN.
				   X=1 is 1X GAIN.

0b011X000000000000 where X SHUTDOWN.
bit12			   X=0 OUTPUT is DISABLED on selected channel.
				   X=1 OUTPUT is ENABLED on selected channel.

0b0111XXXXXXXXXXXX where X is the 12 bits to be written to the active channel.
bit 11 down to bit 0

 */
//************************************************************************

uint8_t highByte(uint16_t value) {
  return (uint8_t)(value >> 8);
}
uint8_t LowByte(uint16_t value) {
  return (uint8_t)(value & 0xFF);
}

void sendIntValueSPI(int A ,int B) {
int channelA = A | 0b0111000000000000;
int channelB = B | 0b1111000000000000;
uint8_t hightByteA = highByte(channelA);
uint8_t lowByteA = LowByte(channelA);
uint8_t hightByteB = highByte(channelB);
uint8_t lowByteB = LowByte(channelB);
HAL_GPIO_WritePin(spiPinDAC.GPIO_p, spiPinDAC.GPIO_pin, GPIO_PIN_RESET);
HAL_SPI_Transmit(spiPinDAC._hspi, &hightByteA,sizeof(hightByteA), 100);
HAL_SPI_Transmit(spiPinDAC._hspi, &lowByteA,sizeof(lowByteA), 100);
HAL_GPIO_WritePin(spiPinDAC.GPIO_p, spiPinDAC.GPIO_pin, GPIO_PIN_SET);
__asm__("nop\n\t");
HAL_GPIO_WritePin(spiPinDAC.GPIO_p, spiPinDAC.GPIO_pin, GPIO_PIN_RESET);
HAL_SPI_Transmit(spiPinDAC._hspi, &hightByteB, sizeof(hightByteB), 100);
HAL_SPI_Transmit(spiPinDAC._hspi, &lowByteB,sizeof(lowByteB), 100);
HAL_GPIO_WritePin(spiPinDAC.GPIO_p, spiPinDAC.GPIO_pin, GPIO_PIN_SET);
__asm__("nop\n\t");
}

void SetDAC(uint16_t channelA, uint16_t channelB) {
#ifdef ADC0_5V
if(channelA>2048)
	{channelA = 2048;}
	if(channelB>2048)
	{channelB=2048;}
#endif
  sendIntValueSPI(channelA,channelB);
}

MCP4922 mcp4922 = { .begin = DAC_begin , .setDAC = SetDAC } ;
