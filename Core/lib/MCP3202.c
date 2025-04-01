/*
 * MCP3202.c
 *
 *  Created on: Jul 25, 2024
 *      Author: ADMIN-HPZ2
 */
#include "MCP3202.h"
mcp3202_pin spiPin;
void _begin (SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	spiPin._hspi = hspi;
	spiPin.GPIO_p = GPIOx;
	spiPin.GPIO_pin = GPIO_Pin;
}

uint16_t _readChannel(uint8_t thisCh) {
	uint8_t txData[3] = { 0b00000001, 0, 0 }; // Khởi tạo mảng txData để truyền dữ liệu
	uint8_t rxData[3] = { 0, 0, 0 }; // Khởi tạo mảng rxData để nhận dữ liệu
	uint16_t result;
	// Gán giá trị bit đầu tiên của txData[0] để chọn kênh đọc dữ liệu
	txData[1] = (thisCh == 0) ? 0b10100000 : 0b11100000;
	// Truyền và nhận dữ liệu qua SPI
	HAL_GPIO_WritePin(spiPin.GPIO_p, spiPin.GPIO_pin, GPIO_PIN_RESET); // Thiết lập tín hiệu CS để bắt đầu truyền nhận
	HAL_SPI_TransmitReceive(spiPin._hspi, txData, rxData, 3, 100); // Truyền nhận dữ liệu
	HAL_GPIO_WritePin(spiPin.GPIO_p, spiPin.GPIO_pin, GPIO_PIN_SET); // Thiết lập tín hiệu CS để kết thúc truyền nhận

	// Gộp 2 byte đầu tiên nhận được thành giá trị 12-bit
	//rxData[1] -=1;
	result = ((rxData[1] & 0x0F) << 8) | rxData[2];

	return result;
}

MCP3202 mcp3202 = { .begin = _begin , .readChannel =  _readChannel};
