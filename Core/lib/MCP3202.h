/*
 * MCP3202.h
 *
 *  Created on: Jul 25, 2024
 *      Author: ADMIN-HPZ2
 */

#ifndef LIB_MCP3202_H_
#define LIB_MCP3202_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

typedef struct {
	void (*begin) (SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	uint16_t (*readChannel)(uint8_t thisCh);
}MCP3202;

extern MCP3202 mcp3202 ;
typedef struct {
    GPIO_TypeDef* GPIO_p;
    uint16_t GPIO_pin;
    SPI_HandleTypeDef* _hspi;
}mcp3202_pin;

#ifdef __cplusplus
}
#endif
#endif /* LIB_MCP3202_H_ */
