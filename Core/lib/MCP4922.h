/*
 * MCP4922.h
 *
 *  Created on: Jul 25, 2024
 *      Author: ADMIN-HPZ2
 */

#ifndef LIB_MCP4922_H_
#define LIB_MCP4922_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
typedef struct {
	void (*begin) (SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	void (*setDAC)(uint16_t chanelA, uint16_t chanelB );
}MCP4922;
extern MCP4922 mcp4922 ;
typedef struct {
    GPIO_TypeDef* GPIO_p;
    uint16_t GPIO_pin;
    SPI_HandleTypeDef* _hspi;
}mcp4922_pin;


#ifdef __cplusplus
}
#endif
#endif /* LIB_MCP4922_H_ */
