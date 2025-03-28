/*
 * CanBus.h
 *
 *  Created on: Nov 3, 2023
 *      Author: ADMIN-HPZ2
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"
/****************************************************/
typedef void(*myfuntion)();
#define MAX_BUFFER_CAN 12
/****************************************************/
#if defined(STM32G4xx_HAL_H)|| defined(STM32H7xx_HAL_H)
typedef struct _CAN_COM
{
	FDCAN_TxHeaderTypeDef   TxHeader;
	FDCAN_RxHeaderTypeDef   RxHeader;
	FDCAN_HandleTypeDef*	CanHandle;
	FDCAN_FilterTypeDef		CanfilterTypedef;
	uint8_t               	Can_rxData[MAX_BUFFER_CAN];
	uint8_t 				IDMode;
	uint32_t				ID_TYPE_U;
}CAN_COM;

typedef enum
{
	DATA_BYTE_0 	= FDCAN_DLC_BYTES_0,
	DATA_BYTE_1 	= FDCAN_DLC_BYTES_1,
	DATA_BYTE_2 	= FDCAN_DLC_BYTES_2,
	DATA_BYTE_3 	= FDCAN_DLC_BYTES_3,
	DATA_BYTE_4 	= FDCAN_DLC_BYTES_4,
	DATA_BYTE_5		= FDCAN_DLC_BYTES_5,
	DATA_BYTE_6 	= FDCAN_DLC_BYTES_6,
	DATA_BYTE_7 	= FDCAN_DLC_BYTES_7,
	DATA_BYTE_8 	= FDCAN_DLC_BYTES_8,
	DATA_BYTE_12 	= FDCAN_DLC_BYTES_12,
	DATA_BYTE_16 	= FDCAN_DLC_BYTES_16,
	DATA_BYTE_20 	= FDCAN_DLC_BYTES_20,
	DATA_BYTE_24 	= FDCAN_DLC_BYTES_24,
	DATA_BYTE_32	= FDCAN_DLC_BYTES_32,
	DATA_BYTE_48 	= FDCAN_DLC_BYTES_48,
	DATA_BYTE_64	= FDCAN_DLC_BYTES_64

}Size_data;
typedef void (*canCallBack)(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void canReciverCallBack(canCallBack cb);
void FDCan_begin(CAN_COM* canhandel, FDCAN_HandleTypeDef* Handle, uint8_t Idmode);
bool FDCan_Write(CAN_COM* canhandle, uint8_t* data, Size_data size);
#else
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
typedef struct _CAN_COM
{
	CAN_FilterTypeDef 	CanfilterTypedef;
	CAN_HandleTypeDef* 	CanHandle;
	CAN_TxHeaderTypeDef Can_txHeader;
	CAN_RxHeaderTypeDef Can_rxHeader;
	uint8_t* 			Can_txData;
	uint8_t             Can_rxData[MAX_BUFFER_CAN];
	uint32_t 			tx_mailBox;
	uint32_t 			Can_id;
	uint8_t 			IDMode;
	myfuntion 			CanRxIT_Callback;
}CAN_COM;
typedef enum
{
	DATA_BYTE_0 	= 0,
	DATA_BYTE_1 	= 1,
	DATA_BYTE_2 	= 2,
	DATA_BYTE_3 	= 3,
	DATA_BYTE_4 	= 4,
	DATA_BYTE_5		= 5,
	DATA_BYTE_6 	= 6,
	DATA_BYTE_7 	= 7,
	DATA_BYTE_8 	= 8
}Size_data;
void Can_begin(CAN_COM* canhandel, CAN_HandleTypeDef* Handle, uint8_t Idmode);
void Can_Write(CAN_COM* canhandle, uint8_t* data, size_t size);
#endif
void CanCofigfilter(CAN_COM* canhandle, uint32_t FilterHigh , uint32_t FilterLow);
void Can_Start(CAN_COM* canhandle, uint32_t CanID );
void Can_reciverData(CAN_COM* canhandle, uint8_t* RxData, size_t size);

#ifdef __cplusplus
}
#endif
#endif /* INC_CANBUS_H_ */
