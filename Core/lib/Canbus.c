/*
 * Canbus.c
 *
 *  Created on: Nov 3, 2023
 *      Author: ADMIN-HPZ2
 */
#include "CanBus.h"
#include "stdint.h"
#if defined(STM32G4xx_HAL_H)|| defined(STM32H7xx_HAL_H)
CAN_COM* Can_communication;
canCallBack can_call_back = NULL;
/* KHAI BAO Struct CANCOM*/
void FDCan_begin(CAN_COM* canhandel, FDCAN_HandleTypeDef* Handle, uint8_t Idmode)
{
	canhandel->ID_TYPE_U = Idmode? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID ;
	canhandel->CanHandle = Handle;
	Can_communication = canhandel;
}
/* HAM DANG KY NHAN DU LIEU*/
void canReciverCallBack(canCallBack cb)
{
	can_call_back = cb ;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Can_communication->RxHeader, Can_communication->Can_rxData) != HAL_OK)
    {
    /* Reception Error */
    Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
	can_call_back(hfdcan,RxFifo0ITs);
  }
}

/* SEND DATA MESSAGE*/
bool FDCan_Write(CAN_COM* canhandle, uint8_t* data, Size_data size)
{
	canhandle->TxHeader.DataLength = size ;
	if(HAL_FDCAN_AddMessageToTxFifoQ(canhandle->CanHandle,&canhandle->TxHeader, data) != HAL_OK)
	{
		return 1;
	}
	return 0;
}
#else
CAN_COM* Can_communication1;
CAN_COM* Can_communication2;
void (*rxDataITcallback)() = NULL;

void Can_begin(CAN_COM* canhandle, CAN_HandleTypeDef* Handle, uint8_t Idmode)
{
	canhandle->CanHandle = Handle;
	canhandle->IDMode = Idmode;
	if(Handle == &hcan1)
	Can_communication1 = canhandle;
	else
		Can_communication2 = canhandle;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
	HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &Can_communication1->Can_rxHeader, Can_communication1->Can_rxData);
	Can_communication1->CanRxIT_Callback();
	} else
	{
		HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &Can_communication2->Can_rxHeader, Can_communication2->Can_rxData);
		Can_communication2->CanRxIT_Callback();
	}
}
void Can_Write(CAN_COM* canhandle, uint8_t* data, size_t size)
{
		canhandle->Can_txHeader.DLC = size;
		if(HAL_CAN_AddTxMessage(canhandle->CanHandle, &canhandle->Can_txHeader, data, &canhandle->tx_mailBox) != HAL_OK)
		{
			Error_Handler();
		}
}
#endif

/* CONFIG FILLTER */
void CanCofigfilter(CAN_COM* canhandle, uint32_t FilterHigh , uint32_t FilterLow)
{

#if defined(STM32G4xx_HAL_H)|| defined(STM32H7xx_HAL_H)
	canhandle->CanfilterTypedef.IdType			 = canhandle->ID_TYPE_U;
	canhandle->CanfilterTypedef.FilterIndex	 	 = 0;
	canhandle->CanfilterTypedef.FilterType		 = FDCAN_FILTER_MASK;
	canhandle->CanfilterTypedef.FilterConfig	 = FDCAN_FILTER_TO_RXFIFO0;
	canhandle->CanfilterTypedef.FilterID1		 = FilterHigh;
	canhandle->CanfilterTypedef.FilterID2		 = FilterLow;
	if(HAL_FDCAN_ConfigFilter(canhandle->CanHandle, &canhandle->CanfilterTypedef) != HAL_OK)
	{
		Error_Handler();
	}
#else
		canhandle->CanfilterTypedef.FilterActivation = CAN_FILTER_ENABLE;
		canhandle->CanfilterTypedef.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		canhandle->CanfilterTypedef.FilterIdHigh = FilterHigh;
		canhandle->CanfilterTypedef.FilterIdLow = FilterLow;
		canhandle->CanfilterTypedef.FilterMaskIdHigh = FilterHigh;
		canhandle->CanfilterTypedef.FilterMaskIdLow = FilterLow;
		canhandle->CanfilterTypedef.FilterMode = CAN_FILTERMODE_IDMASK;
		canhandle->CanfilterTypedef.FilterScale = CAN_FILTERSCALE_32BIT;
		if(canhandle->CanHandle == &hcan1)
		{
			canhandle->CanfilterTypedef.FilterBank = 0; // which filter bank to use from the assigned ones
			canhandle->CanfilterTypedef.SlaveStartFilterBank = 13;
		}
		HAL_CAN_ConfigFilter(canhandle->CanHandle, &canhandle->CanfilterTypedef);
#endif
}

/*  CONFIG DATA TRANSMIT*/
void Can_Start(CAN_COM* canhandle, uint32_t CanID)
{
#if defined(STM32G4xx_HAL_H)|| defined(STM32H7xx_HAL_H)
	  if(HAL_FDCAN_Start(canhandle->CanHandle)!= HAL_OK)
	  {
		  Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(canhandle->CanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }
	  canhandle->TxHeader.Identifier = CanID;
	  canhandle->TxHeader.IdType = canhandle->ID_TYPE_U;
	  canhandle->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	  canhandle->TxHeader.DataLength = FDCAN_DLC_BYTES_8 ;
	  canhandle->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  canhandle->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  canhandle->TxHeader.FDFormat = FDCAN_FD_CAN;
	  canhandle->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  canhandle->TxHeader.MessageMarker = 0;
#else

	canhandle->Can_id =  CanID;
	HAL_CAN_Start(canhandle->CanHandle);
	HAL_CAN_ActivateNotification(canhandle->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
	canhandle->Can_txHeader.DLC = 3;
	canhandle->Can_txHeader.RTR = CAN_RTR_DATA;
	if(canhandle->IDMode)
	{
		canhandle->Can_txHeader.IDE = CAN_ID_EXT;
		canhandle->Can_txHeader.ExtId = canhandle->Can_id;
	}
	else
	{
		canhandle->Can_txHeader.IDE = CAN_ID_STD;
		canhandle->Can_txHeader.StdId = canhandle->Can_id;
	}
#endif
}

void Can_reciverData(CAN_COM* canhandle, uint8_t* RxData, size_t size)
{
	memcpy(RxData, canhandle->Can_rxData,size);
}
