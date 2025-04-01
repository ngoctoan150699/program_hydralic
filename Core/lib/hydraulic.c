/*
 * hydraulic.c
 *
 *  Created on: Feb 20, 2025
 *      Author: ADMIN-HPZ2
 */
#include "hydraulic.h"

#include "MCP4922.h"

#include "cmsis_os.h"
#include "stdio.h"


static const uint64_t TIMER_LIMIT_HYDRAULIC = 5000 ; // 5000 ms
static uint64_t timer_hydarulic = 0 ;
static uint64_t timer_hydarulic1 = 0 ;
static bool error_hydraulic_state = false ;


static const struct HydraulicTableControl wheel_up_state     = {0, 0, 1, 1} ;
static const struct HydraulicTableControl wheel_down_state   = {1, 0, 1, 1} ;
static const struct HydraulicTableControl pallet_up_state    = {1, 1, 0, 1} ;
static const struct HydraulicTableControl pallet_down_state  = {0, 1, 0, 1} ;
static const struct HydraulicTableControl free_all_state     = {0, 0, 0, 0} ;

static uint8_t* lmss_pallet_up_1 ;
static uint8_t* lmss_pallet_up_2 ;
static uint8_t* lmss_pallet_down_1 ;
static uint8_t* lmss_pallet_down_2 ;
static uint8_t* lmss_wheel_up_1 ;
static uint8_t* lmss_wheel_up_2 ;
static uint8_t* lmss_wheel_down_1 ;
static uint8_t* lmss_wheel_down_2 ;

typedef struct
{
	struct HydraulicTableControl wheel_up ;
	struct HydraulicTableControl wheel_down ;
	struct HydraulicTableControl pallet_up ;
	struct HydraulicTableControl pallet_down ;
	struct HydraulicTableControl free_all ;

}CylinderSetState;
bool detectFlagRisingEdge(bool currentState)
{
    static bool lastState = false;
    bool risingEdge = (currentState && !lastState);
    lastState = currentState;
    return risingEdge;
}
// t: expiration time, prd: period, now: current time. Return true if expired
bool u_timer_expired1(uint64_t *t, uint64_t prd, uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
/**
 * @brief    Xuất tín hiệu điều khiển xylanh.
 * @param state: bảng trạng thái điều khiển xylanh
 */

void hydraulicSetState(struct HydraulicTableControl state) {
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, (GPIO_PinState) state.valve2);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, (GPIO_PinState) state.valve3);

	if (state.pump == 1) {
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, (GPIO_PinState) state.valve1);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, (GPIO_PinState) state.pump);
		mcp4922.setDAC(1024, 1024);
	} else {
		osDelay(100);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, (GPIO_PinState) state.valve1);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, (GPIO_PinState) state.pump);
		mcp4922.setDAC(0, 0);

	}
}

bool controlCylinder(CylinderState cmd, bool en )
{
	if(detectFlagRisingEdge(en)) { timer_hydarulic = 0 ;}  // Reset lại timer mỗi lần chạy bơm
	if(!en) {
		hydraulicSetState(free_all_state);
		//MG_DEBUG(("TẮT THỦY LỰC \n"));
		return true ;
	}
	bool state = false;
	uint64_t now = (uint64_t) ((osKernelGetTickCount() * 1000) / osKernelGetTickFreq());
	switch (cmd) {
		case CYLINDER_OFF:
			hydraulicSetState(free_all_state);
			break;
		case CYLINDER_PALLET_UP:
			state = *lmss_pallet_up_1 == 0 || *lmss_pallet_up_2 == 0 ;
			if(state)
			{
				hydraulicSetState(pallet_up_state);
		    	//MG_DEBUG(("NÂNG PALLET \n"));
			}
			break;
		case CYLINDER_WHEEL_UP:
			state = *lmss_wheel_up_1 == 0 || *lmss_wheel_up_2 == 0 ;
			if(state)
			{
				hydraulicSetState(wheel_up_state);
		    	//MG_DEBUG(("NÂNG BÁNH XE \n"));
			}
			break;
		case CYLINDER_PALLET_DOWN:
			state = *lmss_pallet_down_1 == 0 || *lmss_pallet_down_2 == 0 ;
			if(state)
			{
				hydraulicSetState(pallet_down_state);
		    	//MG_DEBUG(("HẠ PALLET \n"));

			}
			break;
		case CYLINDER_WHEEL_DOWN:
			state = *lmss_wheel_down_1 == 0 || *lmss_wheel_down_2 == 0 ;
			if(state)
			{
				hydraulicSetState(wheel_down_state);
		    	//MG_DEBUG(("HẠ BÁNH XE \n"));
			}
			break;
		default:
			break;
	}

	if(u_timer_expired1(&timer_hydarulic, TIMER_LIMIT_HYDRAULIC, now) && state)
	{
		// báo lỗi sau 5s nếu không tác động cảm biến
		error_hydraulic_state = true ;
	}

	if(!state) { hydraulicSetState(free_all_state); }
	return !state ;
}

void configCylinderLimitSensor(uint8_t* limitUpPallet1, uint8_t* limitUpPallet2, uint8_t* limitDownPallet1, uint8_t* limitDownPallet2 ,
							   uint8_t* limitUpWheel1, uint8_t* limitUpWheel2, uint8_t* limitDownWheel1, uint8_t* limitDownWheel2)
{
	lmss_pallet_down_1 = limitDownPallet1 ;
	lmss_pallet_down_2 = limitDownPallet2 ;
	lmss_pallet_up_1   = limitUpPallet1 ;
	lmss_pallet_up_2   = limitUpPallet2 ;
	lmss_wheel_down_1  = limitDownWheel1 ;
	lmss_wheel_down_2  = limitDownWheel2 ;
	lmss_wheel_up_1    = limitUpWheel1   ;
	lmss_wheel_up_2    = limitUpWheel2  ;
}

bool checkErrorHydraulic() {
	return error_hydraulic_state ;
}

void resetErrorHydraulic(){
	error_hydraulic_state = false ;
}
