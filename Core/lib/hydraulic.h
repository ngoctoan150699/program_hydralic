/*
 * hydraulic.h
 *
 *  Created on: Feb 20, 2025
 *      Author: ADMIN-HPZ2
 */

#ifndef HYDRAULIC_H_
#define HYDRAULIC_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"

typedef enum {
	CYLINDER_OFF ,
	CYLINDER_PALLET_UP ,
	CYLINDER_WHEEL_UP ,
	CYLINDER_PALLET_DOWN ,
	CYLINDER_WHEEL_DOWN
}CylinderState;
struct HydraulicTableControl
{
	uint8_t valve1 ;
	uint8_t valve2 ;
	uint8_t valve3 ;
	uint8_t pump ;
};
void configCylinderLimitSensor(uint8_t* limitUpPallet1, uint8_t* limitUpPallet2, uint8_t* limitDownPallet1, uint8_t* limitDownPallet2,
							   uint8_t* limitUpWheel1, uint8_t* limitUpWheel2, uint8_t* limitDownWheel1, uint8_t* limitDownWheel2) ;

bool controlCylinder(CylinderState cmd, bool en ) ;
void hydraulicSetState(struct HydraulicTableControl state );
bool checkErrorHydraulic();
void resetErrorHydraulic();
#ifdef __cplusplus
}
#endif
#endif /* HYDRAULIC_H_ */
