/*
 * MotorControl.h
 *
 *  Created on: Jun 12, 2024
 *      Author: ADMIN-HPZ2
 */

#ifndef MACHINE_MOTORCONTROL_H_
#define MACHINE_MOTORCONTROL_H_
#include "main.h"
#include "CanBus.h"
#include "stdbool.h"
#define FilterHigh                ((uint32_t) 0x11U )
#define FilterLow                ((uint32_t) 0x11U )

#define OperationMode    0x3U
#define ControlWord_EN   0x0FU
#define ControlWord_DIS  0x06U
#define OperationModeRes 0x60600008U
#define ControlWordRes   0x60400010U
#define TargetSpeedRes   0x60FF0020U
#define ProfileAccRes    0x60830020U
#define ProfileDecRes    0x60840020U
#define ResetErrorsRes   0x20100210U
#define MNum 2

extern uint32_t MotorID[MNum];
extern uint32_t SetSpeedPDO[MNum];
extern uint32_t EncoderID[MNum];
extern uint32_t CurrentID[MNum];
/*****************MACRO********************/
#define INT_TO_UINT8_ARRAY(num, array) \
    do { \
        (array)[0] = ((num) >> 24) & 0xFF; \
        (array)[1] = ((num) >> 16) & 0xFF; \
        (array)[2] = ((num) >> 8) & 0xFF; \
        (array)[3] = (num) & 0xFF; \
    } while(0)
#define RPM_TO_DEC(rpm) (((int64_t)(rpm) * 512 * 2500 * 4) / 1875)
#define RPS_TO_DEC(rps) (((int64_t)(rps) * 65536 * 2500 * 4) / 4000000)
#define DEC_TO_RPM(dec) (((int64_t)(dec) * 1875) / (512 * 2500 * 4))
#define UINT8_TO_INT32(data, index, value) \
		value = (data[index] << 24) | (data[index - 1] << 16) | \
          (data[index - 2] << 8) | data[index - 3];
#define UINT8_TO_INT16(data, index, value) \
		value = (data[index] << 8) | data[index - 1];

/********************TYPEDEF****************************/
struct WheelConfig {
	double PI ;
	double wheelDiameter ;   //unit: m
	int ratio ;
};

typedef enum {
	Open = 0x01,
	Close = 0x02,
	ResetNode = 0x81,
	Resetcom = 0x82,
	ComOperationStatus = 0x80
}NMT_Command;

typedef struct {
	uint32_t Canid;
	uint16_t Res;
	uint8_t value[4];
	uint8_t mes[8];
}Message_Kinco_Can ;

struct KincoErrorStatus{
	bool Extended_error;
	bool Encoder_connected;
	bool Encoder_internal;
	bool Encoder_CRC;
	bool Driver_temperature;
	bool Over_voltage;
	bool Under_voltage;
	bool Over_current;
	bool Chop_resistor;
	bool Position_following;
	bool Low_logicvoltage ;
	bool Motor_or_driver_IIt;
	bool Over_frequency;
	bool Motor_temperature;
	bool Motor_communication;
	bool EEPROM_data;
};
typedef struct {
	bool motorconnect;
	int16_t I_q ;		    // high byte Res2/ Unit: mA
	uint16_t Error_code;
	struct KincoErrorStatus Error_status;
	int32_t TargetSpeed;
	int32_t MaxSpeed;
	int32_t PosActual;		// high byte Res1
	int32_t SpeedReal;		// Low byte Res1 / Unit: rpm/s
}KinCoParameter;


extern KinCoParameter Kincoparameter;
extern KinCoParameter Kincoparam[MNum];
/************************FUNTION CODE *******************************/
Message_Kinco_Can CreateMessage( uint32_t Res, uint32_t value );
void MotorInit();
bool SetOperationMode(uint32_t vl, uint32_t id);
bool SetControlWord (uint32_t vl, uint32_t id) ;
bool SDOSetSpeed(int vl, uint32_t id);
bool SDOProfileAcc(int vl, uint32_t id);
bool SDOProfileDec(int vl, uint32_t id);
bool SDOResetErrors(int vl, uint32_t id);
void SetSpeed(int vl, uint32_t TPDO);
bool NMTmanagement (NMT_Command cmd, uint32_t MotorID);
void motorControl( bool en, bool error, uint8_t dir, double speed );
void MotorIsConnected(uint32_t id);
void motorErrorReset();
double rpmToSpeed (double rpm);
int speedToRpm (double speed);
int speedToRps (double speed);
#endif /* MACHINE_MOTORCONTROL_H_ */
