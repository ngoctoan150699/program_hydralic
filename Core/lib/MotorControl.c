/*
 * MotorControl.c
 *
 *  Created on: Jun 12, 2024
 *      Author: ADMIN-HPZ2
 */
#include "MotorControl.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "math.h"
#if defined(STM32G4xx_HAL_H)|| defined(STM32H7xx_HAL_H)
/**************** Variables****************/
struct WheelConfig whellConfig = { .PI = CONF_PI , .ratio = CONF_RATIO , .wheelDiameter = CONF_DIAMETER, .pulsePerRev = 2500 } ;

CAN_COM *Cankinco;
Message_Kinco_Can MesRx;
KinCoParameter Kincoparameter;
KinCoParameter Kincoparam[MNum];
/****************Set id for Motor in mode SDO************/
uint32_t MotorID[MNum] = {0x601, 0x602};

/****************Set id for Speed in mode PDO************/
uint32_t SetSpeedPDO[MNum] = {0x201, 0x202};

/****************Set id Encoder Feedback in mode PDO************/
uint32_t EncoderID[MNum] = {0x181, 0x182};

/****************Set id Current Feedback in mode PDO************/
uint32_t CurrentID[MNum] = {0x281, 0x282};
/*********************FunctionCode***********************/
void readParameter();

void setKincoErrorStatus(struct KincoErrorStatus *status, uint16_t value) {
    status->Extended_error = value & (1 << 0);
    status->Encoder_connected = value & (1 << 1);
    status->Encoder_internal = value & (1 << 2);
    status->Encoder_CRC = value & (1 << 3);
    status->Driver_temperature = value & (1 << 4);
    status->Over_voltage = value & (1 << 5);
    status->Under_voltage = value & (1 << 6);
    status->Over_current = value & (1 << 7);
    status->Chop_resistor = value & (1 << 8);
    status->Position_following = value & (1 << 9);
    status->Low_logicvoltage = value & (1 << 10);
    status->Motor_or_driver_IIt = value & (1 << 11);
    status->Over_frequency = value & (1 << 12);
    status->Motor_temperature = value & (1 << 13);
    status->Motor_communication = value & (1 << 14);
    status->EEPROM_data = value & (1 << 15);
}

void MotorIsConnected(uint32_t id)
{
	if(id == 0x701)
	{
		Kincoparam[0].flagMotorConnecting = true ;
	}
}

void CanRecieverCallback()
{
	MesRx.Canid = Cankinco->Can_rxHeader.Identifier;
	memcpy(MesRx.mes, Cankinco->Can_rxData,8);
	if(MesRx.Canid > 0x180 && MesRx.Canid < 0x280 )
	{
		UINT8_TO_INT32(MesRx.mes,3,Kincoparameter.PosActual);
		int32_t tmp;
		UINT8_TO_INT32(MesRx.mes,7,tmp);
		Kincoparameter.SpeedReal = DEC_TO_RPM(tmp) ;
	} else if (MesRx.Canid > 0x580 && MesRx.Canid < 0x600)
	{
		for(int x=4; x <8; x++)
		{
			MesRx.value[x-4] = Cankinco->Can_rxData[x];
		}
	} else if (MesRx.Canid > 0x280 && MesRx.Canid < 0x300)
	{
		Kincoparameter.flagMotorStarted = true ;
		UINT8_TO_INT16(MesRx.mes,1,Kincoparameter.I_q);
		UINT8_TO_INT16(MesRx.mes,3,Kincoparameter.Error_code);
		setKincoErrorStatus(&Kincoparameter.Error_status, Kincoparameter.Error_code);
	}else if (MesRx.Canid > 0x700)
	{
		MotorIsConnected(MesRx.Canid);
	}

	readParameter();
}
Message_Kinco_Can CreateMessage( uint32_t Res, uint32_t value )
{
	uint8_t byteAdd[4];
	uint8_t byteval[4];
	uint8_t mes[8];
	Message_Kinco_Can candata;
	//candata.Canid = motorid ;
	INT_TO_UINT8_ARRAY(Res, byteAdd);
	INT_TO_UINT8_ARRAY(value, byteval);
	uint8_t ObjectSubindex = 0x23;

	mes[0] = ObjectSubindex;
	mes[1] = byteAdd[1];
	mes[2] = byteAdd[0];
	mes[3] = byteAdd[2];
	mes[4] = byteval[3];
	mes[5] = byteval[2];
	mes[6] = byteval[1];
	mes[7] = byteval[0];
//	mes[8] = '\00';
	candata.Res = ((uint16_t)byteAdd[0]<< 8) | byteAdd[1];
	memcpy(candata.mes,mes,8);
	for(int x=4; x <8; x++)
	{
		candata.value[x-4] = mes[x];
	}
	return candata;
}

void MotorInit(CAN_COM *ucan)
{
//	  FDCan_begin(Cankinco, hfdcan, 0 );
//	  CanCofigfilter(Cankinco, FilterHigh, FilterLow);
//	  Can_Start(Cankinco, MotorID[0]);
	Cankinco = ucan ;
}

bool SetOperationMode(uint32_t vl, uint32_t id)
{
    Message_Kinco_Can temp;
    Cankinco->TxHeader.Identifier = id ;
    uint16_t timeout = 0;
    temp = CreateMessage(OperationModeRes, vl);
    temp.mes[0] = 0x2F;
    do {
        FDCan_Write(Cankinco, temp.mes, DATA_BYTE_6);
        timeout++;
        osDelay(100);
    } while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true  ;
}

bool SetControlWord (uint32_t vl, uint32_t id)
{
	Message_Kinco_Can temp ;
	Cankinco->TxHeader.Identifier = id ;
	uint16_t timeout = 0;
	temp = CreateMessage(ControlWordRes, vl);
	temp.mes[0] = 0x2B;
	do {
		FDCan_Write(Cankinco, temp.mes, DATA_BYTE_6);
		timeout ++ ;
		osDelay(100);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ? false :  true ;
}
bool SDOSetSpeed(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->TxHeader.Identifier = id ;
	int64_t dec = RPM_TO_DEC(vl);
	temp = CreateMessage(TargetSpeedRes, dec);
	uint16_t timeout = 0;
	do {
		FDCan_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;
}
bool SDOProfileAcc(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->TxHeader.Identifier = id ;
	int64_t dec = RPS_TO_DEC(vl);
	temp = CreateMessage(ProfileAccRes, dec);
	uint16_t timeout = 0;
	do {
		FDCan_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;
}
bool SDOProfileDec(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->TxHeader.Identifier = id ;
	int64_t dec = RPS_TO_DEC(vl);
	temp = CreateMessage(ProfileDecRes, dec);
	uint16_t timeout = 0;
	do {
		FDCan_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;

}
bool SDOResetErrors(int vl, uint32_t id)
{
	Message_Kinco_Can temp ;
	Cankinco->TxHeader.Identifier = id ;
	uint16_t timeout = 0;
	temp = CreateMessage(ResetErrorsRes, vl);
	temp.mes[0] = 0x2B;
	do {
		FDCan_Write(Cankinco, temp.mes, DATA_BYTE_6);
		timeout ++ ;
		osDelay(100);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ? false :  true ;
}

void SetSpeed(int vl, uint32_t TPDO)
{
	Message_Kinco_Can temp;
	int64_t dec = RPM_TO_DEC(vl);
	temp = CreateMessage(TargetSpeedRes, dec);
	Cankinco->TxHeader.Identifier = TPDO;
	FDCan_Write(Cankinco, temp.value, DATA_BYTE_4);

}

bool NMTmanagement (NMT_Command cmd, uint32_t MotorID)
{
	uint8_t buf[3] = {cmd, MotorID };
	Cankinco->TxHeader.Identifier = 0x0000;
	uint16_t timeout = 0;
	do {
		FDCan_Write(Cankinco, buf, DATA_BYTE_2);
		timeout++;
		osDelay(50);
	} while (MesRx.Canid != EncoderID[MotorID-1] && timeout < 10);

	return MesRx.Canid == EncoderID[MotorID-1]? false : true ;
}

void readParameter()
{
	for(int i=0; i<MNum; i++)
	{
		if(MesRx.Canid == EncoderID[i])
		{
			Kincoparam[i].SpeedReal = Kincoparameter.SpeedReal;
			Kincoparam[i].PosActual = Kincoparameter.PosActual;
		} else if (MesRx.Canid == CurrentID[i])
		{
			Kincoparam[i].Error_code   = Kincoparameter.Error_code ;
			Kincoparam[i].Error_status = Kincoparameter.Error_status;
			Kincoparam[i].I_q          = Kincoparameter.I_q;
			Kincoparam[i].flagMotorStarted = Kincoparameter.flagMotorStarted ;
		}

	}
}

void motorControl( bool en, bool error, uint8_t dir, double speed )
{
	static bool isStop;
	if ((!en || error) && isStop == false)
	{
		Kincoparam[0].TargetSpeed = 0 ;
		SetControlWord(ControlWord_DIS, MotorID[0]); // disable motor
		isStop = true ;
	} else if(en && !error)
	{
		if(isStop)
		{
			SetControlWord(ControlWord_EN, MotorID[0]); // enable motor
			isStop = false;
		}

		if(dir == 1 || dir == 4)
		{
			Kincoparam[0].TargetSpeed = speedToRpm(speed);
		}else if(dir == 3 || dir == 2)
		{
			Kincoparam[0].TargetSpeed = -speedToRpm(speed);
		}else
		{
			Kincoparam[0].TargetSpeed = 0 ;
		}
	}
	SetSpeed(Kincoparam[0].TargetSpeed, SetSpeedPDO[0]);
}

void motorErrorReset()
{
	SetControlWord(ControlWord_DIS, MotorID[0]);
	SDOResetErrors(0x01, MotorID[0]);
	SetControlWord(ControlWord_EN, MotorID[0]);
	SDOResetErrors(0x0, MotorID[0]);
}

double rpmToSpeed (double rpm)
{
	// Calculate rpm
	double wheelRPM = rpm / whellConfig.ratio;
    // Calculate Circumference (C = pi * D)
    double wheelCircumference = whellConfig.PI * whellConfig.wheelDiameter;
    // Calculate speed (m/s)
    double speed = (wheelCircumference * wheelRPM) / 60.0;
	return speed ;
}
int speedToRpm (double speed)
{
    // Calculate Circumference (C = pi * D)
    double wheelCircumference =  whellConfig.PI * whellConfig.wheelDiameter;
	// Calculate rpm
	double wheelRPM = (speed*60)/wheelCircumference;
	int rpm = wheelRPM * whellConfig.ratio ;
	return rpm ;
}

int speedToRps (double speed)
{
	return (speedToRpm(speed)/60);
}

int distanceToPulses(float distance)
{
	// Calculate Circumference (C = pi * D)
	double wheelCircumference = whellConfig.PI * whellConfig.wheelDiameter;
	// Calculate rotations of whell
	double wheelRotations	  = distance / wheelCircumference ;
	// Calculate rotations of motor
	double motorRotations	  = wheelRotations * whellConfig.ratio ;
	// Calculate pulses
	return (int) (motorRotations * whellConfig.pulsePerRev)  ;

}

double calculateAcceleration( double v ,double v0, double s)
{
	// equation: v^2 - v0^2 = 2as
	double deltaV = pow(v,2) - pow(v0,2);
	double a = abs(deltaV / (2*s));
	return (a);
}
#else
/**************** Variables****************/
struct WheelConfig whellConfig = { .PI = CONF_PI , .ratio = CONF_RATIO , .wheelDiameter = CONF_DIAMETER, .pulsePerRev = 2500 } ;

CAN_COM *Cankinco;
Message_Kinco_Can MesRx;
KinCoParameter Kincoparameter;
KinCoParameter Kincoparam[MNum];
/****************Set id for Motor in mode SDO************/
uint32_t MotorID[MNum] = {0x601, 0x602};

/****************Set id for Speed in mode PDO************/
uint32_t SetSpeedPDO[MNum] = {0x201, 0x202};

/****************Set id Encoder Feedback in mode PDO************/
uint32_t EncoderID[MNum] = {0x181, 0x182};

/****************Set id Current Feedback in mode PDO************/
uint32_t CurrentID[MNum] = {0x281, 0x282};
/*********************FunctionCode***********************/
void readParameter();

void setKincoErrorStatus(struct KincoErrorStatus *status, uint16_t value) {
    status->Extended_error = value & (1 << 0);
    status->Encoder_connected = value & (1 << 1);
    status->Encoder_internal = value & (1 << 2);
    status->Encoder_CRC = value & (1 << 3);
    status->Driver_temperature = value & (1 << 4);
    status->Over_voltage = value & (1 << 5);
    status->Under_voltage = value & (1 << 6);
    status->Over_current = value & (1 << 7);
    status->Chop_resistor = value & (1 << 8);
    status->Position_following = value & (1 << 9);
    status->Low_logicvoltage = value & (1 << 10);
    status->Motor_or_driver_IIt = value & (1 << 11);
    status->Over_frequency = value & (1 << 12);
    status->Motor_temperature = value & (1 << 13);
    status->Motor_communication = value & (1 << 14);
    status->EEPROM_data = value & (1 << 15);
}

void MotorIsConnected(uint32_t id)
{
	if(id == 0x701)
	{
		Kincoparam[0].flagMotorConnecting = true ;
	}
}

void CanRecieverCallback()
{
	MesRx.Canid = Cankinco->Can_rxHeader.StdId;
	memcpy(MesRx.mes, Cankinco->Can_rxData,8);
	if(MesRx.Canid > 0x180 && MesRx.Canid < 0x280 )
	{
		UINT8_TO_INT32(MesRx.mes,3,Kincoparameter.PosActual);
		int32_t tmp;
		UINT8_TO_INT32(MesRx.mes,7,tmp);
		Kincoparameter.SpeedReal = DEC_TO_RPM(tmp) ;
	} else if (MesRx.Canid > 0x580 && MesRx.Canid < 0x600)
	{
		for(int x=4; x <8; x++)
		{
			MesRx.value[x-4] = Cankinco->Can_rxData[x];
		}
	} else if (MesRx.Canid > 0x280 && MesRx.Canid < 0x300)
	{
		Kincoparameter.flagMotorStarted = true ;
		UINT8_TO_INT16(MesRx.mes,1,Kincoparameter.I_q);
		UINT8_TO_INT16(MesRx.mes,3,Kincoparameter.Error_code);
		setKincoErrorStatus(&Kincoparameter.Error_status, Kincoparameter.Error_code);
	}else if (MesRx.Canid > 0x700)
	{
		MotorIsConnected(MesRx.Canid);
	}

	readParameter();
}
Message_Kinco_Can CreateMessage( uint32_t Res, uint32_t value )
{
	uint8_t byteAdd[4];
	uint8_t byteval[4];
	uint8_t mes[8];
	Message_Kinco_Can candata;
	//candata.Canid = motorid ;
	INT_TO_UINT8_ARRAY(Res, byteAdd);
	INT_TO_UINT8_ARRAY(value, byteval);
	uint8_t ObjectSubindex = 0x23;

	mes[0] = ObjectSubindex;
	mes[1] = byteAdd[1];
	mes[2] = byteAdd[0];
	mes[3] = byteAdd[2];
	mes[4] = byteval[3];
	mes[5] = byteval[2];
	mes[6] = byteval[1];
	mes[7] = byteval[0];
//	mes[8] = '\00';
	candata.Res = ((uint16_t)byteAdd[0]<< 8) | byteAdd[1];
	memcpy(candata.mes,mes,8);
	for(int x=4; x <8; x++)
	{
		candata.value[x-4] = mes[x];
	}
	return candata;
}

void MotorInit(CAN_COM *ucan)
{
//	  FDCan_begin(Cankinco, hfdcan, 0 );
//	  CanCofigfilter(Cankinco, FilterHigh, FilterLow);
//	  Can_Start(Cankinco, MotorID[0]);
	Cankinco = ucan ;
}

bool SetOperationMode(uint32_t vl, uint32_t id)
{
    Message_Kinco_Can temp;
    Cankinco->Can_txHeader.StdId = id ;
    uint16_t timeout = 0;
    temp = CreateMessage(OperationModeRes, vl);
    temp.mes[0] = 0x2F;
    do {
        Can_Write(Cankinco, temp.mes, DATA_BYTE_6);
        timeout++;
        osDelay(100);
    } while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true  ;
}

bool SetControlWord (uint32_t vl, uint32_t id)
{
	Message_Kinco_Can temp ;
	Cankinco->Can_txHeader.StdId = id ;
	uint16_t timeout = 0;
	temp = CreateMessage(ControlWordRes, vl);
	temp.mes[0] = 0x2B;
	do {
		Can_Write(Cankinco, temp.mes, DATA_BYTE_6);
		timeout ++ ;
		osDelay(100);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ? false :  true ;
}
bool SDOSetSpeed(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->Can_txHeader.StdId = id ;
	int64_t dec = RPM_TO_DEC(vl);
	temp = CreateMessage(TargetSpeedRes, dec);
	uint16_t timeout = 0;
	do {
		Can_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;
}
bool SDOProfileAcc(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->Can_txHeader.StdId = id ;
	int64_t dec = RPS_TO_DEC(vl);
	temp = CreateMessage(ProfileAccRes, dec);
	uint16_t timeout = 0;
	do {
		Can_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;
}
bool SDOProfileDec(int vl, uint32_t id)
{
	Message_Kinco_Can temp;
	Cankinco->Can_txHeader.StdId = id ;
	int64_t dec = RPS_TO_DEC(vl);
	temp = CreateMessage(ProfileDecRes, dec);
	uint16_t timeout = 0;
	do {
		Can_Write(Cankinco, temp.mes, DATA_BYTE_8);
		timeout++;
		osDelay(20);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 100);

	return memcmp(temp.value, MesRx.value, 4) == 0 ?  false : true ;

}
bool SDOResetErrors(int vl, uint32_t id)
{
	Message_Kinco_Can temp ;
	Cankinco->Can_txHeader.StdId = id ;
	uint16_t timeout = 0;
	temp = CreateMessage(ResetErrorsRes, vl);
	temp.mes[0] = 0x2B;
	do {
		Can_Write(Cankinco, temp.mes, DATA_BYTE_6);
		timeout ++ ;
		osDelay(100);
	} while (memcmp(temp.value, MesRx.value, 4) != 0 && timeout < 20);

    return memcmp(temp.value, MesRx.value, 4) == 0 ? false :  true ;
}

void SetSpeed(int vl, uint32_t TPDO)
{
	Message_Kinco_Can temp;
	int64_t dec = RPM_TO_DEC(vl);
	temp = CreateMessage(TargetSpeedRes, dec);
	Cankinco->Can_txHeader.StdId = TPDO;
	Can_Write(Cankinco, temp.value, DATA_BYTE_4);

}

bool NMTmanagement (NMT_Command cmd, uint32_t MotorID)
{
	uint8_t buf[3] = {cmd, MotorID };
	Cankinco->Can_txHeader.StdId = 0x0000;
	uint16_t timeout = 0;
	do {
		Can_Write(Cankinco, buf, DATA_BYTE_2);
		timeout++;
		osDelay(50);
	} while (MesRx.Canid != EncoderID[MotorID-1] && timeout < 10);

	return MesRx.Canid == EncoderID[MotorID-1]? false : true ;
}

void readParameter()
{
	for(int i=0; i<MNum; i++)
	{
		if(MesRx.Canid == EncoderID[i])
		{
			Kincoparam[i].SpeedReal = Kincoparameter.SpeedReal;
			Kincoparam[i].PosActual = Kincoparameter.PosActual;
		} else if (MesRx.Canid == CurrentID[i])
		{
			Kincoparam[i].Error_code   = Kincoparameter.Error_code ;
			Kincoparam[i].Error_status = Kincoparameter.Error_status;
			Kincoparam[i].I_q          = Kincoparameter.I_q;
			Kincoparam[i].flagMotorStarted = Kincoparameter.flagMotorStarted ;
		}

	}
}

void motorControl( bool en, bool error, uint8_t dir, double speed )
{
	static bool isStop;
	if ((!en || error) && isStop == false)
	{
		Kincoparam[0].TargetSpeed = 0 ;
		SetControlWord(ControlWord_DIS, MotorID[0]); // disable motor
		isStop = true ;
	} else if(en && !error)
	{
		if(isStop)
		{
			SetControlWord(ControlWord_EN, MotorID[0]); // enable motor
			isStop = false;
		}

		if(dir == 1 || dir == 4)
		{
			Kincoparam[0].TargetSpeed = speedToRpm(speed);
		}else if(dir == 3 || dir == 2)
		{
			Kincoparam[0].TargetSpeed = -speedToRpm(speed);
		}else
		{
			Kincoparam[0].TargetSpeed = 0 ;
		}
	}
	SetSpeed(Kincoparam[0].TargetSpeed, SetSpeedPDO[0]);
}

void motorErrorReset()
{
	SetControlWord(ControlWord_DIS, MotorID[0]);
	SDOResetErrors(0x01, MotorID[0]);
	SetControlWord(ControlWord_EN, MotorID[0]);
	SDOResetErrors(0x0, MotorID[0]);
}

double rpmToSpeed (double rpm)
{
	// Calculate rpm
	double wheelRPM = rpm / whellConfig.ratio;
    // Calculate Circumference (C = pi * D)
    double wheelCircumference = whellConfig.PI * whellConfig.wheelDiameter;
    // Calculate speed (m/s)
    double speed = (wheelCircumference * wheelRPM) / 60.0;
	return speed ;
}
int speedToRpm (double speed)
{
    // Calculate Circumference (C = pi * D)
    double wheelCircumference =  whellConfig.PI * whellConfig.wheelDiameter;
	// Calculate rpm
	double wheelRPM = (speed*60)/wheelCircumference;
	int rpm = wheelRPM * whellConfig.ratio ;
	return rpm ;
}

int speedToRps (double speed)
{
	return (speedToRpm(speed)/60);
}

int distanceToPulses(float distance)
{
	// Calculate Circumference (C = pi * D)
	double wheelCircumference = whellConfig.PI * whellConfig.wheelDiameter;
	// Calculate rotations of whell
	double wheelRotations	  = distance / wheelCircumference ;
	// Calculate rotations of motor
	double motorRotations	  = wheelRotations * whellConfig.ratio ;
	// Calculate pulses
	return (int) (motorRotations * whellConfig.pulsePerRev)  ;

}

double calculateAcceleration( double v ,double v0, double s)
{
	// equation: v^2 - v0^2 = 2as
	double deltaV = pow(v,2) - pow(v0,2);
	double a = abs(deltaV / (2*s));
	return (a);
}
#endif
