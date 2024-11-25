#ifndef CANBUS_H_
#define CANBUS_H_

#include "main.h"
#include "string.h"

//Header ID cho các sensor
#define ENC_SID			0x100
#define ENC_FBID		0x200
#define ENC_TXID		0x201
#define ENC_RSID		0x101

#define IMU_SID			0x102
#define IMU_FBID		0x202
#define IMU_TXID		0x203
#define IMU_RSID		0x103

#define SLA_FILID 	0x100
#define SLA_FILIDM	0x7F0
#define MAS_FILID 	0x200
#define MAS_FILIDM	0x7F0

#define X_POS_ADDR	0x01
#define Y_POS_ADDR	0x02
#define X_ANG_ADDR	0x03
#define Y_ANG_ADDR	0x04
#define Z_ANG_ADDR	0x05

typedef struct
{
	CAN_TxHeaderTypeDef StaHeader;
	CAN_TxHeaderTypeDef FbHeader;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_TxHeaderTypeDef RxHeader;
	uint16_t						frequency;
	uint8_t							fbflag;
}CAN_SensorTypeDef;

typedef struct
{
	CAN_RxHeaderTypeDef	Header;
	uint8_t 						aData[8];
	uint8_t							rxflag;
}CAN_FIFO_RxTypeDef ;

void CAN_Fifo_RxFlag(CAN_HandleTypeDef *hcan, uint32_t Fifo, CAN_FIFO_RxTypeDef  *Fifo_Rx);
void CAN_Sensor_TxHeader_Init(CAN_SensorTypeDef *Sensor, uint32_t StaId, uint32_t FbId, uint32_t TxId, uint32_t RsId);
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canfilterconfig, uint32_t FilterBank, uint32_t FIFO, uint32_t FilterId, uint32_t FilterIdMask);
void CAN_Sensor_Start(CAN_HandleTypeDef *hcan, uint16_t delay, CAN_SensorTypeDef *Sensor);
void CAN_Sensor_Feedback(CAN_HandleTypeDef *hcan, CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor);
void CAN_Sensor_RecieveFb(CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor);
uint8_t CAN_Sensor_GetFbFlag(CAN_SensorTypeDef *Sensor);
void CAN_Float_TxData(CAN_HandleTypeDef *hcan, CAN_SensorTypeDef *Sensor, float fData, uint8_t addr);
void CAN_Float_RxData(CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor, float *fData, uint8_t addr);
void CAN_Sensor_ResetRQ(CAN_HandleTypeDef *hcan, CAN_SensorTypeDef *Sensor);
void CAN_Sensor_ResetHandle(CAN_SensorTypeDef *Sensor, CAN_FIFO_RxTypeDef *Fifo_Rx);

#endif



