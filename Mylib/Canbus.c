#include "Canbus.h"

static uint32_t empty_mailbox;
//Nhâp ID Header cho Sensor
void CAN_Sensor_TxHeader_Init(CAN_SensorTypeDef *Sensor, uint32_t StaId, uint32_t FbId, uint32_t TxId, uint32_t RsId)
{
	Sensor->StaHeader.DLC									= 2;
	Sensor->StaHeader.ExtId								= 0;
	Sensor->StaHeader.IDE									= CAN_ID_STD;
	Sensor->StaHeader.RTR									= CAN_RTR_DATA;
	Sensor->StaHeader.StdId								=	StaId;
	Sensor->StaHeader.TransmitGlobalTime	= DISABLE;
	Sensor->FbHeader.DLC									= 2;
	Sensor->FbHeader.ExtId								= 0;
	Sensor->FbHeader.IDE									= CAN_ID_STD;
	Sensor->FbHeader.RTR									= CAN_RTR_DATA;
	Sensor->FbHeader.StdId								= FbId;
	Sensor->FbHeader.TransmitGlobalTime		= DISABLE;
	Sensor->TxHeader.DLC									= 8;
	Sensor->TxHeader.ExtId								= CAN_ID_STD;
	Sensor->TxHeader.RTR									= CAN_RTR_DATA;
	Sensor->TxHeader.StdId								= TxId;
	Sensor->TxHeader.TransmitGlobalTime		= DISABLE;
	Sensor->RxHeader.DLC									= 0;
	Sensor->RxHeader.ExtId								= 0;
	Sensor->RxHeader.IDE									= CAN_ID_STD;
	Sensor->RxHeader.RTR									= CAN_RTR_DATA;
	Sensor->RxHeader.StdId								= RsId;
}
//Tìm mailbox trông
uint32_t CAN_Get_EmptyMailbox(void)
{
	if (CAN1->TSR & CAN_TSR_TME0)
	{
		return CAN_TX_MAILBOX0;
	}
	else if (CAN1->TSR & CAN_TSR_TME1)
	{
		return CAN_TX_MAILBOX1;
	}
	else if (CAN1->TSR & CAN_TSR_TME2)
	{
		return CAN_TX_MAILBOX2;
	}
	return HAL_BUSY;
}
//Thiêt lâp CAN Filter
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canfilterconfig, 
												uint32_t FilterBank, uint32_t FIFO, uint32_t FilterId, uint32_t FilterIdMask)
{
	canfilterconfig->FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig->FilterBank = FilterBank;  
  canfilterconfig->FilterFIFOAssignment = FIFO;
  canfilterconfig->FilterIdHigh = FilterId<<5;
  canfilterconfig->FilterIdLow = 0x0000;
  canfilterconfig->FilterMaskIdHigh = FilterIdMask<<5;
  canfilterconfig->FilterMaskIdLow = 0x0000;
  canfilterconfig->FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig->FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig->SlaveStartFilterBank = 0;  
	
	HAL_CAN_ConfigFilter(hcan, canfilterconfig);
}
//Flag nhân message qua CAN
void CAN_Fifo_RxFlag(CAN_HandleTypeDef *hcan, uint32_t Fifo, CAN_FIFO_RxTypeDef  *Fifo_Rx)
{
	HAL_CAN_GetRxMessage(hcan, Fifo, &Fifo_Rx->Header, Fifo_Rx->aData);
	Fifo_Rx->rxflag = 1;
}
//Lênh khoi dông sensor duoc gui tù master
void CAN_Sensor_Start(CAN_HandleTypeDef *hcan, uint16_t frequency, CAN_SensorTypeDef *Sensor)
{
	uint8_t data[2];
	//Chuyên uint16_t sang uint8_t
	data[0] = (uint8_t)(frequency >> 8);
	data[1] = (uint8_t)(frequency & 0xFF);
	//Tìm mailbox trông
	empty_mailbox = CAN_Get_EmptyMailbox();
	HAL_CAN_AddTxMessage(hcan, &Sensor->StaHeader, data, &empty_mailbox);
}
//Lênh feedback duoc gui tu Slave sau khi nhan lenh start
void CAN_Sensor_Feedback(CAN_HandleTypeDef *hcan, CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor)
{
	uint8_t data[2];
	if ((Fifo_Rx->Header.StdId == Sensor->StaHeader.StdId)&& Fifo_Rx->rxflag && !Sensor->fbflag)
	{
		data[0] = Fifo_Rx->aData[0];
		data[1] = Fifo_Rx->aData[1];
		//Chuyen uint8_t sang uint16_t
		Sensor->frequency = ((uint16_t)Fifo_Rx->aData[0] << 8) | (uint16_t)Fifo_Rx->aData[1];
		//Kiem empty mailbox
		while(CAN_Get_EmptyMailbox() == HAL_BUSY);
		empty_mailbox = CAN_Get_EmptyMailbox();
		//Feedback data
		HAL_CAN_AddTxMessage(hcan, &Sensor->FbHeader, data, &empty_mailbox);
		Sensor->fbflag = 1;
		Fifo_Rx->rxflag = 0;
	}
}
//Lenh nhân feedback duoc gui tù slave
void CAN_Sensor_RecieveFb(CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor)
{
	if ((Fifo_Rx->Header.StdId == Sensor->FbHeader.StdId) && Fifo_Rx->rxflag && !Sensor->fbflag)
	{
		Sensor->frequency = ((uint16_t)Fifo_Rx->aData[0] << 8) | (uint16_t)Fifo_Rx->aData[1];
		Sensor->fbflag = 1;
		Fifo_Rx->rxflag = 0;
	}
}
//Lây feedback flag de debug
uint8_t CAN_Sensor_GetFbFlag(CAN_SensorTypeDef *Sensor)
{
	return Sensor->fbflag;
}
//Gui du lieu kieu float qua CAN
void CAN_Float_TxData(CAN_HandleTypeDef *hcan, CAN_SensorTypeDef *Sensor, float fData, uint8_t addr)
{
	static uint32_t time;
	uint8_t data[8];
	data[0] = addr;
	memcpy(&data[1], &fData, sizeof(float));
	
	if (Sensor->fbflag && Sensor->frequency)
	{
		if ((HAL_GetTick() - time) >= 1000.0/Sensor->frequency)
		{		
			empty_mailbox = CAN_Get_EmptyMailbox();
			HAL_CAN_AddTxMessage(hcan, &Sensor->TxHeader, data, &empty_mailbox);
		}
	}
}
//Nhan du lieu kieu float qua CAN
void CAN_Float_RxData(CAN_FIFO_RxTypeDef  *Fifo_Rx, CAN_SensorTypeDef *Sensor, float *fData, uint8_t addr)
{
	
	if(Fifo_Rx->rxflag)
	{
		if (Fifo_Rx->Header.StdId == Sensor->TxHeader.StdId && Fifo_Rx->aData[0] == addr) 
		{
			memcpy(fData, &Fifo_Rx->aData[1], sizeof(float));
			Fifo_Rx->rxflag = 0;
		}
	}
}
//Lenh yêu c?u reset sensor duoc gui tù master
void CAN_Sensor_ResetRQ(CAN_HandleTypeDef *hcan, CAN_SensorTypeDef *Sensor)
{
	uint8_t data[1];
	empty_mailbox = CAN_Get_EmptyMailbox();
	HAL_CAN_AddTxMessage(hcan, &Sensor->RxHeader, data, &empty_mailbox); 
}
//Thuc hien reset sensor
__weak void CAN_Sensor_Reset(CAN_SensorTypeDef *Sensor)
{
	
}
//Handle Reset
void CAN_Sensor_ResetHandle(CAN_SensorTypeDef *Sensor, CAN_FIFO_RxTypeDef *Fifo_Rx)
{
	if (Fifo_Rx->Header.StdId == Sensor->RxHeader.StdId)
	{
		CAN_Sensor_Reset(Sensor);
		Fifo_Rx->rxflag = 0;
	}
}
