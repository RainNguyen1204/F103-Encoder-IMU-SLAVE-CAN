#ifndef ENCODERPOSITION_H_
#define ENCODERPOSITION_H_

#include "main.h"

#define PI			3.14159265358979323846 
#define ZX_PIN	GPIO_PIN_3
#define ZY_PIN	GPIO_PIN_4


typedef struct
{
	int16_t						round_counter			;
	uint8_t						current_direction	; // 0 = CNT d�m l�n || 1 = CNT d�m xu�ng
	uint8_t						last_direction		;
	int32_t						pulse							;
	uint16_t					current_CNT_value	;
	uint16_t					last_CNT_value		;
	int16_t						offset						;
	uint16_t 					resolution				;
	float							position					;

}Encoder_HandleTypeDef;

void Encoder_Update_Data(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolutin); //Cap nh�t du li�u
void Encoder_Round_Counter(Encoder_HandleTypeDef *encoder);//��m v�ng
void Encoder_Pulse_Counter(Encoder_HandleTypeDef *encoder);// �em xung
void Encoder_Pulse_Calibration(Encoder_HandleTypeDef *encoder); // Hieu chinh sai so so xung
uint8_t Encoder_Offset_Detect(Encoder_HandleTypeDef *encoder); //Tim d� lech so voi vi tri dat robot
void Encoder_Position_Handle(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution, float wheel_diameater); //T�nh vi tr�
void Encoder_Reset(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim); //Reset Encoder + CNT

#endif



