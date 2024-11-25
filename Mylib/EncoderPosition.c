
#include "EncoderPosition.h"

uint8_t offset_flag = 0;



//Cap nhat c�c gi� tri cho encoder qua timer
void Encoder_Update_Data(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution)
{
	//Gan c�c gi� tri cua timer v�o encoder
	encoder->current_direction = (htim->Instance->CR1 & TIM_CR1_DIR) >> 4;
	encoder->current_CNT_value = htim->Instance->CNT;
	encoder->resolution = resolution;
}

void Encoder_Round_Counter(Encoder_HandleTypeDef *encoder) //��m s� vong quay cua encoder qua xung Z
{
		if (encoder->current_direction != 0)
		{
			encoder->round_counter--;
		}
		else
		{
			encoder->round_counter++;
		}	
		//skip 1 xung Z n�u encoder quay nguoc chi�u dang quay
		if(encoder->last_direction != encoder->current_direction)
		{
			if (encoder->current_direction != 0)
			{
				encoder->round_counter++;
			}
			else
			{
				encoder->round_counter--;
			}
		}
	encoder->last_direction = encoder->current_direction;
}	

uint8_t Encoder_Offset_Detect(Encoder_HandleTypeDef *encoder) //Tim d� lech so voi vi tri dat robot
{
	if (!offset_flag)
	{
		encoder->offset = encoder->pulse; //Luu so xung lech so voi vi tri dat robot
		encoder->last_direction = encoder->current_direction;
		offset_flag++;
		return 0;
	}
	else
	{
		return 1; 
	}
}


void Encoder_Pulse_Counter(Encoder_HandleTypeDef *encoder)	//��m s� xung
{
	if (encoder->last_CNT_value > encoder->current_CNT_value)
	{
		if (encoder->current_direction != 0) //CNT dem xuong
		{
			if ((encoder->last_CNT_value - encoder->current_CNT_value) >= 4)
			{
				encoder->pulse--;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
		else	//CNT dem l�n
		{
			if ((encoder->last_CNT_value - encoder->current_CNT_value) >= 4)
			{
				encoder->pulse++;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
	}
	if (encoder->last_CNT_value < encoder->current_CNT_value)
	{
		if (encoder->current_direction != 0) //CNT dem xuong
		{
			if ((encoder->current_CNT_value - encoder->last_CNT_value) >= 4)
			{
				encoder->pulse--;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
		else	//CNT dem l�n
		{
			if ((encoder->current_CNT_value - encoder->last_CNT_value) >= 4)
			{
				encoder->pulse++;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
	}
}




void Encoder_Pulse_Calibration(Encoder_HandleTypeDef *encoder) //Hi�u chinh sai s� s� xung 
{
	Encoder_Round_Counter(encoder); //��m so vong qua xung Z
	encoder->pulse = encoder->round_counter * 1000 + encoder->offset;
}

void Encoder_Position_Handle(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution, float wheel_diameater) //T�nh vi tr�
{
	Encoder_Update_Data(encoder, htim, resolution);
	Encoder_Pulse_Counter(encoder);
	encoder->position = PI*wheel_diameater*encoder->pulse/encoder->resolution; //Tinh quang duong

}

void Encoder_Reset(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)//Reset Encoder + CNT
{
	htim->Instance->CNT = 0;
	encoder->current_CNT_value = encoder->current_direction = encoder->last_CNT_value
	= encoder->last_direction = encoder->offset = encoder->position = encoder->pulse
	= encoder->resolution = encoder->round_counter = 0;
	offset_flag = 0;
}	


