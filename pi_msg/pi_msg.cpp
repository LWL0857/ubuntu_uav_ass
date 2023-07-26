/*
 * pi_msg.cpp
 *
 *  Created on: May 25, 2021
 *      Author: lu
 */

#include "pi_msg.hpp"
#include "string.h"


const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/



RASPBERRYPI raspberrypi(USART1,(char *)"raspberrypi");
void USART1_IRQHandler(void)
{//串口的中断回调函数
//判断串口4是否空闲
	if(LL_USART_IsActiveFlag_IDLE(USART1))
	{
		LL_USART_ClearFlag_IDLE(USART1);
		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_5);
		LL_DMA_ClearFlag_DME5(DMA2);
		LL_DMA_ClearFlag_HT5(DMA2);
		LL_DMA_ClearFlag_TC5(DMA2);
		LL_DMA_ClearFlag_TE5(DMA2);
		LL_DMA_ClearFlag_FE5(DMA2);
		raspberrypi.RxDataSize = PIMSG_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5);
		do{
			//if(raspberrypi.RxRawDat[0] != 0x55 || raspberrypi.RxRawDat[1] != 0xaa||(raspberrypi.RxRawDat[2]+6) != raspberrypi.RxDataSize) break;
			if(raspberrypi.RxRawDat[0] != 0x55||(raspberrypi.RxDataSize!=33)) break;
			if(raspberrypi.LockRx == HAL_LOCKED) break;
			raspberrypi.LockRx = HAL_LOCKED;
			memcpy(raspberrypi.RxDat, raspberrypi.RxRawDat, raspberrypi.RxDataSize);
			raspberrypi.RxDat[raspberrypi.RxDataSize] = 0;
			raspberrypi.LockRx = HAL_UNLOCKED;
			raspberrypi.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(raspberrypiReceiveTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);



		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, PIMSG_RX_LEN);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
	}
}

void DMA2_Stream5_IRQHandler(void)
{

}

void DMA2_Stream7_IRQHandler(void)
{
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);
	LL_DMA_ClearFlag_TC7(DMA2);
	raspberrypi.TxFlag = false;

}

void RASPBERRYPI::raspberrypi_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;
	TxDat[0] = 0X55;

	osDelay(400);

	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_5);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, PIMSG_RX_LEN);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
	/* 配置接收DMA */

	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)&huart->TDR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, PIMSG_TX_LEN);
	LL_DMA_ClearFlag_TC7(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
	/* 配置发送DMA */

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_EnableDMAReq_TX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);

	Sta = STA_RUN;

}
bool RASPBERRYPI::uart_Send_Status(void)
{
	if(TxFlag == true) return false;

	xQueuePeek(queueAhrsEuler, &ahrsEuler, 0);
	xQueuePeek(queueEKF,&ekf,0);
    xQueuePeek(queueBattery, &battery, 0);
//	float ang[3];
//	float q0,q1,q2,q3;
//	q0 = ekf.q[0];
//	q1 = ekf.q[1];
//	q2 = ekf.q[2];
//	q3 = ekf.q[3];
//	ang[0] = atan2f(2.0f*(q0*q1+q2*q3),1.0f-2.0f*(q1*q1+q2*q2));
//	ang[1] = asinf(fConstrain(-2.0f * q1 * q3 + 2.0f * q0 * q2,-1.0f,1.0f));
//	ang[2] = atan2f(2.0f*(q0*q3+q1*q2),1.0f-2.0f*(q2*q2+q3*q3));

	//vs16 _temp;
	//vs32 _temp2;
	BIT32 temp32;
	TxDat[1] = 0x01;
	TxDat[2] = 22;
//	_temp = (s16)(ang[0]*R2D*100.0f);
	//_temp = (s16)(ahrsEuler.Ang[0]*R2D*100.0f);
	temp32.f=ahrsEuler.Ang[0]*R2D;
	TxDat[3] = temp32.data[0];
	TxDat[4] = temp32.data[1];
	TxDat[5] = temp32.data[2];
	TxDat[6] = temp32.data[3];
//	_temp = (s16)(ang[1]*R2D*100.0f);
//	_temp = (s16)(ahrsEuler.Ang[1]*R2D*100.0f);
	temp32.f=ahrsEuler.Ang[1]*R2D;
	TxDat[7] = temp32.data[0];
	TxDat[8] = temp32.data[1];
	TxDat[9] = temp32.data[2];
	TxDat[10] = temp32.data[3];
//	_temp = (s16)(ang[2]*R2D*100.0f);
//	_temp = (s16)(ahrsEuler.Ang[2]*R2D*100.0f);
	temp32.f=ahrsEuler.Ang[2]*R2D;
	TxDat[11] = temp32.data[0];
	TxDat[12] = temp32.data[1];
	TxDat[13] = temp32.data[2];
	TxDat[14] = temp32.data[3];    

//	_temp2 = (s32)ekf.pos[2]*(-100.0f);		//height
	temp32.f=ekf.pos[2]*(-1);
	TxDat[15] = temp32.data[0];
	TxDat[16] = temp32.data[1];
	TxDat[17] = temp32.data[2];
	TxDat[18] = temp32.data[3];
	temp32.f = battery.battery_Voltage;
	TxDat[19] = temp32.data[0];		
	TxDat[20] = temp32.data[1];		
	TxDat[21] = temp32.data[2];		
	TxDat[22] = temp32.data[3];		
	TxDat[23] = 0;		//模式
	TxDat[24] = 0;		//0加锁，1解锁

	uint8_t sum = 0;
	for(uint8_t i=3; i<25; i++)
	{
	sum += TxDat[i];
	} 
	TxDat[31] = sum & 0xff;
	TxDat[32]=0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);
}

bool RASPBERRYPI::uart_Send_IMU(void)
{
	if(TxFlag == true) return false;
	xQueuePeek(queueGyrDat, &gyro, 0);
	xQueuePeek(queueAccDat, &acc, 0);
	//vs16 _temp;
	TxDat[1] = 0x02;
	TxDat[2] = 24;
	temp32.f= acc.acc[0];
	TxDat[3] =  temp32.data[0];
	TxDat[4] =  temp32.data[1];
	TxDat[5] =  temp32.data[2];
	TxDat[6] =  temp32.data[3];
	temp32.f=acc.acc[1];
	TxDat[7] = temp32.data[0];
	TxDat[8] = temp32.data[1];
	TxDat[9] = temp32.data[2];
	TxDat[10] =temp32.data[3];
	temp32.f=acc.acc[2];
	TxDat[11] = temp32.data[0];
	TxDat[12] = temp32.data[1];
	TxDat[13] = temp32.data[2];
	TxDat[14] = temp32.data[3];
	temp32.f= gyro.gyro[0]*R2D;
	TxDat[15] = temp32.data[0];
	TxDat[16] = temp32.data[1];
	TxDat[17] = temp32.data[2];
	TxDat[18] = temp32.data[3];
	temp32.f= gyro.gyro[1]*R2D;
	TxDat[19] = temp32.data[0];
	TxDat[20] = temp32.data[1];
	TxDat[21] = temp32.data[2];
	TxDat[22] = temp32.data[3];
	temp32.f= gyro.gyro[2]*R2D;
	TxDat[23] = temp32.data[0];
	TxDat[24] = temp32.data[1];
	TxDat[25] = temp32.data[2];
	TxDat[26] = temp32.data[3];

	uint8_t sum = 0;
	for(uint8_t i=3; i<27; i++) sum += TxDat[i];
	TxDat[31] = sum&0xff;
	TxDat[32] = 0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);


}

bool RASPBERRYPI::uart_Send_MAG(void)
{
	if(TxFlag == true) return false;
	xQueuePeek(queueMagDat, &mag, 0);
	//vs16 _temp;
	TxDat[1] = 0x03;
	TxDat[2] = 12;
	temp32.f= mag.magraw[0];
	TxDat[3] =  temp32.data[0];
	TxDat[4] =  temp32.data[1];
	TxDat[5] =  temp32.data[2];
	TxDat[6] =  temp32.data[3];
	temp32.f=mag.magraw[1];
	TxDat[7] = temp32.data[0];
	TxDat[8] = temp32.data[1];
	TxDat[9] = temp32.data[2];
	TxDat[10] =temp32.data[3];
	temp32.f=mag.magraw[2];
	TxDat[11] = temp32.data[0];
	TxDat[12] = temp32.data[1];
	TxDat[13] = temp32.data[2];
	TxDat[14] = temp32.data[3];
	uint8_t sum = 0;
	for(uint8_t i=3; i<15; i++) sum += TxDat[i];
	TxDat[31] = sum&0xff;
	TxDat[32] = 0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);

}


bool RASPBERRYPI::uart_Send_UWB(void)
{	if(TxFlag == true) return false;
	xQueuePeek(queueUWB, &uwbPos, 0);					//从队列中获取U{WB数据	vs16 _temp;
	TxDat[1] = 0x04;
	TxDat[2] = 12;
	temp32.f= uwbPos.uwbPos[0];
	TxDat[3] =  temp32.data[0];
	TxDat[4] =  temp32.data[1];
	TxDat[5] =  temp32.data[2];
	TxDat[6] =  temp32.data[3];
	temp32.f=uwbPos.uwbPos[1];
	TxDat[7] = temp32.data[0];
	TxDat[8] = temp32.data[1];
	TxDat[9] = temp32.data[2];
	TxDat[10] =temp32.data[3];
	temp32.f=uwbPos.uwbPos[2];
	TxDat[11] = temp32.data[0];
	TxDat[12] = temp32.data[1];
	TxDat[13] = temp32.data[2];
	TxDat[14] = temp32.data[3];
	uint8_t sum = 0;
	for(uint8_t i=3; i<15; i++) sum += TxDat[i];
	TxDat[31] = sum&0xff;
	TxDat[32] = 0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);

}
bool RASPBERRYPI::uart_Send_MotorPWM(void)
{

	xQueuePeek(queueMotorPWM,&motorPWM, 0);
	if (TxFlag == true)
	return false;
	TxDat[1] = 0x05;
	TxDat[2] = 8;
	temp16.us = motorPWM.PWM[0];
	TxDat[3] = temp16.data[0];
	TxDat[4] = temp16.data[1];
	temp16.us = motorPWM.PWM[1];
	TxDat[5] = temp16.data[0];
	TxDat[6] = temp16.data[1];
	temp16.us = motorPWM.PWM[2];
	TxDat[7] = temp16.data[0];
	TxDat[8] = temp16.data[1];
	temp16.us = motorPWM.PWM[3];
	TxDat[9] = temp16.data[0];
	TxDat[10] = temp16.data[1];

	uint8_t sum = 0;
	for (uint8_t i = 3; i < 11; i++)
	sum += TxDat[i];
	TxDat[31] = sum & 0XFF;
	TxDat[32] = 0xbb;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);
}

bool RASPBERRYPI::uart_Send_rcData(void)
{
	if(TxFlag == true) return false;
	xQueuePeek(queueRCData, &rcPPM, 0);
	//vs16 _temp;
	TxDat[1] = 0x06;
	TxDat[2] = 16;
	temp16.us=rcPPM.PPM[2];
	TxDat[3] = temp16.data[0];
	TxDat[4] = temp16.data[1];
	temp16.us = rcPPM.PPM[0];
	TxDat[5] = temp16.data[0];
	TxDat[6] = temp16.data[1];
	temp16.us = rcPPM.PPM[3];
	TxDat[7] = temp16.data[0];
	TxDat[8] = temp16.data[1];
	temp16.us = rcPPM.PPM[1];
	TxDat[9] = temp16.data[0];
	TxDat[10] = temp16.data[1];
	temp16.us = rcPPM.PPM[4];
	TxDat[11] = temp16.data[0];
	TxDat[12] = temp16.data[1];
	temp16.us = rcPPM.PPM[5];
	TxDat[13] = temp16.data[0];
	TxDat[14] = temp16.data[1];
	temp16.us = rcPPM.PPM[6];
	TxDat[15] = temp16.data[0];
	TxDat[16] = temp16.data[1];
	temp16.us = rcPPM.PPM[7];
	TxDat[17] = temp16.data[0];
	TxDat[18] = temp16.data[1];
	uint8_t sum = 0;
	for(uint8_t i=3; i<19; i++) sum += TxDat[i];
	TxDat[31] = sum&0xff;
	TxDat[32] = 0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);


}
bool RASPBERRYPI::uart_Send_Flow(void)
{

	TxDat[1] = 0x07;
	TxDat[2] = 16;
	temp32.f= flow.flowx;
	TxDat[3] =  temp32.data[0];
	TxDat[4] =  temp32.data[1];
	TxDat[5] =  temp32.data[2];
	TxDat[6] =  temp32.data[3];
	temp32.f=flow.flowy;
	TxDat[7] = temp32.data[0];
	TxDat[8] = temp32.data[1];
	TxDat[9] = temp32.data[2];
	TxDat[10] =temp32.data[3];
	temp32.f=flow.flowx2;
	TxDat[11] = temp32.data[0];
	TxDat[12] = temp32.data[1];
	TxDat[13] = temp32.data[2];
	TxDat[14] = temp32.data[3];
	temp32.f= flow.flowy2;
	TxDat[15] = temp32.data[0];
	TxDat[16] = temp32.data[1];
	TxDat[17] = temp32.data[2];
	TxDat[18] = temp32.data[3];
	uint8_t sum = 0;
	for(uint8_t i=3; i<19; i++) sum += TxDat[i];
	TxDat[31] = sum&0xff;
	TxDat[32] = 0xBB;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);


}
/*
bool RASPBERRYPI::uart_Send_PID(uint8_t group)
{

	if(TxFlag == true) return false;
	vs16 temp;
	TxDat[2] = 0x7+group-1;
	TxDat[3] = 18;
	temp= pid.kp[group*3-3] * 1000;
	TxDat[4] = BYTE1(temp);
	TxDat[5] = BYTE0(temp);

	temp = pid.ki[group*3-3] * 100;
	TxDat[6] = BYTE1(temp);
	TxDat[7] = BYTE0(temp);

	temp = pid.kd[group*3-3] * 100;
	TxDat[8] = BYTE1(temp);
	TxDat[9] = BYTE0(temp);

	temp = pid.kp[group*3-2] * 1000;
	TxDat[10] = BYTE1(temp);
	TxDat[11] = BYTE0(temp);

	temp = pid.ki[group*3-2] * 100;
	TxDat[12] = BYTE1(temp);
	TxDat[13] = BYTE0(temp);

	temp = pid.kd[group*3-2] * 100;
	TxDat[14] = BYTE1(temp);
	TxDat[15] = BYTE0(temp);

	temp = pid.kp[group*3-1] * 1000;
	TxDat[16] = BYTE1(temp);
	TxDat[17] = BYTE0(temp);

	temp = pid.ki[group*3-1] * 100;
	TxDat[18] = BYTE1(temp);
	TxDat[19] = BYTE0(temp);

	temp = pid.kd[group*3-1] * 100;
	TxDat[20] = BYTE1(temp);
	TxDat[21] = BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0; i<22; i++) sum += TxDat[i];
	TxDat[22] = sum;
	return raspberrypi_Uart_Send_DMA((u8 *)TxDat, 33);
}
*/
bool RASPBERRYPI::raspberrypi_Uart_Send_DMA(uint8_t * pData,uint16_t Size)
{
	if(TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_7);

	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, Size);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)pData);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
	TxFlag = true;
	return true;
}

void RASPBERRYPI::raspberrypi_Uart_Send_Loop(void)
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	index++;
	switch(index%7)
	{
	case 0:
		uart_Send_Status();
		break;
	case 1:
		uart_Send_IMU();	//IMU和LSM303
		//uart_Send_adding();
		break;
	case 2:
		uart_Send_MAG();
		break;
	case 3:
		uart_Send_UWB();
		break;
	case 4:
		uart_Send_MotorPWM();
		break;
	case 5:
		uart_Send_rcData();
		break;
	case 6:
		/*
		if(send_pid == true)
		{
			xQueuePeek(queuePID, &pid, 0);
			send_pid_group = 1;
			send_pid = false;
		}
		if((send_pid_group > 0)&&(send_pid_group<7))
		{
			uart_Send_PID(send_pid_group);
			send_pid_group++;
		}
		*/
		break;

	}
	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;

}

void RASPBERRYPI::processMocapData(void)
{
	for(uint8_t i=0;i<7;i++)
	{
		memcpy(temp32.data,&RxDat[3+i*4],4);
		mocapPos.mocapPos[i] = temp32.f;
	}
		mocapPos.timestamp = startTimer;
		xQueueOverwrite(queueMOCAP,&mocapPos);
}
void RASPBERRYPI::raspberrypi_Uart_Receive_Update(void)
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;
	if(RxFlag == false) return;
	RxFlag = false;
	if(LockRx == HAL_LOCKED) return;
	LockRx = HAL_LOCKED;
	static unsigned char checkSum             = 0;

	do 
	{	int len=RxDat[2];
    	uint8_t sum=0;
    	for(int i=3;i<(len+3);i++)
    	{   
       	 sum+=RxDat[i];
   		}
		checkSum = sum & 0xff;
		// 检查信息校验值
		//if(checkSum != RxDat[31])break;		//和校验 判断sum
		uint8_t num=RxDat[1];
		switch(num)
		{
			case 0x01:
				processMocapData();
				break;
			case 0x03:
				break;
			default:
				break;

		}
		/*
		
		
		checkSum = getCrc8(RxDat, 3 + num);
		// 检查信息校验值
		if(checkSum != RxDat[num+3])break;		//和校验 判断sum
		for(uint8_t i=0;i<7;i++)
		{
			memcpy(temp64.data,&RxDat[3+i*8],8);
			mocapPos.mocapPos[i] = temp64.d;
		}
		mocapPos.timestamp = startTimer;
		xQueueOverwrite(queueMOCAP,&mocapPos);
		*/
			
	}while(0);

	LockRx = HAL_UNLOCKED;

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;

}

extern "C" void raspberrypi_main(void *argument)
{
	raspberrypi.raspberrypi_Init();
	osDelay(10);
	for(;;)
	{
		osSemaphoreAcquire(semRaspberrypi,0xffffffff);
		raspberrypi.raspberrypi_Uart_Send_Loop();
		led3_Tog;
	}
}

extern "C" void raspberrypiReceive_main(void *argument)
{

	osDelay(500);//等待系统完成初始化
	for(;;)
	{
		vTaskSuspend(raspberrypiReceiveTaskHandle);


		raspberrypi.raspberrypi_Uart_Receive_Update();
		led2_Tog;

	}
}


