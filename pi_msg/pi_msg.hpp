/*
 * pi_msg.hpp
 *
 *  Created on: May 25, 2021
 *      Author: lu
 */

#ifndef MODULES_PI_MSG_PI_MSG_HPP_
#define MODULES_PI_MSG_PI_MSG_HPP_


#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define PIMSG_RX_LEN		150
#define PIMSG_TX_LEN		62
#define START   0X11

void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
using namespace std;

class RASPBERRYPI
{
public:
	RASPBERRYPI(){}
	RASPBERRYPI(USART_TypeDef * h,char * n) : huart(h),name(n){}
	~RASPBERRYPI(){}

	void raspberrypi_Init(void);
	//void raspberrypi_Uart_Send(void);
	bool raspberrypi_Uart_Send_DMA(uint8_t * pData,uint16_t Size);
	//send loop
	void raspberrypi_Uart_Send_Loop(void);
	bool uart_Send_Status(void);//0x01
	bool uart_Send_IMU(void);//0x02
	bool uart_Send_MAG(void);//0x03
	bool uart_Send_UWB(void);//0x04
	bool uart_Send_MotorPWM(void);//0x05
	bool uart_Send_rcData(void);//0x06
	bool uart_Send_Flow(void);//0x07
	bool uart_Send_PID(uint8_t group);
	void processMocapData(void);

	void raspberrypi_Uart_Receive_Update(void);


	HAL_LockTypeDef LockRx;
	bool  RxFlag;		//接收标志位
	uint16_t   RxDataSize;
	unsigned char    RxRawDat[PIMSG_RX_LEN];	//数据
	unsigned char    RxDat[PIMSG_RX_LEN];
	bool  TxFlag;		//接收标志位

private:
	uint32_t  index;		//发送数据序号,每发完一帧数据加一
	BIT64 temp64;
	BIT32 temp32;
	BIT16 temp16;
	
	USART_TypeDef * huart;
	char * name;

	//接收有关的定义数据类型，有待修改
	char    TxHead;
	char    TxSum;
	uint8_t    TxDat[PIMSG_TX_LEN];

	uint32_t  startTimer;			//计时器
	uint32_t  stopTimer;			//计时器
	uint32_t  executionTime_us;			//计时器
	uint32_t  startTimerLast;
	uint32_t  cycleTime_us;
	//用户数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
	mocap_pos_msg mocapPos;
	uint16_t n_seg;
	//未知参数
	float ts[20];
	float poly_coeff[3][160];
	//transfer
	sensor_gyro_msg gyro;
		sensor_acc_msg acc;
		sensor_mag_msg mag;
		ahrs_euler_msg ahrsEuler;
		battery_msg battery;
		RCData rcPPM;
		Pid_msg pid;
		Motor_PWM_msg motorPWM;
		RC_command_msg rcCommand;
		sensor_gyro_filter_msg gyro_filter;
		Inloop_control_msg inloop_control;
		flow_msg flow;
		height_msg height;
		mahonyAhrs_msg mahonyAhrs_Ang;
		ekf_uwb_pos_vel_msg ekf_uwb_pos_vel;
		motor_rps_msg motorRps;
		uwb_pos_msg uwbPos;
		Cb2n_msg MatCb2n;
		q_msg q_loop;
		ekf_msg ekf;
		sensor_baroAlt_msg baroAlt;
		Control_msg control_msg;

};



#endif


#endif /* MODULES_PI_MSG_PI_MSG_HPP_ */
