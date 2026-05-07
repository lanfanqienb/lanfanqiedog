/**
 * Function       CAN1和CAN2接收和发送
 * @author        GORY
 * @date          2025-07-23
 * @brief         
 * @retval  
 * 
 */
#include "gld_can.h"

 /**
 * 
 * @file "gld_can.h"
 * @brief 
 * @brief 制定上下限度
 *
 */
 
#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

 /**
 * 
 * @file "gld_can.h"
 * @brief 
 * @brief 初始化变量
 *
 */
 
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
 
uint8_t CAN1_ID =0;
uint8_t CAN2_ID =0;
extern MOTOR_can_recv UT_motor_data[8];
uint8_t unitree_can_send_data[8];
uint8_t unitree_can_send_kkdata[8]={0};
uint8_t unitree_CAN_ID;

 /**
 * 
 * @file "gld_can.h"
 * @brief 
 * @brief 函数声明
 *
 */

void CAN_modify_data(MOTOR_can_send *motor_s);
void can_filter_init(void);
void unitree_CAN_Send(CAN_HandleTypeDef *hcanx,uint32_t moduleId,uint32_t ctrlmode,MOTOR_can_send *motor_s);
void unitree_CAN_receive(CAN_RxHeaderTypeDef rx_header,MOTOR_can_recv *ptr,uint8_t* Data);
void unitree_setpara(MOTOR_can_send *motor_s,unsigned short id,unsigned short mode,float T,float W,float Pos,float K_P,float K_W);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

 /**
 * 
 * @file "gld_can.h"
 * @brief Configure motor data limits
 * @brief 配置电机数据限值
 *
 */
 
void CAN_modify_data(MOTOR_can_send *motor_s)
{
	SATURATE(motor_s->K_P,  0.0f,   25.599f);
	SATURATE(motor_s->K_W,  0.0f,   25.599f);
	SATURATE(motor_s->T,   -127.99f,  127.99f);
	SATURATE(motor_s->W,   -804.00f,  804.00f);
	SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

	motor_s->motor_can_send_data.mode.id   = motor_s->id;
	motor_s->motor_can_send_data.mode.status  = motor_s->mode;
	motor_s->motor_can_send_data.comd.k_pos  = (int16_t)(motor_s->K_P*1280.0f);
	motor_s->motor_can_send_data.comd.k_spd  = (int16_t)(motor_s->K_W*1280.0f);
	motor_s->motor_can_send_data.comd.pos_des  = (int32_t)(motor_s->Pos/6.2832f*32768);
	motor_s->motor_can_send_data.comd.spd_des  = (int16_t)(motor_s->W/6.2832f*256);
	motor_s->motor_can_send_data.comd.tor_des  = (int16_t)(motor_s->T*256);
}

 /**
 * 
 * @file "gld_can.h"
 * @brief Configure the filter for CAN
 * @brief 配置can的过滤器
 *
 */
 
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st_1;
    can_filter_st_1.FilterActivation = ENABLE;
    can_filter_st_1.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st_1.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st_1.FilterIdHigh = 0x0000;
    can_filter_st_1.FilterIdLow = 0x0000;
    can_filter_st_1.FilterMaskIdHigh = 0x0000;
    can_filter_st_1.FilterMaskIdLow = 0x0000;
    can_filter_st_1.FilterBank = 0;
    can_filter_st_1.FilterFIFOAssignment = CAN_RX_FIFO0;
	
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st_1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	  CAN_FilterTypeDef can_filter_st_2;
    can_filter_st_2.FilterActivation = ENABLE;
    can_filter_st_2.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st_2.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st_2.FilterIdHigh = 0x0000;
    can_filter_st_2.FilterIdLow = 0x0000;
    can_filter_st_2.FilterMaskIdHigh = 0x0000;
    can_filter_st_2.FilterMaskIdLow = 0x0000;
    can_filter_st_2.SlaveStartFilterBank = 14;
    can_filter_st_2.FilterBank = 14;	
    can_filter_st_2.FilterFIFOAssignment = CAN_RX_FIFO0;
	
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st_2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

 /**
 * 
 * @file "gld_can.h"
 * @brief About CAN transmission
 * @brief CAN发送函数
 *29位扩展ID的结构：
 * ┌──────┬──────┬──────────┬──────────┬──────────┐
 * │28:27 │26:17 │  16:16   │  15:8    │   7:0    │
 * │2 bit │10bit │  1 bit   │  8 bit   │  8 bit   │
 * ├──────┼──────┼──────────┼──────────┼──────────┤
 * │module│ 保留 │ ctrlmode │ motor_id │ 其他信息  │
 * │  ID  │      │          │          │          │
 * └──────┴──────┴──────────┴──────────┼──────────┤
 * ctrlmode（控制模式）的含义：
 * ctrlmode	十进制	方向	数据内容	用途
 * 10		主机→电机	τ, ω, θ（PVT）	发送位置速度力矩
 * 11		主机→电机	Kp, Kd（KK）	发送刚度阻尼
 * 12		主机→电机	空	读取Kp和Kd
 * 13		主机→电机	mode2参数	特殊模式

 */
void unitree_CAN_Send(CAN_HandleTypeDef *hcanx,uint32_t moduleId,uint32_t ctrlmode,MOTOR_can_send *motor_s)
{
	uint32_t send_mail_box1,send_mail_box2;
	 // 【步骤1】数据转换
	CAN_modify_data(motor_s);
	
	CAN_TxHeaderTypeDef TxMessage;
	// 【步骤2】配置CAN帧头
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.ExtId =(0|(moduleId<<27)|(((uint8_t *)(motor_s))[0]<<8)|(ctrlmode<<16));// 模块ID,电机ID,控制模式
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC =8;
	TxMessage.TransmitGlobalTime = DISABLE;
	
	switch(ctrlmode){
		case 10:
		unitree_can_send_data[0]=(motor_s->motor_can_send_data.comd.pos_des)&0xFF;
		unitree_can_send_data[1]=((motor_s->motor_can_send_data.comd.pos_des)>>8)&0xFF;
		unitree_can_send_data[2]=((motor_s->motor_can_send_data.comd.pos_des)>>16)&0xFF;
		unitree_can_send_data[3]=((motor_s->motor_can_send_data.comd.pos_des)>>24)&0xFF;
		
		unitree_can_send_data[4]=(motor_s->motor_can_send_data.comd.spd_des)&0xFF;
		unitree_can_send_data[5]=((motor_s->motor_can_send_data.comd.spd_des)>>8)&0xFF;
		
		unitree_can_send_data[6]=(motor_s->motor_can_send_data.comd.tor_des)&0xFF;
		unitree_can_send_data[7]=((motor_s->motor_can_send_data.comd.tor_des)>>8)&0xFF;
		
		HAL_CAN_AddTxMessage(hcanx, &TxMessage,unitree_can_send_data,&send_mail_box1);break;
		case 11:
		unitree_can_send_kkdata[0]=(motor_s->motor_can_send_data.comd.k_spd)&0xFF;
		unitree_can_send_kkdata[1]=((motor_s->motor_can_send_data.comd.k_spd)>>8)&0xFF;
		
		unitree_can_send_kkdata[2]=(motor_s->motor_can_send_data.comd.k_pos)&0xFF;
		unitree_can_send_kkdata[3]=((motor_s->motor_can_send_data.comd.k_pos)>>8)&0xFF;
		
		HAL_CAN_AddTxMessage(hcanx, &TxMessage,unitree_can_send_kkdata,&send_mail_box2);
		
		break;


	}
}

 /**
 * 
 * @file "gld_can.h"
 * @brief About CAN Accept
 * @brief CAN接收函数
 *
 */

void unitree_CAN_receive(CAN_RxHeaderTypeDef rx_header,MOTOR_can_recv *ptr,uint8_t* Data)
{
	ptr->motor_id=ptr->motor_recv_data.mode.id=(((rx_header.ExtId)>>8)&0x0F);
	ptr->mode=ptr->motor_recv_data.mode.status=(((rx_header.ExtId)>>12)&0x07);
	ptr->Temp = ptr->motor_recv_data.fbk.temp=rx_header.ExtId;
	
	if(ptr->Temp==-128){ptr->motor_recv_data.fbk.MError=(rx_header.ExtId)>>16;}	
	else{ptr->motor_recv_data.fbk.force=((rx_header.ExtId)>>16)&0xFF;}	
	
	ptr->back=ptr->motor_recv_data.fbk.back=(rx_header.ExtId)>>24;
	ptr->motor_recv_data.fbk.mokid=(rx_header.ExtId)>>27;
	
	if(ptr->back==1){
		ptr->motor_recv_data.fbk.pos=(int32_t)((Data[3]<<24)|(Data[2]<<16)|(Data[1]<<8)|Data[0]);
		ptr->motor_recv_data.fbk.speed=(int16_t)((Data[5]<<8)|Data[4]);
		ptr->motor_recv_data.fbk.torque=(int16_t)((Data[7]<<8)|Data[6]);
	}	
	else if(ptr->back==2){
		ptr->k_spd=(int16_t)((Data[1]<<8)|Data[0]);
		ptr->k_pos=(int16_t)((Data[3]<<8)|Data[2]);
	}	

	ptr->W = ((float)ptr->motor_recv_data.fbk.speed/256)*6.2832f;
	ptr->T = ((float)ptr->motor_recv_data.fbk.torque) / 256;
	ptr->Pos = 6.2832f*((float)ptr->motor_recv_data.fbk.pos) / 32768;	
}

 /**
 * 
 * @file "gld_can.h"
 * @brief About CAN sends data assignment
 * @brief CAN发送数据赋值函数
 *
 */

void unitree_setpara(MOTOR_can_send *motor_s,unsigned short id,unsigned short mode,float T,float W,float Pos,float K_P,float K_W)
{
	motor_s->id=id;
	motor_s->mode=mode;
	motor_s->K_P=K_P;
	motor_s->K_W=K_W;
	motor_s->Pos=Pos;
	motor_s->T=T;
	motor_s->W=W;
}	

 /**
 * 
 * @file "gld_can.h"
 * @brief About CAN receive interrupt callback
 * @brief CAN接收中断回调
 *
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1){
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
		switch(((rx_header.ExtId)>>8)&0x0F){
			case 0:
			case 1:
			case 2:	
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:	
			case 8:{
				unitree_CAN_ID=((rx_header.ExtId)>>8)&0x0F;
				unitree_CAN_receive(rx_header,&UT_motor_data[unitree_CAN_ID],rx_data);break;}	
			default:{break;}}}
	if(hcan->Instance == CAN2){
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
		switch(((rx_header.ExtId)>>8)&0x0F){
			case 0:
			case 1:
			case 2:	
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:	
			case 8:{
				unitree_CAN_ID=((rx_header.ExtId)>>8)&0x0F;
				unitree_CAN_receive(rx_header,&UT_motor_data[unitree_CAN_ID],rx_data);break;}	
			default:{break;}}}
}
