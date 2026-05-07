/**
 * Function       CAN1和CAN2接收和发送
 * @author        GORY
 * @date          2025-07-23
 * @brief         
 * @retval     
 *
 */

#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include "struct_typedef.h"
#include "motor_control.h"
#include "ris_protocol.h"
#include "gld_can.h"


#include <stdio.h>
#include <math.h>
#include <stdint.h>


#pragma pack(1)

 /**
 * 
 * @file "motor_control.h"
 * @brief 
 * @brief 电机控制命令数据包
 *
 */
typedef struct										
{  
    RIS_Mode_t mode;    					// 电机控制模式  1Byte
    RIS_Comd_t comd;    					// 电机期望数据 12Byte
} ControlData_t_can; 

 /**
 * 
 * @file "motor_control.h"
 * @brief 
 * @brief 
 *
 */
typedef struct
{
    RIS_Mode_t mode;    					// 电机控制模式  1Byte
    RIS_Fbk_t_can   fbk;   				// 电机反馈数据 11Byte
} MotorData_t_can;

#pragma pack()

 /**
 * 
 * @file "motor_control.h"
 * @brief 
 * @brief CAN数据发送
 *
 */

typedef struct
{
    ControlData_t_can motor_can_send_data;
    unsigned short id;                  
    unsigned short mode;
																				//实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float T;                            //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                            //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                          //期望关节位置（rad）
    float K_P;                          //关节刚度系数
    float K_W;                          //关节速度系数
} MOTOR_can_send;

 /**
 * 
 * @file "motor_control.h"
 * @brief 
 * @brief CAN数据接收
 *
 */

typedef struct												
{ 
    MotorData_t_can motor_recv_data;    //电机接收数据结构体，详见motor_msg.h
																				//解读得出的电机数据
    unsigned char motor_id;             //电机ID
    unsigned char mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                           //温度
    unsigned char MError;               //错误码
		uint8_t  back	:2;
    float T;                            // 当前实际电机输出力矩
		float W;															// speed
    float Pos;                          // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
		int16_t k_pos;
    int16_t k_spd;
} MOTOR_can_recv;

 /**
 * 
 * @file "motor_control.h"
 * @brief 
 * @brief 
 *
 */
void unitree_setpara(MOTOR_can_send *motor_s,unsigned short id,unsigned short mode,float T,float W,float Pos,float K_P,float K_W);
void unitree_CAN_Send(CAN_HandleTypeDef *hcanx,uint32_t moduleId,uint32_t ctrlmode,MOTOR_can_send *motor_s);
extern MOTOR_can_send send;

#endif
