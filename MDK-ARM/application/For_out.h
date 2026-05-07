#ifndef FOR_OUT_H
#define FOR_OUT_H
#include "main.h"
#include <math.h>

/***********************************使用方法************************************
********************************************************************************

适用范围：电机位置环、点对点跑点······																									

主函数前进行结构体声明：						Changer angle_changer[8];
主函数中主循环前进行变化器初始化：	for(uint8_t i=0;i<8;i++){Changer_int(&angle_changer[i]);}

主循环中的使用：
将Changer_calc(&angle_changer,target,step,T_A,T_C)赋值给变量

target：	目标值
step：		一次执行步长（速度）
T_A：			加速阶段->（T_A*变化线程）
T_C：			加速阶段->（T_C*变化线程）

例如：
		switch(RC_Ctl.rc.s1)
		{
			case 1:
				angle_CAN2[5]=Changer_calc(&angle_changer[0],30,1000,0.5,0.5);
				break;
			case 3:
				angle_CAN2[5]=8192*19.2f*Changer_calc(&angle_changer[0],0,200,0,0);
				break;
			case 2:
				angle_CAN2[5]=8192*19.2f*Changer_calc(&angle_changer[0],-30,600, 0.05, 0.05);
				break;
		}
																									
&&注意&&：当T_A和T_C同时为0时为匀速，当T_A和T_C单独为0时分别为仅加速和仅减速。			
&&注意&&：每次对程序进行复位将会有一次结构体初始化，也就意味着位置记忆将会消失。		

********************************************************************************
****************************************END************************************/

typedef struct {
	double target_val;     		
	double last_target_val;  
	double current_val;      
	double last_change;     
	double total_distance; 	
	double step_val;         
	double add;						
	int state;     						
	double current_step; 		  
	double current_step_2;   
} Changer;

extern Changer angle_changer[8];
void Changer_int(Changer* changer);
double Changer_calc(Changer*changer,float target,float step,float T_A,float T_C);

#endif

/*****************2747772618@qq.com*****************/
