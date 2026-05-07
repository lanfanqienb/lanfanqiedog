#include "For_out.h"
/***************************************************************************************
  * 
  * 你可以任意查看、使用和修改，并应用到自己的项目之中
  * 程序版权归私人所有，任何人或组织不得将其据为己有
  * 
  * 程序名称：				一种基于高频循环运行环境对数据变化速率的控制算法
  * 程序创建时间：			2025.11.24
  * 当前程序版本：			V1.0
  * 当前版本发布时间：		2025.11.24
  * 
  * 如果你发现程序中的漏洞或者笔误，可通过邮件向我们反馈：2747772618@qq.com
  * 发送邮件之前，你可以先到更新动态页面查看最新程序，如果此问题已经修改，则无需再发邮件
  ***************************************************************************************
  */
	
/**
 * @brief 辅助计算变量定义
 * @param mode_for 运动模式（仅加速/仅减速/匀速/加匀减）
 * @param stage_for 运动阶段（加速/减速/匀速）
 * @param enable_for 执行确认标识
 * @param direction 运动方向
 * @param distance 目标运动线程
 * @param distance_C 已运动线程
 * @param much_one_A 加速一次多少
 * @param much_one_C 减速一次多少
 */
uint8_t mode_for,stage_for,enable_for;
float direction,distance,distance_C;
float much_one_A,much_one_C;

/**
 * @brief 初始化可控速数据变化器
 * @param target_val 目标值
 * @param last_target_val 上一次目标值
 * @param current_val 当前值
 * @param last_change 一次变化后最终停留值
 * @param total_distance 变化线程绝对值
 * @param step_val 定义最大步长
 * @param add 当下步长
 * @param state 是否在变化中状态值
 * @param current_step 加匀速程序执行次数
 * @param current_step_2 减速程序执行次数
 */
void Changer_int(Changer* changer)
{    
  changer->target_val=0;
	changer->last_target_val=0;
	changer->current_val=0;
	changer->last_change=0;
	changer->total_distance=0;	
	changer->step_val=0;
	changer->add=0;
	changer->state=1;
	changer->current_step=0;
	changer->current_step_2=0;
}

/**
 * @brief 可控速数据变化器
 * @param target 目标值
 * @param step 定义步长
 * @param T_A 加速阶段比例(0-1)
 * @param T_C 减速阶段比例 (0-1)
 * @return 当前变化值
 */
double Changer_calc(Changer* changer, float target, float step, float T_A, float T_C)
{	
//复位重启				
	if(changer->last_target_val!=changer->target_val)
	{	
		changer->total_distance=fabs(target-changer->current_val);
		changer->last_change=changer->current_val;
		changer->current_step=0;
		changer->current_step_2=0;
		changer->state=1;
	}
	changer->last_target_val=changer->target_val;
		
//参数检查	
	if(mode_for==2)
	{
		T_A=(T_A<0)?0:((T_A>1)?1:T_A);
		T_C=(T_C<0)?0:((T_C>1)?1:T_C);
		if(T_A+T_C>1){T_A=0;T_C=0;}
	}
	
//参数赋值	
	much_one_A=changer->step_val/((2*changer->total_distance*T_A)/changer->step_val);										
	much_one_C=changer->step_val/((2*changer->total_distance*T_C)/changer->step_val);											 
	direction=(changer->target_val>changer->current_val)?1.0f:-1.0f; 
	distance=fabs(changer->current_val-changer->target_val);				
	distance_C=fabs(changer->current_val-changer->last_change);	
	changer->step_val=step/10000000;
	changer->target_val=target;
	stage_for=3;	
		
//状态判断
	if(T_A==0&&T_C==0){mode_for=1;}
	else if(T_A!=0&&T_C!=0){mode_for=2;}
	else if(T_A!=0&&T_C==0){mode_for=3;}
	else if(T_A==0&&T_C!=0){mode_for=4;}
	if((distance_C<=changer->total_distance*T_A)&&T_A!=0){stage_for=1;}
	else if((distance_C>changer->total_distance*(1-T_C))&&T_C!=0){stage_for=2;}
		
//计算过程		
  if(changer->state==0)return changer->current_val;
  if(changer->total_distance==0)return changer->current_val;
	if(distance<=changer->step_val&&((changer->add<=much_one_C)&&stage_for==2))
	{
		changer->state=0;
		changer->current_val=target;
		changer->last_change=changer->current_val;
		return changer->current_val;
	}
		
	switch(stage_for){
		case 1:
			changer->add=much_one_A*changer->current_step;
			changer->current_val+=direction*changer->add;
			changer->current_step+=1;
			break;
		case 2:
			changer->add=changer->step_val-(much_one_C*changer->current_step_2);
			changer->current_val+=direction*changer->add;
			changer->current_step_2+=1;
			changer->current_step+=1;
			break;
		case 3:
			enable_for=0;
			switch(mode_for)
			{
				case 1:enable_for=1;break;
				case 2:enable_for=(distance_C>changer->total_distance*T_A)&&(distance_C<=changer->total_distance*(1-T_C));break;
				case 3:enable_for=distance_C>changer->total_distance*T_A;break;
				case 4:enable_for=distance_C<=changer->total_distance*(1-T_C);break;
			}	
			if(enable_for==1)
			{
				changer->add=changer->step_val;
				changer->current_val+=direction*changer->add;
				changer->current_step+=1;	
			}
			break;
		}	
	return changer->current_val;
}	


 /*****************2747772618@qq.com*****************/
