#include "pid_app.h"

/* PID 控制器实例 */
PID_T pid_speed_right; // 3508速度环
PID_T pid_angle_right; // 3508角度环


// 增量式PID：P-稳定性，I-响应性，D-准确性
// 位置式PID：P-响应性，I-准确性，D-稳定性

PidParams_t pid_params_right = {
	  .kp = 2.0f,   //1.0f,
    .ki = 0.0f,   //0.11f,
    .kd = 0.0f,  //0.12f,
    .out_min = -10000.0f,
    .out_max = 10000.0f,
};
int8_t i=0;
PidParams_t pid_params_angle = {
	  .kp = 2.0f,   //1.0f,
    .ki = 0.0f,   //0.11f,
    .kd = 0.0f,  //0.12f,
    .out_min = -10000.0f,
    .out_max = 10000.0f,
};

void PID_Init(void)
{
  
  pid_init(&pid_speed_right,
           pid_params_right.kp, pid_params_right.ki, pid_params_right.kd,
           0.0f, pid_params_right.out_max);
	
  pid_init(&pid_angle_right,
           pid_params_angle.kp, pid_params_angle.ki, pid_params_angle.kd,
           0.0f, pid_params_angle.out_max);
	// 1. 设置目标角度（单位：度）
	float target_angle_degree = 540.0f*19;
	// 2. 将角度值转换为编码器值
	float target_angle_enc=degree_to_encoder(target_angle_degree);
	
  pid_set_target(&pid_speed_right, 2000.0f);
  pid_set_target(&pid_angle_right, target_angle_enc);
	
}

// 低通滤波器系数 (Low-pass filter coefficient 'alpha')
// alpha 越小, 滤波效果越强, 但延迟越大。建议从 0.1 到 0.5 之间开始尝试。
#define SPEED_FILTER_ALPHA_LEFT  0.15f 
#define SPEED_FILTER_ALPHA_RIGHT 0.15f 

// 用于存储滤波后速度的变量
//static float filtered_speed_right = 0.0f;

bool pid_running = true; // PID 控制使能开关

unsigned char pid_control_mode = 0; // 0-角度环控制，1-循迹环控制
int output_right;

void PID_Task(void)
{
//    if(pid_running == false) return;

    
                                                     //当前转速
  	//filtered_speed_right = SPEED_FILTER_ALPHA_RIGHT * right_encoder.speed_cm_s + (1.0f - SPEED_FILTER_ALPHA_RIGHT) * filtered_speed_right;

    // 使用位置式 PID 计算利用速度环计算输出
    output_right = pid_calculate_incremental(&pid_speed_right, motor_chassis[0].speed_rpm);
    
	//限幅函数
//    output_right = pid_constrain(output_right, pid_params_right.out_min, pid_params_right.out_max);
   
	// 设置电流
    CAN_cmd_chassis(output_right,0,0,0);
	                                                            //目标    4000            实际                   电流
   //	my_printf(&huart1,"{3508}%.2f,%d,%d\r\n", pid_speed_right.target, motor_chassis[0].speed_rpm, output_right);//,%.2f,,output_right
	
	
}





























PidParams_t pid_params_line = {
	.kp = 4.0f,
    .ki = 0.01f,
    .kd = 0.1f,
    .out_min = -999.0f,
    .out_max = 999.0f,
};


//void PID_Task(void)
//{
//  if(pid_running == false) return;
//  
//	float output_left, output_right;

//	// 使用增量式 PID 计算利用速度环计算输出
//	output_left = pid_calculate_incremental(&pid_speed_left, left_encoder.speed_cm_s);
//	output_right = pid_calculate_incremental(&pid_speed_right, right_encoder.speed_cm_s);
//    
//	// 设置电机速度
//	Motor_SetSpeed(&left_motor, output_left);
////	Motor_SetSpeed(&right_motor, output_right);
////	my_printf(&huart1,"{left}%d,%d\r\n",(int)(pid_speed_left.target*100),(int)(left_encoder.speed_cm_s*100));
////	my_printf(&huart1,"{right}%d,%d\r\n",(int)(pid_speed_right.target*100),(int)(right_encoder.speed_cm_s*100));	
//	my_printf(&huart1,"{left}%.2f,%.2f\r\n",pid_speed_left.target,left_encoder.speed_cm_s);
////	my_printf(&huart1,"{right}%.2f,%.2f\r\n",pid_speed_right.target,right_encoder.speed_cm_s);
//}

