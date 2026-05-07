#ifndef __USART_APP_H__
#define __USART_APP_H__

#include "mydefine.h"

int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
void uart_init(void);
void uart_task(void);
void arm_send_cmd(uint8_t cmd);

/* 机械臂指令码 */
#define ARM_CMD_STOP           0x00
#define ARM_CMD_FORWARD        0x01
#define ARM_CMD_BACKWARD       0x02
#define ARM_CMD_TURN_LEFT      0x03
#define ARM_CMD_TURN_RIGHT     0x04
#define ARM_CMD_FOLD_END       0x05
#define ARM_CMD_FOLD_BEGIN     0x06
#define ARM_CMD_CHASSIS_END    0x07
#define ARM_CMD_CHASSIS_BEGIN  0x08
#define ARM_CMD_PUMP_START     0x09
#define ARM_CMD_PUMP_END       0x10
#define ARM_CMD_AIR1_START     0x11  /* 空气流通1开启 */
#define ARM_CMD_AIR1_END       0x12  /* 空气流通1关闭 */
#define ARM_CMD_PUMP2_START    0x13  /* 吸盘2启动 */
#define ARM_CMD_PUMP2_END      0x14  /* 吸盘2停止 */
#define ARM_CMD_AIR2_START     0x15  /* 空气流通2开启 */
#define ARM_CMD_AIR2_END       0x16  /* 空气流通2关闭 */

/* 导航数据，由 uart_task 更新，main.c Pure Pursuit 读取 */
extern float   uc_dx;
extern float   uc_dy;
extern uint8_t uc_valid;

#endif
