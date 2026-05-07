#include "usart_app.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "CRC.h"

// ============================================================
// 串口调试打印
// ============================================================
int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512];
	va_list arg;
	int len;
	va_start(arg, format);
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}

// ============================================================
// 帧定义（从 dogarmcan 移植）
// 帧格式：[0x7B][cmd][crc_lo][crc_hi]，串口助手用HEX模式发送
// ┌──────────────────┬───────────────────┐
// │  功能            │  完整帧（HEX）    │
// ├──────────────────┼───────────────────┤
// │  停止            │  7B 00 D4 E4      │
// │  前进            │  7B 01 5D F5      │
// │  后退            │  7B 02 C6 C7      │
// │  左转            │  7B 03 4F D6      │
// │  右转            │  7B 04 F0 A2      │
// │  fold → 翻转     │  7B 05 79 B3      │
// │  fold → 收起     │  7B 06 E2 81      │
// │  chassis → 终点  │  7B 07 6B 90      │
// │  chassis → 原点  │  7B 08 9C 68      │
// │  气泵启动        │  7B 09 15 79      │
// │  气泵停止        │  7B 10 55 F4      │
// └──────────────────┴───────────────────┘
#define ARM_FRAME_HEADER  0x7B
#define ARM_FRAME_LEN     4

#define CMD_STOP           0x00
#define CMD_FORWARD        0x01
#define CMD_BACKWARD       0x02
#define CMD_TURN_LEFT      0x03
#define CMD_TURN_RIGHT     0x04
#define CMD_FOLD_END       0x05
#define CMD_FOLD_BEGIN     0x06
#define CMD_CHASSIS_END    0x07
#define CMD_CHASSIS_BEGIN  0x08
#define CMD_PUMP_START     0x09
#define CMD_PUMP_END       0x10

// ============================================================
// 导航数据（0xAA BB 帧，供 main.c Pure Pursuit 使用）
// ============================================================
float   uc_dx    = 0.0f;
float   uc_dy    = 0.0f;
uint8_t uc_valid = 0;

// ============================================================
// 发送机械臂控制帧给 dogarmcan（通过 USART6）
// 帧格式：[0x7B][cmd][crc_lo][crc_hi]
// ============================================================
void arm_send_cmd(uint8_t cmd)
{
	uint8_t frame[4] = {0x7B, cmd, 0x00, 0x00};
	Append_CRC16_Check_Sum(frame, sizeof(frame));
	HAL_UART_Transmit(&huart6, frame, sizeof(frame), 0xFF);
}

// ============================================================
// DMA + 环形缓冲（USART2，上位机）
// ============================================================
static uint8_t uart_rx_dma_buffer[127];
struct rt_ringbuffer uart_ringbuffer;
static uint8_t ringbuffer_pool[127];

void uart_init(void)
{
	rt_ringbuffer_init(&uart_ringbuffer, ringbuffer_pool, sizeof(ringbuffer_pool));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		HAL_UART_DMAStop(huart);
		rt_ringbuffer_put(&uart_ringbuffer, uart_rx_dma_buffer, Size);
		memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}

// ============================================================
// uart_task：在 main 的 while(1) 里调用
// ============================================================
void uart_task(void)
{
	uint8_t byte;
	while(rt_ringbuffer_getchar(&uart_ringbuffer, &byte) == 1){
		//arm_parse(byte);
		(void)byte;
	}
}
