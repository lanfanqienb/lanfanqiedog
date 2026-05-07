#ifndef DWT_TIMING_H
#define DWT_TIMING_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

/* STM32F407 系统时钟 168MHz
 * DWT_CYCCNT 是 32位计数器，每个 CPU clock +1
 * 溢出周期 = 2^32 / 168000000 ≈ 25.57s
 * 采用 uint32 差值法，自动处理溢出，无需关心溢出次数 */
#define DWT_CLOCK_HZ  168000000UL

/* 初始化 DWT 计数器（main里调一次，在 HAL_Init 之后） */
void DWT_Init(void);

/* 返回自 DWT_Init 以来的经过时间（秒）
 * 每次调用读一次 DWT_CYCCNT，做差累加
 * 必须在 TIM6 中断里定期调用（≤25s 调一次），否则会漏掉溢出
 * 用法同 corgi 的 GetTick()：tick_start = DWT_GetTick() 记录起点 */
float DWT_GetTick(void);

#endif /* DWT_TIMING_H */
