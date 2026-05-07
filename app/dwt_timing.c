#include "dwt_timing.h"

/* DWT 寄存器地址（Cortex-M4 与 M7 相同） */
#define DEM_CR      (*(__IO uint32_t *)0xE000EDFC)
#define DWT_CTRL    (*(__IO uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(__IO uint32_t *)0xE0001004)

#define DEM_CR_TRCENA     (1UL << 24)
#define DWT_CTRL_CYCCNTENA (1UL << 0)

static uint8_t  dwt_inited  = 0;
static uint32_t last_cyccnt = 0;
static float    time_sec    = 0.0f;

/* ================================================================
 *  DWT_Init
 *  使能 DWT 周期计数器，清零计时
 *  在 main() 里 HAL_Init() 之后调用一次
 * ================================================================ */
void DWT_Init(void)
{
    DEM_CR    |= DEM_CR_TRCENA;       /* 使能 DWT 模块 */
    DWT_CYCCNT = 0;                   /* 清零计数器 */
    DWT_CTRL  |= DWT_CTRL_CYCCNTENA; /* 启动 CYCCNT */

    dwt_inited  = 1;
    last_cyccnt = 0;
    time_sec    = 0.0f;
}

/* ================================================================
 *  DWT_GetTick
 *  返回自 DWT_Init() 起经过的时间（秒，float精度约1μs）
 *
 *  原理：uint32 差值法
 *    delta = now - last   ← uint32 减法，自动处理 32 位溢出
 *    time  += delta / 168MHz
 *
 *  前提：两次调用间隔 < 25.57s（F4 @ 168MHz 的溢出周期）
 *  TIM6 每 20ms 调一次，远满足此条件
 *
 *  注意：不覆盖 HAL_GetTick()，HAL_Delay() 不受影响
 * ================================================================ */
float DWT_GetTick(void)
{
    if (!dwt_inited) {
        DWT_Init();
    }

    uint32_t now   = DWT_CYCCNT;
    uint32_t delta = now - last_cyccnt;   /* uint32 减法：溢出时自动回绕正确 */
    last_cyccnt    = now;

    time_sec += (float)delta / (float)DWT_CLOCK_HZ;

    return time_sec;
}
