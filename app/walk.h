#ifndef WALK_H
#define WALK_H

#include "stdint.h"

/* ================================================================
 *  步态参数
 *  所有长度单位 m，时间单位 s
 *  坐标系：x向前为正，z向上为正（stance_height 为负值，绝对值越大腿越伸，机体越高）
 * ================================================================ */
typedef struct {
    float stance_height[4]; /* 各腿站立高度(z)，0=右后,1=左前,2=左后,3=右前 */
    float step_length[4];   /* 各腿步长，0=右后,1=左前,2=左后,3=右前         */
    float step_height[4];   /* 各腿摆动抬腿高度，后腿需小于前腿              */
    float term;           /* 步态周期 (s)，如 0.4f                 */
    float kp;             /* FC_Send_Pos 的位置增益                */
    float kw;             /* FC_Send_Pos 的速度增益                */
    float turn_rate;      /* 转向量(m)：正=右转，负=左转
                           * 左腿(1,2) step = step_length + turn_rate
                           * 右腿(0,3) step = step_length - turn_rate
                           * step_length=0 时即原地转 */
} WalkParam;

extern WalkParam walk_param;

/* 进入步态：采样当前足端位置，记录起始时间
 * 在切换到 mode=2 之前调用一次 */
void walk_enter(void);

/* 步态主循环：在 TIM6 中断里每次调用
 * 内部自动推进 *flah（0→1→2→3→0），每次发送一条腿 */
void walk_run(uint8_t *flah);

#endif /* WALK_H */
