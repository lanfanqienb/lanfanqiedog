#include "walk.h"
#include "dog.h"
#include "dwt_timing.h"
#include "math.h"

/* ================================================================
 *  默认步态参数（上机后根据实际效果调整）
 *    stance_height : -0.22m（与 IK 站立保持一致）
 *    step_length   : 0.06m（保守值，稳定后可加大）
 *    step_height   : 0.04m（抬腿高度）
 *    term          : 0.4s （步频 2.5Hz，较慢，便于调试）
 *    kp / kw       : 与 mode=1 站立保持一致，先用相同值
 * ================================================================ */
WalkParam walk_param = {
    .stance_height = {-0.219f, -0.219f, -0.219f, -0.219f},
    .step_length   = { 0.09f,  0.09f,  0.09f,  0.09f},
    .step_height   = { 0.06f,  0.06f,  0.06f,  0.06f},
    .term          =  0.4f,
    .kp            =  0.5f,
    .kw            =  0.01f,
    .turn_rate     =  0.0f,
};

/* ---- 内部状态 ---- */
static float   tick_start   = 0.0f;
static float   t_last       = -1.0f;          /* 上一次的 t，-1表示首帧 */
static float   init_x[4]   = {0, 0, 0, 0};   /* 每段轨迹起点的足端 x    */
static uint8_t leg_swing[4] = {1, 1, 0, 0};  /* 1=摆动, 0=支撑          */
/* 初始：腿0(右后),腿1(左前) 摆动；腿2(左后),腿3(右前) 支撑 */

/* ================================================================
 *  bezier_se — x 方向 smoothstep 型贝塞尔
 *  等价于 P0=P1=start, P2=P3=end 的三阶贝塞尔
 *  特性：起末速度为零，中段平滑过渡
 * ================================================================ */
static float bezier_se(float start, float end, float s)
{
    /* B(s) = start*(1-3s²+2s³) + end*(3s²-2s³) */
    float s2 = s * s;
    float s3 = s2 * s;
    return start * (1.0f - 3.0f*s2 + 2.0f*s3)
         + end   * (3.0f*s2 - 2.0f*s3);
}

/* ================================================================
 *  swing_traj — 摆动相轨迹
 *
 *  x：smoothstep  init_x → +step_length/2
 *  z：三阶贝塞尔  stance → 峰值(stance+step_height) → stance
 *     控制点高度 = stance + step_height*(4/3)，使峰值恰好=step_height
 *
 *  s ∈ [0,1]，夹紧到 [0,1]
 * ================================================================ */
static void swing_traj(float ix, float t_sw, float T_sw,
                       float step_len, float sh, float step_h, float *x, float *z)
{
    float s = t_sw / T_sw;
    if (s > 1.0f) s = 1.0f;
    if (s < 0.0f) s = 0.0f;

    *x = bezier_se(ix, step_len * 0.5f, s);

    float h_ctrl = sh + step_h * (4.0f / 3.0f);
    float s1 = 1.0f - s;
    *z = s1*s1*s1 * sh
       + 3.0f*s1*s1*s * h_ctrl
       + 3.0f*s1*s*s  * h_ctrl
       + s*s*s         * sh;
}

/* ================================================================
 *  stance_traj — 支撑相轨迹
 *
 *  x：smoothstep  init_x → -step_length/2（足端向后推，机体前进）
 *  z：保持 stance_height
 * ================================================================ */
static void stance_traj(float ix, float t_st, float T_st,
                        float step_len, float sh, float *x, float *z)
{
    float s = t_st / T_st;
    if (s > 1.0f) s = 1.0f;
    if (s < 0.0f) s = 0.0f;

    *x = bezier_se(ix, -(step_len * 0.5f), s);
    *z = sh;
}

/* ================================================================
 *  sample_foot_pos — 通过 FK 采样当前足端 x，存入 init_x[]
 *  FK 失败时保持上次值（不更新），避免跳变
 * ================================================================ */
static void sample_foot_pos(void)
{
    for (int i = 0; i < 4; i++) {
        FC_Motor_To_Joint(&fc_leg[i]);
        if (FC_Kinematic_Solution(&fc_leg[i]) == 0) {
            init_x[i] = fc_leg[i].Foot.x;
        }
    }
}

/* ================================================================
 *  walk_enter
 *  进入步态前调用一次：采样足端位置，记录起始时刻，设定初始腿态
 * ================================================================ */
void walk_enter(void)
{
    sample_foot_pos();
    tick_start   = DWT_GetTick();
    t_last       = -1.0f;       /* 标记首帧，跳过切换检测 */

    /* 对角A（腿0右后,腿1左前）先摆动，对角B（腿2左后,腿3右前）先支撑 */
    leg_swing[0] = 1;
    leg_swing[1] = 1;
    leg_swing[2] = 0;
    leg_swing[3] = 0;
}

/* ================================================================
 *  walk_run
 *  步态主循环，在 TIM6 中断里每 20ms 调用一次
 *
 *  流程：
 *    1. 读 DWT 时间，取模到 [0, term)
 *    2. 检测 t 是否跨越 0 或 term/2，切换腿态并采样足端位置
 *    3. 按腿态计算每条腿的 Bezier 轨迹目标
 *    4. IK 逆解 → FC_Send_Pos 发送当前 *flah 腿
 *    5. *flah 自增
 * ================================================================ */
void walk_run(uint8_t *flah)
{
    WalkParam *p   = &walk_param;
    float      T   = p->term;
    float      T_hw = T * 0.5f;   /* 半周期 */

    float t = fmodf(DWT_GetTick() - tick_start, T);

    /* ---- 状态切换检测（仅在非首帧时执行）---- */
    if (t_last >= 0.0f) {

        /* t 过零（fmod 回绕）：last_t > T_hw，t < T_hw
         * → 对角A(腿0,1) 开始摆动，对角B(腿2,3) 开始支撑 */
        if (t < T_hw && t_last > T_hw) {
            sample_foot_pos();
            leg_swing[0] = 1;  leg_swing[1] = 1;
            leg_swing[2] = 0;  leg_swing[3] = 0;
        }
        /* t 过半周期：last_t < T_hw，t >= T_hw
         * → 对角B(腿2,3) 开始摆动，对角A(腿0,1) 开始支撑 */
        else if (t >= T_hw && t_last < T_hw) {
            sample_foot_pos();
            leg_swing[0] = 0;  leg_swing[1] = 0;
            leg_swing[2] = 1;  leg_swing[3] = 1;
        }
    }
    t_last = t;

    /* ---- 差速步长：右腿(0,3) -= turn_rate，左腿(1,2) += turn_rate ---- */
    float sl[4];
    sl[0] = p->step_length[0] - p->turn_rate;  /* 腿0 右后 */
    sl[1] = p->step_length[1] + p->turn_rate;  /* 腿1 左前 */
    sl[2] = p->step_length[2] + p->turn_rate;  /* 腿2 左后 */
    sl[3] = p->step_length[3] - p->turn_rate;  /* 腿3 右前 */

    /* ---- 逐腿生成轨迹 ---- */
    for (int i = 0; i < 4; i++) {
        float fx, fz, t_phase;

        /* 对角A（i=0,1）：摆动在[0,T_hw]，支撑在[T_hw,T]
         * 对角B（i=2,3）：摆动在[T_hw,T]，支撑在[0,T_hw] */
        if (i < 2) {
            t_phase = leg_swing[i] ? t : (t - T_hw);
        } else {
            t_phase = leg_swing[i] ? (t - T_hw) : t;
        }
        if (t_phase < 0.0f) t_phase = 0.0f;

        if (leg_swing[i]) {
            swing_traj(init_x[i], t_phase, T_hw, sl[i], p->stance_height[i], p->step_height[i], &fx, &fz);
        } else {
            stance_traj(init_x[i], t_phase, T_hw, sl[i], p->stance_height[i], &fx, &fz);
        }

        fc_leg[i].Foot_Target.x = fx;
        fc_leg[i].Foot_Target.z = fz;
        FC_Kinematic_Inversion(&fc_leg[i]);
    }

    /* ---- 分时发送位置指令（避免同一中断内 4 帧挤满 TX 邮箱）---- */
    FC_Send_Pos_Slot(*flah, p->kp, p->kw);
    *flah = (*flah + 1) & 3;
}
