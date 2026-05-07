#include "dog.h"
#include "math.h"

#define PI 3.1415926f

extern CAN_HandleTypeDef hcan2;

/* ================================================================
 *  全局变量定义
 * ================================================================ */
FC_LEG fc_leg[4];

/* ================================================================
 *  FC_Init
 *  初始化4条腿的固定参数（电机映射、方向、零点）
 *  调用时机：motor_waiting() 结束后，正式控制开始前调用一次
 * ================================================================ */
void FC_Init(void)
{
    /* 腿0 右后: Alpha=Motor1, Beta=Motor0, dir=-1 */
    fc_leg[0].param.motor_Alpha_id = 1;
    fc_leg[0].param.motor_Beta_id  = 0;
    fc_leg[0].param.dir            = -1;
    fc_leg[0].param.ref_Alpha      = -5.2640f;
    fc_leg[0].param.ref_Beta       = -6.4249f;

    /* 腿1 左前: Alpha=Motor3（奇朝前）, Beta=Motor2（偶朝后）, dir=+1 */
    fc_leg[1].param.motor_Alpha_id = 3;
    fc_leg[1].param.motor_Beta_id  = 2;
    fc_leg[1].param.dir            = 1;
    fc_leg[1].param.ref_Alpha      = 9.4794f;
    fc_leg[1].param.ref_Beta       = 9.2395f;

    /* 腿2 左后: Alpha=Motor5（奇朝前）, Beta=Motor4（偶朝后）, dir=+1 */
    fc_leg[2].param.motor_Alpha_id = 5;
    fc_leg[2].param.motor_Beta_id  = 4;
    fc_leg[2].param.dir            = 1;
    fc_leg[2].param.ref_Alpha      = 10.8468f;
    fc_leg[2].param.ref_Beta       = 11.8030f;

    /* 腿3 右前: Alpha=Motor7, Beta=Motor6, dir=-1 */
    fc_leg[3].param.motor_Alpha_id = 7;
    fc_leg[3].param.motor_Beta_id  = 6;
    fc_leg[3].param.dir            = -1;
    fc_leg[3].param.ref_Alpha      = -4.3599f;
    fc_leg[3].param.ref_Beta       = -6.0914f;
}

/* ================================================================
 *  FC_Motor_To_Joint
 *  从 UT_motor_data[] 读取电机反馈，换算为关节角/速度
 *
 *  公式（已验证，无需除减速比）：
 *    Joint.Pos.Alpha = (Pos[Alpha_id] - ref_Alpha) * dir
 *    Joint.Pos.Beta  = (Pos[Beta_id]  - ref_Beta)  * dir
 *    Joint.Spd.Alpha = W[Alpha_id] * dir
 *    Joint.Spd.Beta  = W[Beta_id]  * dir
 *
 *  零点定义：joint=0 对应曲柄水平（Pos=leg_middle）
 *  站立时：joint_Alpha ≈ joint_Beta ≈ +3.2 rad
 * ================================================================ */
void FC_Motor_To_Joint(FC_LEG *leg)
{
    float dir = (float)leg->param.dir;

    leg->Joint.Pos.Alpha = (UT_motor_data[leg->param.motor_Alpha_id].Pos - leg->param.ref_Alpha) * dir;
    leg->Joint.Pos.Beta  = (UT_motor_data[leg->param.motor_Beta_id ].Pos - leg->param.ref_Beta ) * dir;
    leg->Joint.Spd.Alpha = UT_motor_data[leg->param.motor_Alpha_id].W * dir;
    leg->Joint.Spd.Beta  = UT_motor_data[leg->param.motor_Beta_id ].W * dir;
}

/* ================================================================
 *  FC_Kinematic_Solution
 *  正运动学：关节角 → 足端位置（物理FK，减法公式）
 *
 *  机构：同轴五连杆
 *    Alpha曲柄(rod0) → B点 → 从动连杆(rod1) → 足端E
 *    Beta曲柄 (rod2) → D点 → 从动连杆(rod3) → 足端E
 *
 *  推导：
 *    第一步：joint → 物理角
 *      phys_Alpha = -joint_Alpha / 6.33   （从0朝前，向下为负）
 *      phys_Beta  = π + joint_Beta / 6.33  （从π朝后，向下为正）
 *    第二步：减法FK
 *      B = (rod0·cos(phys_Alpha), rod0·sin(phys_Alpha))
 *      D = (rod2·cos(phys_Beta),  rod2·sin(phys_Beta))
 *      sx = xb-xd, sz = zb-zd
 *      A    = 2·rod1·sx
 *      B    = 2·rod1·sz
 *      C    = rod1²+sx²+sz²-rod3²
 *      disc = A²+B²-C²
 *      cos_Phi1 = (-C·A + B·√disc) / (A²+B²)   ← +B·sqrt（z轴向上��
 *      sin_Phi1 = (-C - A·cos_Phi1) / B
 *    足端：E = B点 + rod1·(cos_Phi1, sin_Phi1)
 *
 *  奇异点：B≈0（joint_Alpha=joint_Beta，两曲柄对称）或 disc<0
 *  站立目标须保持 Alpha(2.2) ≠ Beta(3.2) 以避免奇异
 *  返回值：0=正常, -1=奇异（Foot不更新）
 * ================================================================ */
int8_t FC_Kinematic_Solution(FC_LEG *leg)
{
    /* 第一步：joint → 物理角 */
    float phys_Alpha = -(leg->Joint.Pos.Alpha) / 6.33f;
    float phys_Beta  = PI + (leg->Joint.Pos.Beta) / 6.33f;

    float xb = FC_ROD0 * cosf(phys_Alpha);
    float zb = FC_ROD0 * sinf(phys_Alpha);
    float xd = FC_ROD2 * cosf(phys_Beta);
    float zd = FC_ROD2 * sinf(phys_Beta);

    /* 第二步：减法公式 */
    float sx = xb - xd;
    float sz = zb - zd;

    float A = 2.0f * FC_ROD1 * sx;
    float B = 2.0f * FC_ROD1 * sz;
    float C = FC_ROD1 * FC_ROD1 + sx * sx + sz * sz - FC_ROD3 * FC_ROD3;

    /* 奇异保护：B≈0 时除零（joint_Alpha=joint_Beta时触发） */
    if (fabsf(B) < 1e-4f) {
        return -1;
    }

    float disc = A * A + B * B - C * C;   /* 减法公式：A²+B²-C² */
    if (disc < 0.0f) {
        return -1;
    }

    /* +B*sqrt：yushuV8 z轴向上，与 corgi（-B*sqrt，z轴向下）符号相反 */
    float cos_Phi1 = (-C * A + B * sqrtf(disc)) / (A * A + B * B);
    float sin_Phi1 = (-C - A * cos_Phi1) / B;

    leg->Foot.x = xb + FC_ROD1 * cos_Phi1;
    leg->Foot.z = zb + FC_ROD1 * sin_Phi1;   /* 站立时 < 0 ✓ */

    /* 第三步：足端速度（对位置求时间导数） */
    float phys_Alpha_spd = -(leg->Joint.Spd.Alpha) / 6.33f;
    float phys_Beta_spd  =  (leg->Joint.Spd.Beta)  / 6.33f;

    float vbx = -FC_ROD0 * sinf(phys_Alpha) * phys_Alpha_spd;
    float vbz =  FC_ROD0 * cosf(phys_Alpha) * phys_Alpha_spd;
    float vdx = -FC_ROD2 * sinf(phys_Beta)  * phys_Beta_spd;
    float vdz =  FC_ROD2 * cosf(phys_Beta)  * phys_Beta_spd;

    /* Phi2: D点到足端E的方向角 */
    float cos_Phi2 = (leg->Foot.x - xd) / FC_ROD3;
    float sin_Phi2 = (leg->Foot.z - zd) / FC_ROD3;

    /* k = sin(Phi1 - Phi2)，奇异保护 */
    float k = sin_Phi1 * cos_Phi2 - sin_Phi2 * cos_Phi1;
    if (fabsf(k) < 1e-4f) {
        leg->Foot.vx = 0.0f;
        leg->Foot.vz = 0.0f;
        return 0;  /* 位置有效，速度置零 */
    }

    float w_Phi1 = ((vbx - vdx) * cos_Phi2 + (vbz - vdz) * sin_Phi2) / (FC_ROD1 * k);
    leg->Foot.vx = vbx - FC_ROD1 * w_Phi1 * sin_Phi1;
    leg->Foot.vz = vbz + FC_ROD1 * w_Phi1 * cos_Phi1;

    return 0;
}

/* ================================================================
 *  FC_VMC_Jacobi
 *  雅可比转置：足端力 [Fx, Fz] → 关节力矩 [Torque.Alpha, Torque.Beta]
 *
 *  推导：对五连杆约束方程求时间导数，得
 *    A_mat · [vx, vz]^T = B_mat · [dAlpha, dBeta]^T
 *    J = A_mat⁻¹ · B_mat
 *    τ_joint = J^T · [Fx, Fz]^T
 *
 *  Phi1/Phi2 由差值几何独立求解（与 FK 的 Phi1 不同）：
 *    sx = xb-xd, sz = zb-zd
 *    A  = 2·rod1·sx, B_c = 2·rod1·sz
 *    C  = rod1²+sx²+sz²-rod3²
 *    disc = A²+B_c²-C²  （注意符号与FK不同）
 *
 *  奇异保护：B_c≈0（除零）或 k≈0（两连杆平行） → 力矩清零返回-1
 *  返回值：0=正常, -1=奇异
 * ================================================================ */
int8_t FC_VMC_Jacobi(FC_LEG *leg)
{
    /* 数值雅可比：对FK做有限差分，避免等效FK（α=β）时解析公式退化（k=0）
     * 扰动量 1e-3 rad，精度约 0.1%，50Hz控制周期内计算量可接受 */
    const float deps = 1e-3f;
    FC_LEG tmp;
    float dir = (float)leg->param.dir;

    /* 扰动 Alpha：dE/dAlpha */
    tmp = *leg;
    tmp.Joint.Pos.Alpha += deps;
    if (FC_Kinematic_Solution(&tmp) != 0) {
        leg->Torque.Alpha = 0.0f;
        leg->Torque.Beta  = 0.0f;
        return -1;
    }
    float dEx_da = (tmp.Foot.x - leg->Foot.x) / deps;
    float dEz_da = (tmp.Foot.z - leg->Foot.z) / deps;

    /* 扰动 Beta：dE/dBeta */
    tmp = *leg;
    tmp.Joint.Pos.Beta += deps;
    if (FC_Kinematic_Solution(&tmp) != 0) {
        leg->Torque.Alpha = 0.0f;
        leg->Torque.Beta  = 0.0f;
        return -1;
    }
    float dEx_db = (tmp.Foot.x - leg->Foot.x) / deps;
    float dEz_db = (tmp.Foot.z - leg->Foot.z) / deps;

    /* τ = J^T · F，乘 dir 转换方向 */
    float tau_Alpha = (dEx_da * leg->Fx + dEz_da * leg->Fz) * dir;
    float tau_Beta  = (dEx_db * leg->Fx + dEz_db * leg->Fz) * dir;

    /* 限幅 */
    if (tau_Alpha >  FC_TORQUE_MAX) tau_Alpha =  FC_TORQUE_MAX;
    if (tau_Alpha < -FC_TORQUE_MAX) tau_Alpha = -FC_TORQUE_MAX;
    if (tau_Beta  >  FC_TORQUE_MAX) tau_Beta  =  FC_TORQUE_MAX;
    if (tau_Beta  < -FC_TORQUE_MAX) tau_Beta  = -FC_TORQUE_MAX;

    leg->Torque.Alpha = tau_Alpha;
    leg->Torque.Beta  = tau_Beta;

    return 0;
}

/* ================================================================
 *  FC_Kinematic_Inversion
 *  逆运动学：目标足端位置 → 目标关节角（解析辅助角法）
 *
 *  机构特性：rod0=rod2=0.1m，rod1=rod3=0.2m（对称五连杆）
 *  使足端 E=(x,z) 距离曲柄端点 B/D 恰好等于从动连杆长：
 *    |EB|=rod1  →  x·cos(α)+z·sin(α) = K   （其中 K=(R²-0.03)/0.2）
 *    |ED|=rod3  →  同形方程，K 相同
 *  辅助角法得两解：
 *    phys_Alpha = φ + arccos(K/R)   （Alpha曲柄朝前偏下，站立≈-0.5rad）
 *    phys_Beta  = φ - arccos(K/R)   （Beta曲柄朝后偏下，站立≈3.6rad）
 *
 *  坐标系：x向前，z向上；站立时 z_foot≈-0.22m，x_foot≈0
 *
 *  输入：leg->Foot_Target.x / .z
 *  输出：leg->Joint_Target.Pos.Alpha / .Beta（速度项暂置零）
 *  返回：0=正常, -1=超出工作范围（Foot_Target不可达）
 * ================================================================ */
int8_t FC_Kinematic_Inversion(FC_LEG *leg)
{
    float x  = leg->Foot_Target.x;
    float z  = leg->Foot_Target.z;

    float R2 = x * x + z * z;
    float R  = sqrtf(R2);

    if (R < 1e-4f) return -1;   /* 足端在轴心，奇异 */

    /* K/R = cos(theta)，theta 为两曲柄相对足端方向的分叉角 */
    /* K = (R² + rod0² - rod1²) / (2·rod0) = (R² - 0.03) / 0.2 */
    float K     = (R2 - 0.03f) / 0.2f;
    float ratio = K / R;
    if (ratio > 1.0f || ratio < -1.0f) return -1;  /* 足端超出可达范围 */

    float phi   = atan2f(z, x);         /* 足端相对轴心的方向角 */
    float theta = acosf(ratio);          /* 分叉角，∈[0,π] */

    /* Alpha 曲柄：选 + 号分支（朝前侧） */
    float phys_Alpha = phi + theta;

    /* Beta 曲柄：选 - 号分支（朝后侧，结果为负，加2π映射到(π,2π)） */
    float phys_Beta = phi - theta;
    if (phys_Beta < 0.0f) phys_Beta += 2.0f * PI;

    /* 物理角 → 关节角（与 FK 中公式互逆） */
    leg->Joint_Target.Pos.Alpha = -phys_Alpha * 6.33f;
    leg->Joint_Target.Pos.Beta  = (phys_Beta - PI) * 6.33f;

    /* 速度前馈暂置零（电机 Kw 阻尼已足够，后续可加解析速度IK） */
    leg->Joint_Target.Spd.Alpha = 0.0f;
    leg->Joint_Target.Spd.Beta  = 0.0f;

    return 0;
}

/* ================================================================
 *  FC_PD_Control
 *  笛卡尔空间PD控制：目标足端位置 vs 当前足端位置 → 足端力 [Fx, Fz]
 *
 *  Fx = Kp_x*(x_target - x) - Kd_x*vx
 *  Fz = Kp_z*(z_target - z) - Kd_z*vz + Ff_z
 *
 *  Ff_z：重力前馈（负值，如站立时 = -body_weight/4）
 * ================================================================ */
void FC_PD_Control(FC_LEG *leg, FC_PD *pd)
{
    leg->Fx = pd->Kp_x * (leg->Foot_Target.x - leg->Foot.x)
              - pd->Kd_x * leg->Foot.vx;

    leg->Fz = pd->Kp_z * (leg->Foot_Target.z - leg->Foot.z)
              - pd->Kd_z * leg->Foot.vz
              + pd->Ff_z;
}

/* ================================================================
 *  FC_Send_Torque
 *  向电机发送纯力矩命令（mode=1, K_P=0, K_W=0, Pos=0, W=0）
 *
 *  CAN映射：Motor 0/1/6/7 → hcan2，Motor 2/3/4/5 → hcan1
 * ================================================================ */
void FC_Send_Torque(uint8_t leg_id)
{
    static MOTOR_can_send fc_send;
    static CAN_HandleTypeDef * const can_map[8] = {
        &hcan2, &hcan2, &hcan1, &hcan1,
        &hcan1, &hcan1, &hcan2, &hcan2
    };

    FC_LEG *leg = &fc_leg[leg_id];
    uint8_t aid = leg->param.motor_Alpha_id;
    uint8_t bid = leg->param.motor_Beta_id;

    unitree_setpara(&fc_send, aid, 1, leg->Torque.Alpha, 0.0f, 0.0f, 0.0f, 0.0f);
    unitree_CAN_Send(can_map[aid], 0, 11, &fc_send);
    unitree_CAN_Send(can_map[aid], 0, 10, &fc_send);

    unitree_setpara(&fc_send, bid, 1, leg->Torque.Beta, 0.0f, 0.0f, 0.0f, 0.0f);
    unitree_CAN_Send(can_map[bid], 0, 11, &fc_send);
    unitree_CAN_Send(can_map[bid], 0, 10, &fc_send);
}

/* ================================================================
 *  FC_Update_Leg
 *  单条腿完整更新：Motor→Joint→FK→PD→Jacobi（不含CAN发送）
 *  发送需另外调用 FC_Send_Torque(leg_id)
 * ================================================================ */
void FC_Update_Leg(uint8_t leg_id, FC_PD *pd)
{
    FC_LEG *leg = &fc_leg[leg_id];
    FC_Motor_To_Joint(leg);
    if (FC_Kinematic_Solution(leg) != 0) return;
    FC_PD_Control(leg, pd);
    FC_VMC_Jacobi(leg);
}

/* ================================================================
 *  FC_Send_Pos
 *  向电机发送位置命令（位控模式：K_P>0, K_W>0, T=0）
 *
 *  电机目标位置由 Joint_Target 反算：
 *    motor_Pos = Joint_Target.Pos * dir + ref
 *    （与 FC_Motor_To_Joint 互逆，dir=±1 时 1/dir=dir）
 *
 *  @param kp  位置增益（对应 unitree_setpara 的 K_P）
 *  @param kw  速度增益（对应 unitree_setpara 的 K_W）
 * ================================================================ */
void FC_Send_Pos(uint8_t leg_id, float kp, float kw)
{
    static MOTOR_can_send fc_send;
    static CAN_HandleTypeDef * const can_map[8] = {
        &hcan2, &hcan2, &hcan1, &hcan1,
        &hcan1, &hcan1, &hcan2, &hcan2
    };

    FC_LEG *leg = &fc_leg[leg_id];
    float   dir = (float)leg->param.dir;
    uint8_t aid = leg->param.motor_Alpha_id;
    uint8_t bid = leg->param.motor_Beta_id;

    float pos_Alpha = leg->Joint_Target.Pos.Alpha * dir + leg->param.ref_Alpha;
    float pos_Beta  = leg->Joint_Target.Pos.Beta  * dir + leg->param.ref_Beta;

    unitree_setpara(&fc_send, aid, 1, 0.0f, 0.0f, pos_Alpha, kp, kw);
    unitree_CAN_Send(can_map[aid], 0, 11, &fc_send);
    unitree_CAN_Send(can_map[aid], 0, 10, &fc_send);

    unitree_setpara(&fc_send, bid, 1, 0.0f, 0.0f, pos_Beta, kp, kw);
    unitree_CAN_Send(can_map[bid], 0, 11, &fc_send);
    unitree_CAN_Send(can_map[bid], 0, 10, &fc_send);
}

/* ================================================================
 *  FC_Send_Pos_Slot
 *  分时位置发送：每次只发 2 条腿各 1 个电机，保证每条 CAN 总线
 *  单次中断内最多 2 帧，不会打满 STM32 的 3 个 TX 邮箱。
 *
 *  slot 分配与 mode=0 原始位控完全相同：
 *    slot=0: M0(腿0 Beta,hcan2) + M2(腿1 Beta,hcan1)
 *    slot=1: M1(腿0 Alpha,hcan2)+ M3(腿1 Alpha,hcan1)
 *    slot=2: M4(腿2 Beta,hcan1) + M6(腿3 Beta,hcan2)
 *    slot=3: M5(腿2 Alpha,hcan1)+ M7(腿3 Alpha,hcan2)
 *
 *  调用方在 TIM6 中断里对 flah 做 0→1→2→3→0 循环即可，
 *  每 4 拍覆盖全部 8 个电机（与 mode=0 节奏相同）。
 * ================================================================ */
void FC_Send_Pos_Slot(uint8_t slot, float kp, float kw)
{
    static MOTOR_can_send fc_send;
    static CAN_HandleTypeDef * const can_map[8] = {
        &hcan2, &hcan2, &hcan1, &hcan1,
        &hcan1, &hcan1, &hcan2, &hcan2
    };

    uint8_t mid;
    float   dir, pos;

#define SEND_MOTOR(leg_idx, which)  \
    mid = fc_leg[leg_idx].param.motor_##which##_id;               \
    dir = (float)fc_leg[leg_idx].param.dir;                       \
    pos = fc_leg[leg_idx].Joint_Target.Pos.which * dir            \
          + fc_leg[leg_idx].param.ref_##which;                    \
    unitree_setpara(&fc_send, mid, 1, 0.0f, 0.0f, pos, kp, kw);  \
    unitree_CAN_Send(can_map[mid], 0, 11, &fc_send);              \
    unitree_CAN_Send(can_map[mid], 0, 10, &fc_send)

    switch (slot & 3) {
    case 0:  /* M0(腿0 Beta,hcan2) + M2(腿1 Beta,hcan1) */
        SEND_MOTOR(0, Beta);
        SEND_MOTOR(1, Beta);
        break;
    case 1:  /* M1(腿0 Alpha,hcan2) + M3(腿1 Alpha,hcan1) */
        SEND_MOTOR(0, Alpha);
        SEND_MOTOR(1, Alpha);
        break;
    case 2:  /* M4(腿2 Beta,hcan1) + M6(腿3 Beta,hcan2) */
        SEND_MOTOR(2, Beta);
        SEND_MOTOR(3, Beta);
        break;
    case 3:  /* M5(腿2 Alpha,hcan1) + M7(腿3 Alpha,hcan2) */
        SEND_MOTOR(2, Alpha);
        SEND_MOTOR(3, Alpha);
        break;
    }

#undef SEND_MOTOR
}
