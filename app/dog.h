/**
 * @file    dog.h
 * @brief   yushuV8 四足机器人力控（VMC虚拟模型控制）数据结构与接口
 *
 * 机构类型：同轴五连杆并联机构
 *   两电机轴心重合（hip_offset=0），Alpha/Beta曲柄共轴，通过两条从动连杆驱动足端
 *
 * 控制链路：
 *   电机反馈 → 关节角换算 → 正运动学FK → 笛卡尔PD → 雅可比转置 → 发转矩
 *
 * 腿编号与电机映射（已物理测试确认）：
 *   腿0(右后): Alpha=Motor1, Beta=Motor0, dir=-1
 *   腿1(左前): Alpha=Motor3, Beta=Motor2, dir=+1
 *   腿2(左后): Alpha=Motor5, Beta=Motor4, dir=+1
 *   腿3(右前): Alpha=Motor7, Beta=Motor6, dir=-1
 *
 * 关节角零点：motor_reading = leg_middle[i] 时 joint_angle = 0（曲柄水平）
 * 站立时关节角：Alpha = Beta = 3.2 rad → 足端约 (0, -0.172m)
 */

#ifndef _DOG_H
#define _DOG_H

#include "stdint.h"
#include "mydefine.h"
#include "math.h"

/* ================================================================
 *  物理常数（同轴五连杆，4条腿相同）
 *
 *  机构示意：
 *   Alpha电机轴 ──[FC_ROD0: 曲柄0.1m]── B点
 *   Beta电机轴  ──[FC_ROD2: 曲柄0.1m]── D点
 *   B点 ──[FC_ROD1: 从动连杆0.2m]── E点（足端）
 *   D点 ──[FC_ROD3: 从动连杆0.2m]── E点（足端）
 *   Alpha轴 与 Beta轴 重合（同轴）
 * ================================================================ */
#define FC_ROD0     0.1f    /* Alpha侧曲柄长度 (m)，连接电机轴到B点 */
#define FC_ROD1     0.2f    /* Alpha侧从动连杆长度 (m)，连接B点到足端E */
#define FC_ROD2     0.1f    /* Beta侧曲柄长度 (m)，连接电机轴到D点 */
#define FC_ROD3     0.2f    /* Beta侧从动连杆长度 (m)，连接D点到足端E */

/* ================================================================
 *  控制参数
 * ================================================================ */
#define FC_JACOBI_SINGULAR_THR  0.05f   /* 雅可比奇异点保护阈值 */
#define FC_TORQUE_MAX           10.0f   /* 输出力矩限幅 (N·m) */

/* ================================================================
 *  数据结构
 * ================================================================ */

/** Alpha/Beta 关节对 */
typedef struct {
    float Alpha;
    float Beta;
} FC_Order;

/** 关节空间运动学状态 */
typedef struct {
    FC_Order Pos;   /* 关节角 (rad) */
    FC_Order Spd;   /* 关节角速度 (rad/s) */
} FC_Kine;

/**
 * 足端状态（笛卡尔空间）
 * 坐标系：x向前为正，z向上为正（站立时足端z≈-0.172m）
 */
typedef struct {
    float x, z;    /* 足端位置 (m) */
    float vx, vz;  /* 足端速度 (m/s) */
} FC_Foot;

/** 腿的固定物理参数（FC_Init初始化一次，之后只读） */
typedef struct {
    uint8_t motor_Alpha_id;   /* Alpha电机在 UT_motor_data[] 中的下标 */
    uint8_t motor_Beta_id;    /* Beta电机的下标 */
    int8_t  dir;              /* 方向乘数：右侧腿=-1，左侧腿=+1 */
    float   ref_Alpha;        /* Alpha电机零点（leg_middle对应值，此时关节角=0） */
    float   ref_Beta;         /* Beta电机零点 */
    /* CAN总线由FC_Send_Torque内部静态表决定，无需此处存储 */
} FC_Param;

/**
 * 笛卡尔空间PD控制器
 * 摆动腿和支撑腿分别定义一组参数，按步态切换传入
 */
typedef struct {
    float Kp_x, Kd_x, k_x;  /* x方向：比例/微分/低通滤波系数 */
    float Kp_z, Kd_z, k_z;  /* z方向 */
    float Ff_z;               /* z方向前馈力 (N)，重力补偿 */
    /* 内部状态（自动维护，勿手动赋值） */
    float err_x,  last_err_x,  diff_x,  last_diff_x;
    float err_z,  last_err_z,  diff_z,  last_diff_z;
} FC_PD;

/** 单条腿完整状态（顶层结构） */
typedef struct {
    FC_Param  param;         /* 物理常数（FC_Init后不变） */
    FC_Kine   Joint;         /* 当前关节角/速度（电机反馈换算） */
    FC_Kine   Joint_Target;  /* 目标关节角/速度（IK逆解输出） */
    FC_Foot   Foot;          /* 当前足端位置/速度（FK计算） */
    FC_Foot   Foot_Target;   /* 目标足端位置（上层控制器设定） */
    float     Fx, Fz;        /* PD输出的目标足端力 (N) */
    FC_Order  Torque;        /* 最终关节力矩 (N·m)，待发给电机 */
} FC_LEG;

/* ================================================================
 *  全局变量声明
 * ================================================================ */
extern FC_LEG fc_leg[4];   /* 4条腿，下标0~3 */

/* ================================================================
 *  函数声明
 * ================================================================ */

/** 初始化4条腿的参数（在 motor_waiting() 之后调用一次） */
void FC_Init(void);

/** 从 UT_motor_data[] 读取电机反馈，换算为关节角/速度 */
void FC_Motor_To_Joint(FC_LEG *leg);

/**
 * 正运动学：关节角/速度 → 足端位置/速度
 * @return 0=正常, -1=奇异点（跳过）
 */
int8_t FC_Kinematic_Solution(FC_LEG *leg);

/**
 * 逆运动学：目标足端位置 → 目标关节角
 * 输入：leg->Foot_Target.x / .z
 * 输出：leg->Joint_Target.Pos.Alpha / .Beta（.Spd置零）
 * @return 0=正常, -1=超出工作范围
 */
int8_t FC_Kinematic_Inversion(FC_LEG *leg);

/** 笛卡尔空间PD控制：目标位置 vs 当前位置 → 足端力 Fx/Fz */
void FC_PD_Control(FC_LEG *leg, FC_PD *pd);

/**
 * 雅可比转置：足端力 Fx/Fz → 关节力矩 Torque.Alpha/Beta
 * @return 0=正常, -1=奇异点（力矩清零跳过）
 */
int8_t FC_VMC_Jacobi(FC_LEG *leg);

/** 向电机发送力矩命令（纯转矩模式：K_P=0, K_W=0） */
void FC_Send_Torque(uint8_t leg_id);

/** 向电机发送位置命令（位控模式：K_P=kp, K_W=kw, T=0），需先调用 FC_Kinematic_Inversion */
void FC_Send_Pos(uint8_t leg_id, float kp, float kw);

/**
 * 分时位置发送（推荐用于步态/位控循环）
 * 每次只发 2 个电机（跨两条 CAN 总线各 1 个），避免 TX 邮箱溢出。
 * slot: 0~3 循环，4 拍覆盖全部 8 个电机。
 * slot=0: 腿0 Beta + 腿1 Beta
 * slot=1: 腿0 Alpha + 腿1 Alpha
 * slot=2: 腿2 Beta + 腿3 Beta
 * slot=3: 腿2 Alpha + 腿3 Alpha
 */
void FC_Send_Pos_Slot(uint8_t slot, float kp, float kw);

/** 单条腿完整更新：Motor→Joint→FK→PD→Jacobi（不含发送） */
void FC_Update_Leg(uint8_t leg_id, FC_PD *pd);

#endif /* _DOG_H */
