# 用 Changer_calc 实现非阻塞式前进步态

## 核心目标

**解决问题**：当前 `dog_forward_ready_zero()` 是阻塞式的，遥杆复位无法立即停止

**实现方案**：用 `Changer_calc()` 控制步态进度，实现非阻塞式控制，遥杆复位立即停止

---

## 方案原理

### 当前阻塞式问题

```c
if(RC_Ctl.rc.ch3>=1200){
    dog_forward_ready_zero(0.0004826, 0.00026);  // ❌ 阻塞式，必须执行完
}
```

内部有两个大 for 循环，一旦进入就必须执行完整个步态周期。

### 新方案：Changer 控制步态进度

**核心思想**：
1. 用 **1个 Changer** 控制步态进度（0 → 10.4，代表完整周期）
2. 根据进度值，实时计算 8条腿的目标角度
3. 在 main.c 的 while 循环中**每次调用一次**（非阻塞）
4. 遥杆复位时，停止调用，Changer 不再更新，机器人停止

---

## 步态分析

### 步态周期划分

**第一循环**（i: 5.2 → 0）：
- 前腿(0,1,2,3)：线性运动
- 后腿(4,5,6,7)：三阶段状态机
  - 阶段1(b: 0→2.6)：抬腿
  - 阶段2(c: 0→5.2)：摆腿
  - 阶段3(d: 0→2.6)：着地

**第二循环**（i2: 5.2 → 0）：
- 后腿(4,5,6,7)：线性运动
- 前腿(0,1,2,3)：三阶段状态机
  - 阶段1(b2: 0→2.6)：抬腿
  - 阶段2(c2: 0→5.2)：摆腿
  - 阶段3(d2: 0→2.6)：着地

**总周期长度**：5.2 + 5.2 = 10.4

---

## 实现方案

### 核心设计

用 **1个 Changer** 控制步态进度：

```c
// 步态进度：0 → 10.4
double gait_progress = Changer_calc(&angle_changer[0], 10.4, speed, 0, 0);

// 根据进度计算当前位置
if(gait_progress <= 5.2) {
    // 第一循环：前腿线性，后腿三阶段
    double i = 5.2 - gait_progress;
    calculate_cycle1_positions(i);
} else {
    // 第二循环：后腿线性，前腿三阶段
    double i2 = 5.2 - (gait_progress - 5.2);
    calculate_cycle2_positions(i2);
}
```

### 关键数据结构

```c
// 步态状态（新增全局变量）
typedef struct {
    double progress;           // 当前进度 0-10.4
    uint8_t is_active;        // 是否激活
    uint8_t flag_1;           // 后腿阶段标志
    uint8_t flag_2;           // 前腿阶段标志
    double b, c, d;           // 第一循环三阶段变量
    double b2, c2, d2;        // 第二循环三阶段变量
} GaitState_t;

GaitState_t gait_state = {0};
```

### 函数设计

#### 1. 初始化函数

```c
void dog_forward_init(void) {
    gait_state.progress = 0;
    gait_state.is_active = 1;
    gait_state.flag_1 = 0;
    gait_state.flag_2 = 0;
    gait_state.b = 0; gait_state.c = 0; gait_state.d = 0;
    gait_state.b2 = 0; gait_state.c2 = 0; gait_state.d2 = 0;

    // 重置 Changer
    Changer_int(&angle_changer[0]);  // 用angle_changer[0]控制步态进度
}
```

#### 2. 步态更新函数（非阻塞，每次调用执行一步）

```c
void dog_forward_update(double speed) {
    if(!gait_state.is_active) return;

    // 用 Changer 更新步态进度
    gait_state.progress = Changer_calc(&angle_changer[0], 10.4, speed, 0, 0);

    // 根据进度计算每条腿的位置
    if(gait_state.progress <= 5.2) {
        // 第一循环
        double i = 5.2 - gait_state.progress;

        // 计算前腿目标位置（线性运动）
        leg[0] = leg0 + 2.6 - i * 0.58;
        leg[1] = leg1 - 2.6 + i;
        leg[2] = leg2 - 2.6 + i * 0.58;
        leg[3] = leg3 + 2.6 - i;

        // 计算后腿三阶段位置
        update_rear_legs_phase1();

    } else if(gait_state.progress <= 10.4) {
        // 第二循环
        double i2 = 5.2 - (gait_state.progress - 5.2);

        // 计算后腿目标位置（线性运动）
        leg[4] = leg4 - 2.6 + i2 * 0.58;
        leg[5] = leg5 + 2.6 - i2;
        leg[6] = leg6 + 2.6 - i2 * 0.58;
        leg[7] = leg7 - 2.6 + i2;

        // 计算前腿三阶段位置
        update_front_legs_phase2();

    } else {
        // 周期结束，重置
        gait_state.is_active = 0;
    }
}
```

#### 3. 三阶段位置计算函数

```c
void update_rear_legs_phase1(void) {
    double add = 0.0004826;  // 阶段变量增量

    // 阶段判断
    if(gait_state.b < 2.6) {
        // 阶段1：抬腿
        gait_state.flag_1 = 0;
        gait_state.b += add;
        if(gait_state.b >= 2.6) gait_state.b = 2.6;

        leg[4] = leg4 - 2.6 - gait_state.b;
        leg[5] = leg5 + 2.6 - gait_state.b;
        leg[6] = leg6 + 2.6 + gait_state.b;
        leg[7] = leg7 - 2.6 + gait_state.b;

    } else if(gait_state.c < 5.2) {
        // 阶段2：摆腿
        gait_state.flag_1 = 1;
        gait_state.c += add;
        if(gait_state.c >= 5.2) gait_state.c = 5.2;

        leg[4] = leg4 - 5.2 + gait_state.c * 0.58;
        leg[5] = leg5 - gait_state.c;
        leg[6] = leg6 + 5.2 - gait_state.c * 0.58;
        leg[7] = leg7 + 5.2 + gait_state.c;

    } else if(gait_state.d < 2.6) {
        // 阶段3：着地
        gait_state.flag_1 = 2;
        gait_state.d += add;
        if(gait_state.d >= 2.6) gait_state.d = 2.6;

        leg[4] = leg4 - 5.2 + gait_state.d + 5.2 * 0.58;
        leg[5] = leg5 + gait_state.d - 5.2;
        leg[6] = leg6 + 5.2 - gait_state.d - 5.2 * 0.58;
        leg[7] = leg7 + 5.2 - gait_state.d + 5.2;
    }
}

void update_front_legs_phase2(void) {
    double add = 0.0004826;  // 阶段变量增量

    // 阶段判断
    if(gait_state.b2 < 2.6) {
        // 阶段1：抬腿
        gait_state.flag_2 = 0;
        gait_state.b2 += add;
        if(gait_state.b2 >= 2.6) gait_state.b2 = 2.6;

        leg[0] = leg0 + 2.6 + gait_state.b2;
        leg[1] = leg1 - 2.6 + gait_state.b2;
        leg[2] = leg2 - 2.6 - gait_state.b2;
        leg[3] = leg3 + 2.6 - gait_state.b2;

    } else if(gait_state.c2 < 5.2) {
        // 阶段2：摆腿
        gait_state.flag_2 = 1;
        gait_state.c2 += add;
        if(gait_state.c2 >= 5.2) gait_state.c2 = 5.2;

        leg[0] = leg0 + 5.2 - gait_state.c2 * 0.58;
        leg[1] = leg1 + gait_state.c2;
        leg[2] = leg2 - 5.2 + gait_state.c2 * 0.58;
        leg[3] = leg3 - gait_state.c2;

    } else if(gait_state.d2 < 2.6) {
        // 阶段3：着地
        gait_state.flag_2 = 2;
        gait_state.d2 += add;
        if(gait_state.d2 >= 2.6) gait_state.d2 = 2.6;

        leg[0] = leg0 + 5.2 - gait_state.d2 - 5.2 * 0.58;
        leg[1] = leg1 - gait_state.d2 + 5.2;
        leg[2] = leg2 - 5.2 + gait_state.d2 + 5.2 * 0.58;
        leg[3] = leg3 + gait_state.d2 - 5.2;
    }
}
```

#### 4. 停止函数

```c
void dog_forward_stop(void) {
    gait_state.is_active = 0;
}
```

### 调用方式（main.c）

**原来**（阻塞式）：
```c
if(RC_Ctl.rc.ch3>=1200){
    dog_forward_ready_zero(0.0004826, 0.00026);  // ❌ 阻塞
}
```

**修改为**（非阻塞）：
```c
static uint8_t gait_initialized = 0;

if(RC_Ctl.rc.ch3>=1200){
    // 遥杆推上去：启动/继续步态
    if(!gait_initialized) {
        dog_forward_init();      // 首次初始化
        gait_initialized = 1;
    }
    dog_forward_update(2600);    // ✅ 非阻塞，每次调用推进一步
} else {
    // 遥杆复位：停止步态
    if(gait_initialized) {
        dog_forward_stop();      // 停止
        gait_initialized = 0;
    }
}
```

---

## 实现步骤

### 步骤1：添加全局变量

**文件**：`e:\32project\yushuV5\app\yushu_app.c`（第22行附近）

```c
// 在 double kd,flag_1=0,flag_2=0,b=0,b2=0,c=0,c2=0,d=0,d2=0; 后面添加

// 步态状态结构体
typedef struct {
    double progress;
    uint8_t is_active;
    uint8_t flag_1;
    uint8_t flag_2;
    double b, c, d;
    double b2, c2, d2;
} GaitState_t;

GaitState_t gait_state = {0};
```

### 步骤2：实现核心函数

**文件**：`e:\32project\yushuV5\app\yushu_app.c`（在 dog_forward_ready_zero 后面添加）

1. `dog_forward_init()` - 初始化步态
2. `dog_forward_update()` - 更新步态（非阻塞）
3. `update_rear_legs_phase1()` - 第一循环后腿三阶段
4. `update_front_legs_phase2()` - 第二循环前腿三阶段
5. `dog_forward_stop()` - 停止步态

### 步骤3：修改调用方式

**文件**：`e:\32project\yushuV5\Core\Src\main.c`（第163行）

将阻塞式调用改为非阻塞式周期调用。

### 步骤4：声明函数

**文件**：`e:\32project\yushuV5\app\yushu_app.h`

```c
void dog_forward_init(void);
void dog_forward_update(double speed);
void dog_forward_stop(void);
```

---

## 关键参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| speed | Changer的速度参数 | 2600 (原add2*10000000) |
| T_A, T_C | 加减速比例 | 0, 0 (匀速) |
| target | 步态总进度 | 10.4 (两个循环) |

### 速度对应关系

```
原 add2 = 0.00026
新 speed = 0.00026 * 10000000 = 2600
```

---

## 优势对比

| 特性 | 原方案（阻塞式） | 新方案（Changer） |
|------|----------------|------------------|
| 遥杆响应 | ❌ 必须执行完 | ✅ 立即停止 |
| 代码结构 | for循环嵌套 | 状态机 + Changer |
| 平滑性 | 线性 | 可加减速 |
| 可扩展性 | 低 | 高（易调参数） |

---

## 测试步骤

1. **编译烧录**：确保无语法错误
2. **遥杆测试**：
   - 推上遥杆 → 机器人开始前进
   - 复位遥杆 → 机器人立即停止 ✅
3. **参数调优**：调整 speed 值观察速度效果

---

## 需要修改的文件

1. ✅ `e:\32project\yushuV5\app\yushu_app.c`
   - 添加 GaitState_t 结构体
   - 添加 dog_forward_init/update/stop 函数
   - 添加 update_rear_legs_phase1/update_front_legs_phase2 函数

2. ✅ `e:\32project\yushuV5\app\yushu_app.h`
   - 声明新函数

3. ✅ `e:\32project\yushuV5\Core\Src\main.c`
   - 修改第163行的调用方式

---

## 注意事项

1. **Changer使用**：用 `angle_changer[0]` 控制步态进度，其余7个可用于其他功能
2. **状态保持**：`gait_state` 保存运动状态，支持暂停/继续
3. **线程安全**：在主循环中调用，注意变量访问
4. **周期循环**：如果需要循环前进，在 `gait_state.progress >= 10.4` 时重置进度即可

**关键**：整个实现没有阻塞循环，每次调用只推进一步，遥杆复位立即响应！
