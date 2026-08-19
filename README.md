# R1 机械臂固件 · ROBOCON 2026 深圳技术大学 1034 战队  ZENGKAI 

> 当前版本：`50ff91f`（正式版本）

机器人上下料机械臂固件，负责 ROBOCON 2026 项目中在**梅花林取放 KFS 方块**，并搬运至**九宫格**得分。

---

## 1. 项目简介

系统由两部分构成：

| 部分 | 说明 |
|---|---|
| **R1 机械臂** | 左右双臂，每臂为「**旋转**（灵足05）+ **升降**（2006）+ **真空吸盘**」结构，安装于 R1 舵轮底盘，在梅花林中取放 KFS |

> **抬升R2**：与机械臂作业无直接关联，R1 上独立的 **3508 大抬升机构**（`Raise_task`）唯一用途即为抬升 R2。

主要功能：

- 梅花林中取放 KFS
- 吸取 / 搬运 / 放入九宫格得分
- 三种比赛模式（正常、预选赛红方、预选赛蓝方）切换

**技术栈**

- 主控：STM32H723ZETx（Cortex-M7，550MHz，FPU）
- 实时系统：FreeRTOS（CMSIS-RTOS v2）
- 总线：3× FDCAN（MIT 运控协议 + DJI RM 协议）+ 1× USART
- 开发工具：Keil MDK（ARMCLANG V6.23）、STM32CubeMX

---

## 2. 硬件概览与接线表

### 2.1 机构

- **旋转关节**：灵足05（RoboStride 05，MIT 运控模式），左右各 1，装于 FDCAN1
- **升降关节**：2006 电机，左右各 1，c610 电调，装于 FDCAN3
- **末端执行器**：真空吸盘（吸取/放置 KFS），配合气缸与真空电磁阀
- **大抬升机构**：3508 电机（DJI 电调），独立于机械臂，用于抬升 R2，装于 FDCAN3
- **气源**：M3508 + C620 电调驱动气泵，装于 FDCAN3

### 2.2 引脚分配

| 功能 | 接口 | 引脚 | 说明 |
|---|---|---|---|
| 旋转关节（灵足05，MIT 运控） | FDCAN1 | PD0 / PD1（扩展帧） | CAN ID 1=左臂、2=右臂 |
| 预留扩展总线 | FDCAN2 | PB12 / PB13 | 已初始化，当前未接线 |
| 2006 升降 / 3508 大抬升 / 气泵（RM 协议） | FDCAN3 | PF6 / PF7（标准帧） | 0x200=双臂2006，0x1FF=3508大抬升 |
| 上位机通信（指令协议见 §5.3） | USART6 | PC6(RX) / PC7(TX)，115200，DMA | 串口助手/上位机 |
| 气缸 1/2 | GPIO | PE5 / PE6 | 气缸伸出/缩回 |
| 真空气阀 1/2 | GPIO | PF3 / PF4 | 断电=吸气，通电=断真空 |
| 大抬升限位开关 | GPIO | PG0（上拉输入） | 3508 零点校准 |
| 临时气泵 | GPIO | PA9 | 备用气泵使能 |
| 预留 | GPIO | PB15 | 未使用 |
| 气泵 ESC 油门 | TIM1_CH1 | PE9（PWM） | 40% 油门运行，`esc_control.c` |

> 电气注意：灵足 05 与 2006 共用 24V，上电时序见 §5.2。

---

## 3. 开发环境

| 工具 | 用途 | 说明 |
|---|---|---|
| Keil MDK-ARM | 编译 / 烧录 / 调试 | 工程使用 ARMCLANG **V6.23** 编译，需安装对应 ARM Compiler |
| STM32CubeMX | 外设代码生成 | 可选，用于从 `FDCAN/FDCAN.ioc` 重新生成 |
| ST-Link / DAP-Link | SWD 烧录调试 | 任一即可 |
| 串口助手 | 上位机指令调试 | 115200-8N1，TX→PC7，RX←PC6 |

工程文件：`FDCAN/MDK-ARM/FDCAN.uvprojx`

> 提示：`bsp_fdcan.c/.h`（FDCAN 底层初始化）位于 `FDCAN/MDK-ARM/FDCAN/` 目录下，重新生成 CubeMX 代码时该目录不会被更新，需手动保留。

---

## 4. 机械臂运动学模型

实际机构为**旋转 + 升降**双关节（XZ 平面作业），左右臂对称。

### 4.1 升降关节（2006，平动）

- 由 2006 电机驱动，位置为相对上电零点的编码器偏移（脉冲）
- 四档高度：`100 / 200 / 400 / 600`
- 目标由 `target_red` / `target_blue`（1~4）间接选择，经 `lift_goto_target()` 位置闭环保持

参数位置：`FDCAN/leg_task/3508_control.h`

| 臂 | 高度 100 | 高度 200 | 高度 400 | 高度 600 |
|---|---|---|---|---|
| 1 号臂 | `LIFT_RED_POS0` | `LIFT_RED_POS1` | `LIFT_RED_POS2` | `LIFT_RED_POS3` |
| 2 号臂 | `LIFT_BLUE_POS0` | `LIFT_BLUE_POS1` | `LIFT_BLUE_POS2` | `LIFT_BLUE_POS3` |

> `MATCH_MODE_PRELIM3` 模式下 2 号臂最低只到 200（防碰撞存杆），见 `leg_task.c`。

### 4.2 旋转关节（灵足05，转动）

- 目标角为绝对值（rad），关键角度：展开到前面 = ±π，收回 = -0.3 / +0.3
- 带重力补偿 `K_GRAVITY[]`（单位 Nm），与角度相关
- 由 `robstride_goto_target()` 平滑驱动

参数位置：`FDCAN/ROBSTRIDE/ROBSTRIDE.h`

| 常量 | 值 | 含义 |
|---|---|---|
| `ROBSTRIDE_TARGET_ANGLE_1` | 3.14 | 1 号臂展开到前 |
| `ROBSTRIDE_TARGET_ANGLE_2` | -3.14 | 2 号臂展开到前 |
| `ROBSTRIDE_RETRACT_ANGLE` | -0.3 | 1 号臂收回 |
| `ROBSTRIDE_RETRACT_ANGLE_BLUE` | 0.3 | 2 号臂收回 |
| `K_GRAVITY[2]` | — | 重力补偿系数 |

### 4.3 大抬升机构（3508，Raise_task）

- 独立于机械臂，`raise_target_pos` 相对 PG0 限位零点
- `raise_target_pos = 0` 降到限位零点；`= AIMDIC` 抬升至最高（默认 23 圈）

---

## 5. 上电使用流程

### 5.1 比赛模式切换

在 `FDCAN/Core/Inc/main.h`（USER CODE 区）修改 `MATCH_MODE`：

| 宏 | 值 | 说明 |
|---|---|---|
| `MATCH_MODE_NORMAL` | 0 | 正常模式 |
| `MATCH_MODE_PRELIM` | 1 | 九宫藏宝预选赛·红方（上电左臂预装 KFS） |
| `MATCH_MODE_PRELIM3` | 2 | 崇武探幽预选赛 3（右臂存料只抬升不收灵足） |
| `MATCH_MODE_DEBUG` | 3 | 调试模式（上电双臂预装 KFS） |
| `MATCH_MODE_BLUE` | -1 | 九宫藏宝预选赛·蓝方（上电右臂预装 KFS） |

### 5.2 上电时序（自动执行）

1. `Raise_task`（高优先级任务）：等 3508 通讯建立 → 3508 下降到 **PG0 限位开关** → 记录零点 → 位置闭环待命
2. 等 24V 稳定（检测到 FDCAN3 返回 2006 数据）
3. ESC 电调上电行程校准（约 1.2s）
4. 双臂 2006 记录上电零点，灵足 05 使能并低增益归零
5. 上位机串口就绪，等待指令

> 预选赛 / 调试模式下会按 `MATCH_MODE` 自动吸气并预装 KFS。

### 5.3 上位机指令格式

帧格式：`RC + 指令 + END`（大小写不敏感），固件答 `RCOKEND`。完整解析见 `FDCAN/leg_task/rc_protocol.c`。

| 指令 | 示例 | 说明 |
|---|---|---|
| `KFSx,y` | `RCKFS1,2END` | 开场定位：记录左/右臂九宫格 KFS 高度（值 0~12，查表映射） |
| `ATAKEnn` / `BTAKEnn` | `RCATAKE00END` | 左/右臂取料（nn=动作号 00/01） |
| `LKFSxxx` / `RKFSxxx` | `RCLKFS300END` | 左/右臂高度直接设 100/200/400/600 |
| `LABSORB` / `RABSORB` | `RCLABSORBEND` | 左/右臂吸收（吸取 KFS） |
| `LSWITCH` / `RSWITCH` | `RCRSWITCHEND` | 左/右臂放料切换（开阀→收气缸） |
| `LRISING` / `RRISING` | `RCLRISINGEND` | 单臂上升一档 |
| `LGODOWN` / `RGODOWN` | `RCRGODOWNEND` | 单臂下降一档 |
| `UPLIFT0` / `UPLIFT1` | `RCUPLIFT1END` | 大抬升：0=降到限位零点，1=抬最高 |
| `LRECALL` / `RRECALL` | `RCLRECALLEND` | 左/右臂收回（灵足收后） |
| `LOUTLAY` / `ROUTLAY` | `RCROUTLAYEND` | 左/右臂放料准备（升 600 + 灵足展开） |
| `R2READY` | `RCR2READYEND` | R2 预备姿态（2 号→-90°） |
| `ATREADY` | `RCATREADYEND` | R1 进攻姿态（2 号→-180°） |
| `TRIGGER` | `RCTRIGGEREND` | 触发 1 号臂放料准备 |
| `LAUTOnn` / `RAUTOnn` | `RCLAUTO01END` | 左/右臂自动流程（预选赛/正常） |

> 高度映射：KFS 协议值经 `score_task.c` 的 `protocol_to_height[]` 映射为高度档位（10 个协议值对应 4 档高度）。

---

## 6. 代码结构与入口地图

```
FDCAN/
├─ FDCAN.ioc / .mxproject        # STM32CubeMX 工程配置
├─ Core/                         # CubeMX 生成外设代码
│  ├─ Src/main.c                 # 入口：时钟/MPU/外设初始化 + 启动调度器
│  ├─ Src/freertos.c             # FreeRTOS 任务创建（LEG_task / Raise_task）
│  ├─ Src/fdcan.c / usart.c / tim.c / gpio.c / dma.c
│  └─ Inc/main.h                 # ★ MATCH_MODE 比赛模式宏
├─ leg_task/                     # 机械臂主逻辑（LEG_task）
│  ├─ leg_task.c                 # 双臂控制主循环、上电初始化、24V掉电恢复
│  ├─ 3508_control.c             # 2006 升降位置闭环、梯形速度规划、堵转保护、气缸/真空阀
│  ├─ rc_protocol.c              # 上位机串口协议（状态机、指令表、ACK/重试）
│  ├─ score_task.c               # 得分状态机（取料/存料/放料、自动流程）
│  ├─ relay.c                    # 气泵启停、吸/放 KFS 继电器时序
│  └─ esc_control.c              # 气泵 ESC（TIM1 CH1 PWM）上电校准与启停
├─ Raise_task/                   # 独立 3508 大抬升（抬升 R2）
│  └─ raise_task.c               # 限位开关校准、位置闭环、FreeRTOS 任务
├─ ROBSTRIDE/                    # 灵足05 旋转关节驱动
│  └─ ROBSTRIDE.c / .h           # MIT 运控协议、重力补偿、角度控制
└─ MDK-ARM/
   ├─ FDCAN.uvprojx              # ★ Keil 工程（打开此文件）
   └─ FDCAN/bsp_fdcan.c/.h       # FDCAN 收发底层、RM 协议封装（勿删除）
```

启动/调用链：

```
main.c
 └─ rc_protocol_init()  relay_init()  fdcan_config()      # 上电外设
 └─ osKernelStart()
    ├─ Raise_task   ── 3508 大抬升（限位校准 + 位置闭环，周期 10ms）
    └─ LEG_task     ── 上电初始化 → 主循环
                      ├─ rc 指令轮询 → score_task 状态机（取/放/自动流程）
                      ├─ lift_control_update()   # 2006 双臂升降
                      ├─ arm_gimbal_update()     # 灵足05 旋转
                      └─ esc_update() / 24V 掉电恢复
```

---

## 7. 调参标定指南

### 7.1 高度标定（`LIFT_*_POS`）

1. 上电后手动转 2006 到目标高度
2. Keil Watch 窗口读 `total_ecd`（相对零点值）
3. 写入 `3508_control.h` 对应 `LIFT_*_POS`，再编译烧录

### 7.2 抬升行程（`AIMDIC`）

- `Raise_task/raise_task.h` 的 `AIMDIC` = 3508 抬升至最高所需的编码器脉冲（默认 3618191 ≈ 23 圈）

### 7.3 重力补偿（`K_GRAVITY[]`）

- `ROBSTRIDE.h` 一维数组，单位 Nm；灵足在角度静态保持时补偿臂自重，改变机械结构后需重新整定

### 7.4 PID 参数

| 位置 | 文件 | 说明 |
|---|---|---|
| 2006 升降 | `3508_control.c` `arm_PID_INIT()` | 位置环参数、死区、限幅 |
| 3508 大抬升 | `raise_task.c` `PID_INIT()` | 位置环 + 速度环串级 |
| 灵足05 | `ROBSTRIDE.c` | 内部位置/电流环（MIT 参数） |

### 7.5 保护与滤波

- **堵转保护**：`lift_stall_check()`（2006），切断后需 `lift_stall_clear()` 恢复
- **限位滤波**：`raise_task.c` 一阶低通（`SW_LPF_ALPHA`），数值越大越稳、响应越慢
- **24V 掉电恢复**：`leg_task.c` `power_supervise()`，自动重启灵足并重新归零 2006

---

## 附：版本记录

| commit | 说明 |
|---|---|
| `50ff91f` | 正式版本（当前 HEAD） |
| `1ccce06` | 预选赛正式版本 |
| `3819ce6` | 11 号试运行比赛版本 |