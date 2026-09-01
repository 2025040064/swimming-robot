# 水面垃圾清理机器人（STM32F103C8T6 + K230）

基于 STM32F103C8T6 的水面自主/远程垃圾识别、靠近与回收机器人，具备避障、姿态估计与任务状态管理能力，可将浮漂垃圾集中输送存储。

## 硬件清单

| 模块 | 型号 / 说明 | 数量 |
|------|-------------|------|
| 主控 | STM32F103C8T6（72MHz，标准外设库 SPL） | 1 |
| 视觉 | K230（串口输出目标坐标） | 1 |
| 电机驱动 | TB6612FNG | 2 |
| 推进电机 | 6V 直流（左/右） | 2 |
| 回收电机 | 滚筒 + 传送 | 2 |
| 超声测距 | AJ-SRP04M（前/左/右） | 3 |
| 姿态 | MPU6050 | 1 |


完整引脚接线见 [docs/硬件接线图.md](first_0_1/docs/硬件接线图.md)。

## 引脚速览

| 引脚 | 功能 |
|------|------|
| PA0 / PA1 | TIM2_CH1/CH2 — 左右推进 PWM |
| PA2 / PA3 | TIM2_CH3/CH4 — 滚筒 / 传送 PWM |
| PA4 / PA5 | TB6612 #1 / #2 STBY |
| PA6 / PA7 | 超声前 Trig / Echo |
| PA9 / PA10 | USART1 TX / RX（K230） |
| PA11 / PA12 | TB6612 #2 滚筒方向 AIN1/AIN2 |
| PB0 / PB1 | TB6612 #2 传送方向 BIN1/BIN2 |
| PB5 / PB6 | 超声左 Trig / Echo |
| PB7 / PB8 | 超声右 Trig / Echo |
| PB10 / PB11 | 软件 I2C SCL / SDA（MPU6050） |
| PB12–PB15 | TB6612 #1 推进方向 |
| PC13 | 调试 LED |

> 注意：PA8 **不是** ADC 引脚（原设计误标为电池 ADC，已禁用，见下文）。

## 编译步骤

1. 用 Keil MDK（ARM Compiler）打开 [first_0_1/first_0_1.uvprojx](first_0_1/first_0_1.uvprojx)。
2. 目标器件选择 STM32F103C8（中等密度）。
3. 编译：`Project → Build`。应 0 error / 0 warning。
4. 烧录：ST-Link SWD（PA13=SWDIO、PA14=SWCLK）。

调试日志默认关闭。若需开启，在 `User/stm32f10x_conf.h` 中取消注释 `#define DEBUG_ENABLE`（`bsp_debug.c` 已加入工程，可正常链接）。

## 串口协议

- 物理层：USART1，115200 8N1（PA9=TX，PA10=RX）。
- 视觉（K230 → STM32）：`TRASH,x,y\n`，坐标为 640×640 帧，`x,y ∈ [0,639]`；越界帧会被丢弃。
- 心跳 / 应答：`HB` / `ACK`。
- 遥测（STM32 → 上位机）：`STAT,<state>,<pitch>,<roll>,<front>,<left>,<right>`（每 200ms）。
- 崩溃上报（复位后）：`CRASH SP=0x.. LR=0x.. PC=0x..`。

> 当前调试日志（DBG_PRINT）与 K230 协议**共用 USART1**，调试输出会注入 K230 RXD；量产时应分到不同串口。协议目前为纯文本，后续可升级为 SOF+len+CRC16 二进制帧。

## 状态机

```
INIT → SEARCH → DETECT → APPROACH → COLLECT
                ↑           ↓
                └── AVOID ──┘
（RETURN 预留，未实现）
```

- **SEARCH**：蛇形巡航 + 超声避障（前向 <50cm 连续 2 次 → AVOID）。
- **DETECT**：确认视觉目标存在。
- **APPROACH**：横轴 PID 逼近；目标丢失/超时/超声失效时停车并回 SEARCH。
- **COLLECT**：滚筒 + 传送电机回收。
- **AVOID**：向更空旷一侧原地转向，前端 >80cm 连续 3 次后回 SEARCH。

## 已实现 / 未实现

**已实现**

- 任务状态机 FSM（INIT/SEARCH/DETECT/APPROACH/COLLECT/AVOID）
- 视觉目标解析 + 坐标范围校验
- 目标横轴逼近（yaw PID + 死区 + 斜坡）
- 3 路超声测距（TIM3 微秒时基 + EXTI 上下沿捕获）
- 前向避障与安全停车（目标/传感器失效时停车，不再盲跑）
- MPU6050 姿态（互补滤波，dt 动态系数）
- TB6612 电机驱动（4 路 PWM）
- 故障复位 + BKP 崩溃上下文上报（不卡死）
- IWDG 独立看门狗

**未实现 / 预留**

- RETURN 返航（需 GPS + 罗盘，尚未接入）
- 姿态到推进的补偿（TASK_50MS 槽位预留）
- y 坐标 / 距离闭环（当前仅用 x 做航向误差）
- 电池电压监测（ADC 已禁用，待重接线）
- 遥控 / 地面站接口

## 已知问题与待办

- **电池 ADC 已禁用**：原 PA8 误标为 ADC（PA8 无 ADC 功能，通道 8 实为 PB0 且已被传送方向脚占用）。恢复需重接线，见 [硬件接线图.md](first_0_1/docs/硬件接线图.md) 3.6 节。
- **超声 Echo 电平**：模块 5V 供电，若 Echo 为 5V 电平需加分压/电平转换（PA7/PB6/PB8 非 5V 容忍）。
- 实测 PID 参数、超声精度、续航等**待实测后回填**。

## 目录结构

```
first_0_1/
├── Library/     STM32 标准外设库
├── Start/       启动文件 + 系统时钟
├── System/      Delay
└── User/
    ├── bsp/      外设驱动（systick/led/usart/iic/mpu6050/ultrasonic/adc/debug）
    ├── driver/   电机驱动（TB6612）
    ├── algorithm/ PID / 互补滤波
    └── app/      状态机 / 控制 / 协议 / 任务调度
```
