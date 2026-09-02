# 水面垃圾清理机器人（STM32F103C8T6 扩展版）

本项目以 STM32F103C8T6 为主控，K230 负责视觉目标坐标输出，GPS 与罗盘为后续导航提供输入；两颗 TB6612FNG 分别驱动左右推进与回收机构。

本文档对应当前的“扩展版 STM32F103C8T6 引脚分配”。其中“已适配”仅指源码已按该方案配置和接入，**不代表已完成整机或水上实测**。

完整接线见 [硬件接线图](first_0_1/docs/硬件接线图.md)；软件和硬件关系见 [系统层级图](first_0_1/docs/系统层级图.md)。

## 硬件组成

| 类别 | 模块 / 用途 |
| --- | --- |
| 主控 | STM32F103C8T6（72 MHz，SPL / Keil） |
| 视觉 | K230，通过 USART1 输出目标信息 |
| 定位 | GPS，通过 USART2 输出 NMEA 数据 |
| 姿态 | MPU6050，I2C 地址 `0x68` |
| 罗盘 | QMC5883L，I2C 地址 `0x0D` |
| 距离 | AJ-SRP04M ×3（前、左、右） |
| 电机驱动 | TB6612FNG ×2（推进、回收） |

## 引脚速览

| STM32 引脚 | 功能 / 连接 |
| --- | --- |
| PA0 / PA1 | TIM2_CH1 / CH2：左、右推进 PWM |
| PA2 / PA3 | USART2_TX / RX：GPS RX / TX |
| PA4 / PA5 | TB6612#1 / #2 的 STBY |
| PA6 / PA7 | 前超声波 Trig / Echo |
| PA9 / PA10 | USART1_TX / RX：K230 RX / TX |
| PA11 / PA12 | TB6612#2 AIN1 / AIN2：滚筒方向 |
| PB0 / PB1 | TB6612#2 BIN1 / BIN2：传送机构方向 |
| PB3 / PB4 | 右超声波 Trig / Echo（关闭 JTAG 后使用） |
| PB5 / PB6 | 左超声波 Trig / Echo |
| PB8 / PB9 | 重映射后的 I2C1 SCL / SDA：MPU6050 与 QMC5883L 共线 |
| PB10 / PB11 | 重映射后的 TIM2_CH3 / CH4：滚筒、传送机构 PWM |
| PB12–PB15 | TB6612#1 AIN1 / AIN2 / BIN1 / BIN2：左右推进方向 |
| PA13 / PA14 | SWDIO / SWCLK：保留给 ST-Link 调试 |
| PC13 | 状态 LED |

## 必须启用的 AFIO 重映射

在所有相关外设初始化前启用 AFIO 时钟并配置下列重映射：

```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

/* TIM2: CH1=PA0, CH2=PA1, CH3=PB10, CH4=PB11 */
GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

/* I2C1: SCL=PB8, SDA=PB9 */
GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

/* 关闭 JTAG，保留 PA13/PA14 的 SWD 调试 */
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
```

关闭的是 JTAG，不是 SWD：PB3 / PB4 因而可作为右侧超声波接口，PA13 / PA14 仍可用于下载和调试。

## 通信与导航适配状态

- K230 独占 USART1（PA9/PA10，默认 115200 8N1），避免调试文本混入视觉协议。
- GPS 独占 USART2（PA2/PA3，默认 9600 8N1）。GPS 接收缓冲、NMEA 行接收与带校验和的有效 RMC 定位解析已接入源码。
- MPU6050 与 QMC5883L 共用硬件 I2C1（PB8/PB9，当前总线配置为 100 kHz）。QMC5883L 原始三轴读数已接入；安装后的硬铁/软铁校准和航向标定仍是使用前的必要工作。
- RETURN 状态尚未实现实际返航控制；当前仅执行安全停车占位，不能据此宣称具备返航能力。

## 接线与供电限制

- K230、GPS 与 STM32 都必须是 **3.3 V TTL** 串口电平，TX/RX 交叉连接并共地。
- AJ-SRP04M 若 Echo 输出为 5 V，PA7、PB4、PB6 都必须先经过分压或电平转换后再接入 STM32。不得把 5 V Echo 直接接到 MCU。
- I2C 的 SCL/SDA 上拉接 **3.3 V**，推荐每线 4.7 kΩ；两模块使用不同的 7 位地址 `0x68` / `0x0D`，可以并联在同一总线上。
- 电机驱动的 VM 接电池 A；逻辑电源使用稳定的 3.3 V / 5 V 转换电源。电机、电机驱动、MCU、传感器必须共地。
- PB2 为启动配置相关引脚，应保持下拉，不作为扩展接口使用。

## 可用扩展接口

| 引脚 | 建议用途 | 注意事项 |
| --- | --- | --- |
| PB7 | 水泵继电器、舱门限位或蜂鸣器 | 普通 GPIO，外接负载需另加驱动级 |
| PA8 | TIM1_CH1：舵机 PWM 或编码器输入 | PA8 不是 ADC 输入 |
| PA15 | 普通 GPIO | 关闭 JTAG 后可用 |
| PC14 / PC15 | 低速 GPIO | 仅在未使用外部 32.768 kHz 晶振时使用 |

## 编译与下载

1. 用 Keil MDK 打开 [first_0_1.uvprojx](first_0_1/first_0_1.uvprojx)。
2. 选择 STM32F103C8（中等密度）目标并编译。
3. 通过 ST-Link 的 PA13（SWDIO）、PA14（SWCLK）、3.3 V 和 GND 下载。

在未完成逐项通电检查、I2C 扫描、串口数据检查及电机空载测试前，请保持电机电源断开或让 TB6612 处于 STBY 低电平。

## 目录结构

```text
first_0_1/
├── Library/       STM32 标准外设库
├── Start/         启动文件与系统时钟
├── System/        延时与基础系统支持
└── User/
    ├── bsp/       板级、UART、硬件 I2C、MPU6050、QMC5883L、超声波等
    ├── driver/    TB6612 电机驱动
    ├── algorithm/ PID 与姿态滤波
    └── app/       状态机、控制、协议、导航与任务调度
```
