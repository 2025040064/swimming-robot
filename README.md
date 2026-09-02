# 水面垃圾清理机器人（STM32F103C8T6 + K230 + MPU6050）

当前硬件不使用 GPS。K230 使用 USART1；MPU6050 使用硬件 I2C1；两块 D153C 双路 TB6612 模块分别负责推进与回收机构。

## 固定引脚

- K230：PA9=TX、PA10=RX，3.3 V TTL。
- MPU6050：PB8=SCL、PB9=SDA，3.3 V 供电，SCL/SDA 各一只 4.7 kΩ 上拉到 3.3 V。
- 前超声：PA6=Trig、PA7=Echo；左超声：PB5=Trig、PB6=Echo；右超声：PB3=Trig、PB4=Echo。三个 Echo 都必须先降压至 3.3 V。
- PA13/PA14 保留为 SWD；PB2 不使用。

## D153C TB6612 模块

第一块 D153C：A 路（AO1/AO2）接左推进，B 路（BO1/BO2）接右推进。

```text
PWMA=PA0  AIN1=PB12  AIN2=PB13  STBY=PA4
PWMB=PA1  BIN1=PB14  BIN2=PB15
```

第二块 D153C：A 路接滚筒，B 路接传送机构。

```text
PWMA=PB10 AIN1=PA11  AIN2=PA12  STBY=PA5
PWMB=PB11 BIN1=PB0   BIN2=PB1
```

`VM` 接电机电池正极；D153C 的 GND、STM32 GND、电池负极必须共地。`ADC` 和编码器接口暂不接入代码。电机方向相反时，先交换该路 AO1/AO2 或 BO1/BO2 两根电机线。

## 软件重映射

```c
GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
```

打开 `first_0_1/first_0_1.uvprojx` 后用 Keil 构建。`RETURN` 状态没有 GPS 定位能力，只执行安全停车。
