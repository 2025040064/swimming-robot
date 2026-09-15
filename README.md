# 水面垃圾清理机器人（STM32F103C8T6 + K230 + MPU6050）

当前软件使用船底被动滤网，已移除回收机构电机控制；MPU6050 按 +X 朝上、+Y 朝船头安装，提供倾斜限推力和锁定停机保护。两个水平推进器不能主动调平。新增 0.96 寸 OLED 实时显示原始六轴、姿态、校准进度、运行状态和推进 PWM，详见 [OLED 接线与显示](first_0_1/docs/OLED接线与显示.md)。

当前已选择 **MPU6050 电机台架测试模式**（`User/bsp/test/bsp_mpu_motor_test.h` 中 `ROBOT_IMU_MOTOR_TEST_ENABLE=1`）：静止校准约 3 秒后，左右推进电机自动缓慢起转，随倾角增大而减速；运行 30 秒后停止，倾角超限或 MPU6050 数据异常也会停机。测试绕过视觉和超声波条件。上电前固定船身并确保桨叶周围无人接触，详见 [MPU6050 电机测试](first_0_1/docs/MPU6050电机测试.md)。

用户计划三个超声波探头完全浸水，但现有 AJ-SR04M 驱动使用空气测距换算。因此关闭台架测试后，自动运行配置仍进入 `RANGE_HOLD`，待水下测距硬件和驱动确认。参数与待确认项见 [滤网模式与倾角保护](first_0_1/docs/滤网模式与倾角保护.md)。

当前硬件不使用 GPS。K230 使用 USART1；MPU6050 使用硬件 I2C1；第一块 D153C 双路 TB6612 模块负责左右推进。OLED 使用 PB10/PB11 上的独立软件 I2C，默认 SSD1306 128×64，实际屏幕驱动型号尚待确认。

## 固定引脚

- K230：PA9=TX、PA10=RX，3.3 V TTL。
- MPU6050：PB8=SCL、PB9=SDA，3.3 V 供电，SCL/SDA 各一只 4.7 kΩ 上拉到 3.3 V。
- OLED：PB10=SCK、PB11=SDA、3.3 V=VDD、GND 共地；先断开 PB10/PB11 原回收 PWM 接线。
- 前超声：PA6=Trig、PA7=Echo；左超声：PB5=Trig、PB6=Echo；右超声：PB3=Trig、PB4=Echo。三个 Echo 都必须先降压至 3.3 V。
- PA13/PA14 保留为 SWD；PB2 不使用。

## D153C TB6612 模块

第一块 D153C：A 路（AO1/AO2）接左推进，B 路（BO1/BO2）接右推进。

```text
PWMA=PA0  AIN1=PB12  AIN2=PB13  STBY=PA4
PWMB=PA1  BIN1=PB14  BIN2=PB15
```

第二块 D153C 的滚筒和传送机构控制已移除；PA5 仅保留拉低，防止旧驱动板仍连接时被使能。PB10/PB11 改接 OLED，PA11/PA12/PB0/PB1 不再作为回收电机方向输出。

`VM` 接电机电池正极；D153C 的 GND、STM32 GND、电池负极必须共地。`ADC` 和编码器接口暂不接入代码。电机方向相反时，先交换该路 AO1/AO2 或 BO1/BO2 两根电机线。

## 软件重映射

```c
GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
```

打开 `first_0_1/first_0_1.uvprojx` 后用 Keil 构建。`RETURN` 状态没有 GPS 定位能力，只执行安全停车。
