# MPU 全零数据启动检查（2026-09-11）

实机证据：初始化及 ID 检查成功，进入 STATE_IMU_STOP；ImuStopReason=1、StopCalibCount=0、CalibRejectMask=4、ImuReadFailures=0、CalibRejectNorm=0。说明存在通信接口报告成功但加速度向量全零的样本，并触发有效样本超时。不能据此确定是传感器未正常工作、配置未生效还是数据读取问题。

原初始化只写入唤醒和量程配置，没有回读核对，没有复位传感器、显式清除各轴待机或等待输出。参考 InvenSense [RM-MPU-6000A-00 Rev 4.0 §4.30/4.31](https://www.mouser.com/datasheet/2/306/RM-MPU-60xxA_rev_4-736751.pdf) 的电源管理寄存器定义，加入如下启动流程：

1. 写 DEVICE_RESET，等待 100ms。
2. PWR_MGMT_1=0x01，PWR_MGMT_2=0，启用六轴并选择陀螺仪参考时钟。
3. 写入原采样率、滤波和量程配置，每次回读并验证配置值。
4. 等待 100ms，再最多读取 20 次加速度，每次全零后等待 10ms。
5. 有非零加速度才允许启动应用初始化；这不是校准或姿态有效性的替代，后续全部安全检查继续生效。

初始化检查在 App_Ctrl_Init 之前完成，不占用后续 3.5 秒校准上限。没有增加运行中的自动解锁或重启电机行为。暂不修改 I2C 连续读取算法；若配置均正确但持续全零，仍需结合单寄存器数据读取、波形或替换模块等实机证据继续定位。

错误码 `g_debugImuInit`：0 成功；1～5 沿用 I2C 错误；0x80 配置回读不一致；0x81 等待后加速度仍全零。失败时 BootStage=101，电机保持关闭。

新增 Watch 变量：`g_debugMpuInitStage`（1复位、2唤醒、3六轴、4采样率、5滤波、6陀螺仪量程、7加速度量程、8等待数据、9完成）；`g_debugMpuCheckReg`、`g_debugMpuExpected`、`g_debugMpuActual` 记录最近一次配置检查；`g_debugMpuRawAx/Ay/Az` 为最近成功读取的传感器原始轴数据，尚未映射到船身坐标；`g_debugMpuZeroSamples` 累计成功读取但加速度全零的次数。

验证：`tests/mpu_startup_test.c` 模拟正常启动、延迟出数、持续全零、写入未生效、读写错误以及有符号连续数据解码，全部通过。原三个应用配置回归通过，Keil Target 1 编译 0 错误、0 警告。未自动下载，实机输出及电机转动尚未验证。
