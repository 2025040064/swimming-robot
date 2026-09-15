# I2C BUSY 调试记录（2026-09-10）

实机截图先后出现：MPU 初始化返回 5（等待 BUSY 释放超时）；一次启动读取 ID=0x68 并进入主循环，但随后 STATE_IMU_STOP、FilterReady=0、PWM=0；最新截图 BootStage=101、RunTick=0、State=255，再次在初始化阶段停止。最新截图未包含初始化返回码，不能单凭 BootStage=101 确定此次仍是错误 5。校准/采样停机的具体原因仍待采集。

工程目标为 STM32F103C8。参考 [ST ES096 Rev 15 §2.8.7](https://www.st.com/resource/en/errata_sheet/es096-stm32f101x8b-stm32f102x8b-and-stm32f103x8b-mediumdensity-device-limitations-stmicroelectronics.pdf)：模拟滤波器可能导致 BUSY 锁定，单纯外设复位不能保证解除。该问题与之前现象相符，但尚未通过实机电平及恢复结果确认根因。

`bsp_iic.c` 增加单主机总线恢复：检测到 BUSY 后禁用 I2C，使用开漏 GPIO 验证双线高电平，再依次拉低 SDA、拉低 SCL、释放 SCL、释放 SDA，每一步检查引脚输入，恢复 AF_OD 并执行 SWRST、重新配置 I2C。每次等待均有循环次数上限，线路被持续拉低会返回失败；不强驱动高电平，不无限重试，不解除已经锁定的电机保护。初始化后 BUSY 以及交易前等待 BUSY 超时均可触发一次恢复。

Watch 中直接填写：

- `g_debugIicRecovery`：0 未尝试，1 恢复成功，2 GPIO 电平检查失败，3 恢复后 BUSY 仍未释放。
- `g_debugIicLines`：恢复入口切换开漏并释放双线后的电平；位 0 为 SCL 高，位 1 为 SDA 高，3 表示双线高。0 在未尝试恢复时不代表实测双线低。
- `g_debugImuInit`：0 成功，5 BUSY 超时，其他值见 `bsp_iic.h`。
- `g_debugBootStage`：100 主循环，101 初始化失败，102 ID 检查失败。
- `g_debugState`：7 倾角停机，8 IMU 停机，10 电机测试，11 测试完成；255 尚未进入主循环诊断。

验证：Keil Target 1 编译 0 错误、0 警告；三个应用配置的宿主机回归测试通过；`tests/iic_recovery_test.c` 模拟正常总线、可恢复 BUSY、交易前恢复、SDA/SCL 持续低、BUSY 永久置位，验证有界失败及引脚恢复。模拟测试不能证明板上时序和电平正确。未自动下载或进行实机验证。

实机下一步：下载新固件，复位后静止连续运行 5 秒，暂停查看上述变量；初始化正常后再看 `g_debugImuStopReason`、`g_debugStopCalibCount` 及采样拒绝记录。应重复验证普通复位和完整断电启动，不能仅凭一次成功认为故障消失。
