# 0.96 寸 OLED 接线与显示

用户确认屏幕引脚为 GND、VDD、SCK、SDA。按常见四针 I2C 模块接入；当前默认驱动是 SSD1306、128×64。仅凭尺寸和引脚标注不能确定控制器型号，SSD1306 型号仍属待确认配置。命令和地址格式参考 [Solomon Systech SSD1306 数据手册](https://www.sunrom.com/download/SSD1306.pdf)。

## 接线

- OLED GND → STM32 GND。
- OLED VDD → 3.3V。
- OLED SCK → PB10。
- OLED SDA → PB11。

接线前断开 PB10/PB11 原来通往第二块电机驱动板的 PWM 线。OLED 和 MPU6050 使用不同总线，MPU 保持 PB8/PB9 原接线。软件自动尝试 OLED 的 7 位地址 0x3C 和 0x3D。总线为开漏，需模块自带上拉；若模块没有上拉，SCK/SDA 各接 4.7kΩ 到 3.3V。

## 已移除的回收电机代码

删除滚筒、传送机构的电机编号、方向输出、TIM2 CH3/CH4 PWM 初始化与控制，以及回收电机启动/停止接口。旧 COLLECT 状态只保留被动滤网流程兼容。PA5 仅在初始化拉低，旧回收驱动板不会被软件使能。左右推进器仍使用 PA0/PA1、PB12～PB15、PA4。原回收接线注释按项目规则保留为历史说明。

## 屏幕内容

八行显示示例（示例数值，不是实测）：

```text
MPU6050 LIVE
AX:2253 AY:-10
AZ:20 GX:179
GY:0 GZ:-3
P:+1.5 R:-2.5
T:+3.0 CAL:OK
STATE:IMU_TEST
L:3600 R:3600
```

AX/AY/AZ、GX/GY/GZ 是 MPU 原始轴的有符号计数；P 为船身俯仰、R 为横滚、T 为总倾角，单位度。CAL 显示已累计的连续校准样本数，就绪后显示 OK。未完成校准时 P/R 显示横线；数据过期或读取失败时，原始轴数据和姿态显示横线，避免把旧值当作实时值。L/R 为 PWM 指令，不是实测转速。

常见顶部提示：

- `MPU STARTING`：初始化中。
- `MPU INIT ERR:80`：配置回读不一致。
- `MPU INIT ERR:81`：启动等待后，加速度仍全零。
- `MPU INIT ERR:82`：主程序的芯片 ID 检查失败。
- `MPU READ ERR:xx`：当前读取失败，xx 为十六进制 I2C 错误码。
- `MPU ZERO DATA`：读取成功但三轴加速度全零。
- `MPU DATA STALE`：没有足够新的采样。
- `MPU DATA INVALID`：当前样本未通过幅值有效性检查。

IMU 初始化失败时也会持续刷新屏幕，便于直接读取错误码。OLED 不会绕过原有 MPU 校准、倾角停机或传感器故障保护。

## Keil 文件与运行

- BSP 分组：`User/bsp/oled/bsp_oled.c`、`bsp_oled.h`。
- App 分组：`User/app/app_oled.c`、`app_oled.h`。
- 采样快照接口：`BSP_MPU6050_GetLatest()`，OLED 使用现有采样，不额外读取 MPU。

重新打开 `first_0_1.uvprojx` 可刷新工程文件列表。编译、下载并复位后，先显示启动提示，随后更新数据或错误状态。当前仍开启 MPU 电机台架测试，校准通过约 3 秒后左右推进器可能起转，测试满 30 秒停止。

每 250ms 尝试生成一帧，每次主循环最多传输 16 个显示字节，分块完成整屏。屏幕断开或总线被拉低时有界退出，每秒尝试重新连接一次。显示驱动不改动 MPU 的 PB8/PB9。

若屏幕不亮，先检查实际驱动型号、四线接线及上拉，再看 Watch 的 `g_debugOledStatus`（1正常，2无应答，3总线被拉低）和 `g_debugOledAddress`。如果是 SH1106 或其他分辨率，需要按实际型号调整驱动，不能仅靠修改 I2C 地址替代。

## 验证

Keil Target 1：0 错误、0 警告，Flash 23512 字节、RAM 3992 字节。三种应用配置回归、MPU 启动/原始数据测试、OLED 文本/状态/刷新测试、OLED 位级总线模拟测试均通过。没有自动下载；实际 OLED 控制器兼容性、屏幕效果、刷新耗时以及电机转动仍待实机确认。
