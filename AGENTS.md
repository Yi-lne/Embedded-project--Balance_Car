# AGENTS.md

本文件面向在此仓库中工作的 AI 代理与开发者，说明项目结构、构建方式、代码约定与已知陷阱。

## 1. 项目概述

基于 **STM32F103C8T6** 的两轮自平衡小车（平衡车）。

| 项 | 内容 |
|---|---|
| MCU | STM32F103C8T6（Cortex-M3，72 MHz，64 KB Flash / 20 KB SRAM） |
| 固件库 | STM32Cube HAL，FW_F1 **V1.8.7**（`Drivers/`，已 vendored） |
| 构建 | **CMake ≥ 3.22 + Ninja + arm-none-eabi-gcc** |
| IDE | CLion 2024.1 / STM32CubeMX（`*.ioc`） |
| 系统架构 | **裸机超级循环 + 1 ms 定时中断标志位调度**（无 RTOS） |
| 姿态融合 | 互补滤波（α = 0.96） |
| 控制 | 串级 PID：角度内环 200 Hz + 速度/转向外环 20 Hz |

> ⚠️ **仓库根目录的 `balance_car_program/` 是早期 FreeRTOS + 卡尔曼滤波版本，已被 `.gitignore` 排除，不要修改它。**
> 所有开发都在 `balance_car_program2/` 中进行。

## 2. 快速命令

所有命令都在 `balance_car_program2/` 目录下执行：

```bash
# 配置 + 编译（Debug，-O0 -g3）
cmake --preset Debug
cmake --build --preset Debug

# 发布构建（-Os）
cmake --preset Release
cmake --build --preset Release

# 完整重新配置（换机器 / CMakeCache 路径失效时）
rm -rf build/Debug && cmake --preset Debug
```

产物：`build/<Preset>/balance_car_program2.elf`（同名 `.map`）。
需要 `.bin` 时：

```bash
arm-none-eabi-objcopy -O binary build/Debug/balance_car_program2.elf build/Debug/balance_car_program2.bin
```

烧录：CLion 的 `OpenOCD Download & Run` 运行项（`interface/stlink.cfg` + `target/stm32f1x.cfg`），
或 STM32CubeProgrammer / ST-Link Utility 下载 `.elf`。

依赖：CMake、Ninja、`arm-none-eabi-gcc`（需在 PATH 中）、OpenOCD + ST-Link 驱动。

## 3. 目录结构

```text
balance_car_program2/
├─ Core/
│  ├─ Inc/                # CubeMX 生成头文件（main.h 含全部引脚宏）
│  └─ Src/                # main.c / tim.c / i2c.c / dma.c / gpio.c / stm32f1xx_it.c
├─ Car_system/            # 应用层（与硬件弱相关）
│  ├─ Angle/              # angle.c/h  互补滤波 + 零偏校准
│  ├─ Interruption/       # it_flag.c/h 标志位调度与故障恢复
│  ├─ Motor/              # motor.c/h   TB6612 PWM / 方向 / 编码器测速
│  ├─ PID/                # pid.c/h     速度环、角度环、转向环
│  └─ Bluetooth/          # bluetooth.c/h 目标速度接口（**当前为空桩，恒返回 0**）
├─ Hardware/
│  ├─ MPU6050/            # MPU6050.c/h  寄存器配置 + I2C2 DMA 读取
│  └─ oled/               # oled.c/h     软件 I2C SSD1306 驱动 + 字库
├─ Drivers/               # CMSIS + STM32F1xx HAL（厂商代码，不要改）
├─ cmake/
│  ├─ gcc-arm-none-eabi.cmake   # 工具链与编译选项
│  └─ stm32cubemx/CMakeLists.txt # CubeMX 生成源文件/宏/包含路径清单
├─ CMakeLists.txt         # 用户源文件与包含路径在此登记
├─ CMakePresets.json      # Debug / Release 预设（Ninja）
├─ STM32F103XX_FLASH.ld   # 链接脚本（64K Flash / 20K RAM）
├─ startup_stm32f103xb.s  # 启动文件
└─ balance_car_program2.ioc  # CubeMX 工程
```

## 4. 运行时架构

### 4.1 调度

`TIM2` 每 1 ms 触发 `HAL_TIM_PeriodElapsedCallback()`（`Core/Src/stm32f1xx_it.c`），
按 `it_cnt % N` 置位标志：

| 周期 | 标志 | 处理函数 | 频率 |
|---:|---|---|---|
| 5 ms | `angle_calculate_flag` | `It_Angle_Calculate()` | 200 Hz |
| 50 ms | `speed_get_flag` | `It_Speed_GetVal()` | 20 Hz |
| 91 ms | `oled_show_flag` | `It_OLED_Show()`（`main.c` 中当前被注释） | ~11 Hz |

`main()` 的 `while (1)` 轮询这三个标志。**标志是布尔位而非计数**，主循环繁忙时周期会被合并，控制周期存在抖动。

### 4.2 数据流

```text
MPU6050 --I2C2 + DMA1_CH5(14B)--> rx_data[]
   │
   ▼
Angle_Calculation()  互补滤波 α=0.96  →  com_fil_angle
   │
   ▼
Pid_Angle()  (200 Hz, PD)  ──► aim_pwm_avg
   │
   ▼
Pid_SetPWM()  ──► Motor_SetDirXxx() + Motor_SetPWM_Xxx()  ──► TIM1_CH1/CH2  ──► TB6612
                                                                                 │
编码器 TIM3/TIM4 ──► Motor_Speed_GetLeft/Right()（读后清零）                     ▼
   │                                                                          电机
   ▼
Pid_Speed() / Pid_Turn()  (20 Hz)  ──► aim_angle / aim_pwm_diff
```

上电顺序（`Core/Src/main.c`）：
`HAL_Init` → 使能 DWT → 72 MHz 时钟 → GPIO/DMA/I2C2/TIM1/TIM3/TIM4/TIM2 →
`OLED_Init` → `MPU6050_Init` → **`Angle_GyroY_Calibration()`（阻塞 2 s）** →
`MPU6050_StartRxDMA` → `Motor_Init` → `HAL_TIM_Base_Start_IT(&htim2)`。

## 5. 关键文件与职责

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 时钟、外设初始化、主循环 |
| `Core/Src/stm32f1xx_it.c` | 中断向量、1 ms 调度标志 |
| `Core/Src/tim.c` | TIM1 PWM（14.4 kHz）、TIM2 1 ms、TIM3/TIM4 编码器、MSP |
| `Core/Src/i2c.c` | I2C2 400 kHz + DMA1_CH5 |
| `Core/Src/gpio.c` | TB6612 方向引脚 |
| `Car_system/Angle/angle.c` | 互补滤波、零偏校准、角度获取 |
| `Car_system/PID/pid.c` | 三环 PID、PWM 合成/限幅/死区 |
| `Car_system/Motor/motor.c` | PWM、方向、编码器读取与清零 |
| `Car_system/Interruption/it_flag.c` | 标志处理、I2C 故障恢复、OLED 刷新 |
| `Hardware/MPU6050/MPU6050.c` | 寄存器初始化、阻塞读、DMA 读、完成回调 |
| `Hardware/oled/oled.c` | 软件 I2C（PB8/PB9）、SSD1306 显示 |

## 6. 关键参数（调参入口）

| 参数 | 位置 | 当前值 | 说明 |
|---|---|---:|---|
| `ACCEL_BIAS` | `Angle/angle.c` | 6.0f | 加速度计角度机械平衡偏移（°） |
| `GYRO_PERIOD` | `Angle/angle.c` | 0.005f | 陀螺积分周期（s），必须与 5 ms 调度一致 |
| `ALPHA_VAL` | `Angle/angle.c` | 0.96f | 互补滤波信任系数 |
| `KP_ANGLE` / `KD_ANGLE` | `PID/pid.c` | -42.0 / -129.0 | 角度内环 PD（`-70*0.6`、`-215*0.6`） |
| `KP_SPEED` / `KI_SPEED` | `PID/pid.c` | 0 / 0 | 速度外环（**当前未启用**） |
| `KP_TURN` / `KI_TURN` | `PID/pid.c` | 0 / 0 | 转向环（**当前未启用**） |
| `KI_LIMIT_SPEED` / `KI_LIMIT_TURN` | `PID/pid.c` | 0 / 0 | 积分限幅为 0 ⇒ I 项恒为 0 ⚠️ |
| `PWM_LIMIT` / `PWM_MIN` / `PWM_MAX` | `PID/pid.c` | 2 / 80 / 999 | 死区 / 死区补偿 / 上限 |
| 陀螺量程 | `MPU6050/MPU6050.c` | ±2000 dps（16.4 LSB/dps） | |
| 加速度量程 | `MPU6050/MPU6050.c` | ±16 g（2048 LSB/g） | |
| 编码器模式 | `Core/Src/tim.c` | `TIM_ENCODERMODE_TI1` | 单边沿（1×）；`TI12` 为 4× |
| 调度周期 | `Core/Src/stm32f1xx_it.c` | 5 / 50 / 91 ms | |

## 7. 代码约定

- **CubeMX 生成文件**（`Core/`、`Drivers/`、`cmake/stm32cubemx/`）只允许在
  `/* USER CODE BEGIN ... */` 与 `/* USER CODE END ... */` 之间修改；否则重新生成会被覆盖。
- 修改外设配置时优先改 `.ioc` 并用 CubeMX 重新生成，保持 `.ioc` 与代码一致。
- **新增 `.c` 文件必须同时加入 `balance_car_program2/CMakeLists.txt` 的 `target_sources`，
  新增头文件目录要加入 `target_include_directories`**，否则不会参与编译。
- 文件内私有函数用 `static`；对外接口在对应 `.h` 中声明。
- 不使用动态内存（`malloc/free`），所有状态用 `static` 或全局结构体。
- 中断共享变量加 `volatile`。
- 注释使用**中文**，说明物理意义与调参思路。
- 单位约定：角度为度（°），PWM 为 `0..999`（对应 TIM1 ARR），
  速度为“每 50 ms 的编码器计数”，未换算为 rpm。
- 浮点运算使用 `float` 与 `xxxf` 字面量（Cortex-M3 无 FPU，但避免 double 开销）。

## 8. 已知问题 / 注意事项

1. `KI_LIMIT_SPEED`、`KI_LIMIT_TURN` 为 0，积分项永远被夹到 0；启用 `KI` 前必须先改限幅。
2. 速度环、转向环增益为 0，蓝牙目标恒为 0 ⇒ **目前只能原地自平衡，不能行驶/转向**。
3. `Bluetooth/` 为空桩；README 中的蓝牙控制、超声波避障**尚未实现**（无 USART/EXTI 配置）。
4. MPU6050 通过排针插母固定，机械位置一旦变化，写死的 `ACCEL_BIAS` 与上电零偏即失效。
5. I2C 故障恢复只做 `DeInit/Init`，无法解开 SDA 被从机拉死的总线。
6. PWM 死区补偿 `PWM_MIN = 80` 会让输出从 0 跳到 82，低速段可能抖动。
7. 换向时先改方向、后写 PWM，存在短时反压电流尖峰的风险。
8. 无看门狗、无电池电压 ADC、无堵转/跌倒检测。
9. 标志位调度在 `main()` 轮询，采样时刻存在抖动，速度计算的有效周期并非严格 50 ms。

## 9. 硬件引脚映射

| 功能 | 引脚 | 外设 |
|---|---|---|
| TB6612 PWMA / PWMB | PA8 / PA9 | TIM1_CH1 / CH2 |
| TB6612 AIN1 / AIN2 | PA11 / PA10 | GPIO 输出 |
| TB6612 BIN1 / BIN2 | PA12 / PA15 | GPIO 输出 |
| 左编码器 A1 / B1 | PB4 / PB5 | TIM3_CH1 / CH2（部分重映射） |
| 右编码器 A2 / B2 | PB6 / PB7 | TIM4_CH1 / CH2 |
| MPU6050 SCL / SDA | PB10 / PB11 | I2C2（400 kHz + DMA1_CH5） |
| OLED SCL / SDA | PB8 / PB9 | 软件 I2C |
| HC-08 TX / RX | PA3 / PA2 | **未配置** |
| HC-SR04 Trig / Echo | PB13 / PB12 | **未配置** |
| SWD | PA13 / PA14 | 调试下载 |

## 10. 版本控制

- 只跟踪 `balance_car_program2/` 源码与工程配置；`build/`、`.idea/`、旧工程、
  编译产物均由根目录 `.gitignore` 排除。
- 提交信息建议使用前缀：`feat:`、`fix:`、`tune:`（调参）、`refactor:`、`docs:`。
  调参提交请写明改了哪些参数，例如 `tune: angle PD -42/-129, accel bias 6.0`。
