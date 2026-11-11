# STM32F103C8T6系统板平衡车项目

## 概述：

​	

## 功能：

1. 自平衡
2. 蓝牙控制移动
3. 超声波碰撞检测 
4. 红外循迹移动

## 元器件引脚接口：

- TB6612:

​	PWMA -> PA8 (TIM1_CH1)	AIN2 -> PA10	AIN1-> PA11	STBY(3.3v)	BIN1 -> PA12	BIN2 -> PA15			PWMB -> PA9 (TIM1_CH2)	VM(12v)	VCC(5v)

- OLED:

​	(i2c) SCL -> PB8	SDA -> PB9

- HC-08:

​	TX -> PA3	RX -> PA2

- MPU6050:

​	(i2c) SCK -> PB10	SDA -> PB11

- HC-SR04:

​	Trig -> PB13	Echo -> PB12

- 左编码器：

  A相(A1) -> PB4 (TIM3_CH1)	B相(B1) -> PB5 (TIM3_CH2)

- 右编码器：

  A相(A2) -> PB6 (TIM4_CH1)	B相(B2) -> PB7 (TIM4_CH2)
