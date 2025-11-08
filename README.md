# MEAM5100-T12-Repo
god bless you

========================================================
                        开发日志
========================================================
11.9
1. test.ino-测试两个电机给定速度转动以及encoder速度返回值正常，power supply给了9V，注意公对母杜邦线松动脱落问题
2. 11.9PID。ino-电源保持9V，测试起来设定大概80-90的target counts目前波形比较健康，当然也可以把速度往上调试试，目前看到如果速度太低电机会有一顿一顿的问题，感觉是在克服静摩擦力之类的？
PS：encoder接的时候注意引脚1要接限流电阻330ohms

引脚图
==================================================
 POWER WIRING
==================================================
- M5Stamp [GND]  ---> L298N [GND]
- M5Stamp [GND]  ---> Power Supply [GND] (-)
- L298N   [GND]  ---> Power Supply [GND] (-)
- L298N   [+12V] ---> Power Supply [9V] (+)

==================================================
 LEFT MOTOR (A-Channel / G1)
==================================================
- M5Stamp [G1]   ---> Left Encoder [Signal]
- M5Stamp [G18]  ---> L298N [ENA] (PWM Speed)
- M5Stamp [G19]  ---> L298N [IN1] (Direction)
- M5Stamp [G20]  ---> L298N [IN2] (Direction)
- L298N   [OUT1] ---> Left Motor
- L298N   [OUT2] ---> Left Motor

==================================================
 RIGHT MOTOR (B-Channel / G0)
==================================================
- M5Stamp [G0]   ---> Right Encoder [Signal]
- M5Stamp [G7]   ---> L298N [ENB] (PWM Speed)
- M5Stamp [G4]   ---> L298N [IN3] (Direction)
- M5Stamp [G5]   ---> L298N [IN4] (Direction)
- L298N   [OUT3] ---> Right Motor
- L298N   [OUT4] ---> Right Motor

==================================================
Kp = 3 Ki = 0 Kd = 0.1