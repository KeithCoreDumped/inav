# NEUTRONRCF435MINI_FW 无襟翼双电机支持调研

## 1. 目标与硬件基础

- NEUTRONRCF435MINI_FW 目标将板标识、USB 描述符、LED/Beeper 以及所有 SPI/I2C 资源直接写在 `src/main/target/NEUTRONRCF435MINI_FW/target.h:20`，SPI1 连接 MPU6500/ICM20602 与 ICM42605/42688，I2C1 暴露 SPL06/BMP280 气压计和通用磁力计，体现了该目标默认就具备 IMU+气压+磁力三传感器能力。
- UART 布局只保留 UART1 与 UART7，并把默认接收机设置为 CRSF/ELRS（`DEFAULT_RX_TYPE`、`SERIALRX_PROVIDER` 和 `SERIALRX_UART`），保证了与 ELRS 手柄的即插即用（`src/main/target/NEUTRONRCF435MINI_FW/target.h:122` 与 `:132`）。
- VBAT/电流计 ADC、缺省特性以及 DShot/最大 PWM 端口也在 `target.h:136`、`target.h:148`、`target.h:165` 描述，便于直接驱动两台空心杯电机。
- `timerHardware` 使用 TIM4_CH1/CH2（PB6/PB7）输出电机 PWM，另保留 PA3/PB1 作为可选舵机位（`src/main/target/NEUTRONRCF435MINI_FW/target.c:27`）。目前虽然硬件仍留有两个舵机定时器，但可以在 CLI 中将其解绑，只把 TIM4 通道配置为高频 MOTOR。
- 目标自带的工作记录列举了复制目标、裁剪外设、禁用 OSD/Blackbox 的背景，以及“2 电机 + 2 舵机”资源分配示例（`NEUTRONRCF435MINI_FW-summary.md:3`、`NEUTRONRCF435MINI-notes.md:1`），可作为修改 mmix/resource 的参考。

## 2. 传感器、状态估计与姿态解算

- `sensors/sensors.c:37` 维护温度补偿和校准状态，能够对 IMU 与气压计数据作温漂修正，保障单翼机在长时间盘旋时仍保持一致的重力/压力基准。
- 状态估计器 `navigation/navigation_pos_estimator.c:60` 通过 `positionEstimationConfig_t` 将重力、GPS、气压、光流等权重集中管理，并根据 arm 状态决定是否重置基准高度，为“盘旋飞”提供稳定的 XY/高度估计。
- `flight/imu.c:892` 的 `imuUpdateAttitude` 在每个 looptime 内整合陀螺、加速度与磁力/航迹角权重（`imuCalculateAccelerometerWeightNearness`、`imuMahonyAHRSupdate`），并暴露 `imuUpdateEulerAngles`，为固定翼姿态 PID 提供滚转/俯仰角。

## 3. 固定翼导航与盘旋控制

- 固定翼导航参数（最大倾角、爬升/俯冲角、巡航速度、loiter 半径/方向、起飞着陆参数、`useFwNavYawControl` 等）集中在 `navConfig()->fw` 结构中，由 `src/main/navigation/navigation.c:206` 初始化，默认 loiter 半径 75 m，且支持手动选择顺/逆时针。
- `navigation/navigation_fixedwing.c:277` 通过 `loiterDirection()` 读取设定、RC Yaw、Geozone 等信息，确定盘旋方向，并允许通过 BOX 开关反向。
- `calculateVirtualPositionTarget_FW()`（`navigation/navigation_fixedwing.c:310`）依据当前目标/速度计算虚拟圆心，必要时添加 45° 位移生成圆周目标点，从而实现 Loiter/Waypoint 过渡。
- 姿态修正结果写入 `posControl.rcAdjustment`：高度控制输出俯仰 `rcAdjustment[PITCH]`（`navigation/navigation_fixedwing.c:205`），航向 PID 输出滚转/偏航修正（`navigation/navigation_fixedwing.c:544` 与 `:552`）。这些修正随后被 `applyFixedWingPitchRollThrottleController()` 注入 `rcCommand`，并联动油门微调（`navigation/navigation_fixedwing.c:664` 起）。
- 固定翼最小速度/油门增强利用 `posControl.actualState.velXY` 与 `NAV_FW_THROTTLE_SPEED_BOOST_GAIN` 调整 `throttleSpeedAdjustment`，避免强风下失速（`navigation/navigation_fixedwing.c:597`）。

## 4. 混控与输出链路

- `mixerUpdateStateFlags()` 会根据 `platformType` 与 `navConfig()->fw.useFwNavYawControl` 决定是否启用 `FW_HEADING_USE_YAW`，从而允许导航层把偏航控制权交给动力差速（`src/main/flight/mixer.c:200`）。
- `mixTable()` 对每个电机将 `axisPID` 输出按当前 motor mixer 的 `pitch/roll/yaw` 系数转换成 PWM，之后与油门合成（`src/main/flight/mixer.c:489`、`src/main/flight/mixer.c:537`）。这意味着只要自定义 `mmix` 把滚转/偏航系数分配给左右电机，就能用差速取代襟翼/舵机。
- 由于两台空心杯电机只能正转，可把 `motorstopOnLow` 关闭并提高 `motor_pwm_rate`，保证定速巡航时的线性响应（`src/main/flight/mixer.c:589`）。

## 5. 遥控与通信链路

- 目标默认启用 CRSF，并将 UART7 预留给串口接收机（`src/main/target/NEUTRONRCF435MINI_FW/target.h:122`、`:132`）。
- CRSF/ELRS 驱动实现于 `src/main/rx/crsf.c:45`，支持 420 kbps 半双工、链路统计、MSP 透传等功能，保证遥控信号与遥测（RSSI/LQ）可在 Loiter 模式下持续回传。

## 6. 盘旋飞闭环流程

1. IMU、气压计、磁力计和电池传感器获取的观测在 `sensors/sensors.c:37` 进行温补，然后被 `imuUpdateAttitude()`（`flight/imu.c:892`）与位置估计器（`navigation/navigation_pos_estimator.c:60`）融合成 `posControl.actualState`。
2. `navGetCurrentActualPositionAndVelocity()` 会根据地面/地形跟随模式选择 ABS 或 AGL 坐标（`navigation/navigation.c:2936`），其输出被 Loiter 控制器 `calculateVirtualPositionTarget_FW()` 与 `updatePositionHeadingController_FW()` 使用（`navigation/navigation_fixedwing.c:324`）。
3. 导航层通过 `posControl.rcAdjustment` 写入滚转/俯仰/偏航指令，并由 `applyFixedWingPitchRollThrottleController()` 把它们转化为 `rcCommand` 和期望油门（`navigation/navigation_fixedwing.c:664`）。
4. 姿态 PID 运行后产出的 `axisPID` 会进入 `mixTable()`，按照 motor mixer（即左右电机的 `throttle/roll/yaw` 权重）得出最终 PWM（`src/main/flight/mixer.c:511`）。
5. PWM 由 `timerHardware` 描述的 TIM4_CH1/CH2 输出到两台电机，实现差速闭环。而 `rx/crsf.c:45` 提供的遥控输入在任何时刻都可以与 Loiter 控制叠加，实现“盘旋飞/遥控飞”之间的切换。

## 7. 无襟翼双电机支持改动建议

1. **自定义 motor mixer**  
   - 目标没有舵面时，需要把滚转/偏航 PID 直接映射到左右电机：如 `MOTOR1(throttle=1, roll=+1, yaw=+1)`、`MOTOR2(throttle=1, roll=-1, yaw=-1)`，可在 CLI `mmix` 中设置，或在 `targetConfiguration()` 中写默认混控。这样 `mixTable()`（`src/main/flight/mixer.c:537`）就会把 `axisPID[ROLL/YAW]` 转为差速推力。
   - 结合 `NEUTRONRCF435MINI-notes.md:1` 的资源示例，将 `resource MOTOR 1 B06`、`resource MOTOR 2 B07` 固定为 TIM4 输出，并保持 `timer_output_mode` 为 MOTOR，确保两路 PWM 频率一致。
   - 若需要保留辅助伺服，可把 TIM2/TIM3 输出转成 LED/NONE，否则直接释放。

2. **动力/传感配置**  
   - 空心杯电机对 PWM 频率敏感，可在 CLI 中设置 `set motor_pwm_rate = 16000`，并确认 `USE_DSHOT` 在 `target.h:165` 开启后不会误将电机配置为数字协议。
   - 校准 IMU/气压/磁力计并启用 `USE_BARO_SPL06`、`USE_MAG_ALL`（`src/main/target/NEUTRONRCF435MINI_FW/target.h:94` 起），保证高度与航向在 Loiter 模式下可信。

3. **导航参数与 yaw 闭环**  
   - 启动 `nav_fw_use_fw_yaw_control = ON`，让 `mixerUpdateStateFlags()` 把 `FW_HEADING_USE_YAW` 置位（`src/main/flight/mixer.c:200`），这样 `posControl.rcAdjustment[YAW]` 就能参与差速控制。
   - 根据机翼展弦比和动力输出，调整 `nav_fw_loiter_radius`、`nav_fw_control_smoothness`、`nav_fw_max_bank_angle` 等参数（`src/main/navigation/navigation.c:206`），避免差速造成过大滚转。
   - 如果俯仰无法主动控制，可通过 `nav_fw_pitch_to_throttle`（`navigation/navigation_fixedwing.c:635`）增加姿态-油门耦合，利用总推力变化维持高度。

4. **闭环监控与测试**  
   - 使用 `blackbox` 或 MSP Telemetry（即便板载 HW 未接 Flash，仍可走串口）观察 `navDesiredHeading`/`motor[]`，验证 Loiter 圆心稳定性。
   - 借助 `rx/crsf.c:45` 的链路统计确保 ELRS 在 Loiter 时信噪充足，再逐步加入 RTH/Waypoint 测试。
   - 逐项验证：手动模式差速转弯—姿态模式维持—Loiter 无人干预稳定，再到 RTH/自动盘旋。

通过上述步骤，可在不添加襟翼/尾翼的前提下，让 `NEUTRONRCF435MINI_FW` 靠双电机差速完成“盘旋飞/遥控飞”等基本固定翼功能，同时依然利用 INAV 现成的传感器融合与导航栈实现闭环。
