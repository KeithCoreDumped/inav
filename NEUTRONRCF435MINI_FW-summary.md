# NEUTRONRCF435MINI_FW 工作总结

- 新建目标：复制 `src/main/target/NEUTRONRCF435MINI` 为 `src/main/target/NEUTRONRCF435MINI_FW`，CMake 名称改为 `NEUTRONRCF435MINI_FW`，板标识 `NRFW`，产品串 “NeuronRC F435 MINI FW”。
- 传感器裁剪：保留 MPU6500/ICM20602 与 ICM42605/42688（同 SPI1），移除 BMI270/LSM6DXX；气压计改为 `USE_BARO_SPL06`，关闭 BMP280/DPS310/磁力计。
- 串口与特性：禁用 USB VCP/检测，保留 UART1+UART7，串口计数 2，默认 CRSF 在 UART7。关闭 OSD、Flash/Blackbox（去掉 SPI2、MAX7456、Flash 芯片定义），默认特性移除 OSD。
- PWM 输出：按 2 电机 + 2 舵机配置定时器——电机 TIM4_CH1 PB6、TIM4_CH2 PB7；舵机 TIM2_CH4 PA3、TIM3_CH4 PB1，标记为 MOTOR/SERVO 以支持不同频率。
- OSD/Blackbox 禁用方式说明：INAV 的设置表和 CMS 菜单依赖 `USE_OSD`、`USE_BLACKBOX` 生成大量 `SETTING_*`，直接 `#undef` 会导致编译缺符号。正确做法是保留这两个宏，让编译通过，同时移除具体硬件宏（如 `USE_MAX7456`、Flash 芯片）并在运行时关闭对应 feature（例如 CLI `feature -OSD`/`feature -BLACKBOX`）。

构建命令：`make NEUTRONRCF435MINI_FW`。如需调整引脚或功能，可直接修改该 target 的 `target.h`/`target.c`。测试尚未执行。 
