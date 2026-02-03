# NEUTRONRCF435MINI 说明

- **target.h 宏对编译的影响 / 设备选择逻辑**  
  `src/main/target/NEUTRONRCF435MINI/target.h` 决定哪些外设被编译进固件。每个 `USE_*` 会拉入驱动，并在 `src/main/target/common_hardware.c` 生成 `BUSDEV_REGISTER_*`，供 `busDeviceInit()`/`busDeviceOpen()`（`src/main/drivers/bus.c`）注册。启用多个 IMU 时，`gyroDetect()`/`accDetect()` 按代码顺序探测，首个读取 WHO_AM_I 成功的传感器即被占用。若多个传感器共用同一总线/CS，引导时只能选中该 CS 上第一个响应的芯片，其他芯片不可达。本目标仅构建单陀螺 (`MAX_GYRO_COUNT 1`)，不会同时跑双陀螺。

- **IMU 支持（ICM-42688 与 ICM-20602）**  
  ICM42605 驱动已兼容 42688P，通过 WHO_AM_I 判断并设置 `is42688P`（`accgyro_icm42605.c:288+`）。保持 `USE_IMU_ICM42605`、`ICM42605_SPI_BUS`、`ICM42605_CS_PIN`，固件会在 SPI1/PA4 自动绑定 42605 或 42688。ICM-20602 由 MPU6500 驱动处理（`accgyro_mpu6500.c` 匹配 `ICM20602_WHO_AM_I_CONST`）；保留 `USE_IMU_MPU6500` 及其总线/CS 即可。两宏同时保留可支持任意一个实际焊接在该 CS 的芯片。

- **气压计 SPL06-001**  
  驱动已存在（`DEVHW_SPL06`）。在 `target.h` 添加 `#define USE_BARO_SPL06` 即可。已定义 `BARO_I2C_BUS = BUS_I2C2`，I2C 无需额外宏；若需 SPI 可另行指定 `SPL06_SPI_BUS`/`SPL06_CS_PIN`。

- **UART 作用与 USE_UART3_PIN_SWAP**（见 `target.h`）  
  - `USE_VCP`：USB CDC 口，用于 CLI/刷机，配合 `USE_USB_DETECT`。  
  - `UART1` (PA10/PA9)：通用，可接 GPS/遥测/CLI。  
  - `UART2` (PB0/PA2)：通用串口。  
  - `UART3` (PB10/PB11) + `USE_UART3_PIN_SWAP`：启用外设内部 TX/RX 交换，使 PB10/PB11 这一对生效，适配 PCB 已交换的走线。  
  - `UART5` (PB8/PB9)：通用串口。  
  - `UART7` (PB3/PB4)：默认 CRSF 接收机口（`DEFAULT_RX_TYPE`/`SERIALRX_PROVIDER`/`SERIALRX_UART` 指向它）。  
  未专门预留调试口，UART1/2/3/5 均可作 CLI/遥测，UART7 预选为 RC。

- **关闭 OSD/Blackbox/未用传感器或 UART**  
  这些是编译期开关，不是运行时 `config.h` 选项。直接在 `target.h` 注释/去掉对应 `USE_*`（如 `USE_MAX7456`、`USE_FLASHFS`、`USE_BARO_*`、各 `USE_UARTx`、`USE_VCP`）。若需要，可在包含 `common.h` 后补 `#undef USE_OSD`/`#undef USE_BLACKBOX`。移除宏后链接器会丢弃相关驱动，通常无需改其他源文件，除非同文件里还引用了被删设备的引脚定义。

- **给出的宏含义**（`target.h`）  
  - `USE_SERIAL_4WAY_BLHELI_INTERFACE`：编译 BLHeli 串口透传，用于 ESC 配置。  
  - `TARGET_IO_PORTx`：声明本板暴露的引脚掩码，若引用未开放的引脚，`IO_TAG()` 会编译报错，防止使用不存在的 PAD。`TARGET_IO_PORTE/PORTH` 仅开放 PE2 与 PH1/2/3。  
  - `MAX_PWM_OUTPUT_PORTS 8`：固件可用的马达/舵机输出上限。  
  - `USE_DSHOT`：允许在标记为电机用途的定时器上输出 DShot。  
  - `USE_ESC_SENSOR`：允许读取 DShot ESC 遥测。

- **停用 USB VCP，改用物理串口**  
  在 `target.h` 注释 `USE_VCP`/`USE_USB_DETECT`，保留至少一个硬件 UART，PC 通过该串口连接。运行时在 CLI 设置该串口为 `MSP`/`CLI`。禁用 VCP 后不会再枚举 USB CDC，刷机需用 SWD/UART。

- **2 空心杯螺旋桨 + 2 舵机的 PWM 输出示例**  
  可用定时器输出（`src/main/target/NEUTRONRCF435MINI/target.c`）：TIM4 CH1 PB6、TIM4 CH2 PB7、TIM2 CH4 PA3、TIM3 CH4 PB1，另有 TIM1 CH1 PA8 标为 LED。为区分快/慢频率，可将两电机放 TIM4（PB6/PB7），舵机放非 TIM4 的通道，如 PA3(TIM2_CH4)、PB1(TIM3_CH4)。示例 CLI（先清空资源）：  
  ```
  resource MOTOR 1 B06
  resource MOTOR 2 B07
  resource SERVO 1 A03
  resource SERVO 2 B01
  timer_output_mode 1 MOTOR
  timer_output_mode 2 MOTOR
  timer_output_mode 3 SERVO
  timer_output_mode 4 SERVO
  set motor_pwm_rate = 400   # 或使用 DShot 并调整相关设置
  set servo_pwm_rate = 50
  save
  ```  
  这样电机共用 TIM4（可高频/DShot），舵机各在独立定时器组低频运行。若需同定时器双舵机，可改用 PA8(TIM1_CH1)，并将该输出从 LED 改标为 SERVO。
