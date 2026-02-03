# Role
你是一个资深的嵌入式系统专家，精通STM32/AT32架构、iNav/Betaflight开源飞控代码、CRSF通信协议以及GDB高级调试技巧。

# Context & Goal
我正在开发inav飞控（Target: `NEUTRONRCF435MINI_FW`）。目前遇到一个严重BUG：当通过UART5接收特定的CRSF协议报文序列时，飞控会进入HardFault死机状态。
你需要利用现有的工具链（CMake, OpenOCD, GDB, Python）复现问题，定位崩溃发生的具体代码行，分析协议报文异常原因，并修复该问题。

# Environment Info
* **Build System:** CMake
* **Target:** `NEUTRONRCF435MINI_FW`
* **Debug Tool:** OpenOCD + arm-none-eabi-gdb
* **Serial Tool:** `DEBUG/playback.py` (用于向飞控UART5发送录制好的二进制数据)
* **Connection:** 飞控已通过DAP-Link连接，且串口已映射到宿主机。

# Symptom Data
运行 playback 脚本后，日志如下：
```txt
[Tx #64] Sending 12 bytes... -> [Rx] C8 13 7B EC C8 5C 00 10 20 09 00 00 00 01 01 00 1B 00 0C 12 C6 (✅ CRC OK)
[Tx #65] Sending 12 bytes... -> [Rx] C8 04 07 00 00 23 (✅ CRC OK)
[Tx #66] Sending 12 bytes... -> [Rx] ❌ Timeout / No Valid Packet (System Halted)
```

注意：Tx #65 发送后，飞控似乎还活着并回复了数据，但在处理 Tx #66 或其后续逻辑时崩溃。

# Step-by-Step Investigation Task

请严格按照以下步骤执行任务，每一步都必须验证结果：

## Phase 1: Preparation & Reproduction

1. **检查环境**：确认当前目录结构，查看 `DEBUG/playback.py` 源码，理解它是如何发送数据的，以及它是如何判断“Timeout”的。
2. **建立调试连接**：
* 启动 OpenOCD 后台进程。
* 使用 `arm-none-eabi-gdb` 连接目标。
* **关键设置**：在GDB中设置 `set pagination off` 和 `set logging on`。


3. **复现崩溃**：
* 在GDB中运行固件 (`continue`)。
* 同时在终端运行 `playback.py` 触发故障。
* 等待GDB捕获到 `HardFault` 或程序停止响应。



## Phase 2: Post-Mortem Analysis (Crucial)

一旦复现崩溃，请在GDB中执行以下“尸检”操作：

1. **获取调用栈**：执行 `bt` (backtrace) 和 `bt full`，查看崩溃时的函数调用链。
2. **定位PC指针**：查看 `info registers`，重点关注 `pc`, `lr`, `sp`。
3. **分析异常状态**：查看SCB寄存器（如CFSR, HFSR, MMFAR, BFAR），判断是总线错误(Bus Fault)、存储器管理错误(MemManage Fault)还是非法指令(Usage Fault)。
* *提示：如果是指针越界，MMFAR将显示试图访问的非法地址。*


4. **关联源码**：使用 `list *0x...` (PC地址) 定位具体是哪一行C代码导致了崩溃。

## Phase 3: Protocol & Root Cause Analysis

1. **解码报文**：分析日志中 [Tx #65] 和 [Tx #66] 的原始Hex数据。
* CRSF帧结构：`[Sync] [Len] [Type] [Payload...] [CRC]`
* 分析崩溃前的最后一条报文意图是什么？是Link Statistics？RC Channels？还是MSP封装？


2. **逻辑推演**：结合 Phase 2 定位的代码行和 Phase 3 解码的报文，解释为什么这条报文会导致该行代码崩溃（例如：数组越界、空指针解引用、除以零、看门狗超时等）。

## Phase 4: Fix & Verify

1. **实施修复**：修改代码以处理该异常情况（添加边界检查、空指针检查等）。
2. **验证修复**：重新编译、烧录，再次运行 `playback.py`，确保不再出现 Timeout 且飞控正常响应后续报文。

# Output Requirements

任务完成后，请提交一份Markdown报告，包含：

1. **Bug定位**：崩溃发生的文件名、行号、函数名。
2. **HardFault原因**：具体的寄存器分析（如尝试读取了地址 0x00000000）。
3. **报文分析**：导致崩溃的CRSF报文的具体含义。
4. **修复方案**：Diff格式的代码变更。
