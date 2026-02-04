当前目录是开源飞控inav的固件仓库。我们关注这样的通信链路：

硬件上，ELRS 接收机通过串口连接到 INAV mcu

软件上，INAV Configurator 上位机连接到 ELRS 的 TCP Server(10.0.0.1:5761)，ELRS 将inav configurator的报文包裹进crsf协议中通过串口发送给飞控。

请在`DEBUG/0203-hw-loop/`目录下实现一个端口转发并log的脚本，能够捕捉到inav发送给elrs的msp v2报文(例如开头是`$X<`)，并转发给elrs TCP Server

请你根据inav的代码实现，完善上述脚本，实现log详细的协议解包，需要能看到各种command id，以及payload的各个部分的意义。

脚本输出需要为彩色，将两个方向的报文和发生解码错误的报文分开三种颜色输出

脚本的输出需要优化可读性，不要包含冗余信息。

---

当前目录是开源飞控inav的固件仓库。请修改`DEBUG/0203-hw-loop/msp_v2_bridge.py`，实现支持两种工作模式：1. tcp转发

inav configurator --- localhost:5761 <==> 10.0.0.1:5761 --- elrs

这里没有串口环节，转发裸字节流即可。

1. 串口模式

inav configurator --- localhost:5761 <==> usb_serial --- inav

这里也是只要转发字节流。

请完善`DEBUG/0203-hw-loop/msp_v2_bridge.py`，增加一个可配置的开关，当开启时将fc->cfg的MSP2_INAV_SERVO_MIXER返回报文替换成MSP2_INAV_SERVO_MIXER_RESP（432字节的payload）

---

当前目录是开源飞控inav的固件仓库。我们关注这样的通信链路：硬件上，ELRS 接收机通过串口连接到 INAV mcu；软件上，INAV Configurator 上位机连接到 ELRS 的 TCP Server(10.0.0.1:5761)，ELRS 将inav configurator的msp v2报文包裹进crsf协议中通过串口发送给飞控。

请调查可能的固件问题：通过上面的链路处理MSP2_INAV_SERVO_MIXER(8224)命令时，飞控回复的报文长度限制到了size=176，而不是正常情况（串口直接连接飞控）的432。

```
[FC->CFG] MSPv2 > MSP2_INAV_SERVO_MIXER(8224) flags=0x00(0) size=176 crc=ok
  payload_hex=01 00 32 00 00 FF 01 01 32 00 00 FF 02 00 CE FF 00 FF 02 01 32 ...
```

正常情况下的报文：
```
[FC->CFG] MSPv2 > MSP2_INAV_SERVO_MIXER(8224) flags=0x00(0) size=432 crc=ok
  payload_hex=01 00 32 00 00 FF 01 01 32 00 00 FF 02 00 CE FF 00 FF 02 01 32 ...
```

请先调查有关msp报文、crsf隧道相关的代码。

---

请根据inav相关代码，在`DEBUG/0203-hw-loop/`下用python实现一个crsf的报文分析工具，从串口读取crsf数据，就像`DEBUG/0203-hw-loop/msp_v2_bridge.py`一样，不过读取的数据只需要打印在终端，不需要转发。

---

当前目录是开源飞控inav的固件仓库。请检查其中crsf包裹msp报文通过串口发送的隧道链路，当msp报文比较长（包括头尾和校验432字节时），隧道中传出的分包crsf报文是否有缺漏的可能。