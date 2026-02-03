当前目录是开源飞控inav的固件仓库。我们关注这样的通信链路：

硬件上，ELRS 接收机通过串口连接到 INAV mcu

软件上，INAV Configurator 上位机连接到 ELRS 的 TCP Server(10.0.0.1:5761)，ELRS 将inav configurator的报文包裹进crsf协议中通过串口发送给飞控。

`DEBUG/0203-hw-loop/tcp_proxy.py` 中实现了简单的端口转发并log的机制，能够捕捉到inav发送给elrs的msp v2报文。请你根据inav的代码实现，完善上述脚本，实现log详细的协议解包。