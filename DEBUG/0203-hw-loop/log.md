发现wifi-tcp连接下，飞控对MSP2_INAV_SERVO_MIXER命令的回复长度变为了176，而不是串口直连时的432。排查发现`msp_shared.c`下面这行代码根本容纳不了大于255的长度
```c
const uint8_t size = sbufBytesRemaining(txBuf);
```

上面长度改为uint16_t后，mixer页面仍然打不开，此时飞控对MSP2_INAV_SERVO_MIXER指令不再返回报文。

```sh
python msp_v2_bridge.py --brief --serial /dev/cu.usb* --baud 115200 --log-file ./serial.log
python msp_v2_bridge.py --brief --log-file ./tcp.log
```

---

其实是飞控没能将完整的432字节报文通过crsf隧道发送到elrs，只发送了100多字节。

