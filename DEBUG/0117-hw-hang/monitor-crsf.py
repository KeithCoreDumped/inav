import serial
import threading
import time
import sys

# 配置部分
# PORT_1 = '/dev/cu.usbmodem11202'  # AT32
PORT_2 = '/dev/cu.usbmodem11402'  # ELRS
BAUD_RATE = 420000 

# CRSF CRC8 Poly = 0xD5
def crsf_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

class PacketReassembler:
    """
    用于处理 CRSF 0x7A 类型分包重组的类
    """
    def __init__(self, port_label):
        self.label = port_label
        # 存储结构: {(src, dest): {'seq': int, 'data': bytearray, 'last_time': float}}
        self.buffers = {}
        # 超过这个时间(秒)没有新包，就认为大包结束，强制输出
        self.timeout = 0.1 

    def process(self, packet):
        """处理单帧数据，如果是0x7A则尝试重组"""
        packet_type = packet[2]
        
        # 仅处理 0x7A (MSP/Tunnel) 类型
        if packet_type == 0x7A:
            # 提取头部信息
            dest = packet[3]
            src = packet[4]
            seq = packet[5]
            payload = packet[6:-1] # 去掉头(Sync,Len,Type,Dst,Src,Seq)和尾(CRC)
            
            key = (src, dest)
            current_time = time.time()
            
            # 检查是否需要刷新旧数据（超时或序号不连续）
            if key in self.buffers:
                buf = self.buffers[key]
                time_diff = current_time - buf['last_time']
                expected_seq = (buf['seq'] + 1) % 256
                
                # 如果超时，或者序号跳变（比如丢包了，或者新的一轮开始）
                if time_diff > self.timeout or seq != expected_seq:
                    self.flush(key)
                    # 重新初始化
                    self.buffers[key] = {'seq': seq, 'data': bytearray(payload), 'last_time': current_time}
                else:
                    # 序号连续，追加数据
                    buf['data'].extend(payload)
                    buf['seq'] = seq
                    buf['last_time'] = current_time
            else:
                # 新的源-目标对
                self.buffers[key] = {'seq': seq, 'data': bytearray(payload), 'last_time': current_time}

    def check_timeouts(self):
        """在主循环空闲时调用，检查是否有超时未打印的包"""
        current_time = time.time()
        keys_to_remove = []
        
        for key, buf in self.buffers.items():
            if (current_time - buf['last_time']) > self.timeout:
                self.flush(key)
                keys_to_remove.append(key)
        
        for key in keys_to_remove:
            del self.buffers[key]

    def flush(self, key):
        """输出重组好的大包"""
        if key not in self.buffers:
            return
            
        buf = self.buffers[key]
        src, dest = key
        data = buf['data']
        
        # 只有长度大于一定值才认为是拼装包（可选过滤）
        if len(data) > 0:
            hex_str = data.hex(' ').upper()
            # 尝试识别 MSP 头 (MSP V2 通常以 $X 开始，但封装在CRSF里可能是纯Payload)
            # print(f"\n >>> [{self.label} REASSEMBLED] Src:{src:02X}->Dest:{dest:02X} | Total Payload ({len(data)}B): {hex_str}\n")

def parse_crsf_buffer(port_name, buffer, name, reassembler):
    """
    解析函数，增加 reassembler 参数
    """
    if len(buffer) < 4: return 0

    sync_byte = buffer[0]
    if sync_byte < 0xC0: return 1 # 简单的非法头过滤

    frame_len = buffer[1]
    total_packet_len = frame_len + 2

    if total_packet_len > 64 or total_packet_len < 4: return 1
    if len(buffer) < total_packet_len: return 0

    # 提取完整报文
    packet = buffer[:total_packet_len]
    
    # CRC 校验
    payload_for_crc = packet[2:-1] 
    received_crc = packet[-1]
    calculated_crc = crsf_crc8(payload_for_crc)
    
    is_crc_valid = (calculated_crc == received_crc)
    crc_status = "✅" if is_crc_valid else "❌"

    # --- 打印原始小包 (用户要求) ---
    hex_str = packet.hex(' ').upper()
    # 稍微简化一下小包打印，避免刷屏太快看不清，只保留关键信息
    # Type 0x7A 显示为 MSP
    p_type = packet[2]
    type_str = f"Type:{p_type:02X}"
    if p_type == 0x7A: type_str = "Type:MSP"
    
    print(f"[{name}] {type_str} Len:{total_packet_len:<2} CRC:{crc_status} | {hex_str}")

    # --- 只有校验通过才进行重组 ---
    if is_crc_valid and reassembler:
        reassembler.process(packet)

    return total_packet_len

def read_from_port(port_name, baud, name):
    try:
        ser = serial.Serial(port_name, baud, timeout=0.01) # timeout设短一点，方便快速轮询
        print(f"[系统] {port_name} ({name}) 就绪")
    except Exception as e:
        print(f"[错误] {port_name}: {e}")
        return

    rx_buffer = bytearray()
    
    # 实例化重组器
    reassembler = PacketReassembler(name)

    while True:
        try:
            # 1. 读取数据
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                rx_buffer.extend(new_data)
                
                while True:
                    # 传入 reassembler
                    processed_count = parse_crsf_buffer(port_name, rx_buffer, name, reassembler)
                    if processed_count == 0:
                        break 
                    else:
                        del rx_buffer[:processed_count]
            else:
                # 2. 空闲时，检查重组超时
                # (如果数据流突然停止，这里负责把最后积攒的包打印出来)
                reassembler.check_timeouts()
                time.sleep(0.002)

        except Exception as e:
            print(f"[{name}] 异常: {e}")
            break

def main():
    print(f"--- CRSF 双串口监视器 (带MSP重组功能) ---")
    
    # t1 = threading.Thread(target=read_from_port, args=(PORT_1, BAUD_RATE, "AT32"))
    t2 = threading.Thread(target=read_from_port, args=(PORT_2, BAUD_RATE, "ELRS"))

    # t1.daemon = True
    t2.daemon = True

    # t1.start()
    t2.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sys.exit()

if __name__ == "__main__":
    main()