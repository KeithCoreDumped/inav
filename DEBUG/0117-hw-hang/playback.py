import serial
import time
import sys
import binascii

# --- 配置部分 ---
LOG_FILE = 'inav_crash.txt'       # 回放的日志文件名
SERIAL_PORT = '/dev/cu.usbmodem11202'       # 发送数据的串口 (如 '/dev/ttyUSB0' 或 'COMx')
BAUD_RATE = 420000         # 波特率
TIMEOUT_SEC = 0.5          # 等待回复的超时时间(秒)

# --- CRSF CRC8 算法 ---
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

# --- 接收并校验单个CRSF包 ---
def wait_for_valid_crsf_packet(ser, timeout):
    """
    阻塞读取串口，直到捕获到一个CRC校验正确的CRSF包或超时
    返回: (Packet_Bytes, Status_Msg) 或 (None, Error_Msg)
    """
    start_time = time.time()
    rx_buffer = bytearray()

    while (time.time() - start_time) < timeout:
        # 1. 读取所有缓冲区数据
        if ser.in_waiting > 0:
            rx_buffer.extend(ser.read(ser.in_waiting))
        
        # 2. 尝试解析缓冲区
        # 最小包长 4: [Sync] [Len] [Type] [CRC]
        while len(rx_buffer) >= 4:
            sync_byte = rx_buffer[0]
            
            # (A) 校验帧头 (CRSF通常 >= 0xC0)
            if sync_byte < 0xC0:
                rx_buffer.pop(0) # 丢弃非法的头，滑动窗口
                continue

            # (B) 校验长度
            payload_len = rx_buffer[1] # Len字节 = Type + Payload + CRC
            total_packet_len = payload_len + 2 # 总长 = Sync + Len + (Len字节的值)
            
            # 长度合法性检查 (4 ~ 64)
            if total_packet_len < 4 or total_packet_len > 64:
                rx_buffer.pop(0) # 长度不可能，丢弃头
                continue

            # (C) 检查数据是否接收完整
            if len(rx_buffer) < total_packet_len:
                break # 数据不够，跳出内层循环，继续读串口

            # (D) 提取完整包并校验 CRC
            packet = rx_buffer[:total_packet_len]
            
            # CRC范围: Type(index 2) 到 Payload结束 (倒数第2个)
            payload_for_crc = packet[2:-1]
            received_crc = packet[-1]
            calc_crc = crsf_crc8(payload_for_crc)

            if calc_crc == received_crc:
                return packet, "✅ CRC OK"
            else:
                # CRC 错误，可能是碰巧凑出的头和长度，丢弃这个包的头，继续找
                # 注意：这里不能把整个包丢弃，只能丢弃头，防止错过了粘在一起的真包
                rx_buffer.pop(0) 
                continue
        
        # 稍微休眠，释放CPU
        time.sleep(0.005)

    return None, "❌ Timeout / No Valid Packet"

# --- 主程序 ---
def main():
    try:
        # 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[System] Serial {SERIAL_PORT} opened @ {BAUD_RATE}")
    except Exception as e:
        print(f"[Error] Failed to open serial: {e}")
        return

    try:
        with open(LOG_FILE, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"[Error] File {LOG_FILE} not found.")
        return

    print(f"[System] Starting playback of {len(lines)} lines...")
    print("-" * 60)

    success_count = 0
    fail_count = 0

    for i, line in enumerate(lines):
        line = line.strip()
        if not line: continue

        # 1. 解析日志行
        parts = line.split('|')
        if len(parts) < 2:
            print(f"[Line {i+1}] Skip (No separator '|')")
            continue
        
        hex_str = parts[1].strip()
        
        try:
            # 将Hex字符串转为bytes
            send_data = bytes.fromhex(hex_str)
        except binascii.Error:
            print(f"[Line {i+1}] Skip (Invalid Hex): {hex_str[:20]}...")
            continue

        # 2. 发送数据
        print(f"[Tx #{i+1}] Sending {len(send_data)} bytes...", end='', flush=True)
        ser.reset_input_buffer() # 发送前清空接收缓存，确保收到的是针对这次的回复
        ser.write(send_data)

        # 3. 等待回复
        reply_packet, status = wait_for_valid_crsf_packet(ser, TIMEOUT_SEC)

        if reply_packet:
            success_count += 1
            print(f" -> [Rx] {reply_packet.hex(' ').upper()} ({status})")
        else:
            fail_count += 1
            print(f" -> [Rx] {status}")

        # 可选：行间延迟
        time.sleep(0.01)

    print("-" * 60)
    print(f"Playback Finished. Success: {success_count}, Fail/Timeout: {fail_count}")
    ser.close()

if __name__ == "__main__":
    main()