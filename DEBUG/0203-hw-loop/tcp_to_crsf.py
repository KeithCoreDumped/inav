import socket
import serial
import threading
import time
import binascii
import sys

# Configuration
TCP_HOST = '0.0.0.0'
TCP_PORT = 5761
SERIAL_PORT = '/dev/cu.usbmodem11402'
SERIAL_BAUDRATE = 420000  # Standard ELRS baud

# CRSF/MSP Constants (mirror C++ headers)
CRSF_SYNC_BYTE = 0xC8
CRSF_MAX_PACKET_LEN = 64
CRSF_FRAME_NOT_COUNTED_BYTES = 2
CRSF_TELEMETRY_LENGTH_INDEX = 1
CRSF_TELEMETRY_TYPE_INDEX = 2
CRSF_TELEMETRY_CRC_LENGTH = 1

CRSF_FRAMETYPE_MSP_REQ = 0x7A
CRSF_FRAMETYPE_MSP_RESP = 0x7B

CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA

CRSF_MSP_FRAME_OFFSET = 6
CRSF_MSP_STATUS_BYTE_OFFSET = 5
CRSF_MSP_SRC_OFFSET = 4
CRSF_MSP_DEST_OFFSET = 3
CRSF_FRAME_PAYLOAD_LEN_IDX = 1
CRSF_EXT_FRAME_PAYLOAD_LEN_SIZE_OFFSET = 5
CRSF_MSP_MAX_BYTES_PER_CHUNK = 57
CRSF_MSP_TYPE_IDX = 2

MSP_FRAME_MAX_LEN = 512

MSP_FRAME_V1 = 1
MSP_FRAME_V2 = 2
MSP_FRAME_V1_JUMBO = 3
MSP_FRAME_UNKNOWN = 4

# CRC8 Poly 0xD5 implementation

def crc8_dvb_s2(crc, ch):
    crc ^= ch
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF


def calc_crsf_crc(data):
    crc = 0
    for byte in data:
        crc = crc8_dvb_s2(crc, byte)
    return crc


# ANSI Color Codes
class Colors:
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'


# MSP helpers (mirror lib/CRSF2MSP)

def msp_get_version(data):
    if len(data) < 4:
        return MSP_FRAME_UNKNOWN
    if data[1] == ord('M'):
        if data[3] == 0xFF:
            return MSP_FRAME_V1_JUMBO
        return MSP_FRAME_V1
    if data[1] == ord('X'):
        return MSP_FRAME_V2
    return MSP_FRAME_UNKNOWN


def msp_get_payload_len(data, version):
    if version == MSP_FRAME_V1:
        return data[3] if len(data) > 3 else 0
    if version == MSP_FRAME_V2:
        if len(data) > 7:
            return data[6] | (data[7] << 8)
        return 0
    if version == MSP_FRAME_V1_JUMBO:
        if len(data) > 6:
            return data[5] | (data[6] << 8)
        return 0
    return 0


def msp_frame_len_from_payload(payload_len, version):
    if version == MSP_FRAME_V1:
        return payload_len + 2
    if version == MSP_FRAME_V2:
        return payload_len + 5
    if version == MSP_FRAME_V1_JUMBO:
        return payload_len + 4
    return 0


def msp_dir_to_crsf_type(direction_byte):
    if direction_byte == ord('<'):
        return CRSF_FRAMETYPE_MSP_REQ
    if direction_byte == ord('>'):
        return CRSF_FRAMETYPE_MSP_RESP
    return 0


def crsf_type_to_msp_dir(frame_type):
    if frame_type == CRSF_FRAMETYPE_MSP_REQ:
        return ord('<')
    if frame_type == CRSF_FRAMETYPE_MSP_RESP:
        return ord('>')
    return ord('!')


def msp_status_get_version(status_byte, payload_len_byte):
    header_version = (status_byte & 0x60) >> 5
    if header_version == 1:
        if payload_len_byte == 0xFF:
            return MSP_FRAME_V1_JUMBO
        return MSP_FRAME_V1
    if header_version == 2:
        return MSP_FRAME_V2
    return MSP_FRAME_UNKNOWN


def msp_checksum(data, msp_version):
    if msp_version in (MSP_FRAME_V1, MSP_FRAME_V1_JUMBO):
        check_sum = 0
        for byte in data:
            check_sum ^= byte
        return check_sum & 0xFF
    if msp_version == MSP_FRAME_V2:
        return calc_crsf_crc(data)
    return 0


class Msp2CrsfEncoder:
    def __init__(self):
        self.seq_num = 0

    def encode(self, msp_frame, src=CRSF_ADDRESS_CRSF_RECEIVER, dest=CRSF_ADDRESS_FLIGHT_CONTROLLER):
        if len(msp_frame) < 4:
            return []

        version = msp_get_version(msp_frame)
        payload_len = msp_get_payload_len(msp_frame, version)
        msp_frame_len = msp_frame_len_from_payload(payload_len, version)

        num_chunks = (msp_frame_len // CRSF_MSP_MAX_BYTES_PER_CHUNK) + 1
        chunk_remainder = msp_frame_len % CRSF_MSP_MAX_BYTES_PER_CHUNK

        crsf_type = msp_dir_to_crsf_type(msp_frame[2])

        version_bits = 0
        if version in (MSP_FRAME_V1, MSP_FRAME_V1_JUMBO):
            version_bits = (1 << 5)
        elif version == MSP_FRAME_V2:
            version_bits = (2 << 5)

        frames = []
        for i in range(num_chunks):
            status = version_bits
            status = (status & ~0x0F) | (self.seq_num & 0x0F)
            self.seq_num = (self.seq_num + 1) & 0x0F
            if i == 0:
                status |= 0x10
            else:
                status &= ~0x10
            status &= 0x7F  # clear error bit

            start_idx = (i * CRSF_MSP_MAX_BYTES_PER_CHUNK) + 3
            chunk_len = chunk_remainder if i == (num_chunks - 1) else CRSF_MSP_MAX_BYTES_PER_CHUNK
            chunk = msp_frame[start_idx:start_idx + chunk_len]

            frame_len_field = chunk_len + CRSF_EXT_FRAME_PAYLOAD_LEN_SIZE_OFFSET

            frame = bytearray()
            frame.append(CRSF_ADDRESS_FLIGHT_CONTROLLER)
            frame.append(frame_len_field)
            frame.append(crsf_type)
            frame.append(dest)
            frame.append(src)
            frame.append(status)
            frame.extend(chunk)

            crc = calc_crsf_crc(frame[2:])
            frame.append(crc)

            frames.append(frame)

        return frames


class Crsf2MspDecoder:
    def __init__(self):
        self.seq_prev = 0
        self.out_buffer = bytearray(MSP_FRAME_MAX_LEN)
        self.reset()

    def reset(self):
        self.pkt_len = 0
        self.idx = 0
        self.msp_version = MSP_FRAME_UNKNOWN

    def parse(self, data):
        if len(data) <= CRSF_MSP_FRAME_OFFSET:
            return []

        crsf_payload_len = data[CRSF_FRAME_PAYLOAD_LEN_IDX] - CRSF_EXT_FRAME_PAYLOAD_LEN_SIZE_OFFSET
        status = data[CRSF_MSP_STATUS_BYTE_OFFSET]

        error = (status & 0x80) != 0
        new_frame = (status & 0x10) != 0

        seq_number = status & 0x0F
        seq_next = (self.seq_prev + 1) & 0x0F
        seq_error = (seq_next != seq_number)
        self.seq_prev = seq_number

        if ((not new_frame and seq_error) or error):
            self.reset()
            return []

        if new_frame:
            self.idx = 3
            payload_len_byte = data[CRSF_MSP_FRAME_OFFSET]
            self.msp_version = msp_status_get_version(status, payload_len_byte)

            if self.msp_version in (MSP_FRAME_V1, MSP_FRAME_V1_JUMBO):
                self.out_buffer[0] = ord('$')
                self.out_buffer[1] = ord('M')
            else:
                self.out_buffer[0] = ord('$')
                self.out_buffer[1] = ord('X')

            self.out_buffer[2] = ord('!') if error else crsf_type_to_msp_dir(data[CRSF_MSP_TYPE_IDX])
            self.pkt_len = self._get_frame_len_from_crsf(data, self.msp_version)

            if self.pkt_len + 4 > MSP_FRAME_MAX_LEN:
                self.reset()
                return []

        frame_len = self.pkt_len - (self.idx - 3)
        min_len = frame_len if frame_len < crsf_payload_len else crsf_payload_len

        if min_len > 0:
            start = CRSF_MSP_FRAME_OFFSET
            self.out_buffer[self.idx:self.idx + min_len] = data[start:start + min_len]
        self.idx += min_len

        if self.idx - 3 == self.pkt_len:
            checksum = msp_checksum(self.out_buffer[3:3 + self.pkt_len], self.msp_version)
            self.out_buffer[self.idx] = checksum
            frame = bytes(self.out_buffer[:self.idx + 1])
            return [frame]

        return []

    def _get_frame_len_from_crsf(self, data, msp_version):
        if msp_version == MSP_FRAME_V1:
            payload_len = data[CRSF_MSP_FRAME_OFFSET]
            return payload_len + 2
        if msp_version == MSP_FRAME_V1_JUMBO:
            low = data[CRSF_MSP_FRAME_OFFSET + 2]
            high = data[CRSF_MSP_FRAME_OFFSET + 3]
            return ((high << 8) | low) + 4
        if msp_version == MSP_FRAME_V2:
            low = data[CRSF_MSP_FRAME_OFFSET + 3]
            high = data[CRSF_MSP_FRAME_OFFSET + 4]
            return ((high << 8) | low) + 5
        return 0


class CrsfStreamParser:
    STATE_IDLE = 0
    STATE_LENGTH = 1
    STATE_DATA = 2

    def __init__(self):
        self.state = self.STATE_IDLE
        self.current_len = 0
        self.current_idx = 0
        self.buffer = bytearray(CRSF_MAX_PACKET_LEN)

    def feed(self, data):
        frames = []
        for byte in data:
            if self.state == self.STATE_IDLE:
                if byte in (CRSF_SYNC_BYTE, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_CRSF_RECEIVER):
                    self.buffer[0] = byte
                    self.state = self.STATE_LENGTH
                continue

            if self.state == self.STATE_LENGTH:
                if byte >= CRSF_MAX_PACKET_LEN:
                    self.state = self.STATE_IDLE
                    continue
                self.current_len = byte
                self.buffer[CRSF_TELEMETRY_LENGTH_INDEX] = byte
                self.current_idx = 0
                self.state = self.STATE_DATA
                continue

            if self.state == self.STATE_DATA:
                self.buffer[self.current_idx + CRSF_FRAME_NOT_COUNTED_BYTES] = byte
                self.current_idx += 1
                if self.current_idx == self.current_len:
                    crc = calc_crsf_crc(self.buffer[CRSF_FRAME_NOT_COUNTED_BYTES:CRSF_FRAME_NOT_COUNTED_BYTES + self.current_len - CRSF_TELEMETRY_CRC_LENGTH])
                    self.state = self.STATE_IDLE
                    if byte == crc:
                        frame = bytes(self.buffer[:self.current_len + CRSF_FRAME_NOT_COUNTED_BYTES])
                        frames.append(frame)
        return frames


class TunnelServer:
    def __init__(self):
        self.tcp_server = None
        self.client_socket = None
        self.serial_conn = None
        self.running = True
        self.msp2crsf = Msp2CrsfEncoder()
        self.crsf2msp = Crsf2MspDecoder()
        self.crsf_parser = CrsfStreamParser()

    def log(self, direction, message, data=None):
        timestamp = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() * 1000) % 1000:03d}"

        color = Colors.WHITE
        if "TCP->SER" in direction:
            color = Colors.GREEN
        elif "SER->TCP" in direction:
            color = Colors.BLUE
        elif "GEN" in direction:
            color = Colors.CYAN
        elif "SYS" in direction:
            color = Colors.YELLOW

        log_msg = f"{color}[{timestamp}] [{direction}] {message}"
        if data:
            hex_data = binascii.hexlify(data).decode('utf-8').upper()
            hex_formatted = ' '.join(hex_data[i:i + 2] for i in range(0, len(hex_data), 2))
            log_msg += f" | Data: {hex_formatted}"

        log_msg += Colors.RESET
        print(log_msg)

    def log_error(self, message):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"{Colors.RED}[{timestamp}] [ERROR] {message}{Colors.RESET}", file=sys.stderr)

    def start(self):
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
            self.log("SYS", f"Connected to Serial Port: {SERIAL_PORT} @ {SERIAL_BAUDRATE}")

            self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server.bind((TCP_HOST, TCP_PORT))
            self.tcp_server.listen(1)
            self.log("SYS", f"TCP Server listening on {TCP_HOST}:{TCP_PORT}")

            threading.Thread(target=self.serial_reader, daemon=True).start()

            while self.running:
                try:
                    client, addr = self.tcp_server.accept()
                    self.log("TCP", f"Client connected: {addr}")
                    self.client_socket = client
                    self.handle_client(client)
                except Exception as e:
                    self.log_error(f"TCP Accept/Handle Error: {e}")
                    if self.client_socket:
                        self.client_socket.close()
                        self.client_socket = None

        except Exception as e:
            self.log_error(f"Startup Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        if self.tcp_server:
            self.tcp_server.close()
        if self.serial_conn:
            self.serial_conn.close()
        self.log("SYS", "Shutdown complete")

    def serial_reader(self):
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        self.log("SER->TCP", f"Received {len(data)} bytes", data)
                        frames = self.crsf_parser.feed(data)
                        for frame in frames:
                            frame_type = frame[CRSF_TELEMETRY_TYPE_INDEX]
                            if frame_type not in (CRSF_FRAMETYPE_MSP_REQ, CRSF_FRAMETYPE_MSP_RESP):
                                continue
                            if frame[CRSF_MSP_SRC_OFFSET] != CRSF_ADDRESS_FLIGHT_CONTROLLER:
                                continue
                            msp_frames = self.crsf2msp.parse(frame)
                            for msp_frame in msp_frames:
                                if self.client_socket:
                                    try:
                                        self.client_socket.sendall(msp_frame)
                                        self.log("SER->TCP", f"Sent MSP frame ({len(msp_frame)} bytes)", msp_frame)
                                    except Exception as e:
                                        self.log_error(f"TCP Send Error: {e}")
            except Exception as e:
                self.log_error(f"Serial Read Error: {e}")
                time.sleep(1)

    def handle_client(self, client):
        while self.running:
            try:
                data = client.recv(4096)
                if not data:
                    break

                self.log("TCP->SER", f"Received {len(data)} bytes from TCP", data)
                self.process_msp_to_crsf(data)

            except ConnectionResetError:
                break
            except Exception as e:
                self.log_error(f"Client Loop Error: {e}")
                break

        self.log("TCP", "Client disconnected")
        if self.client_socket == client:
            self.client_socket = None
        client.close()

    def process_msp_to_crsf(self, msp_data):
        if not msp_data:
            return
        frames = self.msp2crsf.encode(msp_data)
        for frame in frames:
            self.log("GEN", "Built CRSF frame", frame)
            if self.serial_conn:
                self.serial_conn.write(frame)


if __name__ == "__main__":
    print("Starting TCP-CRSF Tunnel...")
    server = TunnelServer()
    try:
        server.start()
    except KeyboardInterrupt:
        server.cleanup()
    except Exception as e:
        print(f"Fatal Error: {e}")
