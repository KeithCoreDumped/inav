import socket
import serial
import threading
import time
import binascii
import sys
import re
import json
from pathlib import Path

# Configuration
TCP_HOST = '0.0.0.0'
TCP_PORT = 5761
SERIAL_PORT = '/dev/cu.usbmodem11402'
SERIAL_BAUDRATE = 420000  # Standard ELRS baud

# Log filters (mirrors crsf_monitor.py) + JSONL file logging
CONFIG = {
    "FILTER_MSP_CMDS": {
        "MSP_ATTITUDE",
        "MSP_SENSOR_STATUS",
        "MSP2_INAV_STATUS",
        "MSP2_INAV_ANALOG",
        "MSP_ACTIVEBOXES",
    },
    "FILTER_CRSF_TYPES": {
        "ATTITUDE",
        "BATTERY",
        "FLIGHT_MODE",
        "VARIO",
        "BARO_ALT",
        "LINK_STATS",
    },
    "LOG_FILE": "crsf.log",
    "LOG_FILE_APPEND": False,
}

FILTER_MSP_CMDS = set(CONFIG["FILTER_MSP_CMDS"])
FILTER_CRSF_TYPES = set(CONFIG["FILTER_CRSF_TYPES"])
LOG_FILE = CONFIG["LOG_FILE"]
LOG_FILE_APPEND = CONFIG["LOG_FILE_APPEND"]

# CRSF/MSP Constants (mirror C++ headers)
CRSF_SYNC_BYTE = 0xC8
CRSF_MAX_PACKET_LEN = 64
CRSF_FRAME_NOT_COUNTED_BYTES = 2
CRSF_TELEMETRY_LENGTH_INDEX = 1
CRSF_TELEMETRY_TYPE_INDEX = 2
CRSF_TELEMETRY_CRC_LENGTH = 1

CRSF_FRAMETYPE_MSP_REQ = 0x7A
CRSF_FRAMETYPE_MSP_RESP = 0x7B
CRSF_FRAMETYPE_MSP_WRITE = 0x7C
CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D

CRSF_FRAMETYPE_GPS = 0x02
CRSF_FRAMETYPE_VARIO_SENSOR = 0x07
CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
CRSF_FRAMETYPE_BAROMETER_ALTITUDE = 0x09
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_ATTITUDE = 0x1E
CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
CRSF_FRAMETYPE_DEVICE_PING = 0x28
CRSF_FRAMETYPE_DEVICE_INFO = 0x29
CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B
CRSF_FRAMETYPE_PARAMETER_READ = 0x2C
CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D
CRSF_FRAMETYPE_COMMAND = 0x32

FRAME_TYPES = {
    0x02: "GPS",
    0x07: "VARIO",
    0x08: "BATTERY",
    0x09: "BARO_ALT",
    0x14: "LINK_STATS",
    0x16: "RC_CHANNELS",
    0x1E: "ATTITUDE",
    0x21: "FLIGHT_MODE",
    0x28: "DEVICE_PING",
    0x29: "DEVICE_INFO",
    0x2B: "PARAM_ENTRY",
    0x2C: "PARAM_READ",
    0x2D: "PARAM_WRITE",
    0x32: "COMMAND",
    0x7A: "MSP_REQ",
    0x7B: "MSP_RESP",
    0x7C: "MSP_WRITE",
    0x7D: "DISPLAYPORT",
}

CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA

CRSF_ADDR_NAME = {
    CRSF_ADDRESS_FLIGHT_CONTROLLER: "FC",
    CRSF_ADDRESS_RADIO_TRANSMITTER: "TX",
    CRSF_ADDRESS_CRSF_RECEIVER: "RX",
}

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


def load_msp_command_map(repo_root: Path):
    files = [
        repo_root / "src/main/msp/msp_protocol.h",
        repo_root / "src/main/msp/msp_protocol_v2_common.h",
        repo_root / "src/main/msp/msp_protocol_v2_inav.h",
        repo_root / "src/main/msp/msp_protocol_v2_sensor.h",
        repo_root / "src/main/msp/msp_protocol_v2_sensor_msg.h",
    ]
    cmd_map = {}
    pattern = re.compile(r"^#define\s+(MSP[0-9A-Z_]+)\s+([0-9xA-Fa-f]+)")

    for path in files:
        if not path.exists():
            continue
        try:
            for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
                match = pattern.match(line.strip())
                if not match:
                    continue
                name, value = match.group(1), match.group(2)
                try:
                    cmd_id = int(value, 0)
                except ValueError:
                    continue
                if cmd_id not in cmd_map:
                    cmd_map[cmd_id] = name
        except Exception:
            pass
    return cmd_map


def msp_cmd_label(cmd, version, cmd_map):
    if cmd is None:
        return "-"
    name = cmd_map.get(cmd)
    if version == MSP_FRAME_V2:
        cmd_id = f"0x{cmd:04X}"
    else:
        cmd_id = f"{cmd}"
    if name:
        return f"{name}({cmd_id})"
    return cmd_id


def describe_msp_frame(data, cmd_map, filter_cmds=None):
    if not data or len(data) < 4:
        return None, False
    if data[0] != ord('$'):
        return None, False

    version = msp_get_version(data)
    if version == MSP_FRAME_UNKNOWN:
        return None, False

    dir_byte = data[2]
    dir_label = {ord('<'): "REQ", ord('>'): "RESP", ord('!'): "ERR"}.get(
        dir_byte, f"0x{dir_byte:02X}"
    )
    version_label = {
        MSP_FRAME_V1: "v1",
        MSP_FRAME_V1_JUMBO: "v1J",
        MSP_FRAME_V2: "v2",
    }.get(version, "v?")

    payload_len = None
    cmd = None
    cmd_name = None
    flags = None
    header_end = None

    if version == MSP_FRAME_V1:
        if len(data) >= 5:
            payload_len = data[3]
            cmd = data[4]
            header_end = 5
    elif version == MSP_FRAME_V1_JUMBO:
        if len(data) >= 7:
            cmd = data[4]
            payload_len = data[5] | (data[6] << 8)
            header_end = 7
    elif version == MSP_FRAME_V2:
        if len(data) >= 8:
            flags = data[3]
            cmd = data[4] | (data[5] << 8)
            payload_len = data[6] | (data[7] << 8)
            header_end = 8

    if payload_len is None or header_end is None:
        return f"MSP {version_label} dir={dir_label} header_incomplete len={len(data)}", False

    total_len = header_end + payload_len + 1
    complete = len(data) >= total_len
    checksum_ok = None
    if complete:
        checksum = data[header_end + payload_len]
        calc = msp_checksum(data[3:header_end + payload_len], version)
        checksum_ok = checksum == calc

    filtered = False
    if cmd is not None:
        cmd_name = cmd_map.get(cmd)
        if filter_cmds and cmd_name in filter_cmds:
            filtered = True

    parts = [
        f"MSP {version_label}",
        f"dir={dir_label}",
        f"cmd={msp_cmd_label(cmd, version, cmd_map)}",
        f"len={payload_len}",
    ]
    if flags is not None:
        parts.append(f"flags=0x{flags:02X}")
    if checksum_ok is not None:
        parts.append("ck=OK" if checksum_ok else "ck=BAD")
    if not complete:
        parts.append(f"partial={len(data)}/{total_len}")

    return " ".join(parts), filtered


def _addr_name(addr):
    name = CRSF_ADDR_NAME.get(addr)
    if name:
        return f"{name}(0x{addr:02X})"
    return f"0x{addr:02X}"


def describe_crsf_frame(frame):
    if not frame or len(frame) < 4:
        return None
    length = frame[CRSF_TELEMETRY_LENGTH_INDEX]
    if length < 2 or len(frame) < length + CRSF_FRAME_NOT_COUNTED_BYTES:
        return f"CRSF len={length} incomplete"
    type_id = frame[CRSF_TELEMETRY_TYPE_INDEX]
    payload_len = max(0, length - 2)
    payload_start = 3
    payload_end = payload_start + payload_len
    crc = frame[payload_end]
    calc = calc_crsf_crc(frame[CRSF_TELEMETRY_TYPE_INDEX:payload_end])
    name = FRAME_TYPES.get(type_id, f"UNKNOWN(0x{type_id:02X})")
    crc_s = "OK" if crc == calc else "BAD"
    return (
        f"CRSF type={name} addr={_addr_name(frame[0])} "
        f"len={length} pl={payload_len} crc={crc_s}"
    )


def is_filtered_crsf_type(type_id, filter_types):
    if not filter_types:
        return False
    name = FRAME_TYPES.get(type_id)
    return bool(name and name in filter_types)


def is_filtered_msp_cmd(cmd, cmd_map, filter_cmds):
    if cmd is None:
        return False
    cmd_name = cmd_map.get(cmd)
    return bool(filter_cmds and cmd_name in filter_cmds)


def msp_cmd_from_crsf_frame(frame):
    if not frame or len(frame) < CRSF_MSP_FRAME_OFFSET + 1:
        return None
    type_id = frame[CRSF_TELEMETRY_TYPE_INDEX]
    if type_id not in (CRSF_FRAMETYPE_MSP_REQ, CRSF_FRAMETYPE_MSP_RESP, CRSF_FRAMETYPE_MSP_WRITE):
        return None
    payload = frame[3:-1]  # [type payload ...] without crc
    if len(payload) < 3:
        return None
    msp_data = payload[2:]
    if not msp_data:
        return None
    status_byte = msp_data[0]
    is_start = bool(status_byte & 0x10)
    if not is_start:
        return None
    version = (status_byte & 0x60) >> 5
    if version == 1:
        if len(msp_data) >= 3:
            return msp_data[2]
    elif version == 2:
        if len(msp_data) >= 4:
            return msp_data[2] | (msp_data[3] << 8)
    return None


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
        repo_root = Path(__file__).resolve().parents[2]
        self.msp_cmd_map = load_msp_command_map(repo_root)
        self.log_file = None
        self.log_lock = threading.Lock()
        if LOG_FILE:
            try:
                mode = "a" if LOG_FILE_APPEND else "w"
                self.log_file = open(LOG_FILE, mode, buffering=1, encoding="utf-8")
            except Exception as e:
                print(f"{Colors.RED}[ERROR] Failed to open log file: {e}{Colors.RESET}", file=sys.stderr)

    def log(self, direction, message, data=None):
        ts = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime(ts)) + f".{int(ts * 1000) % 1000:03d}"

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
        self._write_log_file(ts, timestamp, "INFO", direction, message, data)

    def log_error(self, message):
        ts = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime(ts)) + f".{int(ts * 1000) % 1000:03d}"
        log_msg = f"{Colors.RED}[{timestamp}] [ERROR] {message}{Colors.RESET}"
        print(log_msg, file=sys.stderr)
        self._write_log_file(ts, timestamp, "ERROR", "ERROR", message, None)

    def _write_log_file(self, ts, ts_local, level, direction, message, data):
        if not self.log_file:
            return
        if data:
            data_hex = binascii.hexlify(data).decode("ascii").upper()
            data_len = len(data)
        else:
            data_hex = ""
            data_len = 0
        entry = {
            "ts": ts,
            "ts_local": ts_local,
            "level": level,
            "dir": direction,
            "msg": message,
            "data_len": data_len,
            "data_hex": data_hex,
        }
        with self.log_lock:
            try:
                self.log_file.write(
                    json.dumps(entry, separators=(",", ":"), ensure_ascii=True) + "\n"
                )
            except Exception:
                pass

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
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
        self.log("SYS", "Shutdown complete")

    def serial_reader(self):
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        frames = self.crsf_parser.feed(data)
                        if frames:
                            for frame in frames:
                                if is_filtered_crsf_type(frame[CRSF_TELEMETRY_TYPE_INDEX], FILTER_CRSF_TYPES):
                                    continue
                                cmd = msp_cmd_from_crsf_frame(frame)
                                if is_filtered_msp_cmd(cmd, self.msp_cmd_map, FILTER_MSP_CMDS):
                                    continue
                                summary = describe_crsf_frame(frame)
                                msg = "CRSF frame"
                                if summary:
                                    msg += f" | {summary}"
                                self.log("SER->TCP", msg, frame)
                        else:
                            pass
                            # self.log("SER->TCP", f"Received {len(data)} bytes", data)
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
                                        msp_summary, filtered = describe_msp_frame(
                                            msp_frame, self.msp_cmd_map, FILTER_MSP_CMDS
                                        )
                                        if filtered:
                                            continue
                                        msg = f"Sent MSP frame ({len(msp_frame)} bytes)"
                                        if msp_summary:
                                            msg += f" | {msp_summary}"
                                        self.log("SER->TCP", msg, msp_frame)
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

                msp_summary, filtered = describe_msp_frame(
                    data, self.msp_cmd_map, FILTER_MSP_CMDS
                )
                msg = f"Received {len(data)} bytes from TCP"
                if filtered:
                    self.process_msp_to_crsf(data)
                    continue
                if msp_summary:
                    msg += f" | {msp_summary}"
                self.log("TCP->SER", msg, data)
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
        msp_summary, filtered = describe_msp_frame(
            msp_data, self.msp_cmd_map, FILTER_MSP_CMDS
        )
        if filtered:
            for frame in frames:
                if self.serial_conn:
                    self.serial_conn.write(frame)
            return
        total = len(frames)
        for idx, frame in enumerate(frames, start=1):
            msg = "Built CRSF frame"
            if total > 1:
                msg += f" chunk {idx}/{total}"
            if msp_summary and idx == 1:
                msg += f" | {msp_summary}"
            self.log("GEN", msg, frame)
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
