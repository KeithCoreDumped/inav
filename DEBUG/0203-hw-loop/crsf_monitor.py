#!/usr/bin/env python3
import argparse
import sys
import time
import struct
from dataclasses import dataclass
from typing import List, Optional

try:
    import serial
except ImportError:
    print("Error: pyserial not installed. Install with 'pip install pyserial'", file=sys.stderr)
    sys.exit(1)

# CRSF Constants
CRSF_MAX_PACKET_SIZE = 64
CRSF_SYNC_BYTE = 0xC8  # Often 0xC8 (Flight Controller) or 0xEA, 0xEC etc.
# We will accept any valid address byte if the CRC matches.

# Frame Types
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
CRSF_FRAMETYPE_MSP_REQ = 0x7A
CRSF_FRAMETYPE_MSP_RESP = 0x7B
CRSF_FRAMETYPE_MSP_WRITE = 0x7C
CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D

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

# Colors
RESET = "\033[0m"
GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
RED = "\033[31m"
MAGENTA = "\033[35m"

def crc8_dvb_s2(crc: int, byte: int) -> int:
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0xD5) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc

def calc_crsf_crc(type_byte: int, payload: bytes) -> int:
    crc = crc8_dvb_s2(0, type_byte)
    for b in payload:
        crc = crc8_dvb_s2(crc, b)
    return crc

@dataclass
class CrsfFrame:
    address: int
    length: int
    type_id: int
    payload: bytes
    crc: int
    valid: bool

class CrsfParser:
    def __init__(self):
        self.buffer = bytearray()
    
    def process(self, data: bytes) -> List[CrsfFrame]:
        self.buffer.extend(data)
        frames = []
        
        while len(self.buffer) >= 4: # Min size: Addr + Len + Type + CRC
            # CRSF Frame: [Address] [Length] [Type] [Payload...] [CRC]
            # Length = sizeof(Type) + sizeof(Payload) + sizeof(CRC)
            # So total frame size = 1 (Addr) + 1 (Len) + Length
            
            # Simple heuristic: scan for valid length and CRC
            # We don't enforce specific addresses because we might see RX or TX packets
            
            packet_found = False
            # Check if current buffer start looks like a packet
            length_byte = self.buffer[1]
            
            # Sanity check length
            # Min length: Type(1) + CRC(1) = 2. 
            # Max length: CRSF_MAX_PACKET_SIZE - 2 (Addr, Len) = 62
            if 2 <= length_byte <= 62:
                total_packet_len = 2 + length_byte
                if len(self.buffer) >= total_packet_len:
                    addr = self.buffer[0]
                    type_id = self.buffer[2]
                    payload_len = length_byte - 2
                    payload = self.buffer[3 : 3 + payload_len]
                    received_crc = self.buffer[2 + length_byte - 1] # Last byte of the packet frame
                    
                    calculated_crc = calc_crsf_crc(type_id, payload)
                    
                    if received_crc == calculated_crc:
                        frames.append(CrsfFrame(addr, length_byte, type_id, bytes(payload), received_crc, True))
                        del self.buffer[:total_packet_len]
                        packet_found = True
                    else:
                        # CRC mismatch, maybe not a packet start. 
                        # But also could be a corrupted packet.
                        # For now, let's assume if it fails CRC it's garbage and skip 1 byte
                        pass
            
            if not packet_found:
                del self.buffer[0]
                
        return frames

def dump_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

import re
from pathlib import Path

# ... (rest of configuration) ...

def load_msp_command_map(repo_root: Path) -> dict[int, str]:
    files = [
        repo_root / "src/main/msp/msp_protocol.h",
        repo_root / "src/main/msp/msp_protocol_v2_common.h",
        repo_root / "src/main/msp/msp_protocol_v2_inav.h",
        repo_root / "src/main/msp/msp_protocol_v2_sensor.h",
        repo_root / "src/main/msp/msp_protocol_v2_sensor_msg.h",
    ]
    cmd_map: dict[int, str] = {}
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

MSP_CMD_MAP = {}

# Configuration
CONFIG = {
    "PORT": "/dev/cu.usbmodem11302",
    "BAUD": 420000,
    "FILTER_RC": True,
    "FILTER_TYPES": [
        "ATTITUDE", 
        "BATTERY", 
        "FLIGHT_MODE", 
        "VARIO", 
        "BARO_ALT",
        "LINK_STATS"
    ],
    "FILTER_MSP_CMDS": {
        "MSP_ATTITUDE",
        "MSP_SENSOR_STATUS",
        "MSP2_INAV_STATUS",
        "MSP2_INAV_ANALOG",
        "MSP_ACTIVEBOXES",
    },
    "SHOW_HEX": True
}

def parse_msp_payload(type_id: int, payload: bytes):
    if len(payload) < 2:
        return None
        
    origin = payload[0]
    dest = payload[1]
    msp_data = payload[2:]
    
    info = f"Org:0x{origin:02X} Dst:0x{dest:02X} "
    
    if len(msp_data) > 0:
        status_byte = msp_data[0]
        seq = status_byte & 0x0F
        is_start = bool(status_byte & 0x10)
        version = (status_byte & 0x60) >> 5
        is_error = bool(status_byte & 0x80)
        
        info += f"Seq:{seq} "
        if is_start: info += "START "
        if is_error: info += "ERR "
        info += f"Ver:{version} "
        
        if is_start and len(msp_data) > 1:
            cmd_name = None
            if version == 1 and len(msp_data) >= 3:
                msp_size = msp_data[1]
                msp_cmd = msp_data[2]
                cmd_name = MSP_CMD_MAP.get(msp_cmd, f"{msp_cmd}")
                if cmd_name in CONFIG["FILTER_MSP_CMDS"]:
                    return None
                info += f"MSPv1 Cmd:{cmd_name}({msp_cmd}) Sz:{msp_size}"
                
            elif version == 2 and len(msp_data) >= 6:
                msp_flags = msp_data[1]
                msp_cmd = msp_data[2] | (msp_data[3] << 8)
                msp_size = msp_data[4] | (msp_data[5] << 8)
                cmd_name = MSP_CMD_MAP.get(msp_cmd, f"0x{msp_cmd:04X}")
                if cmd_name in CONFIG["FILTER_MSP_CMDS"]:
                    return None
                info += f"MSPv2 Cmd:{cmd_name}(0x{msp_cmd:04X}) Sz:{msp_size}"
                
    return info

def main():
    # Load MSP command map
    repo_root = Path(__file__).resolve().parents[2]
    global MSP_CMD_MAP
    MSP_CMD_MAP = load_msp_command_map(repo_root)
    print(f"Loaded {len(MSP_CMD_MAP)} MSP commands from {repo_root}")

    # Create set of filtered types from CONFIG
    filtered_types = set()
    
    if CONFIG["FILTER_RC"]:
        filtered_types.add(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        
    # Map name back to ID for filtering
    name_to_id = {v: k for k, v in FRAME_TYPES.items()}
    for name in CONFIG["FILTER_TYPES"]:
        name_upper = name.upper()
        if name_upper in name_to_id:
            filtered_types.add(name_to_id[name_upper])
        else:
            print(f"Warning: Unknown frame type '{name}' in filter config", file=sys.stderr)

    port = CONFIG["PORT"]
    baud = CONFIG["BAUD"]

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return

    print(f"Listening on {port} at {baud} baud...")
    if filtered_types:
        filter_names = [FRAME_TYPES.get(tid, f"0x{tid:02X}") for tid in filtered_types]
        print(f"Filtering out: {', '.join(filter_names)}")
    
    crsf_parser = CrsfParser()
    
    try:
        while True:
            data = ser.read(1024)
            if data:
                frames = crsf_parser.process(data)
                for f in frames:
                    name = FRAME_TYPES.get(f.type_id, f"UNKNOWN(0x{f.type_id:02X})")
                    
                    if f.type_id in filtered_types:
                        continue
                        
                    color = GREEN
                    if f.type_id == CRSF_FRAMETYPE_LINK_STATISTICS: color = CYAN
                    if f.type_id in (CRSF_FRAMETYPE_MSP_REQ, CRSF_FRAMETYPE_MSP_RESP, CRSF_FRAMETYPE_MSP_WRITE): color = MAGENTA
                    
                    print(f"{color}[{time.strftime('%H:%M:%S')}] ADDR:0x{f.address:02X} {name} (Len:{f.length}) CRC:OK{RESET}")
                    
                    if f.type_id in (CRSF_FRAMETYPE_MSP_REQ, CRSF_FRAMETYPE_MSP_RESP, CRSF_FRAMETYPE_MSP_WRITE):
                        msp_info = parse_msp_payload(f.type_id, f.payload)
                        if msp_info:
                            print(f"  MSP Info: {msp_info}")

                    if CONFIG["SHOW_HEX"] or f.type_id not in FRAME_TYPES:
                        print(f"  HEX: {dump_hex(f.payload)}")
                        
            # Sleep briefly to avoid 100% CPU
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
