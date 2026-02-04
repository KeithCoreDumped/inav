#!/usr/bin/env python3
import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple, cast

try:
    import serial
except ImportError:
    print(
        "Error: pyserial not installed. Install with 'pip install pyserial'",
        file=sys.stderr,
    )
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
    raw_len: int


class CrsfParser:
    def __init__(self):
        self.buffer = bytearray()

    def process(self, data: bytes) -> List[CrsfFrame]:
        self.buffer.extend(data)
        frames = []

        while len(self.buffer) >= 4:  # Min size: Addr + Len + Type + CRC
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
                    received_crc = self.buffer[
                        2 + length_byte - 1
                    ]  # Last byte of the packet frame

                    calculated_crc = calc_crsf_crc(type_id, bytes(payload))

                    if received_crc == calculated_crc:
                        frames.append(
                            CrsfFrame(
                                addr,
                                length_byte,
                                type_id,
                                bytes(payload),
                                received_crc,
                                True,
                                total_packet_len,
                            )
                        )
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


def _format_ts(ts: float) -> str:
    # HH:MM:SS.mmm local time
    lt = time.localtime(ts)
    ms = int((ts - int(ts)) * 1000)
    return time.strftime("%H:%M:%S", lt) + f".{ms:03d}"


def dump_hex(data: bytes, *, group: int = 1, cols: int = 0) -> str:
    """Return hex string, optionally grouped and multi-line.

    group: number of bytes per group (1 disables grouping)
    cols: bytes per line (0 disables wrapping)
    """
    if not data:
        return ""
    if group <= 0:
        group = 1
    if cols < 0:
        cols = 0

    def fmt_line(chunk: bytes) -> str:
        parts = []
        for i in range(0, len(chunk), group):
            g = chunk[i : i + group]
            parts.append(" ".join(f"{b:02X}" for b in g))
        return "  ".join(parts)

    if cols == 0:
        return fmt_line(data)
    lines = []
    for i in range(0, len(data), cols):
        lines.append(fmt_line(data[i : i + cols]))
    return "\n".join(lines)


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


msp_cmd_map: dict[int, str] = {}

CRSF_ADDR_NAME = {
    0xC8: "FC",
    0xEA: "TX",
    0xEC: "RX",
}


def _addr_name(addr: int) -> str:
    name = CRSF_ADDR_NAME.get(addr)
    if name:
        return f"{name}(0x{addr:02X})"
    return f"0x{addr:02X}"


@dataclass
class MspFragment:
    origin: int
    dest: int
    seq: int
    is_start: bool
    version: int
    is_error: bool
    status_byte: int
    msp_flags: Optional[int]
    cmd: Optional[int]
    cmd_name: Optional[str]
    declared_size: Optional[int]
    frag_payload_bytes: int
    raw_msp_data_len: int


@dataclass
class MspReassemblyState:
    version: int
    origin: int
    dest: int
    cmd: Optional[int]
    declared_size: Optional[int]
    bytes_received: int = 0
    last_seq: Optional[int] = None
    started_ts: Optional[float] = None


class MspReassemblyTracker:
    def __init__(self):
        self._states: Dict[Tuple[int, int, int], MspReassemblyState] = {}

    def update(
        self, frag: MspFragment, ts: float
    ) -> Tuple[MspReassemblyState, Optional[str]]:
        key = (frag.origin, frag.dest, frag.version)
        state = self._states.get(key)
        note = None

        if frag.is_start or state is None:
            state = MspReassemblyState(
                version=frag.version,
                origin=frag.origin,
                dest=frag.dest,
                cmd=frag.cmd,
                declared_size=frag.declared_size,
                bytes_received=0,
                last_seq=None,
                started_ts=ts,
            )
            self._states[key] = state
        else:
            # Keep cmd/size from last START; continuation fragments usually omit these.
            if state.cmd is None and frag.cmd is not None:
                state.cmd = frag.cmd
            if state.declared_size is None and frag.declared_size is not None:
                state.declared_size = frag.declared_size

        if state.last_seq is not None:
            expected = (state.last_seq + 1) & 0x0F
            if frag.seq != expected:
                note = f"SEQ_GAP exp={expected} got={frag.seq}"

        state.last_seq = frag.seq
        state.bytes_received += max(0, frag.frag_payload_bytes)
        return state, note


# Configuration
CONFIG: dict[str, Any] = {
    "PORT": "/dev/cu.usbmodem11302",
    "BAUD": 420000,
    "FILTER_RC": True,
    "FILTER_TYPES": [
        "ATTITUDE",
        "BATTERY",
        "FLIGHT_MODE",
        "VARIO",
        "BARO_ALT",
        "LINK_STATS",
    ],
    "FILTER_MSP_CMDS": {
        "MSP_ATTITUDE",
        "MSP_SENSOR_STATUS",
        "MSP2_INAV_STATUS",
        "MSP2_INAV_ANALOG",
        "MSP_ACTIVEBOXES",
    },
    "SHOW_HEX": True,
}

FILTER_TYPES: list[str] = CONFIG["FILTER_TYPES"]
FILTER_MSP_CMDS: set[str] = CONFIG["FILTER_MSP_CMDS"]


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
        if is_start:
            info += "START "
        if is_error:
            info += "ERR "
        info += f"Ver:{version} "

        if is_start and len(msp_data) > 1:
            cmd_name = None
            if version == 1 and len(msp_data) >= 3:
                msp_size = msp_data[1]
                msp_cmd = msp_data[2]
                cmd_name = msp_cmd_map.get(msp_cmd, f"{msp_cmd}")
                if cmd_name in FILTER_MSP_CMDS:
                    return None
                info += f"MSPv1 Cmd:{cmd_name}({msp_cmd}) Sz:{msp_size}"

            elif version == 2 and len(msp_data) >= 6:
                _msp_flags = msp_data[1]
                msp_cmd = msp_data[2] | (msp_data[3] << 8)
                msp_size = msp_data[4] | (msp_data[5] << 8)
                cmd_name = msp_cmd_map.get(msp_cmd, f"0x{msp_cmd:04X}")
                if cmd_name in FILTER_MSP_CMDS:
                    return None
                info += f"MSPv2 Cmd:{cmd_name}(0x{msp_cmd:04X}) Sz:{msp_size}"

    return info


def parse_msp_fragment(payload: bytes) -> Optional[MspFragment]:
    if len(payload) < 3:
        return None

    origin = payload[0]
    dest = payload[1]
    msp_data = payload[2:]
    if not msp_data:
        return None

    status_byte = msp_data[0]
    seq = status_byte & 0x0F
    is_start = bool(status_byte & 0x10)
    version = (status_byte & 0x60) >> 5
    is_error = bool(status_byte & 0x80)

    cmd = None
    cmd_name = None
    declared_size = None
    msp_flags = None
    frag_payload_bytes = 0

    if is_start:
        if version == 1 and len(msp_data) >= 3:
            declared_size = msp_data[1]
            cmd = msp_data[2]
            cmd_name = msp_cmd_map.get(cmd, f"{cmd}")
            frag_payload_bytes = max(0, len(msp_data) - 3)
        elif version == 2 and len(msp_data) >= 6:
            msp_flags = msp_data[1]
            cmd = msp_data[2] | (msp_data[3] << 8)
            declared_size = msp_data[4] | (msp_data[5] << 8)
            cmd_name = msp_cmd_map.get(cmd, f"0x{cmd:04X}")
            frag_payload_bytes = max(0, len(msp_data) - 6)
        else:
            frag_payload_bytes = 0
    else:
        frag_payload_bytes = max(0, len(msp_data) - 1)

    return MspFragment(
        origin=origin,
        dest=dest,
        seq=seq,
        is_start=is_start,
        version=version,
        is_error=is_error,
        status_byte=status_byte,
        msp_flags=msp_flags,
        cmd=cmd,
        cmd_name=cmd_name,
        declared_size=declared_size,
        frag_payload_bytes=frag_payload_bytes,
        raw_msp_data_len=len(msp_data),
    )


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Monitor CRSF frames and MSP-over-CRSF fragments"
    )
    p.add_argument(
        "--port", default=CONFIG["PORT"], help="Serial port (default: from CONFIG)"
    )
    p.add_argument(
        "--baud",
        type=int,
        default=CONFIG["BAUD"],
        help="Baud rate (default: from CONFIG)",
    )
    p.add_argument(
        "--hex",
        dest="show_hex",
        action="store_true",
        default=CONFIG["SHOW_HEX"],
        help="Show payload hex dump",
    )
    p.add_argument(
        "--no-hex",
        dest="show_hex",
        action="store_false",
        help="Disable payload hex dump",
    )
    p.add_argument(
        "--hex-group", type=int, default=1, help="Hex dump grouping (bytes per group)"
    )
    p.add_argument(
        "--hex-cols",
        type=int,
        default=0,
        help="Hex dump wrap width in bytes (0 disables wrapping)",
    )
    p.add_argument(
        "-v", "--verbose", action="store_true", help="Richer per-frame metadata"
    )
    p.add_argument(
        "--summary", action="store_true", help="One line per frame; implies --no-hex"
    )
    return p.parse_args(argv)


def main():
    args = parse_args()

    # Load MSP command map
    repo_root = Path(__file__).resolve().parents[2]
    global msp_cmd_map
    msp_cmd_map = load_msp_command_map(repo_root)
    print(f"Loaded {len(msp_cmd_map)} MSP commands from {repo_root}")

    # Create set of filtered types from CONFIG
    filtered_types = set()

    if CONFIG["FILTER_RC"]:
        filtered_types.add(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)

    # Map name back to ID for filtering
    name_to_id = {v: k for k, v in FRAME_TYPES.items()}
    for name in FILTER_TYPES:
        name_upper = name.upper()
        if name_upper in name_to_id:
            filtered_types.add(name_to_id[name_upper])
        else:
            print(
                f"Warning: Unknown frame type '{name}' in filter config",
                file=sys.stderr,
            )

    port = args.port
    baud = args.baud

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
    msp_tracker = MspReassemblyTracker()
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
                    if f.type_id == CRSF_FRAMETYPE_LINK_STATISTICS:
                        color = CYAN
                    if f.type_id in (
                        CRSF_FRAMETYPE_MSP_REQ,
                        CRSF_FRAMETYPE_MSP_RESP,
                        CRSF_FRAMETYPE_MSP_WRITE,
                    ):
                        color = MAGENTA

                    ts = time.time()
                    ts_s = _format_ts(ts)
                    crc_s = "OK" if f.valid else "BAD"

                    dir_s = f"?->dst:{_addr_name(f.address)}"
                    msp_frag = None
                    if f.type_id in (
                        CRSF_FRAMETYPE_MSP_REQ,
                        CRSF_FRAMETYPE_MSP_RESP,
                        CRSF_FRAMETYPE_MSP_WRITE,
                    ):
                        msp_frag = parse_msp_fragment(f.payload)
                        if msp_frag:
                            dir_s = f"{_addr_name(msp_frag.origin)}->{_addr_name(msp_frag.dest)}"

                    line = (
                        f"{color}[{ts_s}] "
                        f"dir={dir_s} raw={f.raw_len} crsf_len={f.length} "
                        f"type={name}(0x{f.type_id:02X}) addr={_addr_name(f.address)} "
                        f"crc={crc_s}"
                    )

                    if args.verbose and not args.summary:
                        line += f" rx_crc=0x{f.crc:02X} pl={len(f.payload)}"
                    else:
                        line += f" pl={len(f.payload)}"

                    if msp_frag:
                        # Keep existing filter behavior: filtered MSP cmds omit MSP details.
                        filtered_cmd = False
                        if msp_frag.is_start and msp_frag.cmd_name is not None:
                            filtered_cmd = msp_frag.cmd_name in FILTER_MSP_CMDS

                        if not filtered_cmd:
                            state, note = msp_tracker.update(msp_frag, ts)
                            cmd_s = "-"
                            if msp_frag.cmd_name:
                                cmd_s = msp_frag.cmd_name
                            elif state.cmd is not None:
                                cmd_s = msp_cmd_map.get(
                                    state.cmd,
                                    f"0x{state.cmd:04X}"
                                    if state.cmd > 0xFF
                                    else f"{state.cmd}",
                                )
                            decl = (
                                msp_frag.declared_size
                                if msp_frag.declared_size is not None
                                else state.declared_size
                            )
                            start_s = "START" if msp_frag.is_start else "CONT"
                            err_s = " ERR" if msp_frag.is_error else ""
                            flags_s = (
                                f" flags=0x{msp_frag.msp_flags:02X}"
                                if msp_frag.msp_flags is not None
                                else ""
                            )
                            done_s = ""
                            if decl is not None and state.bytes_received >= decl:
                                done_s = " DONE"

                            line += (
                                f" msp={start_s} v={msp_frag.version} seq={msp_frag.seq}{err_s} "
                                f"status=0x{msp_frag.status_byte:02X}{flags_s} "
                                f"org=0x{msp_frag.origin:02X} dst=0x{msp_frag.dest:02X} "
                                f"cmd={cmd_s} decl={decl if decl is not None else '-'} "
                                f"frag={msp_frag.frag_payload_bytes} cum={state.bytes_received}"
                            )
                            if args.verbose and not args.summary:
                                line += f" msp_raw={msp_frag.raw_msp_data_len}"
                            line += done_s
                            if note and not args.summary:
                                line += f" gap={note}"

                    print(line + RESET)

                    show_hex = args.show_hex
                    if args.summary:
                        show_hex = False
                    if show_hex or f.type_id not in FRAME_TYPES:
                        payload_hex = dump_hex(
                            f.payload, group=args.hex_group, cols=args.hex_cols
                        )
                        if "\n" in payload_hex:
                            print(f"  HEX_PAYLOAD({len(f.payload)}):")
                            print("  " + payload_hex.replace("\n", "\n  "))
                        else:
                            print(f"  HEX_PAYLOAD({len(f.payload)}): {payload_hex}")
            # Sleep briefly to avoid 100% CPU
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
