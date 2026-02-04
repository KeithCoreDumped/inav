#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

RESET = "\033[0m"
GREEN = "\033[32m"
CYAN = "\033[36m"
RED = "\033[31m"

# Filter: ignore these MSP commands in output (still forwarded, just not logged)
FILTERED_COMMANDS = {
    "MSP_ATTITUDE",
    "MSP_SENSOR_STATUS",
    "MSP2_INAV_STATUS",
    "MSP2_INAV_ANALOG",
    "MSP_ACTIVEBOXES",
}


@dataclass
class MspFrame:
    direction: str
    flags: int
    cmd: int
    size: int
    payload: bytes
    crc: int
    crc_ok: bool


class MspV2Parser:
    def __init__(self, direction: str):
        self.direction: str = direction
        self.buffer: bytearray = bytearray()

    def feed(self, data: bytes) -> tuple[list[MspFrame], list[str]]:
        self.buffer.extend(data)
        frames: list[MspFrame] = []
        errors: list[str] = []

        while True:
            start = self._find_start()
            if start is None:
                break
            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 3:
                break

            if self.buffer[0] != ord("$") or self.buffer[1] != ord("X"):
                del self.buffer[0]
                continue

            direction_byte = self.buffer[2]
            if direction_byte not in (ord("<"), ord(">"), ord("!")):
                del self.buffer[0]
                continue

            if len(self.buffer) < 3 + 5:
                break

            flags = self.buffer[3]
            cmd = self.buffer[4] | (self.buffer[5] << 8)
            size = self.buffer[6] | (self.buffer[7] << 8)
            total_len = 3 + 5 + size + 1
            if len(self.buffer) < total_len:
                break

            payload = bytes(self.buffer[8 : 8 + size])
            crc = self.buffer[8 + size]
            crc_calc = crc8_dvb_s2_update(0, bytes(self.buffer[3 : 8 + size]))
            crc_ok = crc_calc == crc

            frames.append(
                MspFrame(
                    direction=chr(direction_byte),
                    flags=flags,
                    cmd=cmd,
                    size=size,
                    payload=payload,
                    crc=crc,
                    crc_ok=crc_ok,
                )
            )

            del self.buffer[:total_len]

        return frames, errors

    def _find_start(self) -> int | None:
        try:
            return self.buffer.index(ord("$"))
        except ValueError:
            self.buffer.clear()
            return None


def crc8_dvb_s2_update(crc: int, data: bytes) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class Cursor:
    def __init__(self, data: bytes):
        self.data: bytes = data
        self.pos: int = 0

    def remaining(self) -> int:
        return len(self.data) - self.pos

    def read_u8(self) -> int | None:
        if self.remaining() < 1:
            return None
        val = self.data[self.pos]
        self.pos += 1
        return val

    def read_u16(self) -> int | None:
        if self.remaining() < 2:
            return None
        val = self.data[self.pos] | (self.data[self.pos + 1] << 8)
        self.pos += 2
        return val

    def read_i16(self) -> int | None:
        val = self.read_u16()
        if val is None:
            return None
        return val - 0x10000 if val & 0x8000 else val

    def read_u32(self) -> int | None:
        if self.remaining() < 4:
            return None
        b0 = self.data[self.pos]
        b1 = self.data[self.pos + 1]
        b2 = self.data[self.pos + 2]
        b3 = self.data[self.pos + 3]
        self.pos += 4
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)

    def read_i32(self) -> int | None:
        val = self.read_u32()
        if val is None:
            return None
        return val - 0x100000000 if val & 0x80000000 else val


def decode_payload(cmd_name: str, payload: bytes) -> str | None:
    c = Cursor(payload)
    fields: list[str] = []

    if cmd_name == "MSP_API_VERSION":
        proto = c.read_u8()
        major = c.read_u8()
        minor = c.read_u8()
        if None in (proto, major, minor):
            return None
        fields.append(f"protocol_version={proto}")
        fields.append(f"api_version={major}.{minor}")

    elif cmd_name == "MSP_FC_VARIANT":
        fields.append(f"fc_variant={payload[:4].decode(errors='replace')}")

    elif cmd_name == "MSP_FC_VERSION":
        major = c.read_u8()
        minor = c.read_u8()
        patch = c.read_u8()
        if None in (major, minor, patch):
            return None
        fields.append(f"fc_version={major}.{minor}.{patch}")

    elif cmd_name == "MSP_BOARD_INFO":
        board_id = payload[:4].decode(errors="replace")
        c.pos = 4
        hw_rev = c.read_u16()
        osd = c.read_u8()
        comm = c.read_u8()
        name_len = c.read_u8()
        if None in (hw_rev, osd, comm, name_len):
            return None
        assert name_len is not None
        target = payload[c.pos : c.pos + name_len].decode(errors="replace")
        fields.append(f"board_id={board_id}")
        fields.append(f"hw_revision={hw_rev}")
        fields.append(f"osd_support={osd}")
        fields.append(f"comm_caps=0x{comm:02X}")
        fields.append(f"target_name={target}")

    elif cmd_name == "MSP_BUILD_INFO":
        build_date = payload[:11].decode(errors="replace").strip("\x00")
        build_time = payload[11:19].decode(errors="replace").strip("\x00")
        git_rev = payload[19:26].decode(errors="replace").strip("\x00")
        fields.append(f"build_date={build_date}")
        fields.append(f"build_time={build_time}")
        fields.append(f"git_rev={git_rev}")

    elif cmd_name in ("MSP_STATUS", "MSP_STATUS_EX"):
        cycle_time = c.read_u16()
        i2c_errors = c.read_u16()
        sensor_status = c.read_u16()
        box_flags = c.read_u32()
        profile = c.read_u8()
        if None in (cycle_time, i2c_errors, sensor_status, box_flags, profile):
            return None
        fields.append(f"cycle_time_us={cycle_time}")
        fields.append(f"i2c_errors={i2c_errors}")
        fields.append(f"sensor_status=0x{sensor_status:04X}")
        fields.append(f"box_flags=0x{box_flags:08X}")
        fields.append(f"profile={profile}")
        if cmd_name == "MSP_STATUS_EX":
            cpu_load = c.read_u16()
            arming_flags = c.read_u16()
            acc_cal = c.read_u8()
            if None in (cpu_load, arming_flags, acc_cal):
                return None
            fields.append(f"cpu_load={cpu_load}")
            fields.append(f"arming_flags=0x{arming_flags:04X}")
            fields.append(f"acc_calib_flags=0x{acc_cal:02X}")

    elif cmd_name == "MSP_RAW_IMU":
        acc = [c.read_i16(), c.read_i16(), c.read_i16()]
        gyro = [c.read_i16(), c.read_i16(), c.read_i16()]
        mag = [c.read_i16(), c.read_i16(), c.read_i16()]
        if any(v is None for v in acc + gyro + mag):
            return None
        fields.append(f"acc={acc}")
        fields.append(f"gyro_dps={gyro}")
        fields.append(f"mag={mag}")

    elif cmd_name == "MSP_ATTITUDE":
        roll = c.read_i16()
        pitch = c.read_i16()
        yaw = c.read_i16()
        if None in (roll, pitch, yaw):
            return None
        fields.append(f"roll_decideg={roll}")
        fields.append(f"pitch_decideg={pitch}")
        fields.append(f"yaw_deg={yaw}")

    elif cmd_name == "MSP_ALTITUDE":
        est_alt = c.read_i32()
        est_vel = c.read_i16()
        baro_alt = c.read_i32()
        if None in (est_alt, est_vel, baro_alt):
            return None
        fields.append(f"est_alt_cm={est_alt}")
        fields.append(f"est_vel_cms={est_vel}")
        fields.append(f"baro_alt_cm={baro_alt}")

    elif cmd_name == "MSP_ANALOG":
        vbat = c.read_u8()
        mah = c.read_u16()
        rssi = c.read_u16()
        amps = c.read_i16()
        if None in (vbat, mah, rssi, amps):
            return None
        fields.append(f"vbat_0_1v={vbat}")
        fields.append(f"mah_drawn={mah}")
        fields.append(f"rssi={rssi}")
        fields.append(f"amperage_cA={amps}")

    elif cmd_name == "MSP_UID":
        uid0 = c.read_u32()
        uid1 = c.read_u32()
        uid2 = c.read_u32()
        if None in (uid0, uid1, uid2):
            return None
        fields.append(f"uid0=0x{uid0:08X}")
        fields.append(f"uid1=0x{uid1:08X}")
        fields.append(f"uid2=0x{uid2:08X}")

    elif cmd_name == "MSP_BATTERY_STATE":
        cell_count = c.read_u8()
        capacity = c.read_u16()
        vbat = c.read_u8()
        mah = c.read_u16()
        amps = c.read_i16()
        state = c.read_u8()
        vbat_01 = c.read_u16()
        if None in (cell_count, capacity, vbat, mah, amps, state, vbat_01):
            return None
        fields.append(f"cell_count={cell_count}")
        fields.append(f"capacity_mah={capacity}")
        fields.append(f"vbat_0_1v={vbat}")
        fields.append(f"mah_drawn={mah}")
        fields.append(f"amperage_cA={amps}")
        fields.append(f"battery_state={state}")
        fields.append(f"vbat_0_01v={vbat_01}")

    elif cmd_name == "MSP_NAME":
        fields.append(f"craft_name={payload.decode(errors='replace')}")

    elif cmd_name == "MSP2_INAV_ANALOG":
        flags = c.read_u8()
        vbat = c.read_u16()
        amps = c.read_u16()
        power = c.read_u32()
        mah = c.read_u32()
        mwh = c.read_u32()
        remaining = c.read_u32()
        pct = c.read_u8()
        rssi = c.read_u16()
        if None in (flags, vbat, amps, power, mah, mwh, remaining, pct, rssi):
            return None
        fields.append(f"battery_flags=0x{flags:02X}")
        fields.append(f"vbat_0_01v={vbat}")
        fields.append(f"amperage_cA={amps}")
        fields.append(f"power_mw={power}")
        fields.append(f"mah_drawn={mah}")
        fields.append(f"mwh_drawn={mwh}")
        fields.append(f"remaining_capacity={remaining}")
        fields.append(f"battery_pct={pct}")
        fields.append(f"rssi={rssi}")

    elif cmd_name == "MSP2_INAV_STATUS":
        cycle_time = c.read_u16()
        i2c_errors = c.read_u16()
        sensor_status = c.read_u16()
        cpu_load = c.read_u16()
        profile = c.read_u8()
        arming_flags = c.read_u32()
        if None in (
            cycle_time,
            i2c_errors,
            sensor_status,
            cpu_load,
            profile,
            arming_flags,
        ):
            return None
        fields.append(f"cycle_time_us={cycle_time}")
        fields.append(f"i2c_errors={i2c_errors}")
        fields.append(f"sensor_status=0x{sensor_status:04X}")
        fields.append(f"cpu_load={cpu_load}")
        fields.append(f"profile={profile}")
        fields.append(f"arming_flags=0x{arming_flags:08X}")

    else:
        return None

    return ", ".join(fields)


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
    return cmd_map


def full_hex(data: bytes) -> str:
    """Full hex dump without truncation (for log file)."""
    return " ".join(f"{b:02X}" for b in data)


def short_hex(data: bytes, limit: int = 64) -> str:
    if len(data) <= limit:
        return " ".join(f"{b:02X}" for b in data)
    head = data[:limit]
    return f"{' '.join(f'{b:02X}' for b in head)} …(+{len(data) - limit})"


def flags_to_str(flags: int) -> str:
    parts: list[str] = []
    if flags & 0x01:
        parts.append("DONT_REPLY")
    if flags & 0x02:
        parts.append("ILMI")
    return "|".join(parts) if parts else "0"


def log_frame(
    frame: MspFrame,
    cmd_map: dict[int, str],
    color: str,
    label: str,
    error: str | None = None,
    brief: bool = False,
    log_file: object | None = None,
    no_filter: bool = False,
) -> None:
    cmd_name = cmd_map.get(frame.cmd, "UNKNOWN")

    # Filter out noisy commands (unless --no-filter is set)
    if not no_filter and cmd_name in FILTERED_COMMANDS:
        return

    direction_char = frame.direction
    crc_status = "ok" if frame.crc_ok else "bad"
    header = (
        f"[{label}] MSPv2 {direction_char} {cmd_name}({frame.cmd}) "
        f"flags=0x{frame.flags:02X}({flags_to_str(frame.flags)}) "
        f"size={frame.size} crc={crc_status}"
    )
    if error:
        print(f"{RED}{header} err={error}{RESET}")
        if log_file:
            log_file.write(f"{header} err={error}\n")
            log_file.flush()
        return

    print(f"{color}{header}{RESET}")

    decoded = decode_payload(cmd_name, frame.payload)
    detail_line = ""
    if decoded:
        detail_line = f"  {decoded}"
    elif frame.payload:
        detail_line = f"  payload_hex={short_hex(frame.payload)}"

    if log_file:
        log_file.write(f"{header}\n")
        if decoded:
            log_file.write(f"  {decoded}\n")
        elif frame.payload:
            # Write full payload to log file (no truncation)
            log_file.write(f"  payload_hex={full_hex(frame.payload)}\n")
        log_file.flush()

    if brief:
        return

    if detail_line:
        print(f"{color}{detail_line}{RESET}")


def read_loop(
    read_fn: Callable[[], bytes],
    write_fn: Callable[[bytes], None],
    parser: MspV2Parser,
    cmd_map: dict[int, str],
    color: str,
    label: str,
    stop_event: threading.Event,
    brief: bool = False,
    log_file: object | None = None,
    no_filter: bool = False,
):
    while not stop_event.is_set():
        try:
            data = read_fn()
        except Exception as exc:
            print(f"{RED}[{label}] read error: {exc}{RESET}")
            break

        if not data:
            time.sleep(0.002)
            continue

        try:
            write_fn(data)
        except Exception as exc:
            print(f"{RED}[{label}] write error: {exc}{RESET}")
            break

        frames, _ = parser.feed(data)
        for frame in frames:
            if not frame.crc_ok:
                log_frame(
                    frame,
                    cmd_map,
                    RED,
                    label,
                    error=f"crc_mismatch got=0x{frame.crc:02X}",
                    brief=brief,
                    log_file=log_file,
                    no_filter=no_filter,
                )
            else:
                log_frame(frame, cmd_map, color, label, brief=brief, log_file=log_file, no_filter=no_filter)


def _tcp_recv_nonblocking(sock: socket.socket) -> bytes:
    try:
        data = sock.recv(4096)
    except BlockingIOError:
        return b""
    if data == b"":
        raise ConnectionError("peer closed")
    return data


def _tcp_send_all_nonblocking(sock: socket.socket, data: bytes) -> None:
    view = memoryview(data)
    sent = 0
    while sent < len(view):
        try:
            n = sock.send(view[sent:])
        except BlockingIOError:
            time.sleep(0.001)
            continue
        if n == 0:
            raise ConnectionError("peer closed")
        sent += n


class _SerialLike(Protocol):
    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


@dataclass
class _CliArgs:
    listen_host: str = "127.0.0.1"
    listen_port: int = 5761
    tcp_host: str = "10.0.0.1"
    tcp_port: int = 5761
    serial: str | None = None
    baud: int = 115200
    brief: bool = False
    log_file: str | None = None
    no_filter: bool = False
    repo_root: str = str(Path(__file__).resolve().parents[2])


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "MSPv2 proxy/bridge with decode logs. Default: TCP proxy "
            "(localhost:5761 <-> 10.0.0.1:5761)."
        )
    )

    actions: list[argparse.Action] = []

    actions.append(
        parser.add_argument(
            "--listen-host",
            default="127.0.0.1",
            type=str,
            help="Local listen host for Configurator (default: localhost)",
        )
    )
    actions.append(
        parser.add_argument(
            "--listen-port",
            type=int,
            default=5761,
            help="Local listen port for Configurator",
        )
    )
    actions.append(
        parser.add_argument(
            "--tcp-host",
            default="10.0.0.1",
            type=str,
            help="Remote ELRS TCP host",
        )
    )
    actions.append(
        parser.add_argument(
            "--tcp-port",
            type=int,
            default=5761,
            help="Remote ELRS TCP port",
        )
    )
    actions.append(
        parser.add_argument(
            "--serial",
            type=str,
            help="Enable serial<->TCP bridge mode (requires pyserial); path to serial device (e.g. /dev/ttyUSB0)",
        )
    )
    actions.append(
        parser.add_argument(
            "--baud",
            type=int,
            default=115200,
            help="Serial baudrate (only used with --serial)",
        )
    )
    actions.append(
        parser.add_argument(
            "--repo-root",
            default=str(Path(__file__).resolve().parents[2]),
            type=str,
            help="INAV repo root for MSP command map",
        )
    )
    actions.append(
        parser.add_argument(
            "--brief",
            action="store_true",
            help="Brief output mode (only show one line per frame, no payload details)",
        )
    )
    actions.append(
        parser.add_argument(
            "--log-file",
            type=str,
            help="Log full output to file (always includes payload details)",
        )
    )
    actions.append(
        parser.add_argument(
            "--no-filter",
            action="store_true",
            help="Disable command filtering (show all commands including filtered ones)",
        )
    )

    _ = len(actions)

    args = parser.parse_args(namespace=_CliArgs())

    listen_host = args.listen_host
    listen_port = args.listen_port
    tcp_host = args.tcp_host
    tcp_port = args.tcp_port
    serial_path = args.serial
    baud = args.baud
    brief = args.brief
    log_file_path = args.log_file
    no_filter = args.no_filter

    log_file: object | None = None
    if log_file_path:
        log_file = open(log_file_path, "w", encoding="utf-8")
        print(f"{CYAN}[log] writing full output to {log_file_path}{RESET}")

    repo_root = Path(args.repo_root)
    cmd_map = load_msp_command_map(repo_root)

    stop_event = threading.Event()

    ser: _SerialLike | None = None
    listen_sock: socket.socket | None = None
    local_sock: socket.socket | None = None
    remote_sock: socket.socket | None = None

    label_fwd: str
    label_rev: str
    fwd_color: str = GREEN
    rev_color: str = CYAN
    fwd_parser: MspV2Parser
    rev_parser: MspV2Parser
    fwd_read: Callable[[], bytes]
    fwd_write: Callable[[bytes], None]
    rev_read: Callable[[], bytes]
    rev_write: Callable[[bytes], None]

    try:
        if serial_path:
            # Serial mode: Configurator <-> localhost:5761 <-> USB serial <-> FC
            try:
                from serial import Serial
            except ImportError:
                print(
                    "Missing dependency: pyserial (required for --serial). Install with: pip install pyserial",
                    file=sys.stderr,
                )
                return 2

            ser = Serial(serial_path, baud, timeout=0)
            print(f"{CYAN}[serial] opened {serial_path} @ {baud} baud{RESET}")

            listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listen_sock.bind((listen_host, listen_port))
            listen_sock.listen(1)
            print(f"{CYAN}[tcp] listening on {listen_host}:{listen_port}{RESET}")

            local_sock, addr = listen_sock.accept()
            print(f"{CYAN}[tcp] client connected from {addr[0]}:{addr[1]}{RESET}")
            local_sock.setblocking(False)

            fwd_parser = MspV2Parser(direction="CFG->FC")
            rev_parser = MspV2Parser(direction="FC->CFG")

            label_fwd = "CFG->FC"
            label_rev = "FC->CFG"

            def cfg_read_serial() -> bytes:
                assert local_sock is not None
                return _tcp_recv_nonblocking(local_sock)

            def cfg_write_serial(data: bytes) -> None:
                assert local_sock is not None
                _tcp_send_all_nonblocking(local_sock, data)

            def serial_read() -> bytes:
                assert ser is not None
                return ser.read(4096)

            def serial_write(data: bytes) -> None:
                assert ser is not None
                ser.write(data)

            fwd_read = cfg_read_serial
            fwd_write = serial_write
            rev_read = serial_read
            rev_write = cfg_write_serial
        else:
            listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listen_sock.bind((listen_host, listen_port))
            listen_sock.listen(1)
            print(f"{CYAN}[tcp] listening on {listen_host}:{listen_port}{RESET}")

            local_sock, addr = listen_sock.accept()
            print(f"{CYAN}[tcp] client connected from {addr[0]}:{addr[1]}{RESET}")
            local_sock.setblocking(False)

            remote_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            remote_sock.connect((tcp_host, tcp_port))
            remote_sock.setblocking(False)
            print(f"{CYAN}[tcp] connected to {tcp_host}:{tcp_port}{RESET}")

            fwd_parser = MspV2Parser(direction="CFG->FC")
            rev_parser = MspV2Parser(direction="FC->CFG")

            label_fwd = "CFG->FC"
            label_rev = "FC->CFG"

            def cfg_read() -> bytes:
                assert local_sock is not None
                return _tcp_recv_nonblocking(local_sock)

            def cfg_write(data: bytes) -> None:
                assert local_sock is not None
                _tcp_send_all_nonblocking(local_sock, data)

            def elrs_read() -> bytes:
                assert remote_sock is not None
                return _tcp_recv_nonblocking(remote_sock)

            def elrs_write(data: bytes) -> None:
                assert remote_sock is not None
                _tcp_send_all_nonblocking(remote_sock, data)

            fwd_read = cfg_read
            fwd_write = elrs_write
            rev_read = elrs_read
            rev_write = cfg_write
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"{RED}setup error: {exc}{RESET}", file=sys.stderr)
        return 1

    t_serial = threading.Thread(
        target=read_loop,
        args=(
            fwd_read,
            fwd_write,
            fwd_parser,
            cmd_map,
            fwd_color,
            label_fwd,
            stop_event,
            brief,
            log_file,
            no_filter,
        ),
        daemon=True,
    )
    t_tcp = threading.Thread(
        target=read_loop,
        args=(
            rev_read,
            rev_write,
            rev_parser,
            cmd_map,
            rev_color,
            label_rev,
            stop_event,
            brief,
            log_file,
            no_filter,
        ),
        daemon=True,
    )

    _ = t_serial.start()
    _ = t_tcp.start()

    try:
        while t_serial.is_alive() and t_tcp.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    try:
        if ser is not None:
            ser.close()
    except Exception:
        pass
    try:
        if local_sock is not None:
            local_sock.close()
    except Exception:
        pass
    try:
        if remote_sock is not None:
            remote_sock.close()
    except Exception:
        pass
    try:
        if listen_sock is not None:
            listen_sock.close()
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
