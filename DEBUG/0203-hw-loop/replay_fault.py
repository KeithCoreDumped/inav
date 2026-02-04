import sys
import time
import json
import socket
import struct
import threading
import binascii
import serial
from pathlib import Path

# --- Constants & Configuration ---
SERIAL_PORT = '/dev/cu.usbmodem11402'
SERIAL_BAUDRATE = 420000
LOG_FILE_PATH = '/Users/kcd/WorkLoad/Flight/inav/DEBUG/0203-hw-loop/crsf-faulty.log'

# CRSF Constants
CRSF_SYNC_BYTE = 0xC8
CRSF_MAX_PACKET_LEN = 64
CRSF_FRAME_NOT_COUNTED_BYTES = 2
CRSF_TELEMETRY_LENGTH_INDEX = 1
CRSF_TELEMETRY_TYPE_INDEX = 2
CRSF_TELEMETRY_CRC_LENGTH = 1

CRSF_FRAMETYPE_MSP_REQ = 0x7A
CRSF_FRAMETYPE_MSP_RESP = 0x7B
CRSF_FRAMETYPE_MSP_WRITE = 0x7C

CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA

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

# --- CRC Helper ---
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

# --- CRSF Parser ---
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
                    # CRC Check
                    expected_crc = calc_crsf_crc(self.buffer[CRSF_FRAME_NOT_COUNTED_BYTES : CRSF_FRAME_NOT_COUNTED_BYTES + self.current_len - CRSF_TELEMETRY_CRC_LENGTH])
                    actual_crc = self.buffer[CRSF_FRAME_NOT_COUNTED_BYTES + self.current_len - CRSF_TELEMETRY_CRC_LENGTH]
                    
                    self.state = self.STATE_IDLE
                    if actual_crc == expected_crc:
                        frame = bytes(self.buffer[:self.current_len + CRSF_FRAME_NOT_COUNTED_BYTES])
                        frames.append(frame)
        return frames

def describe_crsf_frame(frame):
    if not frame or len(frame) < 4:
        return None
    length = frame[CRSF_TELEMETRY_LENGTH_INDEX]
    type_id = frame[CRSF_TELEMETRY_TYPE_INDEX]
    payload_len = max(0, length - 2)
    name = FRAME_TYPES.get(type_id, f"UNKNOWN(0x{type_id:02X})")
    
    # Simple summary
    return f"CRSF type={name} len={length} pl={payload_len}"

# --- Main Replayer ---
class FaultReplayer:
    def __init__(self):
        self.serial_conn = None
        self.running = True
        self.parser = CrsfStreamParser()
        self.packets_to_send = []

    def load_log(self):
        print(f"Loading log from {LOG_FILE_PATH}...")
        try:
            with open(LOG_FILE_PATH, 'r') as f:
                for line in f:
                    try:
                        entry = json.loads(line)
                        # We want the packets that were SENT to serial (TCP->SER flow resulted in Built CRSF frame)
                        # The log entry for the construction is: msg="Built CRSF frame | ..."
                        if entry.get("msg", "").startswith("Built CRSF frame"):
                            hex_data = entry.get("data_hex", "")
                            if hex_data:
                                data = binascii.unhexlify(hex_data)
                                ts = entry.get("ts", 0)
                                self.packets_to_send.append((ts, data))
                                
                                # Stop after the first MSP2_INAV_SERVO_MIXER command
                                if "MSP2_INAV_SERVO_MIXER" in entry.get("msg", ""):
                                    print("Found first MSP2_INAV_SERVO_MIXER, stopping load.")
                                    break
                    except ValueError:
                        continue
        except Exception as e:
            print(f"Error loading log: {e}")
            sys.exit(1)
        
        print(f"Loaded {len(self.packets_to_send)} packets to replay.")

    def start(self):
        try:
            print(f"Opening serial port {SERIAL_PORT} @ {SERIAL_BAUDRATE}...")
            self.serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)

        # Start reader thread
        reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        reader_thread.start()

        # Replay loop
        if not self.packets_to_send:
            print("No packets found to replay.")
            return

        print("Starting replay...")
        start_time_log = self.packets_to_send[0][0]
        start_time_real = time.time()

        for ts_log, data in self.packets_to_send:
            # Calculate delay
            # We add a small offset to ensure we don't rush too much at the start
            current_time = time.time()
            target_delay = (ts_log - start_time_log)
            # Adjust if we are behind? No, just sleep if needed.
            elapsed = current_time - start_time_real
            wait = target_delay - elapsed
            
            if wait > 0:
                time.sleep(wait)
            
            # Send
            # print(f"[{time.strftime('%H:%M:%S')}] Sending {len(data)} bytes...")
            try:
                self.serial_conn.write(data)
                # print(f"Sent: {binascii.hexlify(data).decode().upper()}")
            except Exception as e:
                print(f"Write error: {e}")

        print("Replay finished. Waiting for trailing responses...")
        time.sleep(3) # Wait for any final responses
        self.running = False
        reader_thread.join(timeout=1)
        if self.serial_conn:
            self.serial_conn.close()

    def reader_loop(self):
        bg_types = {0x08, 0x1E, 0x21, 0x14, 0x07, 0x02, 0x09} # Battery, Attitude, FlightMode, LinkStats, Vario, GPS, Baro
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        frames = self.parser.feed(data)
                        for frame in frames:
                            type_id = frame[CRSF_TELEMETRY_TYPE_INDEX]
                            # Filter background telemetry to reduce noise
                            if type_id in bg_types:
                                continue
                            
                            summary = describe_crsf_frame(frame)
                            print(f"[RECV] {summary}")
                            # HEX dump for MSP responses to see if they are split
                            if type_id in (CRSF_FRAMETYPE_MSP_RESP, CRSF_FRAMETYPE_MSP_REQ):
                                hex_dump = binascii.hexlify(frame).decode().upper()
                                print(f"       DATA: {hex_dump}")

            except Exception as e:
                print(f"Reader error: {e}")
                time.sleep(0.1)

if __name__ == "__main__":
    replayer = FaultReplayer()
    replayer.load_log()
    replayer.start()
