#!/usr/bin/env python3
"""
FPGA ADC Host v6
================

Standalone FPGA acquisition host with V3 continuous receive, hourly manifests,
adaptive threshold mode, and FPGA temperature metadata.
"""

import csv
import os
import queue
import re
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    plt = None
    HAS_MATPLOTLIB = False


PC_IP = "192.168.1.102"
PC_PORT = 1234
FPGA_IP = "192.168.1.10"
FPGA_PORT = 1234

SAMPLE_RATE = 20_000_000
CAPTURE_SECONDS = 1.0
POINTS_PER_PACKET = 625
ADC_HEADER_SIZE = 6
ADC_PAYLOAD_SIZE = 1250
ADC_PACKET_SIZE = ADC_HEADER_SIZE + ADC_PAYLOAD_SIZE
TOTAL_POINTS = int(SAMPLE_RATE * CAPTURE_SECONDS)
TOTAL_PACKETS = TOTAL_POINTS // POINTS_PER_PACKET
TIMESTAMP_PACKET_SIZE_LEGACY = 10
TIMESTAMP_PACKET_SIZE_TEMPERATURE = 14
TIMESTAMP_PACKET_SIZES = (
    TIMESTAMP_PACKET_SIZE_LEGACY,
    TIMESTAMP_PACKET_SIZE_TEMPERATURE,
)
TIMESTAMP_PACKET_SIZE = TIMESTAMP_PACKET_SIZE_LEGACY
LEGACY_PACKET_SIZE = 1250

SYNC0 = 0xA5
SYNC1 = 0x5A
PKT_TYPE_TIMESTAMP = 0x01
PKT_TYPE_ADC = 0x02

CMD_TEST_MODE = 0x01
CMD_TRIGGER_MODE = 0x02
CMD_THRESHOLD = 0x80
CMD_THRESHOLD_MASK = CMD_THRESHOLD
THRESHOLD_STEP = 10
THRESHOLD_MULTIPLIER = THRESHOLD_STEP
THRESHOLD_PROTOCOL_MAX = 65530
THRESHOLD_EFFECTIVE_MAX = 32760

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name == "src" else SCRIPT_DIR
DATA_DIR_NAME = "data"
SHEET_DIR_NAME = "sheet"
TEST_DIR_NAME = "test"
SITE_NAME_FILE = "site_name.txt"
DEFAULT_SITE_CODE = "XXX"
DEFAULT_BUFFER_COUNT = 8
REQUESTED_RCVBUF = 64 * 1024 * 1024
DEFAULT_ADAPTIVE_INTERVAL_HOURS = 12.0
DEFAULT_ADAPTIVE_MULTIPLIER = 5.0
DEFAULT_NO_TRIGGER_SAFE_HOURS = 1.0
NOISE_WINDOW_SIZE = 1_000_000
MIN_VALID_POINTS = 1_000_000

GPS_LOCK_TEXT = {0: "UNLOCKED", 1: "LOCKED", 2: "ERROR", 3: "RESERVED"}
WORK_MODE_TEXT = {1: "TEST", 2: "TRIGGER"}


class SiteCodeConfigError(RuntimeError):
    """Raised when the deployment site code cannot be used safely."""


def load_site_code(root_dir):
    """Load one three-letter site code and normalize it to uppercase."""
    path = Path(root_dir) / SITE_NAME_FILE
    if not path.exists():
        try:
            path.write_text(DEFAULT_SITE_CODE + "\n", encoding="ascii")
        except OSError as exc:
            raise SiteCodeConfigError(
                "cannot create {}: {}".format(path, exc)
            )
        return DEFAULT_SITE_CODE, True

    try:
        raw_text = path.read_text(encoding="ascii")
    except (OSError, UnicodeError) as exc:
        raise SiteCodeConfigError(
            "cannot read {}: {}".format(path, exc)
        )

    raw_code = raw_text.strip()
    if re.fullmatch(r"[A-Za-z]{3}", raw_code) is None:
        raise SiteCodeConfigError(
            "{} must contain exactly three English letters".format(path)
        )

    site_code = raw_code.upper()
    if raw_text != site_code + "\n":
        try:
            path.write_text(site_code + "\n", encoding="ascii")
        except OSError as exc:
            raise SiteCodeConfigError(
                "cannot normalize {}: {}".format(path, exc)
            )
    return site_code, False


def build_threshold_command(threshold):
    """Encode threshold as the current RTL-defined payload: 80 HH LL."""
    if isinstance(threshold, bool) or not isinstance(threshold, int):
        raise TypeError("threshold must be an integer")
    if threshold % THRESHOLD_STEP != 0:
        raise ValueError(f"threshold must be a multiple of {THRESHOLD_STEP}")
    if not 0 <= threshold <= THRESHOLD_PROTOCOL_MAX:
        raise ValueError(
            "threshold is outside FPGA protocol range "
            f"0..{THRESHOLD_PROTOCOL_MAX}"
        )

    step = threshold // THRESHOLD_STEP
    return bytes([
        CMD_THRESHOLD,
        (step >> 8) & 0xFF,
        step & 0xFF,
    ])


def normalize_command_payload(command):
    """Accept one-byte integer mode commands or an encoded byte payload."""
    if isinstance(command, int) and not isinstance(command, bool):
        if not 0 <= command <= 0xFF:
            raise ValueError("one-byte command must be in range 0..255")
        return bytes([command])
    if isinstance(command, (bytes, bytearray, memoryview)):
        payload = bytes(command)
        if not payload:
            raise ValueError("command payload cannot be empty")
        return payload
    raise TypeError("command must be an integer or bytes-like payload")


def format_payload_hex(payload):
    """Format bytes without using Python 3.8+ bytes.hex(sep)."""
    return " ".join(f"{byte:02X}" for byte in payload)


def atomic_write_int16(final_path, data):
    """Write little-endian int16 data and atomically publish the final file."""
    final_path = Path(final_path)
    part_path = Path(str(final_path) + ".part")
    little_endian_data = np.ascontiguousarray(data, dtype="<i2")
    with part_path.open("wb") as handle:
        little_endian_data.tofile(handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(str(part_path), str(final_path))


def ensure_runtime_directories(root_dir, now=None):
    """Create V6 output folders and today's date subfolders."""
    root = Path(root_dir)
    day_text = (now or datetime.now()).strftime("%Y%m%d")
    paths = {}
    root.mkdir(parents=True, exist_ok=True)
    for folder_name in (DATA_DIR_NAME, SHEET_DIR_NAME, TEST_DIR_NAME):
        base_dir = root / folder_name
        day_dir = base_dir / day_text
        day_dir.mkdir(parents=True, exist_ok=True)
        paths[folder_name] = day_dir
    return paths


def find_noise_rms(signal, window_size=NOISE_WINDOW_SIZE):
    """Return RMS of the lowest-energy non-overlapping window."""
    num_windows = len(signal) // window_size
    if num_windows < 1:
        return None

    signal_f = signal[:num_windows * window_size].astype(np.float64)
    reshaped = signal_f.reshape(-1, window_size)
    energies = np.mean(reshaped ** 2, axis=1)
    noise_segment = reshaped[int(np.argmin(energies))]
    return float(np.sqrt(np.mean(noise_segment ** 2)))


def calculate_auto_threshold(noise_rms, multiplier):
    """Round adaptive threshold to protocol step and clamp to ADC range."""
    raw_threshold = abs(float(noise_rms)) * float(multiplier)
    rounded = int((raw_threshold / THRESHOLD_STEP) + 0.5) * THRESHOLD_STEP
    return max(0, min(rounded, THRESHOLD_EFFECTIVE_MAX))


class BufferStarvationError(RuntimeError):
    """Raised when all capture buffers are waiting to be saved."""


class CaptureConfig:
    def __init__(
        self,
        sample_rate=SAMPLE_RATE,
        capture_seconds=CAPTURE_SECONDS,
        points_per_packet=POINTS_PER_PACKET,
        total_points=TOTAL_POINTS,
        total_packets=TOTAL_PACKETS,
        adc_payload_size=ADC_PAYLOAD_SIZE,
        adc_packet_size=ADC_PACKET_SIZE,
        timestamp_packet_size=TIMESTAMP_PACKET_SIZE,
        max_datagram_size=2048,
    ):
        self.sample_rate = sample_rate
        self.capture_seconds = capture_seconds
        self.points_per_packet = points_per_packet
        self.total_points = total_points
        self.total_packets = total_packets
        self.adc_payload_size = adc_payload_size
        self.adc_packet_size = adc_packet_size
        self.timestamp_packet_size = timestamp_packet_size
        self.max_datagram_size = max_datagram_size


class EngineStats:
    def __init__(self):
        self.datagrams_received = 0
        self.adc_packets_received = 0
        self.timestamp_packets_received = 0
        self.orphan_adc_packets = 0
        self.bad_packets = 0
        self.legacy_packets = 0
        self.duplicate_packets = 0
        self.captures_completed = 0
        self.captures_saved = 0
        self.captures_with_loss = 0
        self.buffer_starvation = 0
        self.save_errors = 0
        self.save_queue_peak = 0


def _bcd_to_int(value):
    """Decode one packed-BCD byte, or return None for an invalid value."""
    high = (value >> 4) & 0x0F
    low = value & 0x0F
    if high > 9 or low > 9:
        return None
    return high * 10 + low


def _parse_timestamp_datetime(packet, encodings):
    """Return timestamp datetime and encoding from bytes 4..9."""
    for encoding in encodings:
        if encoding == "BCD":
            values = [_bcd_to_int(value) for value in packet[4:10]]
            if any(value is None for value in values):
                continue
        else:
            values = list(packet[4:10])
        year, month, day, hour, minute, second = values
        try:
            return (
                datetime(2000 + year, month, day, hour, minute, second),
                encoding,
            )
        except ValueError:
            continue
    return None, None


def _temperature_celsius(temp_raw):
    return temp_raw * 503.975 / 65536.0 - 273.15


def parse_timestamp_packet(packet):
    packet_size = len(packet)
    if packet_size not in TIMESTAMP_PACKET_SIZES:
        return None
    if packet[0] != SYNC0 or packet[1] != SYNC1:
        return None
    if packet[2] != PKT_TYPE_TIMESTAMP:
        return None

    encodings = ("BCD", "BINARY") if packet_size == 14 else ("BINARY", "BCD")
    parsed, timestamp_encoding = _parse_timestamp_datetime(packet, encodings)
    if parsed is None:
        return None

    b3 = packet[3]
    work_mode = (b3 >> 2) & 0x03
    gps_lock = b3 & 0x03
    info = {
        "work_mode": work_mode,
        "work_mode_text": WORK_MODE_TEXT.get(work_mode, f"UNK({work_mode})"),
        "gps_lock": gps_lock,
        "gps_lock_text": GPS_LOCK_TEXT.get(gps_lock, f"UNK({gps_lock})"),
        "fpga_time": parsed.strftime("%Y-%m-%d %H:%M:%S"),
        "year": parsed.year,
        "month": parsed.month,
        "day": parsed.day,
        "hour": parsed.hour,
        "minute": parsed.minute,
        "second": parsed.second,
        "timestamp_packet_size": packet_size,
        "timestamp_encoding": timestamp_encoding,
        "protocol_ext_version": None,
        "temperature_valid": False,
        "temperature_c": None,
        "temp_raw": None,
        "temp_raw12": None,
        "temp_flags": None,
        "temp_stale": False,
        "temp_alarm": False,
    }
    if packet_size == TIMESTAMP_PACKET_SIZE_TEMPERATURE:
        temp_raw = (packet[10] << 8) | packet[11]
        temp_flags = packet[12]
        temperature_valid = bool(temp_flags & 0x01)
        info.update({
            "protocol_ext_version": packet[13],
            "temperature_valid": temperature_valid,
            "temperature_c": (
                _temperature_celsius(temp_raw) if temperature_valid else None
            ),
            "temp_raw": temp_raw,
            "temp_raw12": temp_raw >> 4,
            "temp_flags": temp_flags,
            "temp_stale": bool(temp_flags & 0x02),
            "temp_alarm": bool(temp_flags & 0x04),
        })
    return info


def compress_missing(missing_indices):
    if len(missing_indices) == 0:
        return ""
    ranges = []
    start = previous = int(missing_indices[0])
    for value in missing_indices[1:]:
        值 = 整数(值)
        如果值 == 上一个 + 1:
            上一个 = 值
            继续
        ranges.append(
            f"{start}-{previous}" if start != previous else str(start)
        )
        开始 = 上一个 = 值
    ranges.append(
        f"{start}-{previous}" if start != previous else str(start)
    )
    return ";".join(ranges)


def classify_loss_position(missing_indices, total_packets):
    if len(missing_indices) == 0:
        返回 "无"
    ranges = compress_missing(missing_indices).split(";")
    if len(ranges) > 1:
        返回 "分散的"
    第一个 = int(缺失索引[0])
    最后一个 = int(缺失索引[-1])
    if first == 0:
        返回 "头部"
    如果最后 == 总数据包数 - 1:
        返回 "尾部"
    返回 "中部"

    @属性
    def丢失的数据包(self):
        return self.config.total_packets - self.received_count

    @属性
    def 接收时长（秒）(self):
        如果 self.receive_end_monotonic 为 None:
            返回 None
        return self.receive_end_monotonic - self.receive_start_monotonic

    def 添加ADC数据包(self, packet):
        pkt_idx = (packet[4] << 8) | packet[5]
        if pkt_idx >= self.config.total_packets:
            self.bad_count += 1
            返回 False
        if self.received[pkt_idx]:
            self.duplicate_count += 1
            返回 False

        self.received[pkt_idx] = True
        self.received_count += 1
        self.sec_idx_set.add(packet[3] & 0x07)
        start = pkt_idx * self.config.points_per_packet
        end = min(
            start + self.config.points_per_packet,
            self.config.total_points,
        )
        values = np.frombuffer(
            packet,
            dtype="<i2",
            count=self.config.points_per_packet,
            offset=ADC_HEADER_SIZE,
        )
        self.data[start:end] = values[: end - start]
        return 真

    def close(self, reason):
        self.end_reason = reason
        self.receive_end_monotonic = time.monotonic()
        self.receive_end_time = datetime.now()

    def prepare_for_save(self):
        self.missing_indices = np.flatnonzero(~self.received)
        self.missing_ranges = compress_missing(self.missing_indices)
        self.loss_position = classify_loss_position(
            self.missing_indices,
            self.config.total_packets,
        )

        if len(self.missing_indices):
            if (
                self.config.total_points
                == self.config.total_packets * self.config.points_per_packet
            ):
                blocks = self.data.reshape(
                    self.config.total_packets,
                    self.config.points_per_packet,
                )
                blocks[self.missing_indices] = 0
            else:
                for pkt_idx in self.missing_indices:
                    start = int(pkt_idx) * self.config.points_per_packet
                    end = min(
                        start + self.config.points_per_packet,
                        self.config.total_points,
                    )
                    self.data[start:end] = 0

        self.minimum = int(self.data.min())
        self.maximum = int(self.data.max())
        self.mean = round(float(self.data.mean()), 2)


class CaptureSnapshot:
    def __init__(
        self,
        data,
        ts_info,
        capture_index,
        received_count,
        missing_packets,
        missing_ranges,
        minimum,
        maximum,
        mean,
    ):
        self.data = data
        self.ts_info = ts_info
        self.capture_index = capture_index
        self.received_count = received_count
        self.missing_packets = missing_packets
        self.missing_ranges = missing_ranges
        self.minimum = minimum
        self.maximum = maximum
        self.mean = mean


class InMemoryCaptureWriter:
    """Writer used by TEST/calibration paths; it never writes waveform files."""

    def __init__(self):
        self.last_capture = None
        self.captures = []

    def write(self, capture):
        snapshot = CaptureSnapshot(
            data=capture.data.copy(),
            ts_info=dict(capture.ts_info or {}),
            capture_index=capture.capture_index,
            received_count=capture.received_count,
            missing_packets=capture.missing_packets,
            missing_ranges=capture.missing_ranges,
            minimum=capture.minimum,
            maximum=capture.maximum,
            mean=capture.mean,
        )
        self.last_capture = snapshot
        self.captures.append(snapshot)
        return snapshot


MANIFEST_FIELDS = [
    "event_type",
    "data_file",
    "capture_index",
    "base_name",
    "timestamp_source",
    "system_save_time",
    "fpga_time",
    "work_mode",
    "work_mode_text",
    "gps_lock",
    "gps_lock_text",
    "timestamp_packet_size",
    "timestamp_encoding",
    "protocol_ext_version",
    "temperature_valid",
    "temperature_c",
    "temp_raw",
    "temp_raw12",
    "temp_flags",
    "temp_stale",
    "temp_alarm",
    "current_threshold",
    "points",
    "sample_rate",
    "duration_sec",
    "minimum",
    "maximum",
    "mean",
    "packets_received",
    "total_packets",
    "missing_packets",
    "missing_ranges",
    "loss_position",
    "sec_idx_set",
    "sec_idx_consistent",
    "duplicate_packets",
    "bad_packets",
    "segment_end_reason",
    "receive_start_time",
    "receive_end_time",
    "receive_duration_sec",
    "data_save_duration_sec",
    "queue_depth_at_submit",
    "noise_rms",
    "multiplier",
    "raw_threshold",
    "old_threshold",
    "new_threshold",
    "calibration_result",
    "calibration_note",
]


class ManifestCaptureWriter:
    def __init__(
        self,
        root_dir,
        config,
        event_type="trigger",
        current_threshold_getter=None,
        on_trigger_saved=None,
        site_code=DEFAULT_SITE_CODE,
    ):
        if event_type not in ("trigger", "test"):
            raise ValueError("event_type must be 'trigger' or 'test'")
        if re.fullmatch(r"[A-Z]{3}", site_code or "") is None:
            raise ValueError("site_code must be three uppercase letters")
        self.root_dir = Path(root_dir)
        self.config = config
        self.event_type = event_type
        self.site_code = site_code
        self.current_threshold_getter = current_threshold_getter or (lambda: "")
        self.on_trigger_saved = on_trigger_saved
        self.last_data_path = None
        ensure_runtime_directories(self.root_dir)

    @staticmethod
    def _parse_fpga_time(ts_info):
        fpga_time = (ts_info or {}).get("fpga_time")
        try:
            return datetime.strptime(fpga_time, "%Y-%m-%d %H:%M:%S"), "FPGA"
        except (TypeError, ValueError):
            return datetime.now(), "SYSTEM_NO_TS"

    @staticmethod
    def _timestamp_text(dt):
        return dt.strftime("%Y%m%d_%H%M%S")

    def _capture_day_dir(self, dt):
        folder_name = (
            TEST_DIR_NAME if self.event_type == "test" else DATA_DIR_NAME
        )
        path = self.root_dir / folder_name / dt.strftime("%Y%m%d")
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _manifest_path(self, dt):
        day_dir = self.root_dir / SHEET_DIR_NAME / dt.strftime("%Y%m%d")
        day_dir.mkdir(parents=True, exist_ok=True)
        return day_dir / f"capture_manifest_{dt:%Y%m%d_%H00}.csv"

    def _current_threshold(self):
        value = self.current_threshold_getter()
        return "" if value is None else value

    @staticmethod
    def _timestamp_manifest_fields(timestamp_info):
        ts = timestamp_info or {}
        temperature_c = ts.get("temperature_c")
        return {
            "timestamp_packet_size": ts.get("timestamp_packet_size", ""),
            "timestamp_encoding": ts.get("timestamp_encoding", ""),
            "protocol_ext_version": ts.get("protocol_ext_version", ""),
            "temperature_valid": ts.get("temperature_valid", False),
            "temperature_c": (
                "" if temperature_c is None else f"{temperature_c:.2f}"
            ),
            "temp_raw": (
                "" if ts.get("temp_raw") is None else ts.get("temp_raw")
            ),
            "temp_raw12": (
                "" if ts.get("temp_raw12") is None else ts.get("temp_raw12")
            ),
            "temp_flags": (
                "" if ts.get("temp_flags") is None else ts.get("temp_flags")
            ),
            "temp_stale": ts.get("temp_stale", False),
            "temp_alarm": ts.get("temp_alarm", False),
        }

    def _append_manifest_row(self, dt, row):
        manifest_path = self._manifest_path(dt)
        new_file = not manifest_path.exists()
        clean = {field: row.get(field, "") for field in MANIFEST_FIELDS}
        with manifest_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(clean)

    def _base_name(self, capture, dt, timestamp_source):
        ts_info = capture.ts_info or {}
        gps_suffix = "_l" if ts_info.get("gps_lock") == 1 else "_ul"
        no_ts_suffix = "" if timestamp_source == "FPGA" else "_no_ts"
        base = (
            self.site_code + self._timestamp_text(dt) + no_ts_suffix
            + gps_suffix
        )
        capture_dir = self._capture_day_dir(dt)
        candidate = base
        suffix = 1
        while (
            (capture_dir / f"{candidate}.bin").exists()
            or (capture_dir / f"{candidate}.bin.part").exists()
        ):
            candidate = f"{base}_c{capture.capture_index:06d}_{suffix}"
            suffix += 1
        return candidate, capture_dir / f"{candidate}.bin"

   

    def log_calibration_event(
        self,
        timestamp_info,
        noise_rms,
        multiplier,
        old_threshold,
        new_threshold,
        result,
        note,
    ):
        dt, timestamp_source = self._parse_fpga_time(timestamp_info)
        if noise_rms is None:
            noise_rms_text = ""
            raw_threshold_text = ""
        else:
            raw_threshold = abs(float(noise_rms)) * float(multiplier)
            noise_rms_text = f"{float(noise_rms):.2f}"
            raw_threshold_text = f"{raw_threshold:.2f}"
        ts = timestamp_info or {}
        row = {
            "event_type": "calibration",
            "timestamp_source": timestamp_source,
            "system_save_time": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "fpga_time": ts.get("fpga_time") or "",
            "work_mode": ts.get("work_mode") or "",
            "work_mode_text": ts.get("work_mode_text") or "",
            "gps_lock": ts.get("gps_lock") if ts.get("gps_lock") is not None else "",
            "gps_lock_text": ts.get("gps_lock_text") or "",
            "current_threshold": self._current_threshold(),
            "noise_rms": noise_rms_text,
            "multiplier": f"{float(multiplier):.2f}",
            "raw_threshold": raw_threshold_text,
            "old_threshold": "" if old_threshold is None else old_threshold,
            "new_threshold": "" if new_threshold is None else new_threshold,
            "calibration_result": result,
            "calibration_note": note,
        }
        row.update(self._timestamp_manifest_fields(ts))
        self._append_manifest_row(dt, row)


class ContinuousCaptureEngine:
    def __init__(
        self,
        config=None,
        writer=None,
        buffer_count=DEFAULT_BUFFER_COUNT,
        buffer_wait_timeout_s=0.01,
        live_feedback=False,
    ):
        self.config = config or CaptureConfig()
        self.writer = writer or ManifestCaptureWriter(PROJECT_ROOT, self.config)
        self.buffer_count = buffer_count
        self.buffer_wait_timeout_s = buffer_wait_timeout_s
        self.live_feedback = live_feedback
        self.stats = EngineStats()

        self._free_buffers = queue.Queue(maxsize=buffer_count)
        self._save_queue = queue.Queue(maxsize=buffer_count)
        for _ in range(buffer_count):
            self._free_buffers.put(CaptureBuffer(self.config))

        self._active = None
        self.last_timestamp_info = None
        self._capture_index = 0
        self._saver_thread = None
        self._receiver_thread = None
        self._stop_event = threading.Event()
        self._fatal_error = None
        self._closed = False

    @property
    def free_buffer_count(self):
        return self._free_buffers.qsize()

    @property
    def fatal_error(self):
        return self._fatal_error

    @property
    def receiver_alive(self):
        return self._receiver_thread is not None and self._receiver_thread.is_alive()

    def start_saver(self):
        if self._saver_thread is not None:
            return
        self._saver_thread = threading.Thread(
            target=self._saver_loop,
            name="fpga-save",
            daemon=True,
        )
        self._saver_thread.start()

    def start_receiver(self, sock):
        self.start_saver()
        if self._receiver_thread is not None:
            raise RuntimeError("receiver already started")
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop,
            args=(sock,),
            name="fpga-recv",
            daemon=True,
        )
        self._receiver_thread.start()

    def _acquire_buffer(self, ts_info):
        try:
            capture = self._free_buffers.get(
                timeout=self.buffer_wait_timeout_s
            )
        except queue.Empty as exc:
            self.stats.buffer_starvation += 1
            raise BufferStarvationError(
                "capture buffer pool exhausted; disk writer is not keeping up"
            ) from exc
        self._capture_index += 1
        capture.reset(ts_info, self._capture_index)
        self._active = capture

    def _submit_active(self, reason):
        capture = self._active
        if capture is None:
            return
        if capture.received_count == 0:
            self._active = None
            self._free_buffers.put(capture)
            return

        capture.close(reason)
        self._active = None
        queue_depth = self._save_queue.qsize() + 1
        self.stats.save_queue_peak = max(
            self.stats.save_queue_peak,
            queue_depth,
        )
        capture.queue_depth_at_submit = queue_depth
        self.stats.captures_completed += 1
        if capture.missing_packets:
            self.stats.captures_with_loss += 1
        self._save_queue.put_nowait(capture)

    
    @staticmethod
    def _print_timestamp_feedback(ts_info):
        fpga_time = ts_info.get("fpga_time") or "NO_TIMESTAMP"
        if ts_info.get("timestamp_packet_size") == TIMESTAMP_PACKET_SIZE_LEGACY:
            temperature_text = "temperature=N/A (legacy packet)"
        elif not ts_info.get("temperature_valid"):
            temperature_text = "temperature=INVALID"
        else:
            temperature_text = f"temperature={ts_info['temperature_c']:.2f} C"
        flags = []
        if ts_info.get("temp_stale"):
            flags.append("STALE")
        if ts_info.get("temp_alarm"):
            flags.append("ALARM")
        suffix = "" if not flags else " " + ",".join(flags)
        print(
            f"[Status] FPGA={fpga_time}  {temperature_text}{suffix}",
            flush=True,
        )

    def _receiver_loop(self, sock):
        packet_buffer = bytearray(self.config.max_datagram_size)
        packet_view = memoryview(packet_buffer)
        sock.settimeout(0.1)
        try:
            while not self._stop_event.is_set():
                try:
                    packet_length, _ = sock.recvfrom_into(packet_buffer)
                except socket.timeout:
                    continue
                self.process_datagram(packet_view[:packet_length])
        except Exception as exc:
            self._fatal_error = exc
            self._stop_event.set()

    def _saver_loop(self):
        while True:
            capture = self._save_queue.get()
            if capture is None:
                self._save_queue.task_done()
                return
            try:
                capture.prepare_for_save()
                save_started = time.monotonic()
                self.writer.write(capture)
                save_duration = time.monotonic() - save_started
                self.stats.captures_saved += 1
                if self.live_feedback:
                    self._print_capture_feedback(capture, save_duration)
            except Exception as exc:
                self.stats.save_errors += 1
                self._fatal_error = exc
                self._stop_event.set()
            finally:
                self._free_buffers.put(capture)
                self._save_queue.task_done()

    def _print_capture_feedback(self, capture, save_duration):
        ts = capture.ts_info or {}
        fpga_time = ts.get("fpga_time") or "NO_TIMESTAMP"
        display_time = (
            fpga_time.split(" ")[1]
            if " " in fpga_time
            else fpga_time
        )
        gps_text = "L" if ts.get("gps_lock") == 1 else "UL"
        receive_duration = capture.receive_duration_s or 0.0
        if capture.missing_packets == 0:
            print(
                f"[{capture.capture_index:04d}] {display_time}  "
                f"GPS={gps_text}  OK  "
                f"recv={receive_duration:.3f}s  "
                f"save={save_duration:.3f}s",
                flush=True,
            )
        else:
            print(
                f"[{capture.capture_index:04d}] {display_time}  "
                f"GPS={gps_text}  LOSS  "
                f"{capture.received_count}/{self.config.total_packets}  "
                f"range={capture.missing_ranges}  "
                f"reason={capture.end_reason}",
                flush=True,
            )

        if capture.queue_depth_at_submit > 1:
            print(
                f"[WARN] save backlog={capture.queue_depth_at_submit}/"
                f"{self.buffer_count}",
                flush=True,
            )

    def request_stop(self):
        self._stop_event.set()

    def stop_receiver_and_drain(self):
        self.request_stop()
        if self._receiver_thread is not None:
            self._receiver_thread.join(timeout=2.0)
            if self._receiver_thread.is_alive():
                raise RuntimeError("receiver thread did not stop")
        self.stop_and_drain()

    def stop_and_drain(self):
        if self._closed:
            return
        self._submit_active("STOP")
        self._save_queue.join()
        self._save_queue.put(None)
        self._save_queue.join()
        if self._saver_thread is not None:
            self._saver_thread.join(timeout=5.0)
            if self._saver_thread.is_alive():
                raise RuntimeError("saver thread did not stop")
        self._closed = True


class FPGAHostV6:
    def __init__(self):
        self.config = CaptureConfig()
        self.project_root = PROJECT_ROOT
        self.site_code, site_file_created = load_site_code(
            self.project_root
        )
        self.current_threshold = None
        self.last_trigger_time = None
        self.last_calibration_time = None
        self.adaptive_interval_hours = DEFAULT_ADAPTIVE_INTERVAL_HOURS
        self.adaptive_multiplier = DEFAULT_ADAPTIVE_MULTIPLIER
        self.no_trigger_safe_hours = DEFAULT_NO_TRIGGER_SAFE_HOURS
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            REQUESTED_RCVBUF,
        )
        self.sock.bind((PC_IP, PC_PORT))
        self.sock.settimeout(5.0)
        actual_rcvbuf = self.sock.getsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
        )

  
    @classmethod
    def plot_data_file(cls, data_path):
        if not HAS_MATPLOTLIB:
            print("[Plot] matplotlib not installed, skip plotting")
            return

        data_path = Path(data_path)
        data = np.fromfile(str(data_path), dtype="<i2")
        if data.size == 0:
            print(f"[Plot] empty data file: {data_path}")
            return

        print(f"[Plot] Drawing waveform: {data_path.name}")
        cls.plot_data_array(
            data,
            title=f"FPGA ADC Data (Total: {data.size} points)",
        )

    def _make_manifest_writer(self):
        return ManifestCaptureWriter(
            root_dir=self.project_root,
            config=self.config,
            current_threshold_getter=lambda: self.current_threshold,
            on_trigger_saved=self._on_trigger_saved,
            site_code=self.site_code,
        )

    def _make_test_writer(self):
        return ManifestCaptureWriter(
            root_dir=self.project_root,
            config=self.config,
            event_type="test",
            current_threshold_getter=lambda: self.current_threshold,
            site_code=self.site_code,
        )

    def _on_trigger_saved(self, _capture):
        self.last_trigger_time = time.time()

    def _receive_single_test_capture(
        self,
        writer=None,
        timeout_s=30.0,
        live_feedback=True,
        print_summary=True,
    ):
        if not self.send_command(CMD_TEST_MODE):
            return None

        if writer is None:
            writer = InMemoryCaptureWriter()
        engine = ContinuousCaptureEngine(
            config=self.config,
            writer=writer,
            buffer_count=DEFAULT_BUFFER_COUNT,
            live_feedback=live_feedback,
        )
        engine.start_receiver(self.sock)
        started = time.monotonic()
        if isinstance(writer, InMemoryCaptureWriter):
            print("[TEST] receiving one capture in memory; no waveform is saved")
        else:
            print("[TEST] receiving one capture; waveform will be saved")
        try:
            while engine.receiver_alive:
                if engine.fatal_error is not None:
                    raise engine.fatal_error
                if engine.stats.captures_saved >= 1:
                    break
                if time.monotonic() - started > timeout_s:
                    print(f"[TEST] timeout after {timeout_s:.1f}s")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[Stop] user requested stop")
        except Exception as exc:
            print(f"\n[Error] capture stopped: {exc}")
        finally:
            engine.stop_receiver_and_drain()
            elapsed = time.monotonic() - started
            if print_summary:
                self._print_summary(engine.stats, elapsed)
        return writer

    def _run_test_capture(self, plot_after_capture=True):
        writer = self._receive_single_test_capture(
            writer=self._make_test_writer(),
            live_feedback=True,
            print_summary=True,
        )
        if writer is None or writer.last_data_path is None:
            print("[TEST] no valid capture received")
            return None
        if plot_after_capture:
            self.plot_data_file(writer.last_data_path)
        return writer.last_data_path

    def _start_trigger_engine(self):
        writer = self._make_manifest_writer()
        engine = ContinuousCaptureEngine(
            config=self.config,
            writer=writer,
            buffer_count=DEFAULT_BUFFER_COUNT,
            live_feedback=True,
        )
        engine.start_receiver(self.sock)
        print("[Listen] trigger receiver started; Ctrl+C to stop")
        return engine

    def _stop_engine_and_print_summary(self, engine, started):
        if engine is None:
            return
        engine.stop_receiver_and_drain()
        elapsed = time.monotonic() - started
        self._print_summary(engine.stats, elapsed)

    def _run_trigger_capture(self, auto_mode=False):
        if not auto_mode and not self.send_command(CMD_TRIGGER_MODE):
            return

        engine = self._start_trigger_engine()
        started = time.monotonic()
        try:
            while engine.receiver_alive:
                if engine.fatal_error is not None:
                    raise engine.fatal_error
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[Stop] user requested stop")
        except Exception as exc:
            print(f"\n[Error] trigger capture stopped: {exc}")
        finally:
            self._stop_engine_and_print_summary(engine, started)

    @staticmethod
    def _print_summary(stats, elapsed):
        print("\n" + "=" * 60)
        print("V6 capture summary")
        print("=" * 60)
        print(f"ElapsedSec: {elapsed:.3f}")
        print(f"CapturesCompleted: {stats.captures_completed}")
        print(f"CapturesSaved: {stats.captures_saved}")
        print(f"CapturesWithLoss: {stats.captures_with_loss}")
        print(f"AdcPacketsReceived: {stats.adc_packets_received}")
        print(f"DuplicatePackets: {stats.duplicate_packets}")
        print(f"OrphanAdcPackets: {stats.orphan_adc_packets}")
        print(f"BadPackets: {stats.bad_packets}")
        print(f"SaveQueuePeak: {stats.save_queue_peak}")
        print(f"BufferStarvation: {stats.buffer_starvation}")
        print(f"SaveErrors: {stats.save_errors}")
        print("=" * 60)

    def test_mode(self):
        self._run_test_capture(plot_after_capture=True)

    def trigger_mode(self):
        self._run_trigger_capture(auto_mode=False)

    def auto_mode(self):
        self._run_auto_capture()

    def _should_calibrate(self):
        if self.last_calibration_time is None:
            return True
        now = time.time()
        interval_s = self.adaptive_interval_hours * 3600.0
        if now - self.last_calibration_time < interval_s:
            return False

        safe_s = self.no_trigger_safe_hours * 3600.0
        if self.last_trigger_time is None:
            return True
        return now - self.last_trigger_time >= safe_s

    def _perform_calibration(self, note):
        old_threshold = self.current_threshold
        timestamp_info = None
        noise_rms = None
        new_threshold = None
        result = "FAIL"
        result_note = note
        try:
            print(f"[Auto] calibration start: {note}")
            writer = self._receive_single_test_capture(
                writer=InMemoryCaptureWriter(),
                timeout_s=30.0,
                live_feedback=True,
                print_summary=False,
            )
            if writer is None or writer.last_capture is None:
                result_note = f"{note}: no test capture"
                print(f"[Auto] calibration failed: {result_note}")
                return False

            snapshot = writer.last_capture
            timestamp_info = snapshot.ts_info
            if snapshot.missing_packets != 0:
                result_note = (
                    f"{note}: incomplete test capture, "
                    f"missing={snapshot.missing_packets}"
                )
                print(f"[Auto] calibration failed: {result_note}")
                return False
            if snapshot.data.size < MIN_VALID_POINTS:
                result_note = (
                    f"{note}: valid points {snapshot.data.size} "
                    f"< {MIN_VALID_POINTS}"
                )
                print(f"[Auto] calibration failed: {result_note}")
                return False


    @staticmethod
    def show_help():
        print(
            """
==================== Commands (v6) ====================
  test              - save one TEST capture to test/YYYYMMDD and plot it
  trigger           - save trigger captures to data/YYYYMMDD
  auto              - adaptive threshold + trigger save; calibration TEST not saved
  value XXXXX - set threshold (0..65530, step 10; effective <=32760)
  set interval H    - set adaptive interval in hours, default 12
  set multi X       - set noise RMS multiplier, default 5
  set safe H        - require no trigger for H hours before calibration
  show              - show current paths and adaptive settings
  help              - show help
  quit              - exit
=======================================================
"""
        )

    def run(self):
        self.show_help()
        try:
            while True:
                command = input("\nCommand> ").strip().lower()
                if command in ("test", "t", "1"):
                    if self.confirm_manual_capture("test"):
                        self.test_mode()
                elif command in ("trigger", "trig", "2"):
                    if self.confirm_manual_capture("trigger"):
                        self.trigger_mode()
                elif command in ("auto", "a", "3"):
                    if self.confirm_manual_capture("auto"):
                        self.auto_mode()
                elif command.startswith(("value ", "v ")):
                    parts = command.split()
                    if len(parts) != 2:
                        print("Usage: value XXXXX")
                        continue
                    try:
                        self.set_threshold(int(parts[1]))
                    except ValueError:
                        print("Usage: value XXXXX")
                elif command.startswith("set "):
                    self.handle_set_command(command)
                elif command in ("show", "settings", "status"):
                    self.show_settings()
                elif command in ("help", "h", "?"):
                    self.show_help()
                elif command in ("quit", "exit", "q"):
                    break
                elif command:
                    print(f"Unknown command: {command}")
        finally:
            self.sock.close()


if __name__ == "__main__":
    try:
        FPGAHostV6().run()
    except SiteCodeConfigError as exc:
        print(f"[Error] invalid site_name.txt: {exc}")
        sys.exit(2)
    except Exception as exc:
        print(f"Error: {exc}")
        import traceback

        traceback.print_exc()
