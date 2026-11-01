"""Serial port discovery, wire frame parsing, and command packet encoding."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import struct
import serial
import serial.tools.list_ports

from constants import (
    FRAME_START,
    FRAME_END,
    FRAME_LAYOUTS,
    FRAME_SIZE_CONFIG,
    FRAME_SIZE_ERROR,
    TX_CONFIG_STRUCT,
)


@dataclass
class ParsedFrames:
    """Container for frames decoded from a single read of the serial byte buffer."""
    rms: List[float] = field(default_factory=list)
    cw: List[float] = field(default_factory=list)
    cv: List[float] = field(default_factory=list)
    cv_threshold: List[float] = field(default_factory=list)
    status: List[int] = field(default_factory=list)
    latest_config: Optional[Tuple[int, int, float, float]] = None
    latest_error: Optional[Tuple[float, float]] = None


def list_valid_serial_ports() -> List[Tuple[str, str]]:
    """Return a list of (display_label, device_path) for usable serial ports."""
    ports = serial.tools.list_ports.comports()
    valid_ports: List[Tuple[str, str]] = []

    for p in ports:
        if not p.device:
            continue

        desc = (p.description or "").strip()
        if desc.lower() == "n/a" or (p.device.startswith("/dev/ttyS") and not p.hwid):
            continue

        label = f"{p.device} — {desc}" if desc else p.device
        valid_ports.append((label, p.device))

    return valid_ports


def pack_config_packet(ctrl_byte: int, target_window_ms: int, threshold_offset: float, saturation_pct: float) -> bytes:
    """Pack a configuration / command packet to send to the MCU."""
    return TX_CONFIG_STRUCT.pack(
        FRAME_START,
        ctrl_byte,
        target_window_ms,
        threshold_offset,
        saturation_pct,
        FRAME_END,
    )


def parse_byte_buffer(byte_buffer: bytearray) -> ParsedFrames:
    """
    Scan `byte_buffer` for valid 0xAA..0xBB frames, consume processed bytes in-place,
    and return all decoded telemetry samples, config sync frames, and error frames.
    """
    parsed = ParsedFrames()
    n = len(byte_buffer)
    idx = 0

    while idx < n:
        if byte_buffer[idx] != FRAME_START:
            idx += 1
            continue

        matched_struct = None
        matched_size = 0
        need_more = False

        for payload_struct, frame_size in FRAME_LAYOUTS:
            end = idx + frame_size
            if end > n:
                need_more = True
                break
            if byte_buffer[end - 1] != FRAME_END:
                continue
            if end < n and byte_buffer[end] != FRAME_START:
                continue
            matched_struct = payload_struct
            matched_size = frame_size
            break

        if matched_size == 0:
            if need_more:
                break
            idx += 1
            continue

        try:
            values = matched_struct.unpack_from(byte_buffer, idx + 1)
        except struct.error:
            idx += 1
            continue

        if matched_size == FRAME_SIZE_ERROR:
            mse, r_squared = values
            parsed.latest_error = (mse, r_squared)
        elif matched_size == FRAME_SIZE_CONFIG:
            ctrl, t_win, t_offset, sat_pct = values
            parsed.latest_config = (ctrl, t_win, t_offset, sat_pct)
        else:
            rms, cw, cv, cv_threshold, status_code = values
            parsed.rms.append(rms)
            parsed.cw.append(cw)
            parsed.cv.append(cv)
            parsed.cv_threshold.append(cv_threshold)
            parsed.status.append(status_code)

        idx += matched_size

    del byte_buffer[:idx]
    return parsed
