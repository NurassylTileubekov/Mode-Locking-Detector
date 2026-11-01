"""Application constants, wire protocol definitions, and visual theme settings."""

import struct
import numpy as np

# ── Application & Plot Configuration ──────────────────────────────────────────
BAUDRATE           = 460800
MAX_POINTS         = 20000  # Must not exceed Qt QImage width limit (32767)
PLOT_INTERVAL_MS   = 16
PORT_SCAN_MS       = 100
SERIAL_TIMEOUT_SEC = 3.0
CAPTURE_DIR        = "MLDConfig_captures"

# ── Serial Wire Protocol ──────────────────────────────────────────────────────
FRAME_START = 0xAA
FRAME_END   = 0xBB

RESET_TO_DEFAULT_BIT = 0b00000001
START_ACQUIRING_BIT  = 0b00000010
STOP_ACQUIRING_BIT   = 0b00000100
REVERT_TO_PREV_BIT   = 0b00001000

# Wire frame sizes: 1 byte START + struct payload size + 1 byte END
FRAME_SIZE_TELEMETRY = 19
FRAME_SIZE_CONFIG    = 13
FRAME_SIZE_ERROR     = 10

FRAME_LAYOUTS = (
    (struct.Struct('<ffffB'), FRAME_SIZE_TELEMETRY),
    (struct.Struct('<BHff'),  FRAME_SIZE_CONFIG),
    (struct.Struct('<ff'),    FRAME_SIZE_ERROR),
)
TX_CONFIG_STRUCT = struct.Struct('<BBHffB')

# ── Laser Operating Status & Colors ───────────────────────────────────────────
STATUS_LABELS = {
    0: 'LOW SIGNAL',
    1: 'CW',
    2: 'UNSTABLE',
    3: 'MODE LOCKED',
    4: 'SATURATED',
    5: 'INITIATING',
}

STATUS_BADGE_COLOR = {
    0: '#7B4F9E',
    1: '#2E6EA6',
    2: '#B07800',
    3: '#2A7A3A',
    4: '#A02828',
    5: '#1A9BA1',
}

STATE_RGBA = {
    0: np.array([0.73, 0.58, 0.90, 0.30], dtype=float),
    1: np.array([0.50, 0.72, 0.92, 0.30], dtype=float),
    2: np.array([0.95, 0.84, 0.32, 0.42], dtype=float),
    3: np.array([0.38, 0.80, 0.50, 0.30], dtype=float),
    4: np.array([0.88, 0.36, 0.36, 0.42], dtype=float),
    5: np.array([0.10, 0.75, 0.80, 0.35], dtype=float),
}
TRANSPARENT_RGBA = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

# Pre-scaled uint8 RGBA colors for fast PyQtGraph ImageItem updates
STATE_RGBA_U8 = {
    code: np.round(rgba * 255).astype(np.uint8)
    for code, rgba in STATE_RGBA.items()
}
TRANSPARENT_RGBA_U8 = np.zeros(4, dtype=np.uint8)
