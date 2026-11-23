"""CSV telemetry recording manager."""

import csv
import os
import time
from typing import Dict, List, Optional

from constants import CAPTURE_DIR, STATUS_LABELS


class CsvRecorder:
    """Manages writing live telemetry streams to timestamped CSV files."""

    CSV_HEADER = [
        'Sample',
        'Time_s',
        'ML_RMS_mV',
        'CW_mV',
        'Current_CV',
        'CV_Threshold',
        'Status',
    ]

    def __init__(self, out_dir: str = CAPTURE_DIR):
        self.out_dir = out_dir
        self.is_recording: bool = False
        self.current_session_dir: str = ""
        self.current_base_name: str = ""
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None
        self._sample_index: int = 0
        self._start_time: float = 0.0
        self._last_batch_time: float = 0.0

    def start(self, metadata: Optional[Dict[str, str]] = None) -> str:
        """Create a timestamped capture folder, open the CSV file, and write metadata + header row."""
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.current_base_name = f"capture_{timestamp_str}"
        self.current_session_dir = os.path.join(self.out_dir, self.current_base_name)
        os.makedirs(self.current_session_dir, exist_ok=True)

        filename = os.path.join(self.current_session_dir, f"{self.current_base_name}.csv")
        self._csv_file = open(filename, mode='w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)

        if metadata:
            meta_parts = [f"{k}={v}" for k, v in metadata.items()]
            self._csv_file.write(f"# {', '.join(meta_parts)}\n")

        self._csv_writer.writerow(self.CSV_HEADER)
        self._sample_index = 0
        now = time.perf_counter()
        self._start_time = now
        self._last_batch_time = now
        self.is_recording = True
        return filename

    def write_batch(
        self,
        rms_list: List[float],
        cw_list: List[float],
        cv_list: List[float],
        cv_thresh_list: List[float],
        status_list: List[int],
    ) -> None:
        """Write a batch of decoded telemetry samples with interpolated per-sample timestamps."""
        if not self.is_recording or self._csv_writer is None:
            return

        count = len(rms_list)
        if count == 0:
            return

        now = time.perf_counter()
        batch_start = self._last_batch_time
        dt = (now - batch_start) / count if now > batch_start else 0.001
        self._last_batch_time = now

        rows = []
        base_idx = self._sample_index
        t0 = self._start_time

        for i, (rms, cw, cv, cv_thresh, status) in enumerate(
            zip(rms_list, cw_list, cv_list, cv_thresh_list, status_list)
        ):
            sample_time = (batch_start - t0) + (i + 1) * dt
            rows.append([
                base_idx + i,
                f"{sample_time:.4f}",
                f"{rms:.2f}",
                f"{cw:.2f}",
                f"{cv:.6f}",
                f"{cv_thresh:.6f}",
                STATUS_LABELS.get(status, "UNKNOWN"),
            ])

        self._csv_writer.writerows(rows)
        self._sample_index += count

    def stop(self) -> None:
        """Close the active CSV file if open."""
        self.is_recording = False
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            finally:
                self._csv_file = None
                self._csv_writer = None
