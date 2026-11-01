"""CSV telemetry recording manager."""

import csv
import os
import time
from typing import List, Optional

from constants import CAPTURE_DIR


class CsvRecorder:
    """Manages writing live telemetry streams to timestamped CSV files."""

    CSV_HEADER = ['Timestamp', 'ADC_value', 'CW_mV', 'Current_CV', 'CV_Threshold', 'Status_Code']

    def __init__(self, out_dir: str = CAPTURE_DIR):
        self.out_dir = out_dir
        self.is_recording: bool = False
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None

    def start(self) -> str:
        """Open a new timestamped CSV file and write the header row."""
        os.makedirs(self.out_dir, exist_ok=True)
        filename = os.path.join(self.out_dir, time.strftime("capture_%Y%m%d_%H%M%S") + ".csv")
        self._csv_file = open(filename, mode='w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_HEADER)
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
        """Write a batch of decoded telemetry samples if recording is active."""
        if not self.is_recording or self._csv_writer is None:
            return
        now = time.time()
        for rms, cw, cv, cv_thresh, status in zip(rms_list, cw_list, cv_list, cv_thresh_list, status_list):
            self._csv_writer.writerow([now, rms, cw, cv, cv_thresh, status])

    def stop(self) -> None:
        """Close the active CSV file if open."""
        self.is_recording = False
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            finally:
                self._csv_file = None
                self._csv_writer = None
