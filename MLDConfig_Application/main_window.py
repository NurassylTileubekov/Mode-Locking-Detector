"""Main Qt application window for the MLD Configuration Tool."""

import os
import struct
import time
import serial

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QLineEdit, QGroupBox, QMessageBox, QSpinBox,
    QFileDialog,
)
from PySide6.QtCore import QTimer, Qt, QSettings

from constants import (
    BAUDRATE,
    CAPTURE_DIR,
    MAX_POINTS,
    PLOT_INTERVAL_MS,
    PORT_SCAN_MS,
    RESET_TO_DEFAULT_BIT,
    REVERT_TO_PREV_BIT,
    SERIAL_TIMEOUT_SEC,
    START_ACQUIRING_BIT,
    STOP_ACQUIRING_BIT,
)
from protocol import (
    list_valid_serial_ports,
    pack_config_packet,
    parse_byte_buffer,
)
from plot_widget import TelemetryPlotCanvas
from recorder import CsvRecorder


class MLDConfigWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MLD Configuration Tool")
        self.resize(1000, 700)
        self.setMinimumSize(1000, 700)

        # ── Runtime & Communication State ─────────────────────────────────────
        self.settings = QSettings("MLD", "MLDConfigTool")
        saved_dir = self.settings.value("capture_dir", "", type=str)
        self.ser = None
        self.byte_buffer = bytearray()
        self.recorder = CsvRecorder(out_dir=saved_dir if saved_dir else CAPTURE_DIR)
        self.last_mcu_config_received = None
        self.last_rx_time = 0.0

        # ── Build UI ──────────────────────────────────────────────────────────
        self._init_ui()
        self._set_device_controls_enabled(False)
        self.refresh_ports()

        # ── Timers ────────────────────────────────────────────────────────────
        self.render_timer = QTimer(self)
        self.render_timer.setInterval(PLOT_INTERVAL_MS)
        self.render_timer.timeout.connect(self.render_frame)
        self.render_timer.start()

        self.status_clear_timer = QTimer(self)
        self.status_clear_timer.setSingleShot(True)
        self.status_clear_timer.timeout.connect(self.canvas.hide_toast)

        self.port_scan_timer = QTimer(self)
        self.port_scan_timer.setInterval(PORT_SCAN_MS)
        self.port_scan_timer.timeout.connect(self.refresh_ports)
        self.port_scan_timer.start()

    # ── UI Construction ───────────────────────────────────────────────────────
    def _init_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # 1. Top Row: Connection (stretches to fill left) & Display Settings (compact on right)
        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)
        top_layout.addWidget(self._build_connection_group(), stretch=1)
        top_layout.addWidget(self._build_display_group())
        main_layout.addLayout(top_layout)

        # 2. Middle Row: PyQtGraph Telemetry Canvas
        self.canvas = TelemetryPlotCanvas()
        main_layout.addWidget(self.canvas, stretch=1)

        # 3. Bottom Row: Configurable Device Settings & Calibration
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(10)
        bottom_layout.addWidget(self._build_device_settings_group(), stretch=1)
        bottom_layout.addWidget(self._build_calibration_group(), stretch=1)
        main_layout.addLayout(bottom_layout)

    def _build_connection_group(self) -> QGroupBox:
        conn_group = QGroupBox("Serial Connection")
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setContentsMargins(10, 6, 10, 6)

        conn_layout.addWidget(QLabel("Port:"))
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(180)
        conn_layout.addWidget(self.combo_ports, stretch=1)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setMinimumWidth(80)
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect)
        return conn_group

    def _build_display_group(self) -> QGroupBox:
        disp_group = QGroupBox("Display and Recording")
        disp_layout = QHBoxLayout(disp_group)
        disp_layout.setContentsMargins(10, 6, 10, 6)

        disp_layout.addWidget(QLabel("CV max:"))
        self.txt_cv_max = QLineEdit("1.0")
        self.txt_cv_max.setFixedWidth(45)
        self.txt_cv_max.returnPressed.connect(self.submit_cv_max)
        self.txt_cv_max.editingFinished.connect(self.submit_cv_max)
        disp_layout.addWidget(self.txt_cv_max)

        disp_layout.addSpacing(8)
        disp_layout.addWidget(QLabel("Samples max:"))
        self.spin_window = QSpinBox()
        self.spin_window.setRange(1000, MAX_POINTS)
        self.spin_window.setSingleStep(1000)
        self.spin_window.setValue(MAX_POINTS)
        self.spin_window.setMinimumWidth(90)
        self.spin_window.valueChanged.connect(self.update_plot_window)
        disp_layout.addWidget(self.spin_window)

        disp_layout.addSpacing(8)
        self.btn_folder = QPushButton("📁 Folder")
        self.btn_folder.setToolTip(f"Save folder: {os.path.abspath(self.recorder.out_dir)}")
        self.btn_folder.clicked.connect(self.select_capture_folder)
        disp_layout.addWidget(self.btn_folder)

        self.btn_capture = QPushButton("⏺ Record")
        self.btn_capture.clicked.connect(self.toggle_capture)
        disp_layout.addWidget(self.btn_capture)
        return disp_group

    def _build_device_settings_group(self) -> QGroupBox:
        config_group = QGroupBox("Device Settings")
        config_layout = QHBoxLayout(config_group)
        config_layout.setContentsMargins(10, 6, 10, 6)

        config_layout.addWidget(QLabel("Window (ms):"))
        self.txt_t_win = QLineEdit("1")
        self.txt_t_win.setFixedWidth(50)
        config_layout.addWidget(self.txt_t_win)

        config_layout.addWidget(QLabel("CV Offset:"))
        self.txt_t_offset = QLineEdit("0.10")
        self.txt_t_offset.setFixedWidth(50)
        config_layout.addWidget(self.txt_t_offset)

        config_layout.addWidget(QLabel("Sat. Limit:"))
        self.txt_sat_pct = QLineEdit("0.80")
        self.txt_sat_pct.setFixedWidth(50)
        config_layout.addWidget(self.txt_sat_pct)

        self.btn_send = QPushButton("↑ Send Config")
        self.btn_send.clicked.connect(self.send_config)
        config_layout.addWidget(self.btn_send)

        config_layout.addStretch()
        return config_group

    def _build_calibration_group(self) -> QGroupBox:
        calib_group = QGroupBox("Calibration")
        calib_layout = QHBoxLayout(calib_group)
        calib_layout.setContentsMargins(10, 6, 10, 6)

        self.btn_reset = QPushButton("Reset")
        self.btn_reset.setMinimumWidth(55)
        self.btn_reset.clicked.connect(self.send_reset)
        calib_layout.addWidget(self.btn_reset)

        self.btn_revert = QPushButton("Revert")
        self.btn_revert.setMinimumWidth(55)
        self.btn_revert.setToolTip("Revert to the previous calibration")
        self.btn_revert.clicked.connect(self.send_revert)
        calib_layout.addWidget(self.btn_revert)

        self.btn_calib = QPushButton("Start calibration")
        self.btn_calib.setCheckable(True)
        self.btn_calib.setMinimumWidth(110)
        self.btn_calib.clicked.connect(self.toggle_calibration)
        calib_layout.addWidget(self.btn_calib)

        calib_layout.addSpacing(15)
        self.lbl_mse = QLabel("MSE: —")
        self.lbl_mse.setStyleSheet("font-weight: bold; font-family: monospace; font-size: 11px; color: #FFA500;")
        calib_layout.addWidget(self.lbl_mse)

        calib_layout.addSpacing(10)
        self.lbl_r_squared = QLabel("R²: —")
        self.lbl_r_squared.setStyleSheet("font-weight: bold; font-family: monospace; font-size: 11px; color: #1A5CB0;")
        calib_layout.addWidget(self.lbl_r_squared)

        calib_layout.addStretch()
        return calib_group

    # ── Serial Port Management ────────────────────────────────────────────────
    def refresh_ports(self) -> None:
        if self.ser is not None and self.ser.is_open:
            return

        valid_ports = list_valid_serial_ports()
        current_items = [
            (self.combo_ports.itemText(i), self.combo_ports.itemData(i))
            for i in range(self.combo_ports.count())
        ]
        if valid_ports == current_items:
            return

        current_selected = self.combo_ports.currentData()
        self.combo_ports.blockSignals(True)
        self.combo_ports.clear()

        preferred_idx = 0
        for i, (label, device) in enumerate(valid_ports):
            self.combo_ports.addItem(label, device)
            if device == current_selected or any(k in device for k in ('ttyACM', 'ttyUSB', 'COM')):
                if current_selected == device or preferred_idx == 0:
                    preferred_idx = i

        if valid_ports:
            self.combo_ports.setCurrentIndex(preferred_idx)
            self.combo_ports.setEnabled(True)
            self.btn_connect.setEnabled(True)
        else:
            self.combo_ports.setEnabled(False)
            self.btn_connect.setEnabled(False)

        self.combo_ports.blockSignals(False)

    def toggle_connection(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self) -> None:
        port = self.combo_ports.currentData()
        if not port:
            port = self.combo_ports.currentText().split('—')[0].strip()
        if not port:
            self._flash_feedback("⚠ No port selected", "#A02828")
            return

        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0)
            self.byte_buffer.clear()
            self.last_rx_time = time.monotonic()
            self.btn_connect.setText("Disconnect")
            self.combo_ports.setEnabled(False)
            self._set_device_controls_enabled(True)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self._flash_feedback(f"Error: {e}", "#A02828")

    def disconnect_serial(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if self.recorder.is_recording:
            self.toggle_capture()
        if hasattr(self, 'btn_calib') and self.btn_calib.isChecked():
            self.btn_calib.blockSignals(True)
            self.btn_calib.setChecked(False)
            self.btn_calib.setText("Start calibration")
            self.btn_calib.blockSignals(False)
            self._set_ui_blocked_for_calibration(False)
        self._set_device_controls_enabled(False)
        self.btn_connect.setText("Connect")
        self.refresh_ports()
        self.combo_ports.setEnabled(self.combo_ports.count() > 0)

    # ── Telemetry Processing & Render Loop ────────────────────────────────────
    def render_frame(self) -> None:
        if not self.canvas.is_ready or self.ser is None or not self.ser.is_open:
            return

        try:
            self._render_inner()
            if time.monotonic() - self.last_rx_time > SERIAL_TIMEOUT_SEC:
                self.disconnect_serial()
                self._flash_feedback("⚠ Connection lost (timeout > 3s)", "#A02828")
        except (serial.SerialException, OSError) as e:
            self.disconnect_serial()
            self._flash_feedback(f"Connection lost: {e}", "#A02828")

    def _render_inner(self) -> None:
        waiting = self.ser.in_waiting
        if waiting > 0:
            self.byte_buffer.extend(self.ser.read(waiting))

        parsed = parse_byte_buffer(self.byte_buffer)
        if parsed.rms or parsed.latest_config is not None or parsed.latest_error is not None:
            self.last_rx_time = time.monotonic()

        if parsed.latest_error is not None:
            mse, r_squared = parsed.latest_error
            self.lbl_mse.setText(f"MSE: {mse:.4e}" if abs(mse) < 1e-2 else f"MSE: {mse:.4f}")
            self.lbl_r_squared.setText(f"R²: {r_squared:.4f}")

        if parsed.latest_config is not None:
            if parsed.latest_config != self.last_mcu_config_received:
                self.last_mcu_config_received = parsed.latest_config
                _, t_win, t_offset, sat_pct = parsed.latest_config
                self.txt_t_win.setText(str(t_win))
                self.txt_t_offset.setText(f"{t_offset:.2f}")
                self.txt_sat_pct.setText(f"{sat_pct:.2f}")

        if not parsed.rms:
            return

        self.recorder.write_batch(
            parsed.rms, parsed.cw, parsed.cv, parsed.cv_threshold, parsed.status
        )
        self.canvas.append_telemetry(
            parsed.rms, parsed.cw, parsed.cv, parsed.cv_threshold, parsed.status
        )

    # ── UI Control Callbacks ──────────────────────────────────────────────────
    def update_plot_window(self, val: int) -> None:
        self.canvas.set_sample_window(val)

    def submit_cv_max(self) -> None:
        try:
            val = float(self.txt_cv_max.text())
            self.canvas.set_cv_max(val)
        except ValueError:
            pass

    def _flash_feedback(self, msg: str, color: str) -> None:
        self.canvas.show_toast(msg, color)
        self.status_clear_timer.start(2500)

    def _send_command_packet(self, ctrl_byte: int) -> bool:
        if self.ser is None or not self.ser.is_open:
            self._flash_feedback("⚠ No serial port open", "#A02828")
            return False
        try:
            t_win    = int(self.txt_t_win.text())
            t_offset = float(self.txt_t_offset.text())
            sat_pct  = float(self.txt_sat_pct.text())
        except ValueError:
            self._flash_feedback("⚠ Check inputs", "#A02828")
            return False

        try:
            frame = pack_config_packet(ctrl_byte, t_win, t_offset, sat_pct)
            self.ser.write(frame)
            self._flash_feedback(f"✓ Sent ctrl={ctrl_byte} win={t_win} offset={t_offset} sat={sat_pct}", "#1E7A34")
            return True
        except (struct.error, serial.SerialException):
            self._flash_feedback("⚠ Send failed", "#A02828")
            return False

    def send_config(self) -> None:
        self._send_command_packet(ctrl_byte=0)

    def send_reset(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm Reset',
            'Are you sure you want to reset to default parameters?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._send_command_packet(ctrl_byte=RESET_TO_DEFAULT_BIT)

    def send_revert(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm Revert',
            'Are you sure you want to revert to the previous calibration?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._send_command_packet(ctrl_byte=REVERT_TO_PREV_BIT)

    def toggle_calibration(self) -> None:
        is_starting = self.btn_calib.isChecked()

        if is_starting:
            reply = QMessageBox.question(
                self, 'Confirm Calibration',
                'Are you sure you want to start calibration?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.btn_calib.blockSignals(True)
                self.btn_calib.setChecked(False)
                self.btn_calib.blockSignals(False)
                return

        ctrl_byte = START_ACQUIRING_BIT if is_starting else STOP_ACQUIRING_BIT
        success = self._send_command_packet(ctrl_byte)

        if success:
            self.btn_calib.setText("Stop calibration" if is_starting else "Start calibration")
            self._set_ui_blocked_for_calibration(is_starting)
        else:
            self.btn_calib.blockSignals(True)
            self.btn_calib.setChecked(not is_starting)
            self.btn_calib.blockSignals(False)

    def _set_device_controls_enabled(self, enabled: bool) -> None:
        """Enable or disable device-dependent buttons and numeric inputs (leaving UI controls active)."""
        self.txt_t_win.setEnabled(enabled)
        self.txt_t_offset.setEnabled(enabled)
        self.txt_sat_pct.setEnabled(enabled)
        self.btn_send.setEnabled(enabled)
        self.btn_reset.setEnabled(enabled)
        self.btn_revert.setEnabled(enabled)
        self.btn_calib.setEnabled(enabled)
        self.btn_capture.setEnabled(enabled)

    def _set_ui_blocked_for_calibration(self, blocked: bool) -> None:
        self.btn_reset.setDisabled(blocked)
        self.btn_revert.setDisabled(blocked)
        self.btn_send.setDisabled(blocked)
        self.btn_folder.setDisabled(blocked)
        self.btn_capture.setDisabled(blocked)
        self.txt_t_win.setDisabled(blocked)
        self.txt_t_offset.setDisabled(blocked)
        self.txt_sat_pct.setDisabled(blocked)

    def select_capture_folder(self) -> None:
        initial_dir = os.path.abspath(self.recorder.out_dir)
        os.makedirs(initial_dir, exist_ok=True)
        selected_dir = QFileDialog.getExistingDirectory(
            self,
            "Select Folder for Recorded Data",
            initial_dir,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected_dir:
            self.recorder.out_dir = selected_dir
            self.settings.setValue("capture_dir", selected_dir)
            self.settings.sync()
            self.btn_folder.setToolTip(f"Save folder: {selected_dir}")
            folder_name = os.path.basename(selected_dir.rstrip(os.sep)) or selected_dir
            self._flash_feedback(f"📁 Folder: {folder_name}", "#1E7A34")

    def toggle_capture(self) -> None:
        if not self.recorder.is_recording:
            self.recorder.start()
            self.btn_capture.setText('⏹ Stop')
            self.btn_folder.setDisabled(True)
            self.canvas.set_recording_indicator(True)
        else:
            self.recorder.stop()
            plot_filename = self.canvas.save_snapshot(self.recorder.out_dir)
            self.btn_capture.setText('⏺ Record')
            self.btn_folder.setDisabled(False)
            self._flash_feedback(f"Saved: {os.path.basename(plot_filename)}", "#1E7A34")

    def closeEvent(self, event) -> None:
        self.render_timer.stop()
        self.port_scan_timer.stop()

        if hasattr(self, 'btn_calib') and self.btn_calib.isChecked():
            self._send_command_packet(ctrl_byte=STOP_ACQUIRING_BIT)
            if self.ser and self.ser.is_open:
                self.ser.flush()
                time.sleep(0.05)

        self.recorder.stop()
        self.disconnect_serial()
        super().closeEvent(event)
