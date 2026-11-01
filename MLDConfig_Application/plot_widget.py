"""High-performance PyQtGraph telemetry plot widget and ring-buffer management."""

import os
import time
from typing import List
import numpy as np

from PySide6.QtCore import Qt, QRectF, QTimer
from PySide6.QtGui import QFont, QPainterPath
from PySide6.QtWidgets import QWidget, QLabel, QGraphicsPathItem
import pyqtgraph as pg

from constants import (
    CAPTURE_DIR,
    MAX_POINTS,
    STATE_RGBA_U8,
    STATUS_BADGE_COLOR,
    STATUS_LABELS,
    TRANSPARENT_RGBA_U8,
)

pg.setConfigOptions(antialias=False, background='#F2F3F4', foreground='#444444')


class TelemetryPlotCanvas(QWidget):
    """Dual subplot PyQtGraph widget"""

    AMP_MAX_MV = 3500.0
    DEFAULT_CV_MAX = 1.0
    LEFT_AXIS_WIDTH = 58
    LINE_WIDTH = 2.5
    MAX_IMAGE_TEX_WIDTH = 2048

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: #F2F3F4;")

        # ── Telemetry Ring Buffers ────────────────────────────────────────────
        self.data_x            = np.arange(MAX_POINTS, dtype=float)
        self.data_rms          = np.zeros(MAX_POINTS, dtype=float)
        self.data_cw           = np.zeros(MAX_POINTS, dtype=float)
        self.data_cv           = np.zeros(MAX_POINTS, dtype=float)
        self.data_cv_threshold = np.zeros(MAX_POINTS, dtype=float)
        self.state_rgba        = np.zeros((1, MAX_POINTS, 4), dtype=np.uint8)
        self._valid_count      = 0
        self._window_samples   = MAX_POINTS

        self.latest_status_code   = -1
        self._displayed_status    = -2
        self.latest_cv_raw        = np.nan
        self.latest_cv_thresh_raw = np.nan
        self.latest_cw_raw        = np.nan
        self._cv_max              = self.DEFAULT_CV_MAX

        self._init_ui()

    @property
    def is_ready(self) -> bool:
        return True

    def _init_ui(self) -> None:
        # ── 1. Top Subplot: Signal Magnitude ──────────────────────────────────
        self.plot_amp = pg.PlotWidget(self)
        self._style_plot_widget(
            self.plot_amp,
            title="Signal Magnitude",
            y_label="Magnitude (mV)",
            x_label=None,
            show_bottom_values=False,
        )
        self.plot_amp.setYRange(0, self.AMP_MAX_MV, padding=0)
        self.plot_amp.setXRange(0, MAX_POINTS - 1, padding=0)

        self.state_img1 = pg.ImageItem(axisOrder='row-major')
        self.state_img1.setZValue(-10)
        self._update_image_item(self.state_img1, self.AMP_MAX_MV)
        self.plot_amp.addItem(self.state_img1)

        self.legend_amp = self._add_styled_legend(self.plot_amp)
        self.line_rms = self.plot_amp.plot(
            pen=pg.mkPen('#1A5CB0', width=self.LINE_WIDTH),
            name='ML RMS',
        )
        self.line_cw = self.plot_amp.plot(
            pen=pg.mkPen('#1E9455', width=self.LINE_WIDTH),
            name='CW',
        )

        # ── 2. Bottom Subplot: Coefficient of Variation ───────────────────────
        self.plot_cv = pg.PlotWidget(self)
        self._style_plot_widget(
            self.plot_cv,
            title="Coefficient of Variation",
            y_label="CV (%)",
            x_label="Sample",
            show_bottom_values=True,
        )
        self.plot_cv.setYRange(0, self._cv_max, padding=0)
        self.plot_cv.setXRange(0, MAX_POINTS - 1, padding=0)

        self.state_img2 = pg.ImageItem(axisOrder='row-major')
        self.state_img2.setZValue(-10)
        self._update_image_item(self.state_img2, self._cv_max)
        self.plot_cv.addItem(self.state_img2)

        self.legend_cv = self._add_styled_legend(self.plot_cv)
        self.line_cv = self.plot_cv.plot(
            pen=pg.mkPen('#C0392B', width=self.LINE_WIDTH),
            name='CV',
        )
        threshold_pen = pg.mkPen('#D07000', width=self.LINE_WIDTH, style=Qt.CustomDashLine)
        threshold_pen.setDashPattern([5, 3])
        self.line_cv_threshold = self.plot_cv.plot(
            pen=threshold_pen,
            name='Threshold',
        )

        for curve in (self.line_rms, self.line_cw, self.line_cv, self.line_cv_threshold):
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method='peak')
            curve.opts['autoDownsampleFactor'] = 0.5

        # ── 3. HUD Text Overlays ──────────────────────────────────────────────
        self.lbl_rms = self._make_hud_label(self.plot_amp, 'ML RMS: — mV', '#1A5CB0', min_width=155)
        self.lbl_cw = self._make_hud_label(self.plot_amp, 'CW: — mV', '#1E9455', min_width=130)
        self.lbl_cv = self._make_hud_label(self.plot_cv, 'CV: — %', '#C0392B', min_width=125)
        self.lbl_cv_thresh = self._make_hud_label(self.plot_cv, 'Threshold: — %', '#D07000', min_width=160)

        self.lbl_status = QLabel('  —  ', self.plot_cv)
        self.lbl_status.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self._apply_status_badge_style('#888888')

        self.lbl_rec = QLabel('', self.plot_amp)
        self.lbl_rec.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.lbl_rec.setStyleSheet(
            "background-color: #FADADD; color: #A02828; font-weight: bold; "
            "font-size: 9pt; border-radius: 4px; padding: 2px 6px;"
        )
        self.lbl_rec.hide()

        self.lbl_toast = QLabel('', self.plot_amp)
        self.lbl_toast.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.lbl_toast.hide()

        self.plot_amp.getPlotItem().vb.sigResized.connect(self._reposition_overlays)
        self.plot_cv.getPlotItem().vb.sigResized.connect(self._reposition_overlays)

    def _style_plot_widget(
        self,
        pw: pg.PlotWidget,
        title: str,
        y_label: str,
        x_label: str | None,
        show_bottom_values: bool,
    ) -> None:
        pi = pw.getPlotItem()
        pi.setMenuEnabled(False)
        pi.hideButtons()

        vb = pi.getViewBox()
        vb.setMouseEnabled(x=False, y=False)
        vb.setBackgroundColor('#FFFFFF')
        vb.setBorder(pg.mkPen('#BBBBBB', width=1))

        pi.setTitle(f'<span style="color: #222222; font-size: 11pt; font-weight: 600;">{title}</span>')

        axis_pen = pg.mkPen('#BBBBBB', width=1)
        tick_font = QFont()
        tick_font.setPointSize(9)
        label_style = {'color': '#222222', 'font-size': '10pt'}

        left_axis = pi.getAxis('left')
        left_axis.setPen(axis_pen)
        left_axis.setTextPen('#666666')
        left_axis.setTickFont(tick_font)
        left_axis.setWidth(self.LEFT_AXIS_WIDTH)
        left_axis.setLabel(y_label, **label_style)

        bottom_axis = pi.getAxis('bottom')
        bottom_axis.setPen(axis_pen)
        bottom_axis.setTextPen('#666666')
        bottom_axis.setTickFont(tick_font)
        bottom_axis.setStyle(showValues=show_bottom_values)
        if x_label:
            bottom_axis.setLabel(x_label, **label_style)

        self._add_fast_grid(pi, vb)

    def _add_fast_grid(self, pi: pg.PlotItem, vb: pg.ViewBox) -> None:
        """Draw grid lines via a static QGraphicsPathItem inside the ViewBox (above background & status bands)."""
        grid_item = QGraphicsPathItem()
        grid_item.setPen(pg.mkPen(160, 160, 160, 120, width=1))
        grid_item.setZValue(-5)
        vb.addItem(grid_item, ignoreBounds=True)

        def update_grid() -> None:
            vr = vb.viewRect()
            if vr.width() <= 0 or vr.height() <= 0:
                return
            path = QPainterPath()

            x_levels = pi.getAxis('bottom').tickValues(vr.left(), vr.right(), max(100, int(vb.width())))
            x_ticks = []
            for _, ticks in x_levels[:2]:
                x_ticks.extend(ticks)
                if len(x_ticks) >= 6:
                    break
            for x in x_ticks:
                path.moveTo(x, vr.top())
                path.lineTo(x, vr.bottom())

            y_levels = pi.getAxis('left').tickValues(vr.top(), vr.bottom(), max(100, int(vb.height())))
            y_ticks = []
            for _, ticks in y_levels[:2]:
                y_ticks.extend(ticks)
                if len(y_ticks) >= 6:
                    break
            for y in y_ticks:
                path.moveTo(vr.left(), y)
                path.lineTo(vr.right(), y)

            grid_item.setPath(path)

        vb.sigRangeChanged.connect(lambda *_: update_grid())
        vb.sigResized.connect(update_grid)
        update_grid()

    def _add_styled_legend(self, pw: pg.PlotWidget) -> pg.LegendItem:
        legend = pw.addLegend(
            offset=(-10, 8),
            labelTextSize='9pt',
            labelTextColor='#222222',
            brush=pg.mkBrush(255, 255, 255, 215),
            pen=pg.mkPen('#CCCCCC', width=1),
        )
        legend.setZValue(20)
        return legend

    def _make_hud_label(self, parent: QWidget, text: str, color: str, min_width: int = 130) -> QLabel:
        lbl = QLabel(text, parent)
        lbl.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        lbl.setMinimumWidth(min_width)
        lbl.setStyleSheet(
            f"background-color: rgba(255, 255, 255, 210); color: {color}; "
            f"font-weight: bold; font-size: 10pt; border-radius: 4px; padding: 2px 6px;"
        )
        lbl.adjustSize()
        return lbl

    def _apply_status_badge_style(self, bg_color: str) -> None:
        self.lbl_status.setStyleSheet(
            f"background-color: {bg_color}; color: white; font-weight: bold; "
            f"font-size: 10.5pt; border-radius: 5px; padding: 3px 8px;"
        )
        self.lbl_status.adjustSize()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._layout_subplots()
        QTimer.singleShot(0, self._equalize_and_reposition)

    def _equalize_and_reposition(self) -> None:
        self._layout_subplots()
        self._reposition_overlays()

    def _layout_subplots(self) -> None:
        """Position both subplots so their inner white ViewBoxes have the exact same pixel height."""
        pad = 4
        spacing = 6
        w = max(50, self.width() - 2 * pad)
        avail_h = self.height() - 2 * pad - spacing
        if avail_h <= 60:
            return

        vb1_h = self.plot_amp.getPlotItem().vb.sceneBoundingRect().height()
        vb2_h = self.plot_cv.getPlotItem().vb.sceneBoundingRect().height()
        if vb1_h > 10 and vb2_h > 10 and self.plot_amp.height() > vb1_h and self.plot_cv.height() > vb2_h:
            overhead1 = self.plot_amp.height() - vb1_h
            overhead2 = self.plot_cv.height() - vb2_h
        else:
            overhead1 = 35.0
            overhead2 = 62.0

        target_vb_h = (avail_h - overhead1 - overhead2) / 2.0
        h1 = max(30, int(round(target_vb_h + overhead1)))
        h2 = max(30, avail_h - h1)

        self.plot_amp.setGeometry(pad, pad, w, h1)
        self.plot_cv.setGeometry(pad, pad + h1 + spacing, w, h2)

    def _reposition_overlays(self) -> None:
        # Position HUD labels inside the ViewBox area of each subplot
        vb1_rect = self.plot_amp.getPlotItem().vb.sceneBoundingRect()
        tl1 = self.plot_amp.mapFromScene(vb1_rect.topLeft())
        tr1 = self.plot_amp.mapFromScene(vb1_rect.topRight())

        x1 = int(tl1.x()) + 8
        y1 = int(tl1.y()) + 8
        self.lbl_rms.adjustSize()
        self.lbl_rms.move(x1, y1)

        self.lbl_cw.adjustSize()
        self.lbl_cw.move(x1, y1 + self.lbl_rms.height() + 4)

        if self.lbl_rec.isVisible():
            self.lbl_rec.adjustSize()
            self.lbl_rec.move(int(tr1.x()) - self.lbl_rec.width() - 8, y1)

        if self.lbl_toast.isVisible():
            self.lbl_toast.adjustSize()
            center_x = int((tl1.x() + tr1.x() - self.lbl_toast.width()) / 2)
            self.lbl_toast.move(center_x, y1)

        vb2_rect = self.plot_cv.getPlotItem().vb.sceneBoundingRect()
        tl2 = self.plot_cv.mapFromScene(vb2_rect.topLeft())
        bl2 = self.plot_cv.mapFromScene(vb2_rect.bottomLeft())

        x2 = int(tl2.x()) + 8
        y2 = int(tl2.y()) + 8
        self.lbl_cv.adjustSize()
        self.lbl_cv.move(x2, y2)

        self.lbl_cv_thresh.adjustSize()
        self.lbl_cv_thresh.move(x2, y2 + self.lbl_cv.height() + 4)

        self.lbl_status.adjustSize()
        self.lbl_status.move(x2, int(bl2.y()) - self.lbl_status.height() - 10)

    def _update_image_item(self, img_item: pg.ImageItem, y_max: float) -> None:
        x_start = max(0, MAX_POINTS - self._window_samples)
        visible_slice = self.state_rgba[:, x_start:MAX_POINTS, :]
        width = visible_slice.shape[1]
        if width > self.MAX_IMAGE_TEX_WIDTH:
            step = (width + self.MAX_IMAGE_TEX_WIDTH - 1) // self.MAX_IMAGE_TEX_WIDTH
            visible_slice = np.ascontiguousarray(visible_slice[:, ::step, :])
        img_item.setImage(visible_slice, autoLevels=False)
        img_item.setRect(QRectF(float(x_start), 0.0, float(MAX_POINTS - x_start), float(y_max)))

    # ── Compatibility No-ops & Overlays ───────────────────────────────────────
    def blit_artists(self) -> None:
        pass

    def set_recording_indicator(self, recording: bool) -> None:
        if recording:
            self.lbl_rec.setText("● REC")
            self.lbl_rec.show()
        else:
            self.lbl_rec.clear()
            self.lbl_rec.hide()
        self._reposition_overlays()

    def show_toast(self, msg: str, color: str) -> None:
        """Display a floating pill notification at the top-center of the plot area."""
        self.lbl_toast.setText(msg)
        self.lbl_toast.setStyleSheet(
            f"background-color: rgba(255, 255, 255, 238); color: {color}; "
            f"border: 1.5px solid {color}; font-weight: bold; font-size: 10pt; "
            f"border-radius: 5px; padding: 3px 10px;"
        )
        self.lbl_toast.show()
        self._reposition_overlays()

    def hide_toast(self) -> None:
        self.lbl_toast.clear()
        self.lbl_toast.hide()

    # ── Data & View Updates ───────────────────────────────────────────────────
    def _shift_state_buffer(self, k: int) -> None:
        if k <= 0:
            return
        self.state_rgba[:, :-k, :] = self.state_rgba[:, k:, :]
        self.state_rgba[:, -k:, :] = TRANSPARENT_RGBA_U8

    def _fill_new_status_colors(self, statuses: List[int]) -> None:
        if not statuses:
            return
        k = len(statuses)
        for i, status in enumerate(statuses):
            self.state_rgba[0, MAX_POINTS - k + i, :] = STATE_RGBA_U8.get(status, TRANSPARENT_RGBA_U8)
        self._update_image_item(self.state_img1, self.AMP_MAX_MV)
        self._update_image_item(self.state_img2, self._cv_max)

    def _refresh_curve_data(self) -> None:
        if self._valid_count <= 0:
            return
        start = max(MAX_POINTS - self._valid_count, MAX_POINTS - self._window_samples)
        xs = self.data_x[start:]
        self.line_rms.setData(xs, self.data_rms[start:], skipFiniteCheck=True)
        self.line_cw.setData(xs, self.data_cw[start:], skipFiniteCheck=True)
        self.line_cv.setData(xs, self.data_cv[start:], skipFiniteCheck=True)
        self.line_cv_threshold.setData(xs, self.data_cv_threshold[start:], skipFiniteCheck=True)

    def append_telemetry(
        self,
        new_rms: List[float],
        new_cw: List[float],
        new_cv: List[float],
        new_cv_threshold: List[float],
        new_status: List[int],
    ) -> None:
        """Shift rolling buffers, append new telemetry samples, and update plot curves."""
        k = len(new_rms)
        if k == 0:
            return

        self.latest_cv_raw        = new_cv[-1]
        self.latest_cv_thresh_raw = new_cv_threshold[-1]
        self.latest_cw_raw        = new_cw[-1]
        self.latest_status_code   = new_status[-1]

        if k > MAX_POINTS:
            k                = MAX_POINTS
            new_rms          = new_rms[-k:]
            new_cw           = new_cw[-k:]
            new_cv           = new_cv[-k:]
            new_cv_threshold = new_cv_threshold[-k:]
            new_status       = new_status[-k:]

        self.data_rms[:-k]          = self.data_rms[k:]
        self.data_cw[:-k]           = self.data_cw[k:]
        self.data_cv[:-k]           = self.data_cv[k:]
        self.data_cv_threshold[:-k] = self.data_cv_threshold[k:]

        self._shift_state_buffer(k)

        self.data_rms[-k:]          = new_rms
        self.data_cw[-k:]           = new_cw
        self.data_cv[-k:]           = new_cv
        self.data_cv_threshold[-k:] = new_cv_threshold
        self._valid_count           = min(MAX_POINTS, self._valid_count + k)

        self._refresh_curve_data()
        self._fill_new_status_colors(new_status)

        self.lbl_rms.setText(f"ML RMS: {self.data_rms[-1]:.1f} mV")
        self.lbl_cw.setText(f"CW: {self.latest_cw_raw:.1f} mV")
        self.lbl_cv.setText(f"CV: {self.latest_cv_raw:.3f} %")
        self.lbl_cv_thresh.setText(f"Threshold: {self.latest_cv_thresh_raw:.3f} %")

        if self.latest_status_code != self._displayed_status:
            self._displayed_status = self.latest_status_code
            status_name = STATUS_LABELS.get(self.latest_status_code, '—')
            badge_color = STATUS_BADGE_COLOR.get(self.latest_status_code, '#888888')
            self.lbl_status.setText(f'  {status_name}  ')
            self._apply_status_badge_style(badge_color)
            vb2_rect = self.plot_cv.getPlotItem().vb.sceneBoundingRect()
            bl2 = self.plot_cv.mapFromScene(vb2_rect.bottomLeft())
            self.lbl_status.move(int(bl2.x()) + 8, int(bl2.y()) - self.lbl_status.height() - 10)

    def set_sample_window(self, window_samples: int) -> None:
        self._window_samples = max(1, min(MAX_POINTS, int(window_samples)))
        x_min = MAX_POINTS - self._window_samples
        x_max = MAX_POINTS - 1
        self.plot_amp.setXRange(x_min, x_max, padding=0)
        self.plot_cv.setXRange(x_min, x_max, padding=0)
        self._refresh_curve_data()
        self._update_image_item(self.state_img1, self.AMP_MAX_MV)
        self._update_image_item(self.state_img2, self._cv_max)

    def set_cv_max(self, val: float) -> None:
        if val > 0:
            self._cv_max = val
            self.plot_cv.setYRange(0, val, padding=0)
            self._update_image_item(self.state_img2, self._cv_max)

    def save_snapshot(self, out_dir: str = CAPTURE_DIR) -> str:
        """Save the current plot widget (including HUD overlays) to a timestamped PNG."""
        os.makedirs(out_dir, exist_ok=True)
        self.set_recording_indicator(False)
        self.hide_toast()
        plot_filename = os.path.join(out_dir, time.strftime("capture_%Y%m%d_%H%M%S") + ".png")
        pixmap = self.grab()
        pixmap.save(plot_filename, "PNG")
        return plot_filename
