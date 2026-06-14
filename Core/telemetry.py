import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, TextBox
import sys
import time
import csv
import os
import glob
import re

try:
    import scipy.optimize as opt
except ImportError:
    print("Scipy not installed. Curve fitting will not work. Run: pip install scipy")
    opt = None

# --- Configuration ---
BAUDRATE = 460800
MAX_POINTS = 2000
PLOT_INTERVAL_MS = 20
PLOT_DOWNSAMPLE = 10
MAX_PACKETS_PER_UPDATE = 500
PIXEL_X_MIN = -0.5
PIXEL_X_MAX = MAX_POINTS - 0.5

def find_stm32_port():
    preferred_ports = ['/dev/ttyACM0', '/dev/ttyACM1']
    preferred_ports += glob.glob('/dev/ttyUSB*')
    for port in preferred_ports:
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=0.1)
            ser.close()
            return port
        except:
            pass
    return None

PORT = find_stm32_port()
if not PORT:
    print("Warning: No STM32 device found.")
    PORT = "/dev/ttyACM0"

FRAME_START = 0xAA
FRAME_END = 0xBB

FRAME_LAYOUTS = (
    (struct.Struct('<fffb'), 15),
    (struct.Struct('<fffi'), 18),
    (struct.Struct('<ffffB'), 19),
    (struct.Struct('<ffffi'), 22),
    (struct.Struct('<Hfffff'), 24),
    (struct.Struct('<ifffff'), 26),
)

TX_FRAME_STRUCT = struct.Struct('<BifffffB')

STATUS_LABELS = {
    0: 'NO_SIGNAL',
    1: 'CW',
    2: 'NOT_MODE_LOCKED',
    3: 'MODE_LOCKED',
    4: 'SATURATED',
}

STATE_RGBA = {
    0: np.array([255 / 255, 0 / 255, 255 / 255, 0.50], dtype=float), # Magenta
    1: np.array([122 / 255, 122 / 255, 122 / 255, 0.50], dtype=float), # Grey
    2: np.array([255 / 255, 255 / 255, 0 / 255, 0.50], dtype=float), # Yellow
    3: np.array([0.0, 200 / 255, 83 / 255, 0.50], dtype=float),
    4: np.array([255 / 255, 0.0, 0.0, 0.50], dtype=float), # Red
}
RECORD_RGBA = np.array([1.0, 23 / 255, 68 / 255, 0.35], dtype=float)
TRANSPARENT_RGBA = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

data_x = np.arange(MAX_POINTS)
data_rms = np.full(MAX_POINTS, np.nan)
data_cw = np.full(MAX_POINTS, np.nan)
data_cv = np.full(MAX_POINTS, np.nan)
data_cv_threshold = np.full(MAX_POINTS, np.nan)
data_cv_threshold = np.full(MAX_POINTS, np.nan)

byte_buffer = bytearray()
is_recording = False
csv_file = None
csv_writer = None

# Curve Fit State
is_fitting = False
fit_adc_data = []
fit_cv_data = []

ser = None
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"Connected to {PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9))
fig.canvas.manager.set_window_title('Laser Real-Time Telemetry & Config')
text_config = fig.text(0.5, 0.95, 'MCU Config: --', ha='center', fontsize=11, fontweight='bold', bbox=dict(facecolor='white', alpha=0.7))

line_rms, = ax1.plot(data_x, data_rms, lw=1.5, color='blue', label='ADC mean')
line_cw, = ax1.plot(data_x, data_cw, lw=1.5, color='green', label='CW mV')
ax1.set_ylim(0, 4095)  
ax1.set_xlim(0, MAX_POINTS - 1)
ax1.set_ylabel('ADC mean value / mV')
ax1.set_title('Average ADC value & CW mV (Linear Scale)')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.7)

line_cv, = ax2.plot(data_x, data_cv, lw=1.5, color='red', label='CV')
line_cv_threshold, = ax2.plot(data_x, data_cv_threshold, lw=1.5, color='orange', label='CV Threshold')
line_cv_threshold, = ax2.plot(data_x, data_cv_threshold, lw=1.5, color='orange', label='CV Threshold')
ax2.legend(loc='upper right')
ax2.set_ylim(0, 10.0) 
ax2.set_xlim(0, MAX_POINTS - 1)
ax2.set_ylabel('Current CV (%)')
ax2.set_title('Current Coefficient of Variation (Linear Scale)')
ax2.grid(True, which='both', linestyle='--', alpha=0.7)

state_rgba = np.zeros((1, MAX_POINTS, 4), dtype=float)
record_rgba = np.zeros((1, MAX_POINTS, 4), dtype=float)

state_img1 = ax1.imshow(state_rgba, extent=(PIXEL_X_MIN, PIXEL_X_MAX, 0, 1), transform=ax1.get_xaxis_transform(), aspect='auto', interpolation='none', resample=False, origin='lower', zorder=0)
state_img2 = ax2.imshow(state_rgba, extent=(PIXEL_X_MIN, PIXEL_X_MAX, 0, 1), transform=ax2.get_xaxis_transform(), aspect='auto', interpolation='none', resample=False, origin='lower', zorder=0)
record_img1 = ax1.imshow(record_rgba, extent=(PIXEL_X_MIN, PIXEL_X_MAX, 0.97, 1.0), transform=ax1.get_xaxis_transform(), aspect='auto', interpolation='none', resample=False, origin='lower', zorder=0.1)
record_img2 = ax2.imshow(record_rgba, extent=(PIXEL_X_MIN, PIXEL_X_MAX, 0.97, 1.0), transform=ax2.get_xaxis_transform(), aspect='auto', interpolation='none', resample=False, origin='lower', zorder=0.1)
OVERLAY_IMAGES = (state_img1, state_img2, record_img1, record_img2)

text_rms = ax1.text(0.02, 0.85, 'ADC: --', transform=ax1.transAxes, fontsize=12, fontweight='bold', color='darkblue', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
text_cw = ax1.text(0.20, 0.85, 'CW: --', transform=ax1.transAxes, fontsize=12, fontweight='bold', color='darkgreen', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
text_cv = ax2.text(0.02, 0.85, 'CV: --', transform=ax2.transAxes, fontsize=12, fontweight='bold', color='darkred', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
text_cv_thresh = ax2.text(0.02, 0.72, 'CV Threshold: --', transform=ax2.transAxes, fontsize=12, fontweight='bold', color='darkorange', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
text_status = ax2.text(0.02, 0.59, 'Status: --', transform=ax2.transAxes, fontsize=11, fontweight='bold', color='black', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

serial_frame_count = 0
latest_status_code = -1
latest_cv_raw = np.nan
latest_cv_thresh_raw = np.nan
latest_cw_raw = np.nan
last_mcu_config_received = None

def _apply_recording_mode(recording):
    record_rgba[:] = RECORD_RGBA if recording else TRANSPARENT_RGBA
    record_img1.set_data(record_rgba)
    record_img2.set_data(record_rgba)

def _shift_state_buffers(k):
    if k <= 0: return
    state_rgba[:, :-k, :] = state_rgba[:, k:, :]
    state_rgba[:, -k:, :] = TRANSPARENT_RGBA
    record_rgba[:, :-k, :] = record_rgba[:, k:, :]
    record_rgba[:, -k:, :] = RECORD_RGBA if is_recording else TRANSPARENT_RGBA

def _fill_new_status_colors(statuses):
    if not statuses: return
    k = len(statuses)
    for i, status in enumerate(statuses):
        state_rgba[0, MAX_POINTS - k + i, :] = STATE_RGBA.get(status, TRANSPARENT_RGBA)
    state_img1.set_data(state_rgba)
    state_img2.set_data(state_rgba)
    record_img1.set_data(record_rgba)
    record_img2.set_data(record_rgba)

def update(frame):
    global byte_buffer, data_rms, data_cw, data_cv, data_cv_threshold, serial_frame_count, is_recording, csv_writer, latest_status_code, latest_cv_raw, latest_cv_thresh_raw, latest_cw_raw, last_mcu_config_received
    global is_fitting, fit_adc_data, fit_cv_data

    if ser is None or not ser.is_open:
        return []

    if ser.in_waiting > 0:
        byte_buffer.extend(ser.read(ser.in_waiting))
    
    idx = 0
    packets_processed = 0
    new_rms, new_cw, new_cv, new_cv_threshold, new_status = [], [], [], [], []
    
    while idx < len(byte_buffer):
        if packets_processed >= MAX_PACKETS_PER_UPDATE: break
        if byte_buffer[idx] != FRAME_START:
            idx += 1
            continue

        parsed = False
        for payload_struct, frame_size in FRAME_LAYOUTS:
            if idx + frame_size > len(byte_buffer):
                continue
            if byte_buffer[idx + frame_size - 1] != FRAME_END:
                continue
            try:
                values = payload_struct.unpack_from(byte_buffer, idx + 1)
                
                if frame_size == 24 or frame_size == 26:
                    t_win, p_low, p_high, c_low, c_high, t_offset = values
                    current_config_tuple = (t_win, p_low, p_high, c_low, c_high, t_offset)
                    if current_config_tuple != last_mcu_config_received:
                        last_mcu_config_received = current_config_tuple
                        if 'txt_t_win' in globals():
                            txt_t_win.set_val(str(t_win))
                            txt_p_low.set_val(f"{p_low:.1f}")
                            txt_p_high.set_val(f"{p_high:.1f}")
                            txt_c_low.set_val(f"{c_low:.1f}")
                            txt_c_high.set_val(f"{c_high:.1f}")
                            txt_t_offset.set_val(f"{t_offset:.2f}")
                    idx += frame_size
                    packets_processed += 1
                    parsed = True
                    break

                if frame_size == 22 or frame_size == 19:
                    rms, cw, cv, cv_threshold, status_code = values
                elif frame_size == 18:
                    rms, cv, cv_threshold, status_code = values
                    cw = np.nan
                elif frame_size == 15:
                    rms, cv, cv_threshold, status_code = values
                    cw = np.nan
                else:
                    continue

                if is_recording and csv_writer is not None:
                    csv_writer.writerow([time.time(), rms, cw, cv, cv_threshold, status_code])

                if is_fitting:
                    # Save every point for better fitting resolution
                    fit_adc_data.append(rms)
                    fit_cv_data.append(cv)

                serial_frame_count += 1
                if serial_frame_count % PLOT_DOWNSAMPLE == 0:
                    new_rms.append(rms)
                    new_cw.append(cw)
                    latest_cv_raw = cv
                    latest_cv_thresh_raw = cv_threshold
                    latest_cw_raw = cw
                    new_cv.append(cv) 
                    new_cv_threshold.append(cv_threshold)
                    new_status.append(status_code)
                    latest_status_code = status_code

                idx += frame_size
                packets_processed += 1
                parsed = True
                break
            except struct.error:
                continue

        if not parsed:
            idx += 1
            
    del byte_buffer[:idx]
    
    k = len(new_rms)
    if k == 0: return []

    if k > MAX_POINTS: 
        k = MAX_POINTS
        new_rms = new_rms[-k:]
        new_cw = new_cw[-k:]
        new_cv = new_cv[-k:]
        new_cv_threshold = new_cv_threshold[-k:]
        new_status = new_status[-k:]

    data_rms[:-k] = data_rms[k:]
    data_cw[:-k] = data_cw[k:]
    data_cv[:-k] = data_cv[k:]
    data_cv_threshold[:-k] = data_cv_threshold[k:]

    _shift_state_buffers(k)
    
    data_rms[-k:] = new_rms
    data_cw[-k:] = new_cw
    data_cv[-k:] = new_cv
    data_cv_threshold[-k:] = new_cv_threshold

    line_rms.set_ydata(data_rms)
    line_cw.set_ydata(data_cw)
    _fill_new_status_colors(new_status)
    line_cv.set_ydata(data_cv)
    line_cv_threshold.set_ydata(data_cv_threshold)

    if not np.isnan(data_rms[-1]):
        text_rms.set_text(f"ADC: {data_rms[-1]:.5f}")
        text_cw.set_text(f"CW: {latest_cw_raw:.5f}")
        text_cv.set_text(f"CV: {latest_cv_raw:.5f}")
        text_cv_thresh.set_text(f"CV Threshold: {latest_cv_thresh_raw:.5f}")
        status_name = STATUS_LABELS.get(latest_status_code, f'UNKNOWN({latest_status_code})')
        text_status.set_text(f"Status: {status_name}")

    return [line_rms, line_cw, line_cv, line_cv_threshold, text_rms, text_cw, text_cv, text_cv_thresh, text_status, *OVERLAY_IMAGES]

def on_resize(event):
    if 'ani' in globals():
        blit_cache = getattr(ani, '_blit_cache', None)
        if isinstance(blit_cache, dict):
            blit_cache.clear()
    fig.canvas.draw()

plt.subplots_adjust(bottom=0.35) 

ax_btn = plt.axes([0.05, 0.25, 0.15, 0.05])
btn_capture = Button(ax_btn, 'Start Capture', color='lightgreen', hovercolor='palegreen')

ax_box = plt.axes([0.25, 0.25, 0.15, 0.05])
txt_cv_max = TextBox(ax_box, 'Max CV \nY-Limit: ', initial='10.0')

ax_btn_fit = plt.axes([0.45, 0.25, 0.15, 0.05])
btn_fit = Button(ax_btn_fit, 'Fit Function', color='yellow', hovercolor='gold')

def submit_cv_max(text):
    try:
        val = float(text)
        if val > 0:
            ax2.set_ylim(0, val)
            fig.canvas.draw_idle()
    except ValueError:
        pass

txt_cv_max.on_submit(submit_cv_max)

def toggle_capture(event):
    global is_recording, csv_file, csv_writer
    out_dir = "telemetry_captures"
    if not is_recording:
        if not os.path.exists(out_dir): os.makedirs(out_dir)
        filename = os.path.join(out_dir, time.strftime("capture_%Y%m%d_%H%M%S") + ".csv")
        csv_file = open(filename, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Timestamp', 'ADC_value', 'Current_CV', 'CV_Threshold', 'Status_Code'])
        is_recording = True
        _apply_recording_mode(True)
        btn_capture.label.set_text('Stop Capture')
        btn_capture.color = 'salmon'
        btn_capture.hovercolor = 'lightcoral'
    else:
        is_recording = False
        _apply_recording_mode(False)
        if csv_file is not None: csv_file.close()
        plot_filename = os.path.join(out_dir, time.strftime("capture_%Y%m%d_%H%M%S") + ".png")
        fig.savefig(plot_filename, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {plot_filename}")
        btn_capture.label.set_text('Start Capture')
        btn_capture.color = 'lightgreen'
        btn_capture.hovercolor = 'palegreen'
    fig.canvas.draw_idle()

btn_capture.on_clicked(toggle_capture)

def cv_model(x_adc, a, b, c, d, e):
    x_adc = np.asarray(x_adc)
    x_norm = np.clip(x_adc / 4095.0, 0.001, 1.0)
    # y = a*exp(b*x) + c*(x^d) + e
    return a * np.exp(b * x_norm) + c * np.power(x_norm, d) + e

def toggle_fit(event):
    global is_fitting, fit_adc_data, fit_cv_data
    if opt is None:
        print("Scipy is required to perform fitting.")
        return

    if not is_fitting:
        is_fitting = True
        fit_adc_data = []
        fit_cv_data = []
        btn_fit.label.set_text('Stop & Fit')
        btn_fit.color = 'orange'
        print("\n--- Fit Mode STARTED ---")
        print("Attenuate your signal gradually from max to 0.")
    else:
        is_fitting = False
        btn_fit.label.set_text('Fit Function')
        btn_fit.color = 'yellow'
        print(f"\n--- Fit Mode STOPPED ---")
        print(f"Captured {len(fit_adc_data)} points. Fitting...")
        
        if len(fit_adc_data) < 20:
            print("Not enough data points captured. Need more variation.")
            return

        adc_arr = np.array(fit_adc_data)
        cv_arr = np.array(fit_cv_data)
        
        # Limit the CV scale for the fitting data to max 10.0 to prevent NLSS skew
        valid_mask = cv_arr <= 10.0
        adc_arr = adc_arr[valid_mask]
        cv_arr = cv_arr[valid_mask]
        
        sort_idx = np.argsort(adc_arr)
        adc_sorted = adc_arr[sort_idx]
        cv_sorted = cv_arr[sort_idx]

        # Use current CV.h parameters as initial guess
        p0 = [2.1453, -8.7210, 4.8821, -0.4120, 0.0520]
        try:
            # We constrain the exponential and power law so it doesn't blow up or invert
            bounds = ([-np.inf, -20.0, -np.inf, -5.0, -5.0], 
                      [np.inf,  0.0,    np.inf,  0.0,  5.0])
                      
            popt, pcov = opt.curve_fit(cv_model, adc_sorted, cv_sorted, p0=p0, maxfev=10000, bounds=bounds)
            a, b, c, d, e = popt
            
            print(f"Optimal Fit Found:\n A = {a:.4f}\n B = {b:.4f}\n C = {c:.4f}\n D = {d:.4f}\n E = {e:.4f}")
            
            # Update CV.h in STM32 code
            cv_h_path = "/home/nurassyl/Desktop/stm32projects/MLDS_G4/Core/Inc/CV.h"
            if os.path.exists(cv_h_path):
                with open(cv_h_path, 'r') as f:
                    content = f.read()
                
                content = re.sub(r"#define PARAM_A [0-9.-]+f", f"#define PARAM_A {a:.4f}f", content)
                content = re.sub(r"#define PARAM_B [0-9.-]+f", f"#define PARAM_B {b:.4f}f", content)
                content = re.sub(r"#define PARAM_C [0-9.-]+f", f"#define PARAM_C {c:.4f}f", content)
                content = re.sub(r"#define PARAM_D [0-9.-]+f", f"#define PARAM_D {d:.4f}f", content)
                content = re.sub(r"#define PARAM_E [0-9.-]+f", f"#define PARAM_E {e:.4f}f", content)
                
                with open(cv_h_path, 'w') as f:
                    f.write(content)
                print(f"Successfully updated {cv_h_path} with new coefficients!")
                print("Recompile and flash your STM32 to apply the new LUT.")
            else:
                print(f"Error: Could not find CV.h at {cv_h_path}")
            
            # Plot the fitted curve
            fig_fit = plt.figure('NLSS Fit Results')
            ax_fit = fig_fit.add_subplot(111)
            ax_fit.scatter(adc_sorted, cv_sorted, s=2, color='gray', alpha=0.5, label='Captured Data')
            
            adc_fit_line = np.linspace(0, 4095, 1000)
            cv_fit_line = cv_model(adc_fit_line, *popt)
            ax_fit.plot(adc_fit_line, cv_fit_line, color='red', linewidth=2, label='Fitted Function')
            
            ax_fit.set_xlabel('ADC Value')
            ax_fit.set_ylabel('CV')
            ax_fit.set_ylim(0, 10.0)
            ax_fit.set_title('NLSS Curve Fit: ADC vs CV')
            ax_fit.legend()
            ax_fit.grid(True)
            
            fig_fit.show()
            
        except RuntimeError as err:
            print(f"Curve fitting failed (RuntimeError): {err}")
        except Exception as err:
            print(f"Curve fitting failed: {err}")

btn_fit.on_clicked(toggle_fit)

ax_box_t_win = plt.axes([0.15, 0.15, 0.1, 0.05])
txt_t_win = TextBox(ax_box_t_win, 'Target\nWin(ms): ', initial='50')

ax_box_p_low = plt.axes([0.38, 0.15, 0.1, 0.05])
txt_p_low = TextBox(ax_box_p_low, 'Pulsed\nLow: ', initial='100.0')

ax_box_p_high = plt.axes([0.62, 0.15, 0.1, 0.05])
txt_p_high = TextBox(ax_box_p_high, 'Pulsed\nHigh: ', initial='1000.0')

ax_box_c_low = plt.axes([0.15, 0.05, 0.1, 0.05])
txt_c_low = TextBox(ax_box_c_low, 'Cont\nLow: ', initial='100.0')

ax_box_c_high = plt.axes([0.38, 0.05, 0.1, 0.05])
txt_c_high = TextBox(ax_box_c_high, 'Cont\nHigh: ', initial='1000.0')

ax_box_t_offset = plt.axes([0.62, 0.05, 0.1, 0.05])
txt_t_offset = TextBox(ax_box_t_offset, 'Thresh\nOffset: ', initial='1.0')

ax_btn_send = plt.axes([0.75, 0.05, 0.20, 0.15])
btn_send = Button(ax_btn_send, 'Send Config ->', color='lightblue', hovercolor='skyblue')

def send_config(event):
    if ser is None or not ser.is_open:
        print("Serial port not open. Cannot send config.")
        return
        
    try:
        t_win = int(txt_t_win.text)
        p_low = float(txt_p_low.text)
        p_high = float(txt_p_high.text)
        c_low = float(txt_c_low.text)
        c_high = float(txt_c_high.text)
        t_offset = float(txt_t_offset.text)
        
        frame = TX_FRAME_STRUCT.pack(FRAME_START, t_win, p_low, p_high, c_low, c_high, t_offset, FRAME_END)
        ser.write(frame)
        print(f"Sent config: Window={t_win}ms, Pulsed={p_low}-{p_high}, Cont={c_low}-{c_high}, Offset={t_offset}")
        
    except ValueError as e:
        print(f"Invalid configuration input: {e}")

btn_send.on_clicked(send_config)
fig.canvas.mpl_connect('resize_event', on_resize)

ani = animation.FuncAnimation(fig, update, interval=PLOT_INTERVAL_MS, blit=True, cache_frame_data=False)
plt.show()
