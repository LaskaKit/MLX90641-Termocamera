#!/usr/bin/env python3
"""
Live MLX90641 viewer (CSV over serial) with 640x480 upscaling + blending
and an overlay crosshair + temperature label on the hottest pixel.
Also shows a hover readout for temperature under the mouse.

- Reads 192 floats per line from serial (16x12).
- Upscales to 640x480 (OpenCV if available, else pure-NumPy bilinear).
- Auto-scales color limits with EMA smoothing to reduce flicker.
- Press 's' to save a PNG of the current view.
"""

import sys
import time
import numpy as np
import matplotlib.pyplot as plt

# --- Try OpenCV; fall back to pure NumPy if unavailable ---
try:
    import cv2
    HAS_CV2 = True
except Exception:
    HAS_CV2 = False

# ------------------------- CONFIG -------------------------
PORT = '/dev/tty.wchusbserial5A9B0085741'  # <- set your port (macOS: ls /dev/tty.*)
BAUD = 115200

SRC_W, SRC_H = 16, 12             # MLX90641 native resolution
OUT_W, OUT_H = 640, 480           # Target upscaled resolution

CLIM_SMOOTH = 0.2                 # 0..1, EMA for vmin/vmax
CMAP = 'inferno'                  # matplotlib colormap
CV2_INTERP = 'cubic'              # 'linear' or 'cubic' (if OpenCV present)

PLOT_DT = 0.001                   # Throttle UI updates (seconds)

CROSS_SIZE = 18                   # half-length of crosshair in output pixels
CROSS_LINEWIDTH = 1.5
LABEL_FMT = "{:.2f} °C"

HOVER_READOUT = True              # show temp under mouse cursor

# ----------------------- SERIAL SETUP ----------------------
import serial
def open_serial(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    return serial.Serial(port, baud, timeout=timeout)

# -------------------- UPSCALING (BLENDING) ----------------
def upscale_np_bilinear(src: np.ndarray, out_w: int, out_h: int) -> np.ndarray:
    """Pure-NumPy bilinear upscaling: horizontal pass then vertical pass."""
    h, w = src.shape
    x_src = np.arange(w, dtype=np.float32)
    x_dst = np.linspace(0, w - 1, out_w, dtype=np.float32)
    tmp = np.empty((h, out_w), dtype=np.float32)
    for r in range(h):
        tmp[r, :] = np.interp(x_dst, x_src, src[r, :])
    y_src = np.arange(h, dtype=np.float32)
    y_dst = np.linspace(0, h - 1, out_h, dtype=np.float32)
    out = np.empty((out_h, out_w), dtype=np.float32)
    for c in range(out_w):
        out[:, c] = np.interp(y_dst, y_src, tmp[:, c])
    return out

def upscale(frame_12x16: np.ndarray) -> np.ndarray:
    if HAS_CV2:
        inter = cv2.INTER_CUBIC if CV2_INTERP.lower().startswith('c') else cv2.INTER_LINEAR
        f32 = np.ascontiguousarray(frame_12x16, dtype=np.float32)
        return cv2.resize(f32, (OUT_W, OUT_H), interpolation=inter)
    else:
        return upscale_np_bilinear(frame_12x16.astype(np.float32), OUT_W, OUT_H)

# ----------------------- PLOTTING -------------------------
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
img = ax.imshow(np.zeros((OUT_H, OUT_W), dtype=np.float32),
                vmin=20, vmax=35, cmap=CMAP, origin='lower', aspect='equal')
cbar = plt.colorbar(img, ax=ax)
ax.set_title('MLX90641 (16×12 → 640×480)')
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Crosshair lines (two Line2D objects) + label (Text)
cross_h, = ax.plot([], [], '-', linewidth=CROSS_LINEWIDTH)
cross_v, = ax.plot([], [], '-', linewidth=CROSS_LINEWIDTH)
label = ax.text(0, 0, "", color='white', fontsize=10,
                ha='left', va='bottom',
                bbox=dict(facecolor='black', alpha=0.6, boxstyle='round,pad=0.3'))

# Set crosshair color based on the colormap's brightest color
try:
    # Use the last color from the colormap for visibility
    cross_color = plt.get_cmap(CMAP)(1.0)
except Exception:
    cross_color = (1, 1, 1, 1)
cross_h.set_color(cross_color)
cross_v.set_color(cross_color)

vmin_ema = 20.0
vmax_ema = 35.0

def update_clim_auto(frame: np.ndarray):
    """Exponential moving average for color limits to reduce flicker."""
    global vmin_ema, vmax_ema
    fmin = float(np.min(frame))
    fmax = float(np.max(frame))
    # Expand tiny ranges to avoid flat images
    if fmax - fmin < 0.5:
        pad = 0.25
        fmin -= pad
        fmax += pad
    # EMA
    alpha = CLIM_SMOOTH
    vmin_ema = (1 - alpha) * vmin_ema + alpha * fmin
    vmax_ema = (1 - alpha) * vmax_ema + alpha * fmax
    # Ensure minimum span
    if vmax_ema - vmin_ema < 0.5:
        mid = 0.5 * (vmin_ema + vmax_ema)
        vmin_ema = mid - 0.25
        vmax_ema = mid + 0.25
    img.set_clim(vmin=vmin_ema, vmax=vmax_ema)
    cbar.update_normal(img)

def draw_cross_with_label(frame_up: np.ndarray):
    """Draw crosshair + label at the hottest pixel in the upscaled frame."""
    # Locate hottest pixel
    idx = np.argmax(frame_up)
    y, x = divmod(int(idx), frame_up.shape[1])
    t = float(frame_up[y, x])

    # Crosshair geometry (clamped to image bounds)
    x0 = max(0, x - CROSS_SIZE)
    x1 = min(OUT_W - 1, x + CROSS_SIZE)
    y0 = max(0, y - CROSS_SIZE)
    y1 = min(OUT_H - 1, y + CROSS_SIZE)

    # Update line data
    cross_h.set_data([x0, x1], [y, y])
    cross_v.set_data([x, x], [y0, y1])

    # Update label slightly offset to avoid covering the cross center
    label.set_position((x + 6, y + 6))
    label.set_text(LABEL_FMT.format(t))

# Optional: show temp under mouse cursor
hover_text = ax.text(0.98, 0.02, "", color='white', fontsize=9,
                     ha='right', va='bottom', transform=ax.transAxes,
                     bbox=dict(facecolor='black', alpha=0.4, boxstyle='round,pad=0.3')) if HOVER_READOUT else None

def on_move(event):
    if not HOVER_READOUT or not event.inaxes or img.get_array() is None:
        return
    # Data coordinates align with pixel indices because aspect='equal' and origin='lower'
    x, y = int(round(event.xdata)), int(round(event.ydata))
    arr = img.get_array()
    if 0 <= x < arr.shape[1] and 0 <= y < arr.shape[0]:
        temp = float(arr[y, x])
        hover_text.set_text(f"{temp:.2f} °C  @ ({x},{y})")
        fig.canvas.draw_idle()
fig.canvas.mpl_connect('motion_notify_event', on_move)

# ----------------------- MAIN LOOP ------------------------
def main():
    print(f'Opening serial: {PORT} @ {BAUD}…')
    try:
        ser = open_serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f'ERROR opening serial port {PORT}: {e}')
        sys.exit(1)

    last_plot = time.time()
    print("Running. Press the matplotlib window and hit 's' to save a frame; Ctrl+C to quit.")

    saved_idx = 0
    def on_key(event):
        nonlocal saved_idx
        if event.key == 's':
            fname = f'frame_{saved_idx:04d}.png'
            plt.savefig(fname, dpi=150, bbox_inches='tight')
            print(f'Saved {fname}')
            saved_idx += 1
    fig.canvas.mpl_connect('key_press_event', on_key)

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line or ',' not in line:
            continue
        parts = line.split(',')
        if len(parts) != SRC_W * SRC_H:
            continue
        try:
            vals = np.array([float(x) for x in parts], dtype=np.float32)
        except ValueError:
            continue

        frame = vals.reshape((SRC_H, SRC_W))    # (12,16)
        up = upscale(frame)                      # (480,640)

        img.set_data(up)
        update_clim_auto(up)
        draw_cross_with_label(up)

        now = time.time()
        if now - last_plot >= PLOT_DT:
            plt.pause(0.001)
            last_plot = now

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nBye!')