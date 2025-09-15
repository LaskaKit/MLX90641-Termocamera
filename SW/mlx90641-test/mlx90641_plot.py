import serial, numpy as np, matplotlib.pyplot as plt

PORT = '/dev/tty.wchusbserial5A9B0085741'  # replace with your ESP32 port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(np.zeros((12,16)), vmin=20, vmax=35, cmap='inferno', origin='lower')
plt.colorbar(img)

while True:
    line = ser.readline().decode('utf-8').strip()
    if not line or ',' not in line:
        continue
    try:
        vals = [float(x) for x in line.split(',')]
        if len(vals) == 192:
            frame = np.array(vals).reshape((12,16))
            img.set_data(frame)
            img.set_clim(vmin=frame.min(), vmax=frame.max())  # auto-scale
            plt.pause(0.001)
    except ValueError:
        continue
