# MLX90641 Thermal Camera (ESP32-S3 + Python Visualization)

This project uses the **Melexis MLX90641 (16×12 thermal sensor)** with an **ESP32-S3**.  
The ESP32 reads thermal frames over I²C and streams them as CSV over USB serial.  
A Python script on your PC/Mac listens to the serial data and plots a **live heatmap**.

---

## Hardware Setup
- **LaskaKit ESP32-S3 Dev board** (tested with 16MB Flash variant).
- **MLX90641 sensor** connected via I²C:
  - `SDA → GPIO 42`
  - `SCL → GPIO 2`  
- Power the sensor with **3.3V**.  
  Use `GPIO 47` to turn power ON.

---

## Firmware (ESP32-S3)

1. Clone/download this code into Arduino IDE.
2. Make sure you include:
   - `mlx90641-test.ino` (main sketch)
   - `MLX90641_API.cpp/.h` (from [Melexis MLX90641 library](https://github.com/melexis/mlx90641-library/tree/master/functions))
   - `MLX90641_I2C_Driver.cpp/.h` (Arduino Wire-based driver, not the mbed version!)
3. Upload the sketch to LaskaKit ESP32-S3 Dev board.
4. The firmware will:
    - Initialize the sensor.
    - Set Output:
        - Human-readable **ASCII heatmap** 16x12
        - Output frames as **CSV rows with 192 values** (°C).
        - JSON-like **array** [v0, v1, ..., v191]  
    - You can flif horizontal o vertical
---

## PC Visualization (Python, macOS/Linux/Windows)

### Set Output frames as CSV rows: 
In mlx90641-test.ino:
```bash
#define USE_CSV_OUTPUT    1
```
### Install dependencies
Make sure Python 3 is installed:

```bash
python3 --version
```
If not, install:
```bash
brew install python3
```
Install required packages:
```bash
pip3 install pyserial matplotlib numpy
```
replace port with your in python script:
```bash
PORT = '/dev/tty.wchusbserial5A9B0085741'  # replace with your ESP32 port
```
Run the script:
- 16x12:
```bash
python3 mlx90641_plot.py
```
- 640x480 upscaling + blending:
    - Finds the max temperature in the current (upscaled) frame.
	- Draws a cross at that location and a label with the temperature.
	- Optional: hover the mouse over the image to see live temperature under the cursor.
```bash
python3 mlx90641_plot_640x480.py
```
### Optional OpenCV for fastest, highest-quality resize (INTER_CUBIC/LINEAR):
```bash
pip3 install opencv-python
```