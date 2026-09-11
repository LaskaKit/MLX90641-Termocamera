# Thermal camera module MLX90641 16×12 px, 55°×35° with μSup connector

A compact thermal camera (thermal imaging) module built around the **Melexis MLX90641** far-infrared array sensor. It gives you **192 real temperature readings (16×12 px)** in every frame, at up to 64 Hz, over I²C - no contact, no extra optics, no calibration needed. Point it at a board, a motor, a radiator, a face or a soldering iron tip and you get the temperature of every point in the scene.

![Assembled module](https://github.com/LaskaKit/MLX90641-Termocamera/blob/main/img/1.jpg)

Typical uses: overheating detection on PCBs and in electrical cabinets, presence and people counting, heat-loss inspection, 3D printer and machine monitoring, robotics, or just a DIY thermal camera with a display.

## Why this module and not the bare sensor

The MLX90641 alone is a 4-pin TO-39 can that you have to wire, pull up and power yourself. On this module you get:

- the sensor already soldered on a PCB with the correct decoupling,
- a **[μSup connector](https://blog.laskakit.cz/predstavujeme-univerzalni-konektor-pro-propojeni-modulu-a-cidel-%CE%BCsup/)** - plug it into any LaskaKit board with a single 4-wire cable, no soldering at all,
- solder pads for a standard 2.54 mm 4-pin header (straight or right-angle) if you prefer a breadboard,
- mounting holes so you can actually fix the camera where it should look,
- and **working example code** - Arduino firmware plus a Python live heatmap viewer, see [SW](https://github.com/LaskaKit/MLX90641-Termocamera/tree/main/SW).

## Specification

| | |
|---|---|
| Sensor | Melexis MLX90641 |
| Resolution | 16 × 12 px (192 measuring points) |
| Field of view (FOV) | 55° × 35° |
| Temperature range of the scene | -40 °C to +300 °C |
| Refresh rate | up to 64 Hz (0.5 / 1 / 2 / 4 / 8 / 16 / 32 / 64 Hz) |
| Accuracy | typ. ±1 °C (see the Melexis datasheet) |
| Interface | I²C, up to 1 MHz |
| I²C address | **0x33** |
| Supply voltage | 3.3 V |
| Connectors | μSup (JST-SH 1.0 mm, 4-pin) + pads for a 2.54 mm 4-pin header |
| Module dimensions | 22.9 × 21.6 mm |

![Bottom side of the module](https://github.com/LaskaKit/MLX90641-Termocamera/blob/main/img/2.jpg)

## How to connect it

The μSup connector carries **3V3, GND, SDA, SCL**. Connect the module to any of our boards with a single cable - for example the [ESP32-S3 DevKit](https://www.laskakit.cz/laskakit-esp32-s3-devkit/), the [ESP32-DEVKit](https://www.laskakit.cz/laskakit-esp32-devkit/), the low-power [ESP32-C3 LPKit](https://www.laskakit.cz/laskkit-esp-12-board/), [Meteo Mini](https://www.laskakit.cz/laskakit-meteo-mini/) or the [ESPD-3.5 with a 3.5" TFT display](https://www.laskakit.cz/laskakit-espd-35-esp32-3-5-tft-ili9488-touch/), for which we have a **ready-made thermal camera application** - see [ESPD-35/SW/MLX90641-Termocamera](https://github.com/LaskaKit/ESPD-35/tree/main/SW/MLX90641-Termocamera).

You are of course not limited to our boards - any [Arduino](https://www.laskakit.cz/arduino-2/), [Raspberry Pi or Rock Pi](https://www.laskakit.cz/mini-pc/) with a 3.3 V I²C bus will do. Just keep in mind the module is **3.3 V only**, so use a level shifter with 5 V boards.

## Example code

Everything you need is in [SW/mlx90641-test](https://github.com/LaskaKit/MLX90641-Termocamera/tree/main/SW/mlx90641-test):

- `mlx90641-test.ino` - Arduino sketch for ESP32 (tested on LaskaKit ESP32-S3 DevKit: `SDA = GPIO42`, `SCL = GPIO2`, sensor power enable on `GPIO47`). It prints the frame as an **ASCII heatmap**, as a **CSV line with 192 values in °C**, or as a **JSON array** - pick one or more with `USE_ASCII_HEATMAP` / `USE_CSV_OUTPUT` / `USE_JSON_OUTPUT`. Emissivity, refresh rate, frame averaging and image flipping are all at the top of the sketch.
- `MLX90641_API.cpp/.h` - the official [Melexis MLX90641 library](https://github.com/melexis/mlx90641-library).
- `MLX90641_I2C_Driver.cpp/.h` - I²C driver rewritten by us for the Arduino `Wire` library (do **not** use the mbed version).
- `mlx90641_plot.py` - live 16×12 heatmap on your PC over USB serial.
- `mlx90641_plot_640x480.py` - the same, upscaled to 640×480 with interpolation, a marker on the hottest spot and a live temperature readout under the cursor.

Quick start for the Python viewer:

```bash
pip3 install pyserial matplotlib numpy
# optional, for faster and nicer upscaling:
pip3 install opencv-python

# set USE_CSV_OUTPUT 1 in the sketch, set your port in the script, then:
python3 mlx90641_plot.py
```

## What else is in this repository

- `HW/` - schematic in PDF
- `3D/` - 3D model of the module (STEP, Fusion 360)
- `Production/` - gerbers and BOM/CPL files
- `SW/` - example code
### Ready-made thermal camera with a display

If you have the [ESPD-3.5](https://www.laskakit.cz/laskakit-espd-35-esp32-3-5-tft-ili9488-touch/), you don't have to write anything - a complete application that draws the thermal image straight onto the 3.5" touch display lives in the board's repository: [ESPD-35/SW/MLX90641-Termocamera](https://github.com/LaskaKit/ESPD-35/tree/main/SW/MLX90641-Termocamera). Plug the module in with a μSup cable and flash the sketch.
## Where to buy

**[https://www.laskakit.cz/laskakit-mlx90641-modul-termokamery-16--12px-55--x35/](https://www.laskakit.cz/laskakit-mlx90641-modul-termokamery-16--12px-55--x35/)**

Czech version of this readme: [README_CZ.md](README_CZ.md)

Questions? podpora@laskakit.cz
