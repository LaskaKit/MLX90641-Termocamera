# Modul termokamery MLX90641 16×12 px, 55°×35° s konektorem μŠup

Kompaktní modul termokamery postavený na infračerveném maticovém senzoru **Melexis MLX90641**. V každém snímku dostaneš **192 skutečných hodnot teploty (16×12 px)**, až 64× za sekundu, po I²C - bezkontaktně, bez další optiky a bez kalibrace. Namiř ho na desku plošných spojů, motor, radiátor, obličej nebo hrot pájky a máš teplotu každého bodu scény.

![Osazený modul](https://github.com/LaskaKit/MLX90641-Termocamera/blob/main/img/1.jpg)

K čemu se hodí: hledání přehřívajících se součástek na DPS a v rozvaděčích, detekce přítomnosti a počítání osob, kontrola tepelných úniků, hlídání 3D tiskárny nebo stroje, robotika - nebo prostě domácí termokamera s displejem.

## Proč modul a ne samotný senzor

Samotný MLX90641 je čtyřvývodové pouzdro TO-39, které si musíš sám zapojit, napájet a doplnit pull-upy. Na tomhle modulu dostaneš:

- senzor už zapájený na desce se správným blokováním napájení,
- **[konektor μŠup](https://blog.laskakit.cz/predstavujeme-univerzalni-konektor-pro-propojeni-modulu-a-cidel-%CE%BCsup/)** - k jakékoli naší desce ho připojíš jedním čtyřžilovým kabelem, bez pájení,
- pájecí plošky pro standardní 4pinový hřebínek s roztečí 2.54 mm (přímý i pravoúhlý), když chceš do nepájivého pole,
- montážní otvory, takže kameru opravdu upevníš tam, kam se má dívat,
- a **funkční ukázkový kód** - firmware pro Arduino a k tomu Python vizualizaci s živou teplotní mapou, viz složka [SW](https://github.com/LaskaKit/MLX90641-Termocamera/tree/main/SW).

## Specifikace

| | |
|---|---|
| Senzor | Melexis MLX90641 |
| Rozlišení | 16 × 12 px (192 měřicích bodů) |
| Zorné pole (FOV) | 55° × 35° |
| Rozsah měřených teplot | -40 °C až +300 °C |
| Obnovovací frekvence | až 64 Hz (0,5 / 1 / 2 / 4 / 8 / 16 / 32 / 64 Hz) |
| Přesnost | typ. ±1 °C (viz katalogový list Melexis) |
| Komunikace | I²C, až 1 MHz |
| I²C adresa | **0x33** |
| Napájecí napětí | 3,3 V |
| Konektory | μŠup (JST-SH 1,0 mm, 4pin) + plošky pro hřebínek 2.54 mm |
| Rozměry modulu | 22,9 × 21,6 mm |

![Spodní strana modulu](https://github.com/LaskaKit/MLX90641-Termocamera/blob/main/img/2.jpg)

## Jak ho zapojit

Konektor μŠup vede **3V3, GND, SDA, SCL**. Modul připojíš jedním kabelem k libovolné naší desce - třeba k [ESP32-S3 DevKit](https://www.laskakit.cz/laskakit-esp32-s3-devkit/), [ESP32-DEVKit](https://www.laskakit.cz/laskakit-esp32-devkit/), úspornému [ESP32-C3 LPKit](https://www.laskakit.cz/laskkit-esp-12-board/), [Meteo Mini](https://www.laskakit.cz/laskakit-meteo-mini/) nebo k [ESPD-3.5 s 3,5" TFT displejem](https://www.laskakit.cz/laskakit-espd-35-esp32-3-5-tft-ili9488-touch/), pokud chceš termosnímek zobrazovat přímo na zařízení.

Naše desky samozřejmě nejsou podmínka - postačí jakékoli [Arduino](https://www.laskakit.cz/arduino-2/), [Raspberry Pi nebo Rock Pi](https://www.laskakit.cz/mini-pc/) s 3,3V I²C sběrnicí. Jen pozor, modul je **pouze na 3,3 V**, takže s pětivoltovými deskami použij převodník úrovní.

## Ukázkový kód

Všechno potřebné najdeš ve složce [SW/mlx90641-test](https://github.com/LaskaKit/MLX90641-Termocamera/tree/main/SW/mlx90641-test):

- `mlx90641-test.ino` - sketch pro ESP32 (testováno na LaskaKit ESP32-S3 DevKit: `SDA = GPIO42`, `SCL = GPIO2`, spínání napájení senzoru na `GPIO47`). Snímek umí vypsat jako **ASCII teplotní mapu**, jako **CSV řádek se 192 hodnotami ve °C** nebo jako **JSON pole** - vybereš přepínači `USE_ASCII_HEATMAP` / `USE_CSV_OUTPUT` / `USE_JSON_OUTPUT`. Emisivitu, obnovovací frekvenci, průměrování snímků i překlopení obrazu nastavíš hned na začátku sketche.
- `MLX90641_API.cpp/.h` - oficiální [knihovna Melexis MLX90641](https://github.com/melexis/mlx90641-library).
- `MLX90641_I2C_Driver.cpp/.h` - I²C driver, který jsme přepsali pro Arduino knihovnu `Wire` (mbed verzi **nepoužívej**).
- `mlx90641_plot.py` - živá teplotní mapa 16×12 na počítači přes USB sériovou linku.
- `mlx90641_plot_640x480.py` - totéž, ale převzorkované na 640×480 s interpolací, značkou na nejteplejším místě a živým odečtem teploty pod kurzorem.

Rychlý start Python vizualizace:

```bash
pip3 install pyserial matplotlib numpy
# volitelně, pro rychlejší a hezčí převzorkování:
pip3 install opencv-python

# ve sketchi nastav USE_CSV_OUTPUT 1, ve skriptu svůj port, a pak:
python3 mlx90641_plot.py
```

## Co dalšího v repozitáři najdeš

- `HW/` - schéma v PDF
- `3D/` - 3D model modulu (STEP, Fusion 360)
- `Production/` - gerbery a soubory BOM/CPL
- `SW/` - ukázkový kód

## Kde modul koupíš

**[https://www.laskakit.cz/laskakit-mlx90641-modul-termokamery-16--12px-55--x35/](https://www.laskakit.cz/laskakit-mlx90641-modul-termokamery-16--12px-55--x35/)**

English version: [README.md](README.md)

Dotazy? podpora@laskakit.cz
