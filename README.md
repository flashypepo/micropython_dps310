# micropython_dps310 - also driver for sensor HP303B

Sample micropython code for sensor HP303B which can be found in Lolin barometric sensor shield. Sensor HP303B seems compatible with DSP310 sensor

Basic class `DPS310` is made by @jposada2020. Advanced class `DPS310Advanced` is made by Adafruit and modified by me to use @jposada2020 dps310 modules.

Usage: see `dps310_simpletest.py`

**Configuration**
- Micropython version: 1.20 (MicroPython v1.20.0-124-g17c3f6b6aa on 2023-05-08; LOLIN S3 MINI with ESP32S3 
    - [customized firmware](https://github.com/wemos/micropython/releases) from Wemos github
- Microcontroller: [Lolin ESP32 S3 mini](https://www.wemos.cc/en/latest/s3/s3_mini.html)
- [Barometric Pressure Shield (HP303B)](https://www.wemos.cc/en/latest/d1_mini_shield/barometric_pressure.html)

2023-0922 PP
