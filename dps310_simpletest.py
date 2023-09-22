"""
2023-0922 PP: modified simple test
2023-0921 PP: DPS310 is compatible with HP303B - hopefully

gist-version class DPS310 (2023-0922):
https://gist.github.com/jposada202020/3ea7bde4ec10f0bf1dec147a6bc9f845

Original class DPS310: installatie driver DPS310:
    import mip
    mip.install("github:jposada202020/MicroPython_DPS310")

2023-0922 PP test for DPS310 - gist version
2023-0921 PP: DPS310 is compatible with HP303B - hopefully
"""
import time
from machine import I2C
# 2023-0922: GIST version
from micropython_dps310.dps310 import DPS310

i2c = I2C(0)  # ESP32-s3mini
dps = DPS310(i2c)  # address=0x77)
# PP added: wait until sensor is ready...
#dps.wait_temperature_ready()  # working?
#dps.wait_pressure_ready()   # working?
time.sleep_ms(300)

# sea level pressure A'dam:
# https://barometricpressure.app/amsterdam#
dps.sea_level_pressure = 999.4  # A'dam, 2023-0922 11:10

# simple loop to print sensor values
while True:
    print(f"Temperature {dps.temperature:.2f}°C, Pressure: {dps.pressure:.2f}HPa, Altitude: {dps.altitude:.2f}m")
    time.sleep(1)
