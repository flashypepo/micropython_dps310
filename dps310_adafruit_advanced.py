"""
2023-0922 PP: simple test using Adafruit DPS310_Advanced
              in combination with @jposada2020 DPS310
"""
import time
from machine import I2C
from adafruit_dps310.advanced import DPS310_Advanced as DPS310
# configuration parameters - not used in this example
#from adafruit_dps310.advanced import SampleCount, Rate, Mode

i2c = I2C(0)
dps = DPS310(i2c)
time.sleep_ms(300)  # PP: added wait for sensor ready

# sea level pressure A'dam:
# https://barometricpressure.app/amsterdam#
dps.sea_level_pressure = 999.4  # A'dam, 2023-0922 11:10

while True:
    print(f"Temperature {dps.temperature:.2f}°C, Pressure: {dps.pressure:.2f}HPa, Altitude: {dps.altitude:.2f}m")
    #print("Temperature = %.2f *C"%dps310.temperature)
    #print("Pressure = %.2f hPa"%dps310.pressure)
    #print("")
    time.sleep(1.0)
