# 2023-0922: Dit is een gist-version
# URL: https://gist.github.com/jposada202020/3ea7bde4ec10f0bf1dec147a6bc9f845
# 
# SPDX-FileCopyrightText: Copyright (c) 2023 Jose D. Montoya
#
# SPDX-License-Identifier: MIT
"""
`dps310`
================================================================================

MicroPython Driver for the DPS310 Barametric Sensor


* Author: Jose D. Montoya

Implementation Notes
--------------------

**Software and Dependencies:**

This library depends on Micropython

"""

# pylint: disable=line-too-long

import time
import math
import struct
from micropython import const
from micropython_dps310.i2c_helpers import CBits, RegisterStruct


__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/jposada202020/MicroPython_DPS310.git"


_DPS310_DEFAULT_ADDRESS = const(0x77)  # DPS310 default i2c address
_DPS310_DEVICE_ID = const(0x10)  # DPS310 device identifier

_DPS310_PRSB2 = const(0x00)  # Highest byte of pressure data
_DPS310_TMPB2 = const(0x03)  # Highest byte of temperature data
_DPS310_PRSCFG = const(0x06)  # Pressure configuration
_DPS310_TMPCFG = const(0x07)  # Temperature configuration
_DPS310_MEASCFG = const(0x08)  # Sensor configuration
_DPS310_CFGREG = const(0x09)  # Interrupt/FIFO configuration
_DPS310_RESET = const(0x0C)  # Soft reset
_DPS310_PRODREVID = const(0x0D)  # Register that contains the part ID
_DPS310_TMPCOEFSRCE = const(0x28)  # Temperature calibration src


class DPS310:
    """Main class for the Sensor

    :param ~machine.I2C i2c: The I2C bus the DPS310 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x77`

    :raises RuntimeError: if the sensor is not found


    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`micropython_dps310.DPS310` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        from machine import Pin, I2C
        import micropython_dps310.dps310 as dps310

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(1, sda=Pin(2), scl=Pin(3))
        dps = dps310.DPS310(i2c)

    Now you have access to the :attr:`pressure` attribute

    .. code-block:: python

        press = dps.pressure

    """

    _device_id = RegisterStruct(_DPS310_PRODREVID, ">B")
    _reset_register = RegisterStruct(_DPS310_RESET, ">B")
    _mode_bits = CBits(3, _DPS310_MEASCFG, 0)  #

    _pressure_osbits = CBits(4, _DPS310_PRSCFG, 0)

    _temp_osbits = CBits(4, _DPS310_TMPCFG, 0)

    _temp_measurement_src_bit = CBits(1, _DPS310_TMPCFG, 7)

    _pressure_shiftbit = CBits(1, _DPS310_CFGREG, 2)
    _temp_shiftbit = CBits(1, _DPS310_CFGREG, 3)

    _coefficients_ready = CBits(1, _DPS310_MEASCFG, 7)
    _sensor_ready = CBits(1, _DPS310_MEASCFG, 6)
    _temp_ready = CBits(1, _DPS310_MEASCFG, 5)
    _pressure_ready = CBits(1, _DPS310_MEASCFG, 4)

    _raw_pressure = CBits(24, _DPS310_PRSB2, 0, 3, False)
    _raw_temperature = CBits(24, _DPS310_TMPB2, 0, 3, False)

    _calib_coeff_temp_src_bit = CBits(1, _DPS310_TMPCOEFSRCE, 7)

    _reg0e = CBits(8, 0x0E, 0)
    _reg0f = CBits(8, 0x0F, 0)
    _reg62 = CBits(8, 0x62, 0)

    def __init__(self, i2c, address=0x77) -> None:
        self._i2c = i2c
        self._address = address

        if self._device_id != 0x10:
            raise RuntimeError("Failed to find the DPS310 sensor!")

        self._pressure_scale = None
        self._temp_scale = None
        self._c0 = None
        self._c1 = None
        self._c00 = None
        self._c00 = None
        self._c10 = None
        self._c10 = None
        self._c01 = None
        self._c11 = None
        self._c20 = None
        self._c21 = None
        self._c30 = None
        self._oversample_scalefactor = (
            524288,
            1572864,
            3670016,
            7864320,
            253952,
            516096,
            1040384,
            2088960,
        )
        self._sea_level_pressure = 1013.25

        self.initialize()

    @property
    def pressure(self) -> float:
        """Returns the current pressure reading in hectoPascals (hPa)"""

        temp_reading = self._raw_temperature

        raw_temperature = self._twos_complement(temp_reading, 24)

        pressure_reading = self._raw_pressure

        raw_pressure = self._twos_complement(pressure_reading, 24)

        scaled_rawtemp = raw_temperature / self._temp_scale
        scaled_rawpres = raw_pressure / self._pressure_scale

        pres_calc = (
            self._c00
            + scaled_rawpres
            * (self._c10 + scaled_rawpres * (self._c20 + scaled_rawpres * self._c30))
            + scaled_rawtemp
            * (self._c01 + scaled_rawpres * (self._c11 + scaled_rawpres * self._c21))
        )

        final_pressure = pres_calc / 100

        return final_pressure

    def initialize(self) -> None:
        """Initialize the sensor to continuous measurement"""

        self.reset()

        self._pressure_osbits = 6
        self._pressure_shiftbit = True
        self._pressure_scale = self._oversample_scalefactor[6]

        self._temp_osbits = 6
        self._temp_scale = self._oversample_scalefactor[6]
        self._temp_shiftbit = True

        self._mode_bits = 7

        # wait until we have at least one good measurement
        self.wait_temperature_ready()
        self.wait_pressure_ready()

    # (https://github.com/Infineon/DPS310-Pressure-Sensor#temperature-measurement-issue)
    # similar to DpsClass::correctTemp(void) from infineon's c++ library
    def _correct_temp(self) -> None:
        """Correct temperature readings on ICs with a fuse bit problem"""
        self._reg0e = 0xA5
        self._reg0f = 0x96
        self._reg62 = 0x02
        self._reg0e = 0
        self._reg0f = 0

        # perform a temperature measurement
        # the most recent temperature will be saved internally
        # and used for compensation when calculating pressure
        _unused = self._raw_temperature

    def reset(self) -> None:
        """Reset the sensor"""
        self._reset_register = 0x89
        # wait for hardware reset to finish
        time.sleep(0.010)
        while not self._sensor_ready:
            time.sleep(0.001)
        self._correct_temp()
        self._read_calibration()
        # make sure we're using the temperature source used for calibration
        self._temp_measurement_src_bit = self._calib_coeff_temp_src_bit

    @property
    def pressure(self) -> float:
        """Returns the current pressure reading in hectoPascals (hPa)"""

        temp_reading = self._raw_temperature
        raw_temperature = self._twos_complement(temp_reading, 24)

        pressure_reading = self._raw_pressure
        raw_pressure = self._twos_complement(pressure_reading, 24)

        scaled_rawtemp = raw_temperature / self._temp_scale
        scaled_rawpres = raw_pressure / self._pressure_scale

        pres_calc = (
            self._c00
            + scaled_rawpres
            * (self._c10 + scaled_rawpres * (self._c20 + scaled_rawpres * self._c30))
            + scaled_rawtemp
            * (self._c01 + scaled_rawpres * (self._c11 + scaled_rawpres * self._c21))
        )

        final_pressure = pres_calc / 100
        return final_pressure

    @property
    def altitude(self) -> float:
        """The altitude in meters based on the sea level pressure
        (:attr:`sea_level_pressure`) - which you must enter
        ahead of time
        """
        return 44330 * (
            1.0 - math.pow(self.pressure / self._sea_level_pressure, 0.1903)
        )

    @property
    def temperature(self) -> float:
        """The current temperature reading in degrees Celsius"""
        _scaled_rawtemp = self._raw_temperature / self._temp_scale
        _temperature = _scaled_rawtemp * self._c1 + self._c0 / 2.0
        return _temperature

    @property
    def sea_level_pressure(self) -> float:
        """The local sea level pressure in hectoPascals (aka millibars). This is used
        for calculation of :attr:`altitude`. Values are typically in the range
        980 - 1030."""
        return self._sea_level_pressure

    @sea_level_pressure.setter
    def sea_level_pressure(self, value: float) -> None:
        self._sea_level_pressure = value

    def wait_temperature_ready(self) -> None:
        """Wait until a temperature measurement is available."""

        while self._temp_ready is False:
            time.sleep(0.001)

    def wait_pressure_ready(self) -> None:
        """Wait until a pressure measurement is available"""

        while self._pressure_ready is False:
            time.sleep(0.001)

    @staticmethod
    def _twos_complement(val: int, bits: int) -> int:
        if val & (1 << (bits - 1)):
            val -= 1 << bits

        return val

    def _read_calibration(self) -> None:
        """
        Read the calibration data from the sensor
        """
        while not self._coefficients_ready:
            time.sleep(0.001)

        coeffs = [None] * 18
        for offset in range(18):
            register = 0x10 + offset
            coeffs[offset] = struct.unpack(
                "B", self._i2c.readfrom_mem(self._address, register, 1)
            )[0]

        self._c0 = (coeffs[0] << 4) | ((coeffs[1] >> 4) & 0x0F)
        self._c0 = self._twos_complement(self._c0, 12)

        self._c1 = self._twos_complement(((coeffs[1] & 0x0F) << 8) | coeffs[2], 12)

        self._c00 = (coeffs[3] << 12) | (coeffs[4] << 4) | ((coeffs[5] >> 4) & 0x0F)
        self._c00 = self._twos_complement(self._c00, 20)

        self._c10 = ((coeffs[5] & 0x0F) << 16) | (coeffs[6] << 8) | coeffs[7]
        self._c10 = self._twos_complement(self._c10, 20)

        self._c01 = self._twos_complement((coeffs[8] << 8) | coeffs[9], 16)
        self._c11 = self._twos_complement((coeffs[10] << 8) | coeffs[11], 16)
        self._c20 = self._twos_complement((coeffs[12] << 8) | coeffs[13], 16)
        self._c21 = self._twos_complement((coeffs[14] << 8) | coeffs[15], 16)
        self._c30 = self._twos_complement((coeffs[16] << 8) | coeffs[17], 16)
