#-----------------------------------------------------------------------------
# __init__.py
#
#------------------------------------------------------------------------
#
#
# Written by  SparkFun Electronics, August 2020
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2020 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#==================================================================================
#-----------------------------------------------------------------------------
# this package encapsulates the resources used for the Micro OLED board
#
#-----------------------------------------------------------------------------

#
# This is mostly a port of existing Arduino functionaly, so pylint is sad.
# The goal is to keep the public interface pthonic, but internal is internal
#
# pylint: disable=line-too-long
#

"""
sparkfun_ublox_gps
=================
Python module for the [SparkFun GPS-RTK Dead Reckoning Phat
ZED-F9R](https://www.sparkfun.com/products/16475) and the others found
[here](https://www.sparkfun.com/gps), as long as you use
Serial and SPI.

This python package is a port of the existing [SparkFun Ublox Arduino
Library](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library).
"""

from .ublox_gps import UbloxGps
from .core import *
