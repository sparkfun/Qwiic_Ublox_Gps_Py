#!/usr/bin/env python3
#-----------------------------------------------------------------------------
# modifying_configuration_settings_ex5.py
#
# Simple Example for SparkFun ublox GPS products 
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, July 2020
# 
# Do you like this library? Help support SparkFun. Buy a board!
# https://sparkfun.com
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
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
# Example 5
# Ublox has changed their method for changing your module's settings. The new
# method requires passing the module the "key id" of the configuration you want
# to modify followed by the value you want to set. If you're requesting
# information then you'd pass the key id without a value. In both cases the
# response will be the configuration data that was set upon success and a NAK
# on a failure. Check the interface description datasheet for more information. 

import serial

from ublox_gps import UbloxGps

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():

    try:
        print("Listening for UBX Messages")
        while True:
            try:
                # Get NMEA Protocol Version
                get_set = gps.ubx_get_set_del(0x20930001)
                print(get_set)
            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == '__main__':
    run()
