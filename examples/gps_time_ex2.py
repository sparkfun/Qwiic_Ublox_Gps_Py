#!/usr/bin/env python3
#-----------------------------------------------------------------------------
# gps_time_ex2.py
#
# Simple Example for SparkFun ublox GPS products 
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, July 2020
# 
# Do you like this library? Help support SparkFun. Buy a board!
# https://sparkfun.com
#==================================================================================
# GNU GPL License 3.0
# Copyright (c) 2020 SparkFun Electronics
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#==================================================================================
# Example 2
# This example sets up the serial port and then passes it to the UbloxGPs
# library. From here we call date_time() and to get gps time and check whether
# the data received is "valid" which indicates that the probability of the time
# to be correct is very high. 

import serial

from ublox_gps import UbloxGps

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():

    try:
        print("Listening for UBX Messages")
        while True:
            try:
                gps_time = gps.date_time()
                print("{}/{}/{}".format(gps_time.day, gps_time.month,
                                          gps_time.year))
                print("UTC Time {}:{}:{}".format(gps_time.hour, gps_time.min,
                                          gps_time.sec))
                print("Valid date:{}\nValid Time:{}".format(gps_time.valid.validDate, 
                                                             gps_time.valid.validTime))
            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == '__main__':
    run()
