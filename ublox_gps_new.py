# Python library for the SparkFun's line of u-Blox GPS units.
#
# SparkFun GPS-RTK2 ZED-F9P:
#   https://www.sparkfun.com/products/15136
# SparkFun GPS-RTK2 NEO-M8P:
#   https://www.sparkfun.com/products/15005
# SparkFun GPS-RTK2 ZOE-M8Q:
#   https://www.sparkfun.com/products/15193
# SparkFun GPS-RTK2 SAM-M8Q:
#   https://www.sparkfun.com/products/15210
#
#------------------------------------------------------------------------
#
# Written by SparkFun Electronics, October 2019
#
# This python library supports the SparkFun Electroncis qwiic
# qwiic sensor/board ecosystem
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
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
#
# This is mostly a port of existing Arduino functionaly, so pylint is sad.
# The goal is to keep the public interface pthonic, but internal is internal
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

import ubxTranslator as ubx


class UbloxGps(object):

    def __init__(self, port_settings = None):
        if port_settings is not None: 
            self.port_settings = port_settings
        else:
            self.port_settings = {
                'port': '/dev/serial0',
                'baud': 38400,
                'timeout': 1
            } 

    
    def send_packet(self, port = self.port, packet): 
        


        with serial.Serial(self.port_settings.get('port'),
                           self.port_settings.get('baud'),
                           timeout=self.port_settings.get('timemout')) as ser:

            ser.write(ubc.UBX_SYNCH_1)
            ser.write(ubc.UBX_SYNCH_2)
            ser.write(packet.get('ubx_class'))
            ser.write(packet.get('ubx_id'))
            ser.write(packet.get('ubx_length_lsb'))
            ser.write(packet.get('ubx_length_msb'))
            ser.write(0)#payload
            ser.write(packet.get('ubx_checkA'))
            ser.write(packet.get('ubx_checkB'))

            sleep(.5) # 500ms wait

            ublox_response = []

            for byte in range(ser.in_waiting):
                ublox_response.append(ser.read())

            if ublox_response:
                ubx_response = self.build_response(ublox_response)
                return ubx_response
            else:
                return {}
