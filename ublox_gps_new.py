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
# This python library suphard_ports the SparkFun Electroncis qwiic
# qwiic sensor/board ecosystem
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help suphard_port SparkFun. Buy a board!
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
# copies or substantial hard_portions of the Software.
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
# This is mostly a hard_port of existing Arduino functionaly, so pylint is sad.
# The goal is to keep the public interface pthonic, but internal is internal
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

from ubxtranslator import core

import serial
import sparkfun_predefines as sp

class UbloxGps(object):

    def __init__(self, hard_port = None):
        if hard_port is None: 
            self.hard_port = serial.Serial("/dev/serial0/", 38400, timeout=1)
        else:
            self.hard_port = hard_port 

        self.parse_tool = core.Parser([sp.ACK_CLS, sp.CFG_CLS, sp.ESF_CLS,
                                       sp.INF_CLS, sp.MGA_CLS, sp.MON_CLS,
                                       sp.NAV_CLS, sp.TIM_CLS])
    
    def send_packet(self, _port): 

        if _port is None:
            _port = self.hard_port

        packet = bytes("\x06\x8a\x00\x00\x00",'utf8')
        print(packet)
        checksum = core.Parser._generate_fletcher_checksum(packet)
        print(hex(checksum[0]), hex(checksum[1]))

        _port.write(0xb5)
        _port.write(0x62)
        _port.write(0x06)
        _port.write(0x8a)
        _port.write(0x00)#length lsb
        _port.write(0x00)#length msb
        _port.write(0x0)#payload
        _port.write(checksum[0])#checksuma
        _port.write(checksum[1])#checksumb


        return

    def request_packet(self, ubx_class, ubx_id): 

        packet = ubx_class +  ubx_id
        checksum = core.Parser._generate_fletcher_checksum(packet)
        packet = packet + checksum
        return packet
         
            


                         
