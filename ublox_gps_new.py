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
import struct
import sparkfun_predefines as sp

class UbloxGps(object):

    def __init__(self, hard_port = None):
        if hard_port is None: 
            self.hard_port = serial.Serial("/dev/serial0/", 38400, timeout=1)
        else:
            self.hard_port = hard_port 

        # Add SPI here

#        self.parse_tool = core.Parser([sp.ACK_CLS, sp.CFG_CLS, sp.ESF_CLS,
#                                       sp.INF_CLS, sp.MGA_CLS, sp.MON_CLS,
#                                       sp.NAV_CLS, sp.TIM_CLS])
    
    def send_message(self, ubx_class, ubx_id, ubx_payload = None): 

        SYNC_CHAR1 = 0xB5
        SYNC_CHAR2 = 0x62

        if ubx_payload == b'\x00' or ubx_payload is None:
            payload_length = 0
        elif type(ubx_payload) is not bytes:
            ubx_payload = bytes([ubx_payload])
            payload_length = len(ubx_payload)
            
        if payload_length > 0:
            message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2, 
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF), 
                                  (payload_length >> 8)) + ubx_payload

        else: 
            message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2, 
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF), 
                                  (payload_length >> 8)) 

        checksum = core.Parser._generate_fletcher_checksum(message[2:])

        self.hard_port.write(message + checksum)
        
        return

    def request_message(self, ubx_class, ubx_id): 
        
        self.send_message(ubx_class, ubx_id)

        parse_tool = core.Parser([ubx_class])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)
            
    def message_version(self):

        msg = self.send_message(sp.MON_CLS, 0x0e)

        parse_tool = core.Parser([sp.MON_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port) 

        return(msg)

    def enable_UART1(self, enable):
        if enable is True: 
            self.send_message(sp.CFG_CLS, 0x04, 0x00)

        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)
                         

    def get_nav(self):

        self.send_message(sp.NAV_CLS, i)
        parse_tool = core.Parser([sp.NAV_CLS, sp.ACK_CLS])
        cls_name, msg_name, payload  = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def get_geo_coords(self):

        self.send_message(sp.NAV_CLS, 0x07)
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def get_date_time(self):
         
        self.send_message(sp.NAV_CLS, 0x07)
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def get_satellites(self):

        self.send_message(sp.NAV_CLS, 0x35)
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def get_veh_att(self):

        self.send_message(sp.NAV_CLS, 0x01)
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)
