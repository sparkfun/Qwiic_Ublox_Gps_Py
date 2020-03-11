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

# pylint: disable-all
import ublox_comms

class UbloxGps(object):

    def __init__(self, comm_interface):
        self.comm_interface = comm_interface
    
    def get_altitude(self):
        pass
    
    def clear_configruation(self):
        packet = self.comm_interface.build_packet(UBX_CLASS_CFG, 
                                                  UBX_CFG_CFG,
                                                  0, 0)
        ublox_reponse = self.comm_interface.recieve_command(packet)
        return(ublox_reponse.get('payload'))

    def extract_byte(self, packet, location):
        payload = packet.get('payload')
        byte = payload[location]
        return byte 

    def extract_int(self, packet, location):
        payload = packet.get('payload')
        two_byte = payload[location] << 8
        two_byte = two_byte | payload[location + 1]
        return two_byte 

    def extract_long(self, packet, location):
        payload = packet.get('payload')
        four_byte = payload[location] << 24
        four_byte = four_byte | (payload[location + 1] << 16)
        four_byte = four_byte | (payload[location + 2] << 8)
        four_byte = four_byte | payload[location + 3] 
        return four_byte 

    def get_soft_version(self, packet, location):
        packet = self.comm_interface.build_packet(UBX_CLASS_MON,
                                                  UBX_MON_VER, 
                                                  0, 0)
        ublox_reponse = self.comm_interface.recieve_command(packet)
        payload = ublox_reponse.get('payload') 
        for byte in range(30): 
            soft_vers = soft_vers | payload[byte] 
            soft_vers = soft_vers << 8
            
        return soft_vers 

    def get_hard_version(self, packet, location):
        packet = self.comm_interface.build_packet(UBX_CLASS_MON,
                                                  UBX_MON_VER, 
                                                  0, 0)
        ublox_reponse = self.comm_interface.recieve_command(packet)
        payload = ublox_reponse.get('payload') 
        for byte in range(30, 41): 
            hard_vers = hard_vers | payload[byte] 
            hard_vers = hard_vers << 8
            
        return hard_vers 
