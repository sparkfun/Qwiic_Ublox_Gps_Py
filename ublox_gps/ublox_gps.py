# Python library for the SparkFun's line of u-Blox GPS units.
#
# SparkFun GPS-RTK NEO-M8P:
#   https://www.sparkfun.com/products/15005
# SparkFun GPS-RTK2 ZED-F9P:
#   https://www.sparkfun.com/products/15136
# SparkFun GPS ZOE-M8Q:
#   https://www.sparkfun.com/products/15193
# SparkFun GPS SAM-M8Q:
#   https://www.sparkfun.com/products/15210
# SparkFun GPS-RTK Dead Reckoning Phat ZED-F9R:
#   https://www.sparkfun.com/products/16475
# SparkFun GPS-RTK Dead Reckoning ZED-F9R:
#   https://www.sparkfun.com/products/16344
# SparkFun GPS Dead Reckoning NEO-M9N:
#   https://www.sparkfun.com/products/15712
#
#------------------------------------------------------------------------
# Written by SparkFun Electronics, July 2020
#
# Do you like this library? Help suphard_port SparkFun. Buy a board!
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

import struct
import serial
import spidev

import threading
import time

import sys

from . import sfeSpiWrapper
from . import sparkfun_predefines as sp
from . import core

class UbloxGps(object):
    """
    UbloxGps

    Initialize the library with the given port.

    :param hard_port:   The port to use to communicate with the module, this
                        can be a serial or SPI port. If no port is given, then the library
                        assumes serial at a 38400 baud rate.

    :return:            The UbloxGps object.
    :rtype:             Object
    """

    def __init__(self, hard_port = None):
        if hard_port is None:
            self.hard_port = serial.Serial("/dev/serial0/", 38400, timeout=1)
        elif type(hard_port) == spidev.SpiDev:
            sfeSpi = sfeSpiWrapper(hard_port)
            self.hard_port = sfeSpi
        else:
            self.hard_port = hard_port

        self.packets = {}

        self.pckt_scl = {
            'lon' : (10**-7),
            'lat' :  (10**-7),
            'headMot ' :  (10**-5),
            'headAcc ' :  (10**-5),

            'pDOP' :  0.01,
            'gDOP' : 0.01,
            'tDOP' : 0.01,
            'vDOP' : 0.01,
            'hDOP' : 0.01,
            'nDOP' : 0.01,
            'eDOP' : 0.01,

            'headVeh ' :  (10**-5),
            'magDec ' :  (10**-2),
            'magAcc ' :  (10**-2),

            'lonHp' : (10**-9),
            'latHp' : (10**-9),
            'heightHp' : 0.1,
            'hMSLHp' : 0.1,
            'hAcc' : 0.1,
            'vAcc' : 0.1,

            'errEllipseOrient': (10**-2),

            'ecefX' : 0.1,
            'ecefY' : 0.1,
            'ecefZ' : 0.1,
            'pAcc' : 0.1,

            'prRes' : 0.1,

            'cAcc' : (10**-5),
            'heading' : (10**-5),

            'relPosHeading' : (10**-5),
            'relPosHPN' : 0.1,
            'relPosHPE' : 0.1,
            'relPosHPD' : 0.1,
            'relPosHPLength' : 0.1,
            'accN' : 0.1,
            'accE' : 0.1,
            'accD' : 0.1,
            'accLength' : 0.1,
            'accPitch' : (10**-5),
            'accHeading' : (10**-5),

            'roll' : (10**-5),
            'pitch' : (10**-5),
        }


        # Class message values
        self.cls_ids = {
            'CFG' : sp.CFG_CLS,
            'ACK' : sp.ACK_CLS,
            'ESF' : sp.ESF_CLS,
            'INF' : sp.INF_CLS,
            'MGA' : sp.MGA_CLS,
            'MON' : sp.MON_CLS,
            'NAV' : sp.NAV_CLS
        }

        self.cls_ms = {}

        self.cls_ms['ACK']= {
            'ACK':0x01, 'NAK':0x00
        }
        self.cls_ms['CFG']= {
            'OTP':0x41,    'PIO':0x2c,      'PRT':0x00,     'PT2':0x59,     'RST':0x04,
            'SPT':0x64,    'USBTEST':0x58,  'VALDEL':0x8c,  'VALGET':0x8b,
            'VALSET':0x8a
        }
        self.cls_ms['ESF']= {
            'ALG':0x14,       'INS':0x15,    'MEAS':0x02,  'RAW':0x03,
            'RESETALG':0x13,  'STATUS':0x10
        }
        self.cls_ms['INF']= {
            'DEBUG':0x04,  'ERROR':0x00,   'NOTICE':0x02,
            'TEST':0x03,   'WARNING':0x01
        }
        self.cls_ms['MGA']= {
            'ACK':0x60,       'BDS_EPH':0x03,
            'BDS_ALM':0x03,   'BDS_HEALTH':0x03,      'BDS_UTC':0x03,
            'DBD_POLL':0x80,  'DBD_IO':0x80,          'GAL_EPH':0x02,
            'GAL_ALM':0x02,   'GAL_TIMEOFFSET':0x02,  'GAL_UTC':0x02
        }
        self.cls_ms['MON']= {
            'COMMS':0x36,  'GNSS':0x28,  'HW3':0x37,  'PATCH':0x27,
            'PIO':0x24,    'PT2':0x2b,   'RF':0x38,   'RXR':0x21,
            'SPT':0x2f
        }
        self.cls_ms['NAV']= {
            'ATT':0x05,        'CLOCK':0x22,     'COV':0x36,
            'DOP':0x04,        'EELL':0x3d,      'EOE':0x61,        'GEOFENCE':0x39,
            'HPPOSECEF':0x13,  'HPPOSLLH':0x14,  'ORB':0x34,        'POSECEF':0x01,
            'POSLLH':0x02,     'PVT':0x07,       'RELPOSNED':0x3c,  'SAT':0x35,
            'SBAS':0x32,       'SIG':0x43,       'STATUS':0x03,     'TIMBDS':0x24,
            'TIMEGAL':0x25,    'TIMEGLO':0x23,   'TIMEGPS':0x20,    'TIMELS':0x25,
            'TIMEQZSS':0x27,   'TIMEUTC':0x21,   'VELECEF':0x11,    'VELNED':0x12
        }
        self.cls_ms['TIME']= {
            'TM2':0x03, 'TP':0x01, 'VRFY':0x06
        }

        self.parse_tool = core.Parser(self.cls_ids.values())

        self.stopping = False

        self.thread = threading.Thread(target=self.run_packet_reader, args=())
        self.thread.daemon = True
        self.thread.start()


    def set_packet(self, cls_name, msg_name, payload):
        if not(cls_name in self.packets):
            self.packets[cls_name] = {}

        if (payload is None):
            if msg_name in self.packets[cls_name]:
                del self.packets[cls_name][msg_name]
        else:
            self.packets[cls_name][msg_name] = payload

    def wait_packet(self, cls_name, msg_name):
        if not(cls_name in self.packets):
            self.packets[cls_name] = {}

        while (not(msg_name in self.packets[cls_name])):
            time.sleep(0.1)

        return self.packets[cls_name][msg_name]

    def run_packet_reader(self):
        while True:
            cls_name, msg_name, payload = self.parse_tool.receive_from(self.hard_port)
            self.set_packet(cls_name, msg_name, payload)

            if (self.stopping):
                break

            time.sleep(0.1)

    def stop(self):
        self.stopping = True
        self.thread.join()

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.stop()

    def send_message(self, ubx_class, ubx_id, ubx_payload = None):
        """
        Sends a ublox message to the ublox module.

        :param ubx_class:   The ublox class with which to send or receive the
                            message to/from.
        :param ubx_id:      The message id under the ublox class with which
                            to send or receive the message to/from.
        :param ubx_payload: The payload to send to the class/id specified. If
                            none is given than a "poll request" is
                            initiated.
        :return: True on completion
        :rtype: boolean
        """

        SYNC_CHAR1 = 0xB5
        SYNC_CHAR2 = 0x62

        if ubx_payload == b'\x00' or ubx_payload is None:
            payload_length = 0
        elif type(ubx_payload) is not bytes:
            ubx_payload = bytes([ubx_payload])
            payload_length = len(ubx_payload)
        else:
            payload_length = len(ubx_payload)

        message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
                      ubx_class.id_, ubx_id, (payload_length & 0xFF),
                      (payload_length >> 8))

        if payload_length > 0:
             message = message + ubx_payload

        checksum = core.Parser._generate_fletcher_checksum(message[2:])

        self.hard_port.write(message + checksum)

        return True

    def request_standard_packet(self, ubx_class_id, ubx_id):
        """
        Sends a poll request for the ubx_class_id class with the ubx_id Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then scaled and passed to the user.
        :return: The payload of the ubx_class_id Class and ubx_id Message ID
        :rtype: namedtuple
        """
        self.set_packet(ubx_class_id, ubx_id, None)

        self.send_message(self.cls_ids[ubx_class_id], self.cls_ms[ubx_class_id][ubx_id])

        return self.scale_packet(self.wait_packet(ubx_class_id, ubx_id))

    def scale_packet(self, packet):
        dict_packet= packet._asdict();
        for (k,v) in dict_packet.items():
            if k in self.pckt_scl:
                dict_packet[k] = self.pckt_scl[k] * v

        return type(packet)(**dict_packet)


    def geo_coords(self):
        return self.request_standard_packet('NAV', 'PVT')

    def hp_geo_coords(self):
        return self.request_standard_packet('NAV', 'HPPOSLLH')

    def satellites(self):
        return self.request_standard_packet('NAV', 'SAT')

    def veh_attitude(self):
        return self.request_standard_packet('NAV', 'ATT')

    def port_settings(self):
        return self.request_standard_packet('MON', 'COMMS')

    def module_gnss_support(self):
        return self.request_standard_packet('MON', 'GNSS')

    def rf_ant_status(self):
        return self.request_standard_packet('MON', 'RF')
