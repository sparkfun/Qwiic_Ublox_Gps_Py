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

from . import core

import serial
import spidev
import struct
from . import sparkfun_predefines as sp

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
            sfeSpi = sfe_spi_wrapper(hard_port)
            self.hard_port = sfeSpi
        else: 
            self.hard_port = hard_port

        # Class message values
        self.ack_ms= {
            'ACK':0x01, 'NAK':0x00 
        }
        self.cfg_ms= {
            'OTP':0x41,    'PIO':0x2c,      'PRT':0x00,     'PT2':0x59,     'RST':0x04,
            'SPT':0x64,    'USBTEST':0x58,  'VALDEL':0x8c,  'VALGET':0x8b,
            'VALSET':0x8a
        }
        self.esf_ms= {
            'ALG':0x14,       'INS':0x15,    'MEAS':0x02,  'RAW':0x03,
            'RESETALG':0x13,  'STATUS':0x10
        }
        self.inf_ms= {
            'DEBUG':0x04,  'ERROR':0x00,   'NOTICE':0x02,
            'TEST':0x03,   'WARNING':0x01
        }
        self.mga_ms= {
            'ACK':0x60,       'BDS_EPH':0x03,
            'BDS_ALM':0x03,   'BDS_HEALTH':0x03,      'BDS_UTC':0x03,
            'DBD_POLL':0x80,  'DBD_IO':0x80,          'GAL_EPH':0x02,
            'GAL_ALM':0x02,   'GAL_TIMEOFFSET':0x02,  'GAL_UTC':0x02
        }
        self.mon_ms= {
            'COMMS':0x36,  'GNSS':0x28,  'HW3':0x37,  'PATCH':0x27,
            'PIO':0x24,    'PT2':0x2b,   'RF':0x38,   'RXR':0x21,
            'SPT':0x2f
        }
        self.nav_ms= {
            'ATT':0x05,        'CLOCK':0x22,     'COV':0x36,
            'DOP':0x04,        'EELL':0x3d,      'EOE':0x61,        'GEOFENCE':0x39,
            'HPPOSECEF':0x13,  'HPPOSLLH':0x14,  'ORB':0x34,        'POSECEF':0x01,
            'POSLLH':0x02,     'PVT':0x07,       'RELPOSNED':0x3c,  'SAT':0x35,
            'SBAS':0x32,       'SIG':0x43,       'STATUS':0x03,     'TIMBDS':0x24,
            'TIMEGAL':0x25,    'TIMEGLO':0x23,   'TIMEGPS':0x20,    'TIMELS':0x25,
            'TIMEQZSS':0x27,   'TIMEUTC':0x21,   'VELECEF':0x11,    'VELNED':0x12
        } 
        self.time_ms= {
            'TM2':0x03, 'TP':0x01, 'VRFY':0x06
        }

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
        
        return True

    def ubx_get_val(self, key_id):
        """
        This function takes the given key id and breakes it into individual bytes
        which are then cocantenated together. This payload is then sent along
        with the CFG Class and VALGET Message ID to send_message(). Ublox
        Messages are then parsed for the requested values or a NAK signifying a
        problem. 

        :return: The requested payload or a NAK on failure.
        :rtype: namedtuple
        """
        key_id_bytes = bytes([])
        if type(key_id) != bytes:
            while key_id > 0: 
                key_id_bytes = key_id_bytes + bytes([(key_id & 0xFF)])
                key_id = key_id >> 8 
                
        key_id_bytes = key_id_bytes[::-1]
        msg = self.send_message(sp.CFG_CLS, self.cfg_ms.get('VALGET'), key_id_bytes)
        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def enable_UART1(self, enable):
        if enable is True: 
            self.send_message(sp.CFG_CLS, self.cfg_ms.get('RST'), 0x00)

        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def geo_coords(self):
        """
        Sends a poll request for the NAV class with the PVT Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the NAV Class and PVT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('PVT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def hp_geo_coords(self):
        """
        Sends a poll request for the NAV class with the HPPOSLLH Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the NAV Class and HPPOSLLH Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('HPPOSLLH'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def date_time(self):
        """
        Sends a poll request for the NAV class with the PVT Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the NAV Class and PVT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('PVT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def satellites(self):
        """
        Sends a poll request for the NAV class with the SAT Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the NAV Class and SAT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('SAT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def veh_attitude(self):
        """
        Sends a poll request for the NAV class with the ATT Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the NAV Class and ATT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, self.nav_ms.get('ATT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)
    
    def stream_nmea(self):
        """
        Reads directly from the module's data stream, by default this is NMEA
        data. 

        :return: Returns NMEA data.
        :rtype: string
        """
        return(self.hard_port.readline().decode('utf-8'))

    def imu_alignment(self):
        """
        Sends a poll request for the ESF class with the ALG Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and ALG Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('ALG'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port) 
        return(payload)

    def vehicle_dynamics(self):
        """
        Sends a poll request for the ESF class with the INS Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and INS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, 0x15)
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return(payload)

    def esf_measures(self):
        """
        Sends a poll request for the ESF class with the MEAS Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and MEAS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('MEAS'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return(payload)

    def esf_raw_measures(self):
        """
        Sends a poll request for the ESF class with the RAW Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and RAW Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('RAW'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return(payload)

    def reset_imu_align(self):
        """
        Sends a poll request for the ESF class with the RESETALG Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and RESETALG Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('RESETALG'))
        parse_tool = core.Parser([sp.ACK_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return(payload)

    def esf_status(self):
        """
        Sends a poll request for the ESF class with the STATUS Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and STATUS Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.ESF_CLS, self.esf_ms.get('STATUS'))
        parse_tool = core.Parser([sp.ESF_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return(payload)
    
    def port_settings(self):
        """
        Sends a poll request for the MON class with the COMMS Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and COMMS Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('COMMS'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def module_gnss_support(self):
        """
        Sends a poll request for the MON class with the GNSS Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and GNSS Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('GNSS'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def pin_settings(self):
        """
        Sends a poll request for the MON class with the HW3 Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and HW3 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('HW3'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def installed_patches(self):
        """
        Sends a poll request for the MON class with the PATCH Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and HW3 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('HW3'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def prod_test_pio(self):
        """
        Sends a poll request for the MON class with the PIO Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and PIO Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('PIO'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def prod_test_monitor(self):
        """
        Sends a poll request for the MON class with the PT2 Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and PT2 Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('PT2'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def rf_ant_status(self):
        """
        Sends a poll request for the MON class with the RF Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the ESF Class and RF Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.esf_ms.get('RF'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)


    def module_wake_state(self):
        """
        Sends a poll request for the MON class with the RXR Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the MON Class and RXR Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('RXR'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def sensor_production_test(self):
        """
        Sends a poll request for the MON class with the SPT Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the MON Class and SPT Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('SPT'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def temp_val_state(self):
        """
        Sends a poll request for the MON class with the TEMP Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the MON Class and TEMP Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('TEMP'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)

    def module_software_version(self):
        """
        Sends a poll request for the MON class with the VER Message ID and 
        parses ublox messages for the response. The payload is extracted from 
        the response which is then passed to the user. 

        :return: The payload of the MON Class and VER Message ID
        :rtype: namedtuple
        """
        msg = self.send_message(sp.MON_CLS, self.mon_ms.get('VER'))
        parse_tool = core.Parser([sp.MON_CLS])
        msg = parse_tool.receive_from(self.hard_port) 
        return(msg)
    



class sfe_spi_wrapper(object):
    """
    sfe_spi_wrapper

    Initialize the library with the given port. 

    :param spi_port:    This library simply provides some ducktyping for spi so
                        that the ubxtranslator library doesn't complain. It
                        takes a spi port and then sets it to the ublox module's
                        specifications.
    
    :return:            The sfe_spi_wrapper object.
    :rtype:             Object
    """

    def __init__(self, spi_port = None):

        if spi_port is None:
            self.spi_port = spidev.SpiDev()
        else:
            self.spi_port = spi_port

        self.spi_port.open(0,0)
        self.spi_port.max_speed_hz = 5500 #Hz
        self.spi_port.mode = 0b00
        
    def read(self, read_data = 1): 
        """
        Reads a byte or bytes of data from the SPI port. The bytes are
        converted to a bytes object before being returned.

        :return: The requested bytes
        :rtype: bytes
        """

        data = self.spi_port.readbytes(read_data)
        byte_data = bytes([])
        for d in data: 
            byte_data = byte_data + bytes([d])
        return byte_data

    def write(self, data):
        """
        Writes a byte or bytes of data to the SPI port. 

        :return: True on completion
        :rtype: boolean
        """
        self.spi_port.xfer2(list(data))

        return True
            
            
