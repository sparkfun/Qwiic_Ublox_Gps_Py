#-----------------------------------------------------------------------------
# qwiic_gps_ublox.py
#
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

"""
qwiic_gps_ublox
============
 Python library for the SparkFun's line of u-Blox GPS units: [SparkFun GPS-RTK2\
 ZED-F9P](https://www.sparkfun.com/products/15136), [SparkFun GPS-RTK2\
 NEO-M8P](https://www.sparkfun.com/products/15005), [SparkFun GPS-RTK2\
 ZOE-M8Q](https://www.sparkfun.com/products/15193), [SparkFun GPS-RTK2\
 SAM-M8Q](https://www.sparkfun.com/products/15210).

This python package is a port of the existing [SparkFun u-blox Arduino\
Library](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library).

This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

"""
from __future__ import print_function, division

import time
import math
import sys
import qwiic_i2c

#======================================================================
# NOTE: For Raspberry Pi
#======================================================================
# For this sensor to work on the Raspberry Pi, I2C clock stretching
# must be enabled.
#
# To do this:
#   - Login as root to the target Raspberry Pi
#   - Open the file /boot/config.txt in your favorite editor (vi, nano ...etc)
#   - Scroll down until the bloct that contains the following is found:
#           dtparam=i2c_arm=on
#           dtparam=i2s=on
#           dtparam=spi=on
#   - Add the following line:
#           # Enable I2C clock stretching
#           dtparam=i2c_arm_baudrate=10000
#
#   - Save the file
#   - Reboot the raspberry pi
#======================================================================
def __checkIsOnRPi():

    # Are we on a Pi? First Linux?

    if sys.platform not in ('linux', 'linux2'):
        return False

    # we can find out if we are on a RPI by looking at the contents
    # of /proc/device-tree/compatable

    try:
        with open('/proc/device-tree/compatible', 'r') as fCompat:

            systype = fCompat.read()

            return systype.find('raspberrypi') != -1
    except IOError:
        return False

# check if stretching is set if on a rpi
#
def _checkForRPiI2CClockStretch():

    #are we on a rpi?
    if not __checkIsOnRPi():
        return

    # read the boot config file and see if the clock stretch param is set
    try:
        with open('/boot/config.txt') as fConfig:

            strConfig = fConfig.read()
            for line in strConfig.split('\n'):
                if line.find('i2c_arm_baudrate') == -1:
                    continue

                # start with a comment?
                if line.strip().startswith('#'):
                    break

                # is the value less <= 10000
                params = line.split('=')
                if int(params[-1]) <= 10000:
                    # Stretching is enabled and set correctly.
                    return

                break
    except IOError:
        pass

    # if we are here, then we are on a Raspberry Pi and Clock Stretching isn't
    # set correctly.
    # Print out a message!

    print("""
============================================================================
 NOTE:

 For and of the ublox gps units to work on the Raspberry Pi, I2C clock stretching
 must be enabled.

 The following line must be added to the file /boot/config.txt

    dtparam=i2c_arm_baudrate=10000

 For more information, see the note at:
          https://github.com/sparkfun/qwiic_gps_ublox
============================================================================
        """)

# Define the device name and I2C addresses. These are set in the class defintion
# as class variables, making them avilable without having to create a class instance.
# This allows higher level logic to rapidly create a index of qwiic devices at
# runtine
#
# The name of this device
_DEFAULT_NAME = "Qwiic GPS u-blox"

# Some devices have multiple availabel addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the
# device.
_AVAILABLE_I2C_ADDRESS = [0x42]

class QwiicGpsUblox(object):
    """
    QwiicGpsUblox

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The gpsUblox device object.
        :rtype: Object
    """
    # Constructor
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Library variables
    i2c_polling_wait = .1 # 100ms delay between checking for data
    last_checked     = 0  # set to zero
    command_ack      = False
    payload_config   = [0 for i in range(MAX_PAYLOAD_SIZE)] 
    payload_ack      = [0 for i in range(2)] 

    # This is our C++ 'struct'
    ublox_packet_cfg = { "Class"         : 0,
                         "ID"            : 0,
                         "Length"        : 0,
                         "Counter"       : 0,
                         "Start"         : 0,
                         "Payload"       : payload_config,
                         "Checksum_A"    : 0,
                         "Checksum_B"    : 0,
                         "Checksum_Pass" : False }

    ublox_packet_ack = { "Class"         : 0,
                         "ID"            : 0,
                         "Length"        : 0,
                         "Counter"       : 0,
                         "Start"         : 0,
                         "Payload"       : payload_ack,
                         "Checksum_A"    : 0,
                         "Checksum_B"    : 0,
                         "Checksum_Pass" : False }

    module_queried = { 'gps_iTOW'         : 1,
                       'gps_year'         : 1,
                       'gps_month'        : 1,
                       'gps_day'          : 1,
                       'gps_hour'         : 1,
                       'gps_minute'       : 1,
                       'gps_second'       : 1,
                       'gps_nanosecond'   : 1,

                       'all'              : 1,
                       'longitude'        : 1,
                       'latitude'         : 1,
                       'altitude'         : 1,
                       'altitude_MSL'     : 1,
                       'SIV'              : 1,
                       'fix_type'         : 1,
                       'carrier_solution' : 1,
                       'ground_speed'     : 1,
                       'heading_motion'   : 1,
                       'pDOP'             : 1,
                       'version_num'      : 1  }

    high_res_module_queried = { 'all'                 : 1,
                                'time_of_week'        : 1,
                                'high_res_latitude'   : 1,
                                'high_res_longitude'  : 1,
                                'elipsoid'            : 1,
                                'mean_sea_level'      : 1,
                                'geo_id_separation'   : 1,
                                'horizontal_accuracy' : 1,
                                'vertical_accuracy'   : 1  }

	# Relative Positioning Info in NED frame specific controls.
    relative_pos_info = {    'ref_station_ID'  : 0,
                             'rel_pos_N'        : 0,
                             'rel_pos_E'        : 0,
                             'rel_pos_D'        : 0,

                             'rel_pos_length'   : 0,
                             'rel_pos_heading'  : 0,

                             'rel_pos_HPN'      : 0,
                             'rel_pos_HPE'      : 0,
                             'rel_pos_HPD'      : 0,
                             'rel_pos_HPLength' : 0,

                             'acc_N'            : 0,
                             'acc_E'            : 0,
                             'acc_D'            : 0,

                             'gnss_fix_Ok'      : False,
                             'diff_sol_n'       : False,
                             'rel_pos_valid'    : False,
                             'carr_sol_n'       : 0,
                             'is_moving'        : False,
                             'ref_pos_miss'     : False,
                             'ref_obs_miss'     : False  }

    # u-blox Register List
    UBX_SYNCH_1    = 0xB5
    UBX_SYNCH_2    = 0x62

    UBX_CLASS_NAV  = 0x01
    UBX_CLASS_RXM  = 0x02
    UBX_CLASS_INF  = 0x04
    UBX_CLASS_ACK  = 0x05
    UBX_CLASS_CFG  = 0x06
    UBX_CLASS_UPD  = 0x09
    UBX_CLASS_MON  = 0x0A
    UBX_CLASS_AID  = 0x0B
    UBX_CLASS_TIM  = 0x0D
    UBX_CLASS_ESF  = 0x10
    UBX_CLASS_MGA  = 0x13
    UBX_CLASS_LOG  = 0x21
    UBX_CLASS_SEC  = 0x27
    UBX_CLASS_HNR  = 0x28

    UBX_CFG_PRT    = 0x00    # Used to configure port specifics
    UBX_CFG_RST    = 0x04    # Used to reset device
    UBX_CFG_RATE   = 0x08    # Used to set port baud rates
    UBX_CFG_CFG    = 0x09    # Used to save current configuration
    UBX_CFG_VALSET = 0x8A    # Used for config of higher version Ublox modules (ie protocol v27 and above)
    UBX_CFG_VALGET = 0x8B    # Used for config of higher version Ublox modules (ie protocol v27 and above)
    UBX_CFG_VALDEL = 0x8C    # Used for config of higher version Ublox modules (ie protocol v27 and above)

    UBX_CFG_TMODE3    = 0x71 # Used to enable Survey In Mode
    SVIN_MODE_DISABLE = 0x00
    SVIN_MODE_ENABLE  = 0x01

    UBX_NAV_PVT       = 0x07 # All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites
    UBX_NAV_HPPOSECEF = 0x13 # Find our positional accuracy (high precision)
    UBX_NAV_HPPOSLLH  = 0x14 # Used for obtaining lat/long/alt in high precision
    UBX_NAV_SVIN      = 0x3B # Used for checking Survey In status
    UBX_NAV_RELPOSNED = 0x3C # Relative Positioning Information in NED frame

    UBX_MON_VER   = 0x04     # Used for obtaining Protocol Version
    UBX_MON_TXBUF = 0x08     # Used for query tx buffer size/state

                             # The following constants are used to enable RTCM messages
    UBX_CFG_MSG   = 0x01
    UBX_RTCM_MSB  = 0xF5     # All RTCM enable commands have 0xF5 as MSB
    UBX_RTCM_1005 = 0x05     # Stationary RTK reference ARP
    UBX_RTCM_1074 = 0x4A     # GPS MSM4
    UBX_RTCM_1077 = 0x4D     # GPS MSM7
    UBX_RTCM_1084 = 0x54     # GLONASS MSM4
    UBX_RTCM_1087 = 0x57     # GLONASS MSM7
    UBX_RTCM_1094 = 0x5E     # Galileo MSM4
    UBX_RTCM_1124 = 0x7C     # BeiDou MSM4
    UBX_RTCM_1230 = 0xE6     # GLONASS code-phase biases, set to once every 10 seconds

    UBX_ACK_NACK  = 0x00
    UBX_ACK_ACK   = 0x01

    # The following constants are used to configure the various ports and streams for those ports.
    # See -CFG-PRT.
    COM_PORT_I2C   = 0
    COM_PORT_UART1 = 1
    COM_PORT_UART2 = 2
    COM_PORT_USB   = 3
    COM_PORT_SPI   = 4

    COM_TYPE_UBX   = (1 << 0)
    COM_TYPE_NMEA  = (1 << 1)
    COM_TYPE_RTCM3 = (1 << 5)

    # The following constants are used to generate KEY values for the advanced protocol
    # functions of VELGET/SET/DEL
    VAL_SIZE_1  = 0x01 # One bit
    VAL_SIZE_8  = 0x02 # One byte
    VAL_SIZE_16 = 0x03 # Two bytes
    VAL_SIZE_32 = 0x04 # Four bytes
    VAL_SIZE_64 = 0x05 # Eight bytes

    # These are the Bitfield layers definitions for the UBX-CFG-VALSET message
    # (not to be confused with Bitfield deviceMask in UBX-CFG-CFG).
    VAL_LAYER_RAM   = (1 << 0)
    VAL_LAYER_BBR   = (1 << 1)
    VAL_LAYER_FLASH = (1 << 2)

    # Below are various Groups, IDs, and sizes for various settings
    # These can be used to call getVal/setVal/delVal.
    VAL_GROUP_I2COUTPROT      = 0x72
    VAL_GROUP_I2COUTPROT_SIZE = VAL_SIZE_1 # All fields in I2C group are currently 1 bit

    VAL_ID_I2COUTPROT_UBX   = 0x01
    VAL_ID_I2COUTPROT_NMEA  = 0x02
    VAL_ID_I2COUTPROT_RTCM3 = 0x03

    VAL_GROUP_I2C      = 0x51
    VAL_GROUP_I2C_SIZE = VAL_SIZE_8 # All fields in I2C group are currently 1 byte

    VAL_ID_I2C_ADDRESS = 0x01

    MAX_PAYLOAD_SIZE = 128

    _RPiCheck = False

    def __init__(self, address=None, i2c_driver=None):


        # As noted above, to run this device on a Raspberry Pi,
        # clock streching is needed.
        #
        # Lets check if it's enabled. This is done only once in
        # the session
        if not QwiicCcs811._RPiCheck:
            _checkForRPiI2CClockStretch()
            QwiicCcs811._RPiCheck = True

        # Did the user specify an I2C address?

        self.address = address if address is not None else self.available_addresses[0]

        # load the I2C driver if one isn't provided

        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

    # ----------------------------------
    # is_connected()
    #
    # Is an actual board connected to our system?

    def is_connected(self):
        """
            Determine if a gps device is conntected to the system..

            :return: True if the device is connected, otherwise False.
            :rtype: bool

        """
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(is_connected)
    # ----------------------------------
    # begin()
    #
    # Initialize the system/validate the board.
    def begin(self):
        """
            Initialize the data transmission lines.

            :return: Returns True on success, False on failure
            :rtype: boolean

        """

        return self.is_connected()

    def check_ublox_i2C(self):
        """
            Checks to see if GPS data is available.
            :return: Returns True when GPS data is available, and Flase when
            not.
            :rtype: boolean
        """
        # We only want to poll every 100ms as per the datasheet.
        if (time.perf_counter() - last_checked) >= i2c_polling_wait:

            bytes_avail = self._i2c.readBlock(self.available_addresses, 0xFD, 2)

            # Check LSB for 0xFF  == No bytes available
            if (bytes_avail | 0x00FF)  == 0xFF:
                last_checked = time.perf_counter()
                return False

            elif bytes_avail ==  0:
                last_checked = time.perf_counter()
                return False

            else:
                return True

        # There's some additional error checking for the RTK version that
        # should be added here: bit error (extremely rare edge case).

        def set_i2c_address(device_address, max_wait):
            """
                Changes the software configureable I2C address of the GPS
                product.
                :return: Returns True on success and False otherwise
                :rtype: boolean
            """

            ublox_packet_cfg['Class']  = UBX_CLASS_CFG
            ublox_packet_cfg['ID']     = UBX_CFG_PRT
            ublox_packet_cfg['Length'] = 20
            ublox_packet_cfg['Start']  = 0

            ublox_packet_cfg['Payload'][4] = device_address << 1;

            if send_command(ublox_packet_cfg, max_wait) == true:
                self.available_addresses[0] = device_address
                return True

            return False

        def send_command(outgoing_ubx_packet, max_wait):

            command_ack = False;
            calc_checksum(outgoing_ubx_packet)

            if not self.send_i2c_command(outgoing_ubx_packet, max_wait):
                return False

            if max_wait > 0:
                return self.wait_for_response(outgoing_ubx_packet['Class'],
                                         outgoing_ubx_packet['ID'],
                                         max_wait)
            return True

        def send_i2c_command(outgoing_ubx_packet, max_wait):

            self._i2c.writeByte(self.available_addresses[0], 0xFF,
                                self.UBX_SYNCH_1)
            self._i2c.writeByte(self.available_addresses[0], 0xFF,
                                self.UBX_SYNCH_2)
            self._i2c.writeByte(self.available_addresses[0], 0xFF,
                                self.outgoing_ubx_packet['Class'])
            self._i2c.writeByte(self.available_addresses[0], 0xFF,
                                self.outgoing_ubx_packet['ID'])
            self._i2c.writeWord(self.available_addresses[0], 0xFF,
                                self.outgoing_ubx_packet['Length'])
            
            bytes_to_send = outgoing_ubx_packet['Length']
            
            # I'm going to ignore any limitations of the buffer for now...
            for i in range(len(bytes_to_send - 1)): # Stop before last write
                self._i2c.writeByte(outgoing_ubx_packet['Payload'][i])

            # Now write the last available byte in payload_config
            self._i2c.writeByte(outgoing_ubx_packet['Payload'][bytes_to_send]) #unsure about this
            self._i2c.writeByte(outgoing_ubx_packet['Checksum_A']) 
            self._i2c.writeByte(outgoing_ubx_packet['Checksum_B'])

            return True

        def calc_checksum(ubx_packet):

            ubx_packet['Checksum_A'] = 0
            ubx_packet['Checksum_B'] = 0

            ubx_packet['Checksum_A'] += ubx_packet['Class']
            ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']
            
            ubx_packet['Checksum_A'] += ubx_packet['ID']
            ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']
        
            ubx_packet['Checksum_A'] += ubx_packet['Length'] & 0xFF # LSB first
            ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']

            ubx_packet['Checksum_A'] += ubx_packet['Length'] >> 8 # MSB next
            ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']

            for i in range(len(ubx_packet['Length'])):
                ubx_packet['Checksum_A'] += ubx_packet['Payload'][i]
                ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']

        
        def get_latitude():

            if 
