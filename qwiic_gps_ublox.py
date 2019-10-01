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
    auto_pvt = False
    auto_pvt_implicit_update = False
    i2c_polling_wait = .1 # 100ms delay between checking for data
    last_checked = 0  # set to zero
    command_ack = False
    payload_config = [0 for i in range(MAX_PAYLOAD_SIZE)]
    payload_ack = [0 for i in range(2)]

    # The major datums we want to globally store
    gps_year;
    gps_month;
    gps_day;
    gps_hour;
    gps_minute;
    gps_second;
    gps_millisecond;
    gps_nanosecond;

    latitude;          # Degrees * 10^-7 (more accurate than floats)
    longitude;         # Degrees * 10^-7 (more accurate than floats)
    altitude;          # Number of mm above ellipsoid
    altitude_MSL;      # Number of mm above Mean Sea Level
    SIV;               # Number of satellites used in position solution
    fix_type;          # Tells us when we have a solution aka lock
    carrier_solution;  # Tells us when we have an RTK float/fixed solution
    ground_speed;      # mm/s
    heading_motion; # degrees * 10^-5
    pDOP;              # Positional dilution of precision
    version_low;       # Loaded from getProtocolVersion().
    version_high;

    time_of_week;
    high_res_latitude;
    high_res_longitude;
    elipsoid;
    mean_sea_level;
    geo_id_separation;
    horizontal_accuracy;
    vertical_accuracy;

    rtcm_frame_counter = 0 # Tracks the type of incoming byte inside RTCM frame
    ubx_frame_counter = 0
    rolling_checksum_A = 0
    rolling_checksum_B = 0

    # Dictionaries that hold packets to send to GPS unit
    ublox_packet_cfg = {
        'Class'         : 0,
        'ID'            : 0,
        'Length'        : 0,
        'Counter'       : 0,
        'Start'         : 0,
        'Payload'       : payload_config,
        'Checksum_A'    : 0,
        'Checksum_B'    : 0,
        'Checksum_Pass' : False,
        'Valid'         : False
        }

    ublox_packet_ack = {
        'Class'          : 0,
        'ID'             : 0,
        'Length'         : 0,
        'Counter'        : 0,
        'Start'          : 0,
        'Payload'        : payload_ack,
        'Checksum_A'     : 0,
        'Checksum_B'     : 0,
        'Checksum_Pass'  : False,
        'Valid'          : False
        }

    is_module_queried = {
        'GPS_iTOW'         : True,
        'GPS_year'         : True,
        'GPS_month'        : True,
        'GPS_day'          : True,
        'GPS_hour'         : True,
        'GPS_minute'       : True,
        'GPS_second'       : True,
        'GPS_nanosecond'   : True,
        'All'              : True,
        'Longitude'        : True,
        'Latitude'         : True,
        'Altitude'         : True,
        'Altitude_MSL'     : True,
        'SIV'              : True,
        'fix_type'         : True,
        'carrier_solution' : True,
        'ground_speed'     : True,
        'heading_motion'   : True,
        'pDOP'             : True,
        'version_num'      : True  }

    is_high_res_module_queried = {
        'All'                 : True,
        'time_of_week'        : True,
        'Latitude'            : True,
        'Longitude'           : True,
        'elipsoid'            : True,
        'mean_sea_level'      : True,
        'geo_id_separation'   : True,
        'horizontal_accuracy' : True,
        'vertical_accuracy'   : True }

    # Relative Positioning Info in NED frame specific controls.
    relative_pos_info = {
        'ref_station_ID'   : 0,
        'rel_pos_N'        : 0,
        'rel_pos_E'        : 0,
        'rel_pos_D'        : 0,
        'rel_pos_length'   : 0,
        'rel_pos_heading'  : 0,
        'rel_pos_HPN'      : 0,
        'rel_pos_HPE'      : 0,
        'rel_pos_HPD'      : 0,
        'rel_pos_HP_Length': 0,
        'acc_N'            : 0,
        'acc_E'            : 0,
        'acc_D'            : 0,
        'gnss_fix_ok'      : False,
        'diff_sol_n'       : False,
        'rel_pos_valid'    : False,
        'carr_sol_n'       : 0,
        'is_moving'        : False,
        'ref_pos_miss'     : False,
        'ref_obs_miss'     : False
        }

    # Lists of various settings:
    sentence_type = [
        NMEA,
        UBX,
        RTCM,
        ]

    current_sentence = None

    class_types = [
        CLASS_ACK,
        CLASS_NACK
        ]

    ubx_frame_class = None

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
    # Time in ms till timeout
    MAX_TIME_SHORT = 250
    MAX_TIME_MED = 500
    MAX_TIME_LONG = 1000

    NMEA = 1
    UBX = 2
    RTCM = 3

    CLASS_ACK = 1
    CLASS_NACK = 2

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
        if (time.perf_counter() - self.last_checked) >= self.i2c_polling_wait:

            bytes_avail = self._i2c.readBlock(self.available_addresses, 0xFD, 2)

            # Check LSB for 0xFF  == No bytes available
            if (bytes_avail | 0x00FF)  == 0xFF:
                self.last_checked = time.perf_counter()
                return False

            elif bytes_avail ==  0:
                self.last_checked = time.perf_counter()
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

            self.ublox_packet_cfg['Class']  = UBX_CLASS_CFG
            self.ublox_packet_cfg['ID']     = UBX_CFG_PRT
            self.ublox_packet_cfg['Length'] = 20
            self.ublox_packet_cfg['Start']  = 0

            self.ublox_packet_cfg['Payload'][4] = device_address << 1;

            if send_command(self.ublox_packet_cfg, max_wait) == true:
                self.available_addresses[0] = device_address
                return True

            return False

        def send_command(outgoing_ubx_packet, max_wait):

            self.command_ack = False;
            self.calc_checksum(outgoing_ubx_packet)

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
            for i in range(bytes_to_send - 1): # Stop before last write
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

            for i in range(ubx_packet['Length']):
                ubx_packet['Checksum_A'] += ubx_packet['Payload'][i]
                ubx_packet['Checksum_B'] += ubx_packet['Checksum_A']

        def process(incoming):

            if self.current_sentence == None or self.current_sentence == self.NMEA:

                if incoming == 0xB5:
                    self.ubx_frame_counter = 0
                    self.rolling_checksum_A = 0
                    self.rolling_checksum_B = 0
                    self.current_sentence = self.UBX

                elif incoming == '$':
                    self.current_sentence = self.NMEA

                elif incoming == 0xD3:
                    self.rtcm_frame_counter = 0
                    self.current_sentence = self.RTCM

            if self.current_sentence == self.UBX:

                if self.ubx_frame_counter == 0 and incoming != 0xB5:
                    self.current_sentence = None
                elif self.ubx_frame_counter == 1 and incoming != 0x62:
                    self.current_sentence = None
                elif self.ubx_frame_counter == 2:
                    self.ublox_packet_ack['Counter'] = 0
                    self.ublox_packet_ack['Validate'] = False
                    self.ublox_packet_cfg['Counter'] = 0
                    self.ublox_packet_cfg['Validate'] = False

                    if incoming == self.UBX_CLASS_ACK:
                        self.ubx_frame_class = self.CLASS_ACK
                    else:
                        self.ubx_frame_class = self.CLASS_NACK

                self.ubx_frame_counter += 1

                if self.ubx_frame_class == self.CLASS_ACK:
                    self.process_UBX(incoming, self.ublox_packet_ack)
                elif self.ubx_frame_class == self.CLASS_NACK:
                    self.process_UBX(incoming, self.ublox_packet_cfg)

            elif self.current_sentence == self.NMEA:
                self.process_NMEA(incoming)
            elif self.current_sentence == self.RTCM:
                self.process_RTCM(incoming)

        def process_NMEA(incoming):

            if _nmea_output_port != None:
                _nmea_output_port.write(incoming) # Fix this

        def extract_long(initial_index):

            # LSB to MSB
            val = 0
            val |= ublox_packet_cfg['Payload'][initial_index]
            val |= ublox_packet_cfg['Payload'][initial_index + 1] << 8
            val |= ublox_packet_cfg['Payload'][initial_index + 2] << 16
            val |= ublox_packet_cfg['Payload'][initial_index + 3] << 24
            return val

        def extract_int(initial_index):

            val = 0
            val |= ublox_packet_cfg['Payload'][initial_index]
            val |= ublox_packet_cfg['Payload'][initial_index + 1] << 8
            return val

        def extract_byte(initial_index):

            return(ublox_packet_cfg['Payload'][initial_index])

        def get_latitude(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['Latitude'] == False:
                self.get_pvt();
            self.is_module_queried['Latitude'] = False
            self.is_module_queried['All'] = False

            return self.latitude

        def get_longitude(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['Longitude'] == False:
                self.get_pvt();
            self.is_module_queried['Longitude'] = False
            self.is_module_queried['All'] = False

            return self.longitude

        def get_altitude(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['Altitude'] == False:
                self.get_pvt();
            self.is_module_queried['Altitude'] = False
            self.is_module_queried['All'] = False

            return self.altitude

        def get_year(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_year'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_year'] = False
            self.is_module_queried['All'] = False

            return self.gps_year

        def get_month(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_month'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_month'] = False
            self.is_module_queried['All'] = False

            return self.gps_month

        def get_day(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_day'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_day'] = False
            self.is_module_queried['All'] = False

            return self.get_day

        def get_hour(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_hour'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_hour'] = False
            self.is_module_queried['All'] = False

            return self.gps_hour

        def get_second(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_second'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_second'] = False
            self.is_module_queried['All'] = False

            return self.gps_second

        def get_millisecond(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_iTOW'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_iTOW'] = False
            self.is_module_queried['All'] = False

            return self.gps_millisecond

        def get_nanosecond(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['GPS_nanosecond'] == False:
                self.get_pvt();
            self.is_module_queried['GPS_nanosecond'] = False
            self.is_module_queried['All'] = False

            return self.gps_nanosecond

        def get_time_of_week(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['time_of_week'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['time_of_week'] = False
            self.is_high_res_module_queried['All'] = False

            return self.time_of_week

        def get_high_res_latitude(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['Latitude'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['Latitude'] = False
            self.is_high_res_module_queried['All'] = False

            return self.high_res_latitude

        def get_high_res_longitude(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['Longitude'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['Longitude'] = False
            self.is_high_res_module_queried['All'] = False

            return self.high_res_longitude

        def get_mean_sea_level(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['mean_sea_level'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['mean_sea_level'] = False
            self.is_high_res_module_queried['All'] = False

            return self.mean_sea_level

        def get_geo_id_separation(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['geo_id_separation'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['geo_id_separation'] = False
            self.is_high_res_module_queried['All'] = False

            return self.geo_id_separation


        def get_horizontal_accuracy(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['horizontal_accuracy'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['horizontal_accuracy'] = False
            self.is_high_res_module_queried['All'] = False

            return self.horizontal_accuracy

        def get_vertical_accuracy(max_wait = self.MAX_TIME_SHORT):

            if self.is_high_res_module_queried['vertical_accuracy'] == False:
                self.get_hppos_llh();
            self.is_high_res_module_queried['vertical_accuracy'] = False
            self.is_high_res_module_queried['All'] = False

            return self.vertical_accuracy

        def get_hppos_llh(max_wait = MAX_TIME_LONG):

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
            self.ublox_packet_cfg['ID'] = self.UBX_NAV_HPPOSLLH
            self.ublox_packet_cfg['Length'] = 0

            return self.send_command(self.ublox_packet_cfg, max_wait)

        def get_position_accuracy(max_wait = MAX_TIME_MED):

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
            self.ublox_packet_cfg['ID'] = self.UBX_NAV_HPPOSECEF
            self.ublox_packet_cfg['Length'] = 0
            self.ublox_packet_cfg['Start'] = 0

            if self.send_command(self.ublox_packet_cfg, max_wait) == False:
                return 0

            temp_accuracy = self.extract_long(24)

            if temp_accuracy % 10 >= 5:
                temp_accuracy += 5

            temp_accuracy /= 10

            return temp_accuracy

        def get_altitude_MSL(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['Altitude_MSL'] == False:
                self.get_pvt();
            self.is_module_queried['Altitude_MSL'] = False
            self.is_module_queried['All'] = False

            return self.altitude_MSL

        def get_SIV(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['SIV'] == False:
                self.get_pvt();
            self.is_module_queried['SIV'] = False
            self.is_module_queried['All'] = False

            return self.SIV

        def get_fix_type(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['fix_type'] == False:
                self.get_pvt();
            self.is_module_queried['fix_type'] = False
            self.is_module_queried['All'] = False

            return self.fix_type

        def get_carrier_solution_type(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['carrier_solution'] == False:
                self.get_pvt();
            self.is_module_queried['carrier_solution'] = False
            self.is_module_queried['All'] = False

            return self.carrier_solution

        def get_ground_speed(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['ground_speed'] == False:
                self.get_pvt();
            self.is_module_queried['ground_speed'] = False
            self.is_module_queried['All'] = False

            return self.ground_speed

        def get_heading(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['heading_motion'] == False:
                self.get_pvt();
            self.is_module_queried['heading_motion'] = False
            self.is_module_queried['All'] = False

            return self.heading_motion

        def get_PDOP(max_wait = self.MAX_TIME_SHORT):

            if self.is_module_queried['pDOP'] == False:
                self.get_pvt();
            self.is_module_queried['pDOP'] = False
            self.is_module_queried['All'] = False

            return self.pDOP

        def get_protocol_version_high(max_wait = self.MAX_TIME_LONG):

            if self.is_module_queried['version_num'] == False:
                self.get_pvt();
            self.is_module_queried['version_num'] = False
            self.is_module_queried['All'] = False

            return self.version_high

        def get_protocol_version_low(max_wait = self.MAX_TIME_LONG):

            if self.is_module_queried['version_num'] == False:
                self.get_pvt();
            self.is_module_queried['version_num'] = False
            self.is_module_queried['All'] = False

            return self.version_low

        def get_protocol_version(max_wait = self.MAX_TIME_LONG):

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_MON
            self.ublox_packet_cfg['ID'] = self.UBX_MON_VER

            extension_number = 10

            for i in range(extension_number):

                self.ublox_packet_cfg['Length'] = 0
                self.ublox_packet_cfg['Start'] = 40 + (30 * extension_number)

                if self.send_i2c_command(self.ublox_packet_cfg,
                                         maxWait) == False:
                    return False

                if (self.ublox_packet_cfg['Payload'][0] == 'P' and
                    self.ublox_packet_cfg['Payload'][6] == 'R'):

                    self.version_high = (str(self.ublox_packet_cfg['Payload'][8]) +
                                         str(self.ublox_packet_cfg['Payload'][9]))

                    self.version_low =  (str(self.ublox_packet_cfg['Payload'][11]) +
                                         str(self.ublox_packet_cfg['Payload'][12]))

                    return self.version_low

            is_module_queried['version_num'] = True

            return True

        def get_PVT(max_wait = self.MAX_TIME_LONG):

            if self.auto_pvt and self.auto_pvt_implicit_update:
                self.check_ublox_i2C()
                return self.is_module_queried['All']

            elif self.auto_pvt and not self.auto_pvt_implicit_update:
                return False

            else:
                self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
                self.ublox_packet_cfg['ID'] = self.UBX_NAV_PVT
                self.ublox_packet_cfg['Length'] = 0
                return self.send_command(self.ublox_packet_cfg, max_wait)

        def assume_auto_PVT(enabled, implicit_update):

            change = (self.auto_pvt != enabled |
                      self.auto_pvt_implicit_update != implicit_update)

            if change:
                self.auto_pvt = enabled
                self.auto_pvt_implicit_update = implicit_update

            return change

        def set_auto_pvt(enable, implicit_update = None, max_wait = MAX_TIME_SHORT):

            if implicit_update == None:
                implicit_update = True

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
            self.ublox_packet_cfg['ID'] = self.UBX_CFG_MSG
            self.ublox_packet_cfg['Length'] = 3
            self.ublox_packet_cfg['Start'] = 0
            self.ublox_packet_cfg['Payload'][0] = self.UBX_CLASS_NAV
            self.ublox_packet_cfg['Payload'][1] = self.UBX_NAV_PVT
            self.ublox_packet_cfg['Payload'][2] = enable if True else not enable

            sent = self.send_command(self.ublox_packet_cfg, max_wait)

            if sent:
                self.auto_pvt = enable
                self.auto_pvt_implicit_update = implicit_update

            self.is_module_queried['All'] = False
            return sent

        def hard_rest():

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
            self.ublox_packet_cfg['ID'] = self.UBX_CFG_RST
            self.ublox_packet_cfg['Length'] = 4
            self.ublox_packet_cfg['Start'] = 0
            self.ublox_packet_cfg['Payload'][0] = 0xFF
            self.ublox_packet_cfg['Payload'][1] = 0xFF
            self.ublox_packet_cfg['Payload'][2] = 0
            self.ublox_packet_cfg['Payload'][2] = 0

            self.send_command(self.ublox_packet_cfg, 0)

        def factory_reset():

            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
            self.ublox_packet_cfg['ID'] = self.UBX_CFG_CFG
            self.ublox_packet_cfg['Length'] = 13
            self.ublox_packet_cfg['Start'] = 0

            for i in range(4):
                self.ublox_packet_cfg['Payload'][0 + i] = 0xFF
                self.ublox_packet_cfg['Payload'][4 + i] = 0x00
                self.ublox_packet_cfg['Payload'][8 + i] = 0x00

            self.ublox_packet_cfg['Payload'][12] = 0xFF
            self.send_command(self.ublox_packet_cfg, 0)
            hard_reset()

