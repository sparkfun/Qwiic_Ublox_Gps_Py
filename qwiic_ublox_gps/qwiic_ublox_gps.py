#-----------------------------------------------------------------------------
# qwiic_ublox_gps.py
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
qwiic_ublox_gps
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
import sys
import qwiic_i2c
import pynmea2

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
          https://github.com/sparkfun/qwiic_ublox_gps_py
============================================================================
        """)

# Define the device name and I2C addresses. These are set in the class defintion
# as class variables, making them avilable without having to create a class instance.
# This allows higher level logic to rapidly create a index of qwiic devices at
# runtine
#
# The name of this device
_DEFAULT_NAME = "Qwiic u-blox GPS"

# Some devices have multiple availabel addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the
# device.
_AVAILABLE_I2C_ADDRESS = [0x42]

class QwiicUbloxGps(object):
    """
    QwiicUbloxGps

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The ublox_gps device object.
        :rtype: Object
    """
    # Constructor
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Library variables
    auto_pvt = False
    auto_pvt_implicit_update = False
    command_ack = False
    _print_debug = False
    i2c_polling_wait = .100 # 100ms delay between checking for data
    last_checked = 0      # set to zero
    MAX_PAYLOAD_SIZE = 128
    payload_config = [0 for i in range(MAX_PAYLOAD_SIZE)]
    payload_ack = [0 for i in range(2)]

    # The major datums we want to globally store
    gps_year = 0
    gps_month = 0
    gps_day = 0
    gps_hour = 0
    gps_minute = 0
    gps_second = 0
    gps_millisecond = 0
    gps_nanosecond = 0

    latitude = 0          # Degrees * 10^-7 (more accurate than floats)
    longitude = 0         # Degrees * 10^-7 (more accurate than floats)
    altitude = 0          # Number of mm above ellipsoid
    altitude_MSL = 0      # Number of mm above Mean Sea Level
    SIV = 0               # Number of satellites used in position solution
    fix_type = 0          # Tells us when we have a solution aka lock
    carrier_solution = 0  # Tells us when we have an RTK float/fixed solution
    ground_speed = 0      # mm/s
    heading_motion = 0 # degrees * 10^-5
    pDOP = 0              # Positional dilution of precision
    version_low = 0       # Loaded from getProtocolVersion().
    version_high = 0

    time_of_week = 0
    high_res_latitude = 0
    high_res_longitude = 0
    elipsoid = 0
    mean_sea_level = 0
    geo_id_separation = 0
    horizontal_accuracy = 0
    vertical_accuracy = 0

    rtcm_frame_counter = 0 # Tracks the type of incoming byte inside RTCM frame
    ubx_frame_counter = 0
    rolling_checksum_A = 0
    rolling_checksum_B = 0
    rtcm_length = 0
    word = " "
    package_nmea = False
    new_sentence_flag = True

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
        'version_num'      : True
    }

    is_high_res_module_queried = {

        'All'                 : True,
        'time_of_week'        : True,
        'Latitude'            : True,
        'Longitude'           : True,
        'elipsoid'            : True,
        'mean_sea_level'      : True,
        'geo_id_separation'   : True,
        'horizontal_accuracy' : True,
        'vertical_accuracy'   : True
    }

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

    gnss_messages = {

        'Latitude'       : 0.0,     
        'Lat_Direction'  : "",
        'Longitude'      : 0.0,
        'Long_Direction' : "",
        'Altitude'       : 0.0,
        'Altitude_Units' : "",
        'Sat_Number'     : 0,
        'Geo_Separation' : 0,
        'Geo_Sep_Units'  : "",
        'Data_Age'       : 0,
        'Ref_Station_ID' : 0
    }

    # Lists of various settings:

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

    # Time in ms till timeout
    MAX_TIME_SHORT = 250
    MAX_TIME_MED = 500
    MAX_TIME_LONG = 1000

    I2C_BUFFER_LENGTH = 32

    # Sentence Types
    NMEA = 1
    UBX = 2
    RTCM = 3
    current_sentence = None

    # Class Types
    CLASS_ACK = 1
    CLASS_NACK = 2
    ubx_frame_class = None

    # Data commuincation options/types
    COMM_TYPE_I2C = 1
    COMM_TYPE_SERIAL = 2
    COMM_TYPE_SPI = 3
    outgoing_data_channel = None

    _RPiCheck = False

    def __init__(self, address=None, i2c_driver=None):


        # As noted above, to run this device on a Raspberry Pi,
        # clock streching is needed.
        #
        # Lets check if it's enabled. This is done only once in
        # the session
        if not QwiicUbloxGps._RPiCheck:
            _checkForRPiI2CClockStretch()
            QwiicUbloxGps._RPiCheck = True

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

        self.outgoing_data_channel = self.COM_PORT_I2C

        return self.is_connected()

    def enable_debugging(self):
        """
            This function enables debug messages peppered through library at
            functional points.
            :returns: Does not return anything
        """
        self._print_debug = True
        print("Debugging enabled.")

    def disable_debugging(self):
        """
            This function disables debug messages.
            :returns: Nothing
        """
        self._print_debug = False
        print("Debugging disabled.")

    def disable_nmea_package(self):
        """
            This function disables packaging GNSS data stream for NMEA
            library. Othewise the datastream is coming in a character at a time
            and printed out to command line.
            :returns: Nothing
        """
        self.package_nmea = False

    def get_output_port(self, port, MAX_TIME_SHORT):
        pass

    def get_port_settings(self, port_id, max_wait = MAX_TIME_SHORT):
        """
            Loads the payloadCfg array with the current protocol bits located
            the UBX-CFG-PRT register for a given port.
            :returns: Payload configuration within the UBX-CFG-PRT register
            :rtype: One byte of Payload Configuration data
        """
        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
        self.ublox_packet_cfg['ID'] = self.UBX_CFG_PRT
        self.ublox_packet_cfg['Length'] = 1
        self.ublox_packet_cfg['Start'] = 0
        self.ublox_packet_cfg['Payload'][0] = port_id

        return self.send_command(self.ublox_packet_cfg['Payload'], max_wait)

    def debug_print(self, message):
        """
            This function prints out the given message to the command line.
            :returns: Returns nothing
        """
        if self._print_debug is True:
            print(message)
        else:
            return

    def set_serial_rate(self, baudrate, uart_port, max_wait = MAX_TIME_SHORT):
        """
            This function changes the serial baud rate of the Ublox module.
            :returns: Nothing is returned because messages are lost temporarily
            during baud rate change.
        """
        self.get_port_settings(uart_port, max_wait)

        if self._print_debug is True:

            curr_baudrate = self.ublox_packet_cfg['Payload'][10] << 16
            curr_baudrate |= self.ublox_packet_cfg['Payload'][9] << 8
            curr_baudrate |= self.ublox_packet_cfg['Payload'][8]

            print("Current Baudrate: {}", curr_baudrate)

        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
        self.ublox_packet_cfg['ID'] = self.UBX_CFG_PRT
        self.ublox_packet_cfg['Length'] = 20
        self.ublox_packet_cfg['Start'] = 0

        self.ublox_packet_cfg['Payload'][8] = baudrate
        self.ublox_packet_cfg['Payload'][9] = baudrate >> 8
        self.ublox_packet_cfg['Payload'][10] = baudrate >> 16
        self.ublox_packet_cfg['Payload'][11] = baudrate >> 24

        if self._print_debug is True:

            curr_baudrate = self.ublox_packet_cfg['Payload'][10] << 16
            curr_baudrate |= self.ublox_packet_cfg['Payload'][9] << 8
            curr_baudrate |= self.ublox_packet_cfg['Payload'][8]

            print("Current Baudrate: {}", curr_baudrate)

        self.send_command(self.ublox_packet_cfg['Payload'], max_wait)


    def process_RTCM(self, incoming_data): # Not implemented in Arduino Library
        """
            This function is not implemented but could process RTCM messages.
            :returns: Nothing
        """
        pass

    def process_RTCM_frame(self, incoming_data):
        """
            This function process RTCM messages and sends that message to the
            process_RTCM function which is not implemented.
            :returns: Nothing
        """
        if self.rtcm_frame_counter == 1:
            self.rtcm_length = (incoming_data & 0x03) << 8
        elif self.rtcm_frame_counter == 2:
            self.rtcm_length |= incoming_data
            self.rtcm_length += 6

        self.rtcm_frame_counter += 1

        self.process_RTCM(incoming_data)

        if self.rtcm_frame_counter == self.rtcm_length:
            self.current_sentence = None


    def process(self, incoming_data):
        """
            This function "processes" the data given by the ublox module to
            determine the type of information that it is. It then passes this
            information to a function dedicated to that type.
            :returns: Nothing
        """
        if self.current_sentence is None or self.current_sentence == self.NMEA:

            if incoming_data == 0xB5: # Binary frames start with 0xB5
                self.debug_print("UBX")
                self.ubx_frame_counter = 0
                self.rolling_checksum_A = 0
                self.rolling_checksum_B = 0
                self.current_sentence = self.UBX

            elif chr(incoming_data) == '$':
                self.debug_print("NMEA")
                self.current_sentence = self.NMEA

            elif incoming_data == 0xD3:
                self.debug_print("RTCM")
                self.rtcm_frame_counter = 0
                self.current_sentence = self.RTCM
            else:
                # Missed something
                pass

        if self.current_sentence == self.UBX:
            if self.ubx_frame_counter == 0 and incoming_data != 0xB5:
                self.current_sentence = None # Something went wrong
            elif self.ubx_frame_counter == 1 and incoming_data != 0x62:
                self.current_sentence = None # Someting went wrong
            elif self.ubx_frame_counter == 2: # Class
                self.ublox_packet_ack['Counter'] = 0
                self.ublox_packet_ack['Validate'] = False
                self.ublox_packet_cfg['Counter'] = 0
                self.ublox_packet_cfg['Validate'] = False

                if incoming_data == self.UBX_CLASS_ACK:
                    self.ubx_frame_class = self.CLASS_ACK
                else:
                    self.ubx_frame_class = self.CLASS_NACK

            self.ubx_frame_counter += 1

            if self.ubx_frame_class == self.CLASS_ACK:
                self.process_UBX(incoming_data, self.ublox_packet_ack)
            elif self.ubx_frame_class == self.CLASS_NACK:
                self.process_UBX(incoming_data, self.ublox_packet_cfg)
            else:
                return None

        elif self.current_sentence == self.NMEA:
            return self.process_NMEA(incoming_data)
        elif self.current_sentence == self.RTCM:
            return self.process_RTCM_frame(incoming_data)
        else:
            return None

    def process_NMEA(self, incoming_data):
        """
            This function builds and prints out streaming NMEA sentences
            received from the ublox module.
            :return: No return value.
        """
        complete_sentence = ""
        data = chr(incoming_data)

        # pynmea2 takes full sentences so we're building them here.
        if data == '$' and self.new_sentence_flag is True:
            self.new_sentence_flag = False
            self.word = data
            return None

        elif data == '$' and self.new_sentence_flag is False:
            self.new_sentence_flag = True
            complete_sentence = self.word # Save full sentence before we begin anew
            self.word = data # Start new sentence with current character    
            # Removing new line characters so that we can that the user can use 
            # print in a more simple way.
            if '\n' in complete_sentence:
                complete_sentence = complete_sentence.replace('\n','')
            return complete_sentence

        else: 
            self.word = self.word + data
            return None



    def add_to_nmea_dictionary(self, sentence):
        """
            This function adds relevant information to their respective
            NMEA dictionary locations.
            :returns: Returns True on addition and False otherwise
            :rtype: Boolean
        """

        if sentence is not None: 
            # Not every sentence has the information you need - so we'll fill
            # in what is relevant and pass on what is not. 
            try: 
                self.gnss_messages['Latitude'] = sentence.lat
            except:
                pass
            try:
                self.gnss_messages['Lat_Direction'] = sentence.lat_dir
            except:
                pass
            try:
                self.gnss_messages['Longitude'] = sentence.lon
            except:
                pass
            try:
                self.gnss_messages['Long_Direction'] = sentence.lon_dir
            except:
                pass
            try:
                self.gnss_messages['Altitude'] = sentence.altitude
            except:
                pass
            try:
                self.gnss_messages['Altitude_Units'] = sentence.altitude_units
            except:
                pass
            try:
                self.gnss_messages['Sat_Number'] = sentence.num_sats
            except:
                pass
            try:
                self.gnss_messages['Geo_Separation'] = sentence.geo_sep
            except:
                pass
            try:
                self.gnss_messages['Geo_Sep_Units'] = sentence.geo_sep_units
            except:
                pass
            try:
                self.gnss_messages['Data_Age'] = sentence.age_gps_data
            except:
                pass
            try:
                self.gnss_messages['Ref_Station_ID'] = sentence.ref_station_id
            except:
                pass
            return True

        return False

    def clean_nmea_list(self, raw_gnss_list):
        """
            This function parses raw NMEA data for incomplete sentences or any
            other corruption to the data and generates a new list from that
            data which only contains correct data.
            :return: Returns a procured list of NMEA messages
            :rtype: List
        """

        # Check that there are proper sentences.  
        clean_gnss_list = []
        for sentence in raw_gnss_list:
            if sentence.startswith('$'):
                clean_gnss_list.append(sentence)

        return clean_gnss_list

    def get_raw_nmea(self):
        """
            This function gets NMEA messages returned from the u-blox
            module.
            :return: Returns a list of NMEA messages or False otherwise
            :rtype: list or boolean
        """
        data = self.check_ublox()
        if data is not None:
            # Remove anything that was corrupted in transit. 
            cleaned_data = self.clean_nmea_list(data)
            return cleaned_data

        return False

    def get_parsed_nmea(self):
        """
            This function parses lists of NMEA setences using the pynmea2
            library.
            :return: True on successful parsing and False otherwise
            :rtype: boolean
        """
        data = self.get_raw_nmea()
        msg = []
        if data is not False :
            for sentence in data:
                try:
                    # Build list 
                    msg.append(pynmea2.parse(sentence))
                except pynmea2.nmea.ParseError:
                    continue

            for message in msg:
                # Pass pynmea2 data strings to build dictionary
                added_to_dict = self.add_to_nmea_dictionary(message)
                if added_to_dict is not False:
                    return True
            
        return False


    def check_ublox(self):
        """
            This function checks the which data channel data should be piped
            too before retrieving that data.
            :return: Returns True with successful retrieval of data and False
            otherwise.
            :rtype: boolean
        """

        if self.outgoing_data_channel == self.COM_PORT_I2C:
            return self.check_ublox_i2c()
        elif self.outgoing_data_channel == self.COMM_TYPE_SERIAL:
            pass # Not implemented
        else:
            return None

    def check_ublox_i2c(self):
        """
            Checks to see if GPS data is available via I2C protocol.
            :return: Returns True when GPS data is available, and False when
            not.
            :rtype: boolean
        """

        edge_case = False
        # We only want to poll every 100ms as per the datasheet.
        if (time.monotonic() - self.last_checked) >= self.i2c_polling_wait:

            bytes_avail = 0
            try:
                byte_block = self._i2c.readBlock(self.address, 0xFD, 2)
                bytes_avail = byte_block[0] << 8 | byte_block[1]
                # This is a broad class, but the clock stretching error is
                # consistent with Pi's and the ublox modules.
            except OSError:
                return None

            # Check LSB for (0xFF  == No bytes available)
            if (bytes_avail | 0x00FF)  == 0xFF:
                self.debug_print("No bytes available.")
                self.last_checked = time.monotonic()
                return None

            elif bytes_avail ==  0:
                self.debug_print("Zero bytes available.")
                self.last_checked = time.monotonic()
                return None

            # Something has gone wrong here, checking register again.
            # Data will still be there when we get back.
            elif bytes_avail > 3000:
                return None

            elif bytes_avail > 100:

                self.debug_print("Data available.")
                gnss_sentences = []

                while bytes_avail >= 0:
                    bytes_to_read = bytes_avail


                    if bytes_to_read > self.I2C_BUFFER_LENGTH:
                        bytes_to_read = self.I2C_BUFFER_LENGTH

                    try:
                        incoming = self._i2c.readBlock(self.address,
                                                       0xFF, bytes_to_read)

                    # This is a broad class, but the clock stretching error is
                    # consistent with the amount of clock stretching done by
                    # the ublox module.
                    except OSError:
                        break

                    for index, gnss_data in enumerate(incoming):
                        # Rare edge case - needs to continue back at top:
                        if index == 0 and gnss_data == 0x7F:
                            edge_case = True
                            self.debug_print("Module not ready with data.")
                            time.sleep(.005)

                        if edge_case is True:
                            edge_case = False
                            return None

                        data = self.process(gnss_data)
                        # This will not be "None" when the sentence is
                        # contructued. 
                        if data is not None:
                            gnss_sentences.append(data)

                    # Our end condition.
                    bytes_avail = bytes_avail - bytes_to_read

                cleaned_list = self.clean_nmea_list(gnss_sentences)
                return cleaned_list
        else:
            return None


    def set_i2c_address(self, device_address, max_wait):
        """
            Changes the software configureable I2C address of the GPS
            product.
            :return: Returns True on success and False otherwise
            :rtype: boolean
        """

        self.ublox_packet_cfg['Class']  = self.UBX_CLASS_CFG
        self.ublox_packet_cfg['ID']     = self.UBX_CFG_PRT
        self.ublox_packet_cfg['Length'] = 20
        self.ublox_packet_cfg['Start']  = 0

        self.ublox_packet_cfg['Payload'][4] = device_address << 1

        if self.send_command(self.ublox_packet_cfg, max_wait) is True:
            self.address[0] = device_address
            return True

        return False

    def print_packet(self, outgoing_ubx_packet):
        """
            This function prints the current ublox packet.
            :returns: Nothing
        """
        if self._print_debug is True:
            print("Class:{}, ID:{}, Length:0x{}".format(
                outgoing_ubx_packet['Class'],
                outgoing_ubx_packet['ID'],
                outgoing_ubx_packet['Length']))
            print("Payload: ",end="")
            for i in len(outgoing_ubx_packet['Length']):
                print(hex(outgoing_ubx_packet['Payload'][i]), end="")
            print('\n')

    def wait_for_response(self, requested_class, requested_id,
                          max_time = MAX_TIME_SHORT):
        """
            This function polls the module until an "ACK" is received.
            :returns: Returns True on succesful communication and False
            otherwise
            :rtype: Boolean
        """
        self.command_ack = False
        self.ublox_packet_cfg['Valid'] = False

        start_time = time.monotonic()

        while time.monotonic() - start_time < max_time:

            if not None in self.check_ublox():
                if self.command_ack is True:
                    return True
                if self.ublox_packet_cfg['Valid'] is True:

                    if (self.ublox_packet_cfg['Class'] is requested_class and
                            self.ublox_packet_cfg['ID'] is requested_id):
                        self.debug_print("Class and ID Match!")
                        return True
                    else:
                        self.debug_print("Packet didn't match Class or ID!")
                        self.print_packet(self.ublox_packet_cfg)

            time.sleep(.5)

        self.debug_print("wait_for_response() timeout")
        return False


    def send_i2c_command(self, outgoing_ubx_packet):
        """
            This function takes the given packet and payload, and sends that
            information including CRC bytes via I2C
            :returns: Returns True on succesful communication and False
            otherwise
            :rtype: Boolean
        """
        self._i2c.writeByte(self.address, self.UBX_SYNCH_1)
        self._i2c.writeByte(self.address, self.UBX_SYNCH_2)
        self._i2c.writeByte(self.address, outgoing_ubx_packet['Class'])
        self._i2c.writeByte(self.address, outgoing_ubx_packet['ID'])
        self._i2c.writeByte(self.address, outgoing_ubx_packet['Length'] & 0xFF)
        self._i2c.writeByte(self.address, outgoing_ubx_packet['Length'] >> 8)

        bytes_to_send = outgoing_ubx_packet['Length']
        start_index = 0

        while bytes_to_send > 1:

            length = bytes_to_send
            if length > self.I2C_BUFFER_LENGTH:
                length = self.I2C_BUFFER_LENGTH

            for i in len(length):

                self._i2c.writeByte(self.address,
                                    outgoing_ubx_packet['Payload'][start_index + i])

                start_index = start_index + length
                bytes_to_send = bytes_to_send - length

        if bytes_to_send is 1:
            self._i2c.writeByte(self.address, outgoing_ubx_packet['Payload'][1])
            self._i2c.writeByte(self.address, outgoing_ubx_packet['Checksum_A'])
            self._i2c.writeByte(self.address, outgoing_ubx_packet['Checksum_B'])

        return True

    def send_command(self, outgoing_ubx_packet, max_wait):
        """
            This function take the given packet and payload, and sends that
            information including CRC bytes through to the set
            communication port (default is I2C port).
            :returns: Returns True on succesful communication and False
            otherwise
            :rtype: Boolean
        """

        self.command_ack = False
        self.calc_checksum(outgoing_ubx_packet)

        if not self.send_i2c_command(outgoing_ubx_packet):
            return False

        if max_wait > 0:
            return self.wait_for_response(outgoing_ubx_packet['Class'],
                                          outgoing_ubx_packet['ID'],
                                          max_wait)
        return True

    def calc_checksum(self, ubx_packet):
        """
            This function calculates and stores the two byte "8-Bit Fletcher"
            checksum over the entirety of the message. This is called before
            a command message is sent.
            :returns: Returns nothing
        """

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


    def add_to_checksum(self, incoming):
        """
            This function takes the given byte and adds it to a rolling "8-bit
            Fletcher" checksum. This is used when receiving messages from the
            module.
            :returns: Returns nothing
        """
        self.rolling_checksum_A = self.rolling_checksum_A + incoming
        self.rolling_checksum_B = self.rolling_checksum_B + self.rolling_checksum_A


    def process_UBX(self, incoming, incoming_ubx):
        """
            This function with the passed character, files it away into the UXB packet structure
            Set valid = true once sentence is completely received and passes CRC
            The payload portion of the packet can be 100s of bytes but the max array
            size is roughly 64 bytes. startingSpot can be set so we only record
            a subset of bytes within a larger packet.
            :returns: Returns nothing
        """
        if (incoming_ubx['Counter'] <
                incoming_ubx['Length'] + 4):
            self.add_to_checksum(incoming)

        if incoming_ubx['Counter'] == 0:
            incoming_ubx['Class'] = incoming

        elif incoming_ubx['Counter'] == 1:
            incoming_ubx['ID'] = incoming

        elif incoming_ubx['Counter'] == 2:
            incoming_ubx['Length'] = incoming

        elif incoming_ubx['Counter'] == 3:
            incoming_ubx['Length'] |= incoming << 8

        elif (incoming_ubx['Counter'] ==
              incoming_ubx['Length'] + 4):

            incoming_ubx['Checksum_A'] = incoming

        elif (incoming_ubx['Counter'] ==
              incoming_ubx['Length'] + 5):

            incoming_ubx['Checksum_B'] = incoming
            self.current_sentence = None

        if (incoming_ubx['Checksum_A'] == self.rolling_checksum_A and
                incoming_ubx['Checksum_B'] == self.rolling_checksum_B):

            incoming_ubx['Valid'] = True
            self.process_UBX_packet(incoming_ubx)

        else:
            initial_index = incoming_ubx['Start']
            if (incoming_ubx['Class'] == self.UBX_CLASS_NAV and
                    incoming_ubx['ID'] == self.UBX_NAV_PVT):

                initial_index = 0

                if incoming_ubx['Counter'] - 4 >= initial_index:
                    if ((incoming_ubx['Counter'] - 4) - initial_index <
                            self.MAX_PAYLOAD_SIZE) :

                        incoming_ubx['Payload'][incoming_ubx['Counter'] -
                                                4 - initial_index] = incoming

        incoming_ubx['Counter'] += 1

    def process_UBX_packet(self, packet_message):
        """
            This function takes the packet that has been validated and
            identifies the packet's class and ID, and then updates internal
            flags.
            :returns: Nothing is returned
            :rtype: None
        """
        packet_class = packet_message['Class']

        if packet_class == self.UBX_CLASS_ACK:
            if (packet_message['ID'] == self.UBX_ACK_ACK and
                    packet_message['Payload'][0] ==
                    self.ubx_frame_class['Class'] and
                    packet_message['Payload'][1] ==
                    self.ubx_frame_class['ID']):

                self.command_ack = True

                return None

        elif packet_class == self.UBX_CLASS_NAV:
            if (packet_message['ID'] == self.UBX_NAV_PVT and
                    packet_message['Length'] == 92):

                initial_index = 0

                self.gps_millisecond = self.extract_long(0) % 1000
                self.gps_year = self.extract_int(4)
                self.gps_month = self.extract_byte(6)
                self.gps_day = self.extract_byte(7)
                self.gps_hour = self.extract_byte(8)
                self.gps_minute = self.extract_byte(9)
                self.gps_second = self.extract_byte(10)
                self.gps_nanosecond = self.extract_long(16)

                self.fix_type = self.extract_byte(20 - initial_index)
                self.carrier_solution = self.extract_byte(21 - initial_index) >> 6

                self.SIV = self.extract_byte(23 - initial_index)
                self.longitude = self.extract_long(24 - initial_index)
                self.latitude = self.extract_long(28 - initial_index)
                self.altitude = self.extract_long(32 - initial_index)
                self.altitude_MSL = self.extract_long(36 - initial_index)
                self.ground_speed = self.extract_long(60 - initial_index)
                self.heading_motion = self.extract_long(64 - initial_index)
                self.pDOP = self.extract_long(76 - initial_index)

                self.is_module_queried['GPS_iTOW'] = True
                self.is_module_queried['GPS_year'] = True
                self.is_module_queried['GPS_month'] = True
                self.is_module_queried['GPS_day'] = True
                self.is_module_queried['GPS_hour'] = True
                self.is_module_queried['GPS_minute'] = True
                self.is_module_queried['GPS_second'] = True
                self.is_module_queried['GPS_nanosecond'] = True

                self.is_module_queried['All'] = True
                self.is_module_queried['Longitude'] = True
                self.is_module_queried['Latitude'] = True
                self.is_module_queried['Altitude'] = True
                self.is_module_queried['Altitude_MSL'] = True
                self.is_module_queried['SIV'] = True
                self.is_module_queried['fix_type'] = True
                self.is_module_queried['carrier_solution'] = True
                self.is_module_queried['ground_speed'] = True
                self.is_module_queried['heading_motion'] = True
                self.is_module_queried['pDOP'] = True

                return None

            elif (packet_message['ID'] == self.UBX_NAV_HPPOSLLH
                  and packet_message['Length'] == 36):

                self.time_of_week = self.extract_long(4)
                self.high_res_longitude = self.extract_long(8)
                self.high_res_latitude = self.extract_long(12)
                self.elipsoid = self.extract_long(16)
                self.mean_sea_level = self.extract_long(20)
                self.geo_id_separation = self.extract_long(24)
                self.horizontal_accuracy = self.extract_long(28)
                self.vertical_accuracy = self.extract_long(32)

                self.is_high_res_module_queried['All'] = True
                self.is_high_res_module_queried['time_of_week'] = True
                self.is_high_res_module_queried['high_res_latitude'] = True
                self.is_high_res_module_queried['high_res_longitude'] = True
                self.is_high_res_module_queried['elipsoid'] = True
                self.is_high_res_module_queried['mean_sea_level'] = True
                self.is_high_res_module_queried['geo_id_separation'] = True
                self.is_high_res_module_queried['horizontal_accuracy'] = True
                self.is_high_res_module_queried['vertical_accuracy'] = True

                if self._print_debug is True:
                    self.debug_print("Sec: ")
                    self.debug_print(float(self.extract_long(4)) / 1000)
                    self.debug_print(" ")
                    self.debug_print("LON: ")
                    self.debug_print(float(self.extract_long(8)) / 10000000)
                    self.debug_print(" ")
                    self.debug_print("LAT: ")
                    self.debug_print(float(self.extract_long(12)) / 10000000)
                    self.debug_print(" ")
                    self.debug_print("ELI M: ")
                    self.debug_print(float(self.extract_long(16)) / 1000)
                    self.debug_print(" ")
                    self.debug_print("MSL M: ")
                    self.debug_print(float(self.extract_long(20)) / 1000)
                    self.debug_print(" ")
                    self.debug_print("GEO: ")
                    self.debug_print(float(self.extract_long(24)) / 1000)
                    self.debug_print(" ")
                    self.debug_print("HA 2D M: ")
                    self.debug_print(float(self.extract_long(28)) / 1000)
                    self.debug_print(" ")
                    self.debug_print("VERT M: ")
                    self.debug_print(float(self.extract_long(32)) / 1000)
                    self.debug_print(" ")

        return None

    def extract_long(self, initial_index):
        """
            This function takes an index and builds a four byte long
            integer from that starting index.
            :returns: Return the constructed value.
            :rtype: 4 byte value
        """
        # LSB to MSB
        val = 0
        val |= self.ublox_packet_cfg['Payload'][initial_index]
        val |= self.ublox_packet_cfg['Payload'][initial_index + 1] << 8
        val |= self.ublox_packet_cfg['Payload'][initial_index + 2] << 16
        val |= self.ublox_packet_cfg['Payload'][initial_index + 3] << 24
        return val

    def extract_int(self, initial_index):
        """
            This function takes an index and builds a two byte integer
            integer from that starting index.
            :returns: Return the constructed value.
            :rtype: 2 byte value
        """
        val = 0
        val |= self.ublox_packet_cfg['Payload'][initial_index]
        val |= self.ublox_packet_cfg['Payload'][initial_index + 1] << 8

        return val

    def extract_byte(self, initial_index):
        """
            This function takes an index and returns the byte at that
            location.
            :returns: Return a byte.
            :rtype: Single byte
        """
        return self.ublox_packet_cfg['Payload'][initial_index]

    def get_latitude(self):
        """
            This function returns the current latitude in degrees.
            :returns: Returns a 4 byte long representing the number in
            degrees *10^7
            :rtype: 4 byte long
        """
        if self.is_module_queried['Latitude'] is False:
            self.get_pvt()
        self.is_module_queried['Latitude'] = False
        self.is_module_queried['All'] = False

        return self.latitude

    def get_longitude(self):
        """
            This function returns the current latitude in degrees.
            :returns: Returns a 4 byte long representing the number in
            degrees *10^7
            :rtype: 4 byte long
        """
        if self.is_module_queried['Longitude'] is False:
            self.get_pvt()
        self.is_module_queried['Longitude'] = False
        self.is_module_queried['All'] = False

        return self.longitude

    def get_altitude(self):
        """
            This function gets the current altitude in millimeters
            according to an ellipsoid model.
            :returns: Altitude
            :rtype: 4 byte long
        """
        if self.is_module_queried['Altitude'] is False:
            self.get_pvt()
        self.is_module_queried['Altitude'] = False
        self.is_module_queried['All'] = False

        return self.altitude

    def get_year(self):
        """
            This function gets the current year.
            :returns: Return the year value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_year'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_year'] = False
        self.is_module_queried['All'] = False

        return self.gps_year

    def get_month(self):
        """
            This function gets the current month.
            :returns: Return the month value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_month'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_month'] = False
        self.is_module_queried['All'] = False

        return self.gps_month

    def get_day(self):
        """
            This function gets the current day.
            :returns: Return the day value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_day'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_day'] = False
        self.is_module_queried['All'] = False

        return self.get_day

    def get_hour(self):
        """
            This function gets the current hour.
            :returns: Return the hour value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_hour'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_hour'] = False
        self.is_module_queried['All'] = False

        return self.gps_hour

    def get_second(self):
        """
            This function gets the current second.
            :returns: Return the "second" value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_second'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_second'] = False
        self.is_module_queried['All'] = False

        return self.gps_second

    def get_millisecond(self):
        """
            This function gets the current millisecond.
            :returns: Returns the millisecond value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_iTOW'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_iTOW'] = False
        self.is_module_queried['All'] = False

        return self.gps_millisecond

    def get_nanosecond(self):
        """
            This function gets the current nanosecond.
            :returns: Returns the nanosecond value.
            :rtype: Four byte long
        """
        if self.is_module_queried['GPS_nanosecond'] is False:
            self.get_pvt()
        self.is_module_queried['GPS_nanosecond'] = False
        self.is_module_queried['All'] = False

        return self.gps_nanosecond

    def get_time_of_week(self):
        """
            This function gets the current day of the week.
            :returns: Returns the day of the week value.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['time_of_week'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['time_of_week'] = False
        self.is_high_res_module_queried['All'] = False

        return self.time_of_week

    def get_high_res_latitude(self):
        """
            This function gets the latitude with greater resolution. This
            :returns: Returns a high resolution latitude.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['Latitude'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['Latitude'] = False
        self.is_high_res_module_queried['All'] = False

        return self.high_res_latitude

    def get_high_res_longitude(self):
        """
            This function gets the longitude with greater resolution. This
            :returns: Returns a high resolution longitude.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['Longitude'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['Longitude'] = False
        self.is_high_res_module_queried['All'] = False

        return self.high_res_longitude

    def get_mean_sea_level(self):
        """
            This function gets the mean sea level.
            :returns: Returns the sea level value.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['mean_sea_level'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['mean_sea_level'] = False
        self.is_high_res_module_queried['All'] = False

        return self.mean_sea_level

    def get_geoid_separation(self):
        """
            This function gets the geoidal separation.
            :returns: Returns the geiodal value.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['geo_id_separation'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['geo_id_separation'] = False
        self.is_high_res_module_queried['All'] = False

        return self.geo_id_separation


    def get_horizontal_accuracy(self):
        """
            This function gets the horizontal accuracy of the module.
            :returns: Returns the horizontal accuracy  value.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['horizontal_accuracy'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['horizontal_accuracy'] = False
        self.is_high_res_module_queried['All'] = False

        return self.horizontal_accuracy

    def get_vertical_accuracy(self):
        """
            This function gets the vertical accuracy of the module.
            :returns: Returns the vertical accuracy  value.
            :rtype: Four byte long.
        """
        if self.is_high_res_module_queried['vertical_accuracy'] is False:
            self.get_hppos_llh()
        self.is_high_res_module_queried['vertical_accuracy'] = False
        self.is_high_res_module_queried['All'] = False

        return self.vertical_accuracy

    def get_hppos_llh(self, max_wait = MAX_TIME_LONG):
        """
            This function gets the high-precision latitude/longitude from
            the module.
            :returns: Returns the hpll  value.
            :rtype: Four byte long.
        """
        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
        self.ublox_packet_cfg['ID'] = self.UBX_NAV_HPPOSLLH
        self.ublox_packet_cfg['Length'] = 0

        return self.send_command(self.ublox_packet_cfg, max_wait)

    def get_position_accuracy(self, max_wait = MAX_TIME_MED):
        """
            This function gets the positional  accuracy of the module.
            :returns: Returns the positional accuracy  value.
            :rtype: Four byte long.
        """
        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
        self.ublox_packet_cfg['ID'] = self.UBX_NAV_HPPOSECEF
        self.ublox_packet_cfg['Length'] = 0
        self.ublox_packet_cfg['Start'] = 0

        if self.send_command(self.ublox_packet_cfg, max_wait) is False:
            return 0

        temp_accuracy = self.extract_long(24)

        if temp_accuracy % 10 >= 5:
            temp_accuracy += 5

        temp_accuracy /= 10

        return temp_accuracy

    def get_pvt(self, max_wait = MAX_TIME_LONG):
        """
            This function gets the altitude relative to the mean sea level.
            :returns: Returns the altitude value.
            :rtype: Four byte long.
        """
        if self.auto_pvt and self.auto_pvt_implicit_update:
            self.check_ublox()
            return self.is_module_queried['All']

        elif self.auto_pvt and not self.auto_pvt_implicit_update:
            return False

        else:
            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
            self.ublox_packet_cfg['ID'] = self.UBX_NAV_PVT
            self.ublox_packet_cfg['Length'] = 0
            return self.send_command(self.ublox_packet_cfg, max_wait)

    def get_altitude_MSL(self):
        """
            This function gets the altitude relative to the mean sea level.
            :returns: Returns the altitude value.
            :rtype: Four byte long.
        """
        if self.is_module_queried['Altitude_MSL'] is False:
            self.get_pvt()
        self.is_module_queried['Altitude_MSL'] = False
        self.is_module_queried['All'] = False

        return self.altitude_MSL

    def get_SIV(self):
        """
            This function gets the number of satellites used to get a fix.
            :returns: Returns the number of satellites.
            :rtype: Byte
        """
        if self.is_module_queried['SIV'] is False:
            self.get_pvt()
        self.is_module_queried['SIV'] = False
        self.is_module_queried['All'] = False

        return self.SIV

    def get_fix_type(self):
        """
            This function gets the current fix type.
            :returns: Returns the fix type.
            :rtype: Byte
        """
        if self.is_module_queried['fix_type'] is False:
            self.get_pvt()
        self.is_module_queried['fix_type'] = False
        self.is_module_queried['All'] = False

        return self.fix_type

    def get_carrier_solution_type(self):
        """
            This function gets the carrier phase range solution status
            which is useful when querying the nmodule to see if it has a high
            precision RTK fix.
            :returns: Will return "0" if no solution, "1" if Float
            solution, and "2" if Fixed solution.
            :rtype: Byte
        """
        if self.is_module_queried['carrier_solution'] is False:
            self.get_pvt()
        self.is_module_queried['carrier_solution'] = False
        self.is_module_queried['All'] = False

        return self.carrier_solution

    def get_ground_speed(self):
        """
            This function gets the ground speed measured in millimeters a
            second.
            :returns: Returns ground speed in millimeters a second.
            :rtype: 4 byte long
        """
        if self.is_module_queried['ground_speed'] is False:
            self.get_pvt()
        self.is_module_queried['ground_speed'] = False
        self.is_module_queried['All'] = False

        return self.ground_speed

    def get_heading(self):
        """
            This function gets the heading of motion in degress.
            :returns: Returns heading in degress * 10^5
            :rtype: 4 byte long
        """
        if self.is_module_queried['heading_motion'] is False:
            self.get_pvt()
        self.is_module_queried['heading_motion'] = False
        self.is_module_queried['All'] = False

        return self.heading_motion

    def get_PDOP(self):
        """
            This function gets the positional dillution of precision.
            :returns: Returns positional dilution * 10^2
            :rtype: 2 byte integer.
        """
        if self.is_module_queried['pDOP'] is False:
            self.get_pvt()
        self.is_module_queried['pDOP'] = False
        self.is_module_queried['All'] = False

        return self.pDOP

    def get_protocol_version_high(self):
        """
            This function gets the greater protocol version number of
            the module.
            :returns: The greater protocol version number.
            :rtype: Byte
        """
        if self.is_module_queried['version_num'] is False:
            self.get_pvt()
        self.is_module_queried['version_num'] = False
        self.is_module_queried['All'] = False

        return self.version_high

    def get_protocol_version_low(self):
        """
            This function gets the lower protocol version number of
            the module.
            :returns: The lower protocol version number.
            :rtype: Byte
        """
        if self.is_module_queried['version_num'] is False:
            self.get_pvt()
        self.is_module_queried['version_num'] = False
        self.is_module_queried['All'] = False

        return self.version_low

    def get_protocol_version(self):
        """
            This function gets the protocol version number of
            the module.
            :returns: The protocol version number.
            :rtype: Byte
        """
        if self.is_module_queried['version_num'] is False:
            self.get_pvt()
        self.is_module_queried['version_num'] = False
        self.is_module_queried['All'] = False
        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_MON
        self.ublox_packet_cfg['ID'] = self.UBX_MON_VER

        extension_number = 10

        for i in range(extension_number):

            self.ublox_packet_cfg['Length'] = 0
            self.ublox_packet_cfg['Start'] = 40 + (30 * extension_number)

            if self.send_i2c_command(self.ublox_packet_cfg) is False:
                return False

            if (self.ublox_packet_cfg['Payload'][0] == 'P' and
                    self.ublox_packet_cfg['Payload'][6] == 'R'):

                self.version_high = (str(self.ublox_packet_cfg['Payload'][8]) +
                                     str(self.ublox_packet_cfg['Payload'][9]))

                self.version_low =  (str(self.ublox_packet_cfg['Payload'][11]) +
                                     str(self.ublox_packet_cfg['Payload'][12]))

                return self.version_low

        self.is_module_queried['version_num'] = True

        return True

    def get_PVT(self, max_wait = MAX_TIME_LONG):
        """
            This function gets the latest positional, veloticy, and time
            solution and fills all global variables.
            :returns: True upon success and false otherwise.
            :rtype: Boolean
        """
        if self.auto_pvt and self.auto_pvt_implicit_update:
            self.check_ublox_i2c()
            return self.is_module_queried['All']

        elif self.auto_pvt and not self.auto_pvt_implicit_update:
            return False

        else:
            self.ublox_packet_cfg['Class'] = self.UBX_CLASS_NAV
            self.ublox_packet_cfg['ID'] = self.UBX_NAV_PVT
            self.ublox_packet_cfg['Length'] = 0
            return self.send_command(self.ublox_packet_cfg, max_wait)

    def assume_auto_PVT(self, enabled, implicit_update):
        """
            This function sets the configuration to suitable parameters in
            the case no configuaration access to the GPS is possible and
            PVT is sent cyclically already.
            :returns: True upon success and false otherwise.
            :rtype: Boolean
        """
        change = (self.auto_pvt != enabled |
                  self.auto_pvt_implicit_update != implicit_update)

        if change:
            self.auto_pvt = enabled
            self.auto_pvt_implicit_update = implicit_update

        return change

    def set_auto_pvt(self, enable, implicit_update = None, max_wait = MAX_TIME_SHORT):
        """
            This function enables or disables automatic naviation message
            generation by the GPS. This will change the way get_PVT.
            :returns: True upon success and false otherwise.
            :rtype: Boolean
        """
        if implicit_update is None:
            implicit_update = True

        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
        self.ublox_packet_cfg['ID'] = self.UBX_CFG_MSG
        self.ublox_packet_cfg['Length'] = 3
        self.ublox_packet_cfg['Start'] = 0
        self.ublox_packet_cfg['Payload'][0] = self.UBX_CLASS_NAV
        self.ublox_packet_cfg['Payload'][1] = self.UBX_NAV_PVT
        self.ublox_packet_cfg['Payload'][2] = 1 if enable else 0

        sent = self.send_command(self.ublox_packet_cfg, max_wait)

        if sent != False:
            self.auto_pvt = enable
            self.auto_pvt_implicit_update = implicit_update

        self.is_module_queried['All'] = False
        return sent

    def hard_reset(self):
        """
            This function resets the ublox module.
            :returns: Nothing
        """
        self.ublox_packet_cfg['Class'] = self.UBX_CLASS_CFG
        self.ublox_packet_cfg['ID'] = self.UBX_CFG_RST
        self.ublox_packet_cfg['Length'] = 4
        self.ublox_packet_cfg['Start'] = 0
        self.ublox_packet_cfg['Payload'][0] = 0xFF
        self.ublox_packet_cfg['Payload'][1] = 0xFF
        self.ublox_packet_cfg['Payload'][2] = 0
        self.ublox_packet_cfg['Payload'][2] = 0

        self.send_command(self.ublox_packet_cfg, 0)

    def factory_reset(self):
        """
            This function resets the ublox module.
            :returns: Nothing
        """
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
        self.hard_reset()

