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

    'Time'           : 0,
    'Latitude'       : 0.0,
    'Lat'            : 0.0,
    'Lat_Direction'  : "",
    'Longitude'      : 0.0,
    'Long'            : 0.0,
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

