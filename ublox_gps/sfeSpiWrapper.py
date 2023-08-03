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

import spidev

class sfeSpiWrapper(object):
    """
    sfeSpiWrapper

    Initialize the library with the given port.

    :param spi_port:    This library simply provides some ducktyping for spi so
                        that the ubxtranslator library doesn't complain. It
                        takes a spi port and then sets it to the ublox module's
                        specifications.

    :return:            The sfeSpiWrapper object.
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
