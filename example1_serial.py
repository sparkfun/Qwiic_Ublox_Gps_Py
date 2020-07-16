"""An example of how to use the light parser for parsing UBX messages from a serial device.

You will need to change the port name to that of the port you want to connect to. Also make sure that the baud rate is
correct and that the device has been setup to output the messages via UBX protocol to your desired port!

The serial package could easily be replaced with an alternative.
"""
import serial

from ubxtranslator import core
from ublox_gps_new import UbloxGps

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():


    print("Starting to listen for UBX packets")

    print(gps.get_nav())

    try:
        print("Reading from NAV or ACK(fingers crossed)")
        while True:
            try:
                msg= parser.receive_from(port)
                print(msg)
            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == '__main__':
    run()
