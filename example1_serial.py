"""An example of how to use the light parser for parsing UBX messages from a serial device.

You will need to change the port name to that of the port you want to connect to. Also make sure that the baud rate is
correct and that the device has been setup to output the messages via UBX protocol to your desired port!

The serial package could easily be replaced with an alternative.
"""
import serial

from ublox_gps import UbloxGps

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():

    try:
        print("Listening for UBX Packets")
        while True:
            try:
                print(gps.ublox_debug())

            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == '__main__':
    run()
