Qwiic_Ublox_Gps_Py
----

<p align="center">
   <img src="https://cdn.sparkfun.com/assets/custom_pages/2/7/2/qwiic-logo-registered.jpg"  width=200>  
   <img src="https://www.python.org/static/community_logos/python-logo-master-v3-TM.png"  width=240>   
</p>
<p align="center">
	<a href="https://pypi.org/project/sparkfun-qwiic-ublox-gps/" alt="Package">
		<img src="https://img.shields.io/pypi/pyversions/sparkfun_ublox_gps_py.svg" /></a>
	<a href="https://github.com/sparkfun/Qwiic_Ublox_Gps_Py/issues" alt="Issues">
		<img src="https://img.shields.io/github/issues/sparkfun/Qwiic_Ublox_Gps_Py.svg" /></a>
	<a href="https://qwiic-ublox-gps-py.readthedocs.io/en/latest/index.html" alt="Documentation">
		<img src="https://readthedocs.org/projects/qwiic-ublox-gps-py/badge/?version=latest&style=flat" /></a>
	<a href="https://twitter.com/intent/follow?screen_name=sparkfun">
        	<img src="https://img.shields.io/twitter/follow/sparkfun.svg?style=social&logo=twitter"
           	 alt="follow on Twitter"></a>
	
</p>

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/15136"><img src="https://cdn.sparkfun.com//assets/parts/1/3/5/1/4/15136-SparkFun_GPS-RTK2_Board_-_ZED-F9P__Qwiic_-03.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15005"><img src="https://cdn.sparkfun.com//assets/parts/1/3/3/2/0/15005-SparkFun_GPS-RTK__Qwiic__-_NEO-M8P-2-00.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15193"><img src="https://cdn.sparkfun.com//assets/parts/1/3/6/1/4/15193-SparkFun_GPS_Breakout_-_U.FL__ZOE-M8__Qwiic_-01.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15210"><img src="https://cdn.sparkfun.com//assets/parts/1/3/6/4/8/15210-SparkFun_GPS_Breakout_-_Chip_Antenna__SAM-M8Q__Qwiic_-01.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/15136">SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)</a></td>
    <td><a href="https://www.sparkfun.com/products/15005">SparkFun GPS-RTK - NEO-M8P-2 (GPS-15005)</a></td>
    <td><a href="https://www.sparkfun.com/products/15193">SparkFun ZOE-M8Q Breakout (GPS-15193)</a></td>
    <td><a href="https://www.sparkfun.com/products/15210">SparkFun SAM-M8Q Breakout (GPS-15210)</a></td>
  </tr>
   <td><a href="https://www.sparkfun.com/products/16344"><img src="https://cdn.sparkfun.com/assets/parts/1/5/0/5/9/16344-SparkFun_GPS-RTK_Dead_Reckoning_Breakout_-_ZED-F9R__Qwiic_-01a.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/16475"><img src="https://cdn.sparkfun.com/assets/parts/1/5/3/3/9/16475-SparkFun_GPS-RTK_Dead_Reckoning_pHAT_for_Raspberry_Pi-01.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15733"><img src="https://cdn.sparkfun.com/assets/parts/1/4/2/9/3/15712-SparkFun_GPS_Breakout_-_NEO-M9N__U.FL__Qwiic_-01.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/16344">SparkFun GPS-RTK Dead Reckoning - ZED-F9R (GPS-16344)</a></td>
    <td><a href="https://www.sparkfun.com/products/16475">SparkFun GPS-RTK Dead Reckoning Phat- ZED-F9R (GPS-16475)</a></td>
    <td><a href="https://www.sparkfun.com/products/15733">SparkFun GPS Dead Reckoning - NEO-M9N (GPS-15733)</a></td>
</table>

This is a Python module for the SparkFun GPS products based on u-blox GPS modules.
	
This package is included in the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py). While the module itself does not use I<sup>2</sup>C, it may none the less join the ranks when the Raspberry Pi has better support for clock stretching. None the less, a Qwiic connector has been included onboard so the GPS module can be used with our along side SparkFun's Qwiic products.

## Contents
* [Supported Platforms](#supported-platforms)
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Documentation](#documentation)
* [Example Use](#example-use)

Supported Platforms
--------------------
The u-blox gps Python package currently supports the following platforms:
* [Raspberry Pi](https://www.sparkfun.com/search/results?term=raspberry+pi)
<!-- Platforms to be tested
* [NVidia Jetson Nano](https://www.sparkfun.com/products/15297)
* [Google Coral Development Board](https://www.sparkfun.com/products/15318)
-->


Documentation
-------------
The SparkFun u-blox gps module documentation is hosted at [ReadTheDocs](https://qwiic-ublox-gps-py.readthedocs.io/en/latest/index.html)

Installation
-------------

### PyPi Installation
This repository is hosted on PyPi as the [sparkfun-ublox_gps](https://pypi.org/project/) package. On systems that support PyPi installation via pip, this library is installed using the following commands

For all users (note: the user must have sudo privileges):
```sh
sudo pip install sparkfun-ublox-gps
```
For the current user:

```sh
sudo pip install sparkfun-ublox-gps
```

### Local Installation
To install, make sure the setuptools package is installed on the system.

Direct installation at the command line:
```sh
python setup.py install
```

To build a package for use with pip:
```sh
python setup.py sdist
 ```
A package file is built and placed in a subdirectory called dist. This package file can be installed using pip.
```sh
cd dist
pip install sparkfun_ublox_gps-<version>.tar.gz
  
```
Example Use
---------------

```python

from ublox_gps import UbloxGps
import serial
# Can also use SPI here - import spidev
# I2C is not supported

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():
  
  try: 
    print("Listenting for UBX Messages.")
    while True:
      try: 
        coords = gps.geo_coords()
        print(coords.lon, coords.lat)
      except (ValueError, IOError) as err:
        print(err)
  
  finally:
    port.close()

if __name__ == '__main__':
  run()
 ```

### Examples Directory
---
* geo_coords_ex1.py 
	* Simple example showing how to get and print out latitude, longitude, and
		heading. 
* gps_time_ex2.py 
	* Simple example showing how to UTC time and how to check its' validity.
* dead_reckoning_ex3.py 
	* Simple example showing how to use dead reckoning on dead reckoning modules.
		Make sure to take a look at our [hookup guide](https://learn.sparkfun.com/tutorials/sparkfun-gps-rtk-dead-reckoning-zed-f9r-hookup-guide) for a detailed explanation on
		where to attach the module and how to calibrate it. 
* stream_nmea_gps_ex4.py 
	* Simple example showing how to stream NMEA data from Ublox Modules.
* modifying_configuration_settings_ex5.py 
	* Simple example showing how change the configuration settings for the Ublox
		Module. 
* using_spi_ex6.py 
	* Simple example showing how to use SPI. 
		Module. 

### Attribution
---
This code is dependent on the work by [daylomople](https://github.com/dalymople) and the awesome parsing capabilities of [ubxtranslator](https://github.com/dalymople/ubxtranslator). 

### To Do
---
* [ ] Some bugs associated with SPI writes.
* [ ] Add more Classes and Messages to `sparkfun_predefines.py` so that a
	greater variety of messages can be parsed. 
* [ ] Fix bug when getting configuration settings. 




