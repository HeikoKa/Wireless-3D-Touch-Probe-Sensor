# 3D Touch Probe Edge Finder for CNC/VMC Machines
## Description

An in-depth paragraph about your project and overview of use.

## Features
* client (3D touch probe) and server (base station) code 
* based on UDP communication
* many parameters (see header file)
* debug mode for detailed serial monitor print outs
* optional webserver
* optional round trip delay measurement
* lost UDP message detection
* battery voltage monitoring
* WLAN signal strength monitoring
* client and server alive monitoring
* wake up 3D touch client by touching the sensor by hand or by CNC maschine
* send client asleep feature to save battery life
* easy recharging of the client battery by magnetic USB cable
* many more

## Content of the repository
* C/C++ code for ESP8266 client and server
* 3D Fusion 360 model of the 3d touch sensor
* Schematic of the electronics of both ESP8266 boards (not yet)
* Eding CNC macro for inner/outer center finding (not yet)
* Picture of the hardware and picture during build
* Videos of the function (not yet)
* Bill of material (not yet)

## Getting Started
* get two ESP8266 and two mirco USB cables (not just a power USB cable it must support the serial communication)
* download and setup Arduine IDE for the ESP8266
* download the repository (client, server und header file)
* adjust the include file path in the server and client file (because Arduino IDE does not support relative include paths)
* enable DEBUG mode in the header file for the first runs to get as many infos as possible
* compile and download files for client and server


### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

Any advise for common problems or issues.


## Authors

Contributors names and contact info

ex. Dominique Pizzie  
ex. [@DomPizzie](https://twitter.com/dompizzie)

## Version History

* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

