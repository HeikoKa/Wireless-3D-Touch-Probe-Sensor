# 3D Touch Probe Edge Finder for CNC/VMC Machines
## Description

For CNC machines like VMC (Vertical Machining Center) a regular task is to find the center or an edge of a workpiece as a reference for the following machining. However, this can be extremely annoying to do manually by moving the X, Y and Z axis. Professional machinists use so call 3D Touch Probe or Edge Finder to do this automatically. For hobbyists and semi-professionals these Touch Probes are way too expensive. 

There are a lot of examples where people build a DIY 3D Touch Probe, that is basically nothing more than a switch that is connects by a wire to the CNC controller. For machines that make use of an ATC (Automatic Tool Changer) like a carousel this is not suitable, because of the wire from the probe to the CNC controller. In order to make use of a completely automated probing a wireless connection is required. In this project I share my results for a WLAN UDP based 3D Touch Probe consisting of the Probe and a Basestation. While the Probe is working by a recharable battery and a D1 Mini Pro (DSP8266) microcontroller, the Basestation also hosts a DSP8266 microcontroller and is powered by a 5V power supply and is connected to the CNC controller to hand over e.g. the current switch state of the 3D Touch Probe. Although the electronics suits best for the provided mechanics, it is basically possible to use the code and the electronics for any 3D-Touch Probe.

The following picture shows the 3D-Touch-Probe in a ATC tool holder BT30. The round connector is the magnetic charging connector for the internal battery. The transparent middle part is used to show serveral states by LEDs, e.g. power on, WLAN connected, touch probe closed/open, battery low, general error etc. For pictures of the basestation see the /media/pics/ folder.

![3D_Touch_Probe.jpg](/media/pics/3D_Touch_Probe.jpg)

## Features
* X, Y and Z detection
* Client (3D touch probe) and server (base station) code 
* Wireless communication based on WIFI UDP, especially suitable for automatic toolchanger
* Many parameters (see header file)
* Debug mode for detailed serial monitor print outs
* Optional webserver
* Optional round trip delay measurement
* Lost UDP message detection
* Battery voltage monitoring
* WLAN signal strength monitoring
* Client and Server alive monitoring
* Wake up 3D touch client by touching the sensor by hand or automatically by CNC maschine
* Send client asleep feature to save battery life
* Easy recharging of the client battery by magnetic USB cable
* Many more

## Content of the repository
* C/C++ code for ESP8266 client and server
* 3D Fusion 360 model of the 3d touch sensor (not yet)
* Schematic of the electronics of both ESP8266 boards (not yet)
* Eding CNC macro for inner/outer center finding (not yet)
* Picture of the hardware and picture during build
* Videos of the function (not yet)
* Bill of material (not yet)

## Getting Started
* get two D1 Mini Pro (ESP8266) and two mirco USB cables (not just a power USB cable it must support the serial communication)
* download and setup Arduine IDE for the ESP8266
* download the repository (client, server und header file)
* adjust the include file path in the server and client file (because Arduino IDE does not support relative include paths)
* enable DEBUG mode in the header file for the first runs to get as many infos as possible
* compile and download files for client and server
* watch the serial prints of the Client and Server to understand what ist happening


## Mechanical Hardware Build

* No lathe required, CNC maschine and welding ist necessary


## Authors

Heiko Kalte, h.kalte@gmx.de

## Version History

* 0.1
    * Initial Release
    * 
## Donate
If this project help you reduce time to develop, you can give me a cup of coffee :) 

[![paypal](/media/paypal-donate-button.png)](https://www.paypal.com/donate?hosted_button_id=WHYVXK9DHQQHL)

## License

This project is licensed under the GNU General Public License v3.0 License - see the LICENSE.md file for details

