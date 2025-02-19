# 3D Touch Probe Edge Finder for CNC/VMC Machines

See also announcements under Discussions. 
## Description

For CNC machines like VMC (Vertical Machining Center) a regular task is to find the center or an edge of a workpiece as a reference for the following machining. However, this can be extremely annoying to do manually by moving the X, Y and Z axis. Professional machinists use so call 3D Touch Probe or Edge Finder to do this automatically. For hobbyists and semi-professionals these Touch Probes are way too expensive. 

There are a lot of examples where people build a DIY 3D Touch Probe, that is basically nothing more than a switch that is connects by a wire to the CNC controller. For machines that make use of an ATC (Automatic Tool Changer) like a carousel this is not suitable, because of the wire from the probe to the CNC controller. In order to make use of a completely automated probing a wireless connection is required. In this project I share my results for a WLAN UDP based 3D Touch Probe consisting of the Probe and a Basestation. While the Probe is working by a recharable battery and two ESP32 or ESP8266 microcontroller, the Basestation also hosts a ESP32 or ESP8266 microcontroller and is powered by a 5V power supply and is connected to the CNC controller to hand over e.g. the current switch state of the 3D Touch Probe. Although the electronics suits best for the provided mechanics, it is basically possible to use the code and the electronics for any 3D-Touch Probe.

The following picture shows the 3D-Touch-Probe in a ATC tool holder BT30. The round connector is the magnetic charging connector for the internal battery. The transparent middle part is used to show serveral states by LEDs, e.g. power on, WLAN connected, touch probe closed/open, battery low, general error etc. For pictures of the basestation see the /media/pics/ folder. The videos seem not to work online, you might need to download them first. I did not publised all information, because it take a lot of time and I am not sure if there is any interest. So, contact me (h.kalte@gmx.de) and I will answer all you questions.

<img src="https://github.com/user-attachments/assets/fa8e3e30-4a22-4669-bcde-ced0f382cb05" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/b08b1bd6-aa48-4538-9839-da5430fc2dcd" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/6427e9e0-6a6e-452c-a44e-c63c5c17042a" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/851a62f8-6281-4f8f-982e-8389c36a1bd9" alt="3D_Touch_Probe.jpg" width="500">

| Hardware/PCB Revision | Features    | Pictures    |
| :---:   | :---| :---: |
| <b>Basestation 1.0 </b>(prototype, not available)|n/a|n/a|
|<b>Basestation 2.0 </b>(current revision, available and fully functional)|<ul><li>Powerfull <b>ESP32-WROOM-32N Microcontroller</b></li><li><b>Sleep input</b> (transistor protected) for CNC to send battery powered Sensor asleep (by UDP message)</li><li><b>3 transistor protected outputs</b>. Each output can be used in “normal” and open collector mode to indicate to the CNC controller: sensor event, Wifi ready, sensor battery low</li><li><b>Output mode to CNC controller individually settable</b> by jumpers</li><li><b>6 LEDs for current status</b>: Power On, Wlan Ready, Battery Low, Sleep, Sensor Event, General Error</li><li><b>3 additional GPI/Os</b> of the ESP32 for general purpose features</li><li><b>USB connector and USB to UART chip onboard</b> that handles software update and debugging outputs.</li><li>2 switches for <b>Reset and Boot</b></li><li><b>External Antenna</b> connector</li><li>Specifically design to fit in a <b>DIN Rail Terminal Enclosure</b></li><li><b>2 layer PCB design</b></li><li><b>5V Power Supply recommended</b>, 12 V not recommended and 24V not possible.</li><li>Individually <b>solder by me</b></li></ul> | ![IMG_20250219_104215](https://github.com/user-attachments/assets/2d6c2f27-f6b2-4797-805f-447b445f564c)   |
|<b>Basestation 3.0</b> (designed but yet not manufactured) |<ul><li>All Features of revision 2.0 plus</li><li><b>One additional protected output</b> to the CNC controller (4 in total)</li><li>LEDs and outputs to CNC are independent (allows switching of the output polarity independently from the LED)</li><li><b>4-layer PCB</b> and improved PCB design to better protect against the rough environment of a CNC mill</li><li>Better <b>accessibility of the USB port</b></li><li>Automatic detection of the <b>hardware revision</b></li><li>DC-DC converter to power the hardware up to <b>24V supply</b>.</li><li><b>ESP protection</b> (TVS diode)</li><b>Fuse</b> to protect the board and the environment</li></ul>|![image](https://github.com/user-attachments/assets/6c612234-3904-421a-bafc-f3462f5f83fe)|





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
* C/C++ code for ESP32 and ESP8266 client and server
* 3D Fusion 360 model of the 3d touch sensor (not yet)
* Schematic of the electronics of both ESP32 and ESP8266 boards (not yet)
* Eding CNC macro for inner/outer center finding (not yet)
* Picture of the hardware and picture during build
* Videos of the function
* Bill of material (not yet)

## Getting Started
* get two ESP32 (or ESP8266) boards and two USB-C cables (not just a power USB cable it must support the serial communication)
* download and setup Arduine IDE for the ESP32 or ESP8266
* download the repository (client, server und header file)
* adjust the include file path in the server and client file (because Arduino IDE does not support relative include paths)
* enable DEBUG mode in the header file for the first runs to get as many infos as possible
* compile and download files for client and server
* watch the serial prints of the Client and Server to understand what ist happening


## Mechanical Hardware Build

* No lathe required, CNC maschine and welding ist necessary
* Lathe tools in a vise is required


## Authors

Heiko Kalte, h.kalte@gmx.de

## Version History

* 0.1
    * Initial Release
      
## Donate
If this project help you reduce time to develop, you can give me a cup of coffee :) 

[![paypal](/media/paypal-donate-button.png)](https://www.paypal.com/donate?hosted_button_id=WHYVXK9DHQQHL)

## License

This project is licensed under the GNU General Public License v3.0 License - see the LICENSE.md file for details

