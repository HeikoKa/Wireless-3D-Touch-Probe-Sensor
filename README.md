# Wireless 3D Touch Probe Edge Finder for CNC/VMC Machines

See also announcements under Discussions. 

## Supporters

[<img src="https://github.com/user-attachments/assets/2e52d4b8-d1d0-4040-a53e-2fd4b7a10689" alt="3D_Touch_Probe.jpg" width="300">](https://www.pcbway.com/)

## Description

For CNC machines like VMC (Vertical Machining Center) a regular task is to find the center or an edge of a workpiece as a reference for the following machining. However, this can be extremely annoying to do manually by moving the X, Y and Z axis. Professional machinists use so call 3D Touch Probe or Edge Finder to do this automatically. For hobbyists and semi-professionals these Touch Probes are way too expensive. 

There are a lot of examples where people build a DIY 3D Touch Probe, that is basically nothing more than a switch that is connects by a wire to the CNC controller. For machines that make use of an ATC (Automatic Tool Changer) like a carousel this is not suitable, because of the wire from the probe to the CNC controller. In order to make use of a completely automated probing a wireless connection is required. In this project I share my results for a WLAN UDP based 3D Touch Probe consisting of the Probe and a Basestation. While the Probe is working by a recharable battery and a ESP32 microcontroller, the Basestation also hosts a ESP32 microcontroller and is powered by a 5-24V power supply and is connected to the CNC controller to hand over e.g. the current switch state of the 3D Touch Probe. Although the electronics suits best for the provided mechanics, it is basically possible to use the code and the electronics for any 3D-Touch Probe.

The following picture shows the 3D-Touch-Probe in a ATC tool holder BT30. The round connector is the magnetic charging connector for the internal battery. The transparent middle part is used to show serveral states by LEDs, e.g. power on, WLAN connected, touch probe closed/open, battery low, general error etc. The videos in the media area seem not to work online, you might need to download them first. I did not publised all information, because it take a lot of time and I am not sure if there is any interest. So, contact me (h.kalte@gmx.de) and I will answer all you questions.

<img src="https://github.com/user-attachments/assets/fa8e3e30-4a22-4669-bcde-ced0f382cb05" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/b08b1bd6-aa48-4538-9839-da5430fc2dcd" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/6427e9e0-6a6e-452c-a44e-c63c5c17042a" alt="3D_Touch_Probe.jpg" width="500">
<img src="https://github.com/user-attachments/assets/851a62f8-6281-4f8f-982e-8389c36a1bd9" alt="3D_Touch_Probe.jpg" width="500">

## Sensor Server/Basestation Hardware
| Hardware/PCB Revision | Features    | Pictures    |
| :---:   | :---| :---: |
| <b>Basestation 1.0 </b>(prototype, not available)|n/a|n/a|
|<b>Basestation 2.0 </b>(older revision, available and fully functional)|<ul><li>Powerfull <b>ESP32-WROOM-32N Microcontroller</b></li><li><b>Sleep input</b> (transistor protected) for CNC to send battery powered Sensor asleep (by UDP message)</li><li><b>3 transistor protected outputs</b>. Each output can be used in “normal” and open collector mode to indicate to the CNC controller: sensor event, Wifi ready, sensor battery low</li><li><b>Output mode to CNC controller individually settable</b> by jumpers</li><li><b>6 LEDs for current status</b>: Power On, Wlan Ready, Battery Low, Sleep, Sensor Event, General Error</li><li><b>3 additional GPI/Os</b> of the ESP32 for general purpose features</li><li><b>USB connector and USB to UART chip onboard</b> that handles software update and debugging outputs.</li><li>2 switches for <b>Reset and Boot</b></li><li><b>External Antenna</b> connector</li><li>Specifically design to fit in a <b>DIN Rail Terminal Enclosure</b></li><li><b>2 layer PCB design</b></li><li><b>5V Power Supply recommended</b>, 12 V not recommended and 24V not possible.</li><li>Individually <b>solder by me</b></li></ul> | <img src="https://github.com/user-attachments/assets/bf0c96e3-1395-4ab1-825e-0e64b0bda174" width="600">  |
|<b>Basestation 3.0</b> (available and fully functional) |<ul><li>All Features of revision 2.0 plus</li><li><b>One additional protected output</b> to the CNC controller (4 in total)</li><li><b>LEDs and outputs to CNC are independent</b> (allows switching of the output polarity independently from the LED)</li><li><b>4-layer PCB</b> and improved PCB design to better protect against the rough environment of a CNC mill</li><li>Better <b>accessibility of the USB port</b></li><li>Automatic detection of the <b>hardware revision</b></li><li>DC-DC converter to power the hardware up to <b>24V supply</b>.</li><li><b>ESP protection</b> (TVS diode)</li><b>Fuse</b> to protect the board and the environment</li></ul>| <img src="https://github.com/user-attachments/assets/f84240f4-f113-48ea-a520-0c5cbe9c474a" alt="3D_Touch_Probe.jpg" width="500">|
|<b>Basestation 4.0</b> (current Version, available and fully functional) |<ul><li>All Features of revision 3.0 plus</li><li><b>UART LEDs</b> for TX and RX</li><li>Removed some resistors and capacitors for <b>simplification</b></li><li>Improved <b>DC-DC</b> layout</li><li>Improved <b>Footprint</b></li><li>Improved placement of <b>LEDs and Jumpers</b></li></ul>|<img src="https://github.com/user-attachments/assets/c8e730fd-47c5-489e-bc2d-f417f7367d31" alt="3D_Touch_Probe.jpg" width="500">|
|<b>Basestation 5.0</b> (planning) |<ul><li>All Features of revision 4.0 plus</li><li>Dedicated<b> JTAG PORT</b></li></ul>|<img src="https://github.com/user-attachments/assets/1cacb21f-7009-4f00-b428-5a8b5f2d1896" alt="3D_Touch_Probe.jpg" width="500"><img src="https://github.com/user-attachments/assets/fb75ed11-8288-449e-9f5f-d68fe7428396" alt="3D_Touch_Probe.jpg" width="500">|

## Sensor Client Hardware
| Hardware/PCB Revision | Features    | Pictures    |
| :---:   | :---| :---: |
|<b>Sensor 1.0</b> (older revision, available and fully functional)|<ul><li>Highly integrated <b>ESP32-PICO-V3-02</b></li><li><b>Specifically designed power management</b> to ensure longest possible operation and sleep time when powered by battery Lithium polymer battery (LiPo). During sleep everything is switched off except for one pull up resistor. </li><li>Highly integrated and very small <b>outline 35mm diameter</b></li><li><b>Wake up</b> by simple touch of the sensor mechanic (mechanical switch must be a normally closed switch) </li><li><b>DC-DC converter</b> for efficient power management and to handle changing LiPo voltage during operation. </li><li><b>LiPo charging IC</b> on board to comfortably charge LiPo by e.g., USB cable. </li><li><b>UART connector</b> for Software update and debugging. </li><li><b>Charging status</b> is detected by the Microcontroller and can be displayed via LED (can also be send via UDP message to the basestation and webserver) </li><li><b>Battery voltage is measured</b>, and low and critical battery voltage can be indicated by LED and send to basestation to inform CNC controller by general purpose input. </li><li><b>Connectors</b> for: LiPo battery, Sensor Switch and charging device. </li><li>Individual sensor <b>debounce capacitor</b> possible</li><li><b>RGB LED for status information</b>. Blinking and individual colours are possible, e.g. Power, WLAN initialization, Sensor status, Charging, low and critical battery voltage etc. </li><li>On board <b>ceramic antenna</b> (in this version no BNC connector for an external antenna) </li><li><b>Several additional ESP32 I/O</b> on test points</li><li><b>4-layer PCB design</b> to cope with rough conditions</li></ul> | <img src="https://github.com/user-attachments/assets/0572da01-13b3-4c58-a336-5981b5508ffa" width="700">  |
|<b>Sensor 2.0</b> (revision available and fully functional) |<ul><li><b>Additional BNC connector for external antennas</b> (only necessary in complete closed mechanical designs)</li><li>Improved <b>battery measurement</b></li><li><b>Improved charging</b> detection</li><li>Smaller and <b>more robust buttons</b> for Reset and Boot</li><li>Improved <b>DC-DC Layout</b></li><li><b>Small PCB errors fixed</b></li><li>Automatic <b>PCB revision detection</b></li></ul>|<img src="https://github.com/user-attachments/assets/997f4eee-eff9-4f00-9452-59547570dee8" width="700">|
|<b>Sensor 3.0</b> (current revision, available and fully functional) |<ul><li>Improved <b>Antenna Design</b></li><li><b>Optional fully automated wakeup and sleep</b> if charger is disconnected and connected (usecase LiPo charger is in tool magazin)</li><li><b>5V charging voltage</b> detection</li><li>Fix for <b>analog battery voltage measurement</b></li><li>Improved <b>DC-DC Layout</b></li><li><b>Small PCB errors fixed</b></li><li><b>Bottom Overlay</b> with more infos</li></ul>|<img src="https://github.com/user-attachments/assets/e0adad99-6f2d-4927-8aac-2c8b0d36afc9" width="700">|
|<b>Sensor 4.0</b> (planning) |<ul><li>All Features of revision 3.0 plus</li><li>Dedicated<b> JTAG PORT</b></li></ul>|<img src="https://github.com/user-attachments/assets/e33ef636-db1f-40ec-86c8-3f98f8860e59" alt="3D_Touch_Probe.jpg" width="500"><img src="https://github.com/user-attachments/assets/5632eb0a-e748-4637-9885-9afc0b7008d4" alt="3D_Touch_Probe.jpg" width="500">|

## Features
* **X, Y and Z** detection
* **Wireless communication** based on WIFI, especially suitable for automatic toolchanger
* specifically designe "**hardware deep sleep**"
* **Client Wake up** by:
  - 3D touch action
  - Automatic wake up by disconnecting charging voltage (charger in tool magazin)
* **Client Sleep** by:
  - Sleep by CNC controller interaction
  - Sleep if Basestation is not available
  - Sleep by watchdog (not used for a certain time)
  - Sleep by charging connected (charger in tool magazin)
  
* **Battery voltage monitoring**
* WLAN **signal strength monitoring**
* Client and Server **alive monitoring** 
* **Easy UART Debug mode** for detailed serial monitor print outs
* Optional **webserver**
* Optional **round trip delay measurement**
* **Easy recharging** of the client battery/Lipo by on board charging chip 
* **Many parameters** for highly adaptability and wide range of features (see header file)
* Many more

## Authors

Heiko Kalte, h.kalte@gmx.de
      
## Donate
If this project help you reduce time to develop, you can give me a cup of coffee :) 

[![paypal](/media/paypal-donate-button.png)](https://www.paypal.com/donate?hosted_button_id=WHYVXK9DHQQHL)

## License

This project is licensed under the GNU General Public License v3.0 License - see the LICENSE.md file for details

