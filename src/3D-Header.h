/**  Header
  *   
  *  @author Heiko Kalte  
  *  @date 15.03.2025 
  * 
  *  @version 1.0
  */


//TODOs
//is NANOSEC_PER_TICK for ESP32 the same as for ESP8266?


#ifndef CNCSENSOR_H_
#define CNCSENSOR_H_

//## user defines start ##
//general defines
#define DEBUG                   // enable debug output via serial interface on client and server
//#define DEBUG_SHOW_CORE        // Additional debug info on which cpu core the current function is running
//#define DEBUG_SHOW_ALL_TRANSMISSIONS  // show all wifi transmissions during DEBUG
//#define CYCLETIME              // enable measuring the round trip delay from client to server back to client

// server/basestation specific defines
#define SERVER_ESP32            // defines server as a ESP32 instead of a ESP8266, comment out to switch to ESP8266
#define SERVER_HW_REVISION_3_0  // set this define if basestation/server is hardware revision 3.0 or later
//#define WEBSERVER               // enable webserver

// client specific defines
#define CLIENT_ESP32         // defines client as a ESP32 (e.g. Waveshare ESP32-S3-Zero)instead of a ESP8266, comment out to switch to ESP8266
#define CLIENT_HW_REVISION_2_0  // set this define if client is hardware PCB revision 2.0 or later
//#define SHOW_STATE_COLORS       // if enabled at the startup of the client all state colors are shown one after the other
//## user defines end ##

#include "IPAddress.h"

namespace CncSensor{

  //RGB Colors                                //red                    , green                  , blue
  #define           POWER_COLOR                 CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS    //white  (255, 255, 255)
  #define           CHARGE_COLOR                0                      , CLIENT_RGB_BRIGHTNESS  , 0                        //green  (0  , 255,   0)
  #define           TOUCH_COLOR                 0                      , 0                      , CLIENT_RGB_BRIGHTNESS    //blue   (0  ,   0, 255)
  #define           WLAN_COLOR                  CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS    //white  (255, 255, 255)
  #define           RSSI_ERROR_COLOR            CLIENT_RGB_BRIGHTNESS  , 0                      , CLIENT_RGB_BRIGHTNESS    //magenta(255,   0, 255)
  #define           BATTERY_ERROR_COLOR         CLIENT_RGB_BRIGHTNESS  , 0                      , 0                        //red    (255,   0,   0)
  #define           WIFI_ERROR_COLOR            CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS  , 0                        //yellow (255, 255,   0)
  #define           ALIVE_ERRROR_COLOR          CLIENT_RGB_BRIGHTNESS  , CLIENT_RGB_BRIGHTNESS/2, 0                        //orange (255, 127,   0)

  //datatypes
  enum fadingType  {NONE, UP, DOWN};
  enum batCharType {NORMAL=0, MEAN=1, CRIT=2};   // battery voltage levels (LOW is already used, taking MEAN instead)
  
  //general consts
  const int          BAUD_RATE                   = 9600;                  // Baud rate for serial outputs/debug
  const char*        SSID                        = "3D-Touch-WIFI";       // WLAN network name to search for when connecting to the webserver
  const char*        password                    = "123456789";           // WLAN password

  //server consts
  const char*        SERVER_SLEEP_MSG            = "sleep";               // UDP message from server to client to send client asleep
  const char*        SERVER_HELLO_MSG            = "hello";               // UDP message from server to client to say hello
  const char*        SERVER_REPLY_MSG            = "reply";               // UDP message from server to client to reply to hello msg
  const char*        SERVER_ALIVE_MSG            = "alive";               // UDP message from server to show server is alive  
  const int          CLIENT_ALIVE_CNT_MAX        = 2;                     // maximum client alive counter value
  const long         WIFI_RSSI_REPORT_LEVEL      = -85;                   // RSSI level, WLAN signal strength e.g. -40 is better -70 is worse (best was -44)
  const int          SERVER_TICKS_ARRAY_SIZE     = 10;                    // Array Size of measured ticks
  const bool         SERVER_SLEEP_IN_POLARITY    = true;                  // Invert SLEEP Input polarity. False means high input send sensor aleep. Truee means low input send sensor aleep (be aware that the input is inverted by a transistor, behaviour will be inverted without an external transistor)
  const bool         SERVER_WLAN_OUT_POLARITY    = true;                  // (works only with server PCB version 3 or later) invert polarity of WLAN output, false means high represents active output, true inverts output and low is active
  const bool         SERVER_TOUCH_OUT_POLARITY   = true;                  // (works only with server PCB version 3 or later) invert polarity of TOUCH output, false means high represents active output, true inverts output and low is active
  const bool         SERVER_ERROR_OUT_POLARITY   = true;                  // (works only with server PCB version 3 or later) invert polarity of ERROR output, false means high represents active output, true inverts output and low is active
  const bool         SERVER_BAT_ALM_OUT_POLARITY = true;                  // (works only with server PCB version 3 or later) invert polarity of BAT ALARM output, false means high represents active output, true inverts output and low is active
  #ifdef SERVER_ESP32
    const float      NANOSEC_PER_TICK            = 12.5;                  // ## Attention this value has not been verified for the ESP32 ##
  #else
    const float      NANOSEC_PER_TICK            = 12.5;                  // number of nanosecond per measured tick in CCOUNT register
  #endif

  //client consts
  const char*        CLIENT_HELLO_MSG        = "hello";                   // UDP message from client to server to say hello
  const char*        CLIENT_REPLY_MSG        = "reply";                   // UDP message from client to server to reply to hello msg
  const char*        CLIENT_TOUCH_HIGH_MSG   = "high";                    // UDP message from client if 3D touch probe is high
  const char*        CLIENT_TOUCH_LOW_MSG    = "low";                     // UDP message from client if 3D touch probe is low
  const char*        CLIENT_ALIVE_MSG        = "alive";                   // UDP message from client alive
  const char*        CLIENT_RSSI_MSG         = "rssi";                    // UDP message from client with rssi (WLAN signal strength)
  const char*        CLIENT_CYCLE_MSG        = "CYC:";                    // UDP message from client to server to transmit the last measured cycle time ticks
  const char*        CLIENT_INFO_MSG         = "INFO:";                   // UDP message from client to server with infos about the client
  const int          SERVER_ALIVE_CNT_MAX    = 2;                         // maximum server alive counter value, try to reconnect
  const int          SERVER_ALIVE_CNT_DEAD   = 3;                         // maximum server alive counter value, server seems to be dead
  const int          SERVER_AQUN_CNT_MAX     = 4000;                      // number of loop cycles before the server must acknowledge the high/low messages
  const int          TOUCH_PIN_DEBOUNCE      = 700;                       // debounce time in µs for software touch input pin
  const bool         NO_SLEEP_WHILE_CHARGING = true;                      // prevent sleeping during battery loading, e.g. to keep status LED on
  const uint8_t      CLIENT_RGB_BRIGHTNESS   = 20;                        // brightness of the RGB LED (max 255). Product of CLIENT_RGB_BRIGHTNESS x RGB_FADE_SPEED = Fading Time at start and end
  const uint8_t      CLIENT_RGB_FADE_SPEED   = 100;                       // speed for fading out the LED brightness when going to sleep. Set as delay, smaller value faster fading
  const bool         CLIENT_ALLOW_LED_FADING = true;                      // allow fading LED at startup and sleep. Leads to longer startup time, but looks nice.
  const bool         CLIENT_TOUCH_POLARITY   = true;                      // invert sensor touch input 
  const uint8_t      TRANSMISSION_RETRY_MAX  = 3;                         // maximum wifi transmission retrys before an transmission error is reported
  //client and server consts
  const uint32_t     SERVICE_INTERVALL       = 4615385;     //~15sec      // ESP8266 timer ticks for service interrupt (ESP8266 max 8388607) 
  const uint32_t     SERVICE_INTERVALL_ESP32 = 15000000;                  // ESP32 Time in 1MHz ticks between service interrupt calls (20000000 = 20sec)
  const int          UDP_PACKET_MAX_SIZE     = 128;                       // UDP buffer size
  const uint8_t      SLOW_BLINK              = 1000;                      // delay for slow blinking LED
  const uint8_t      FAST_BLINK              = 500;                       // delay for fast blinking LED
  const uint8_t      WLAN_INIT_PAUSE         = 100;                       // Pause length during WLAN initiation

  //IP addresses
  IPAddress         serverIpAddr(192,168,  2,1);                          // local IP address of server and webserver ip address if enabled
  IPAddress         clientIpAddr(192,168,  2,2);                          // local IP address of client
  IPAddress         gateway     (192,168,  2,0);                          // gateway of server, need to have a value, but does not exist
  IPAddress         subnet      (255,255,255,0);                          // subnet mask of WLAN network

  //Global variables
  unsigned int      serverUdpPort            = 4211;                      // server port
  unsigned int      clientUdpPort            = 4210;                      // client port

  //Battery consts
  const float       BAT_LOW_VOLT             = 3.7;                       // battery voltage below this, is indicated as low battery state
  const float       BAT_CRIT_VOLT            = 3.6;                       // battery voltage below this, is indicated as critical battery state
  const float       BAT_CORRECTION           = 0.2;                       // use this correction for the battery voltage if analog measure differs from multimeter measure
  const char*       CLIENT_BAT_OK_MSG        = "bat ok";                  // UDP message for battery voltage is ok
  const int         CLIENT_BAT_OK            = 1;                         // internal coding for battery is ok
  const char*       CLIENT_BAT_LOW_MSG       = "bat low";                 // UDP message for battery is low 
  const int         CLIENT_BAT_LOW           = 2;                         // internal coding for battery is low
  const char*       CLIENT_BAT_CRITICAL_MSG  = "bat critical";            // UDP message for battery is critical
  const int         CLIENT_BAT_CRITICAL      = 3;                         // internal coding for battery is critical

  #ifdef SERVER_ESP32
    //ESP32 Server specific LEDs
    #ifdef SERVER_HW_REVISION_3_0
      // Sensor Basestation hardware version 3.0 distinguishs between outputs for LED and output for CNC controller
      const uint8_t     SERVER_POWER_LED        = 14;                     // Attention: Power LED, is not used in ESP32 PCB versions
      const uint8_t     SERVER_WLAN_LED         = 21;                     // LED to inducate the current WLAN state
      const uint8_t     SERVER_TOUCH_LED        = 32;                     // LED to indicate a Touch of the 3D sensor
      const uint8_t     SERVER_ERROR_LED        = 19;                     // Error output that can e.g tell the cnc controller to stop
      const uint8_t     SERVER_SLEEP_IN         = 33;                     // Input pin for the CNC controller to indicate that the slave/sensor can go to sleep
      const uint8_t     SERVER_SLEEP_LED        = 22;                     // Sleep output LED, indicates that the server/Basestation request the sensor to go asleep
      const uint8_t     SERVER_BAT_ALM_LED      = 23;                     // Client battery status
      // additional channels for hardware revision 3.0 or later
      const uint8_t     SERVER_WLAN_OUT         = 4;                      // LED to indicate the current WLAN state for the CNC controller
      const uint8_t     SERVER_TOUCH_OUT        = 17;                     // LED to indicate a Touch of the 3D sensor for the CNC controller
      const uint8_t     SERVER_ERROR_OUT        = 5;                      // general error output that can hold the cnc controller
      const uint8_t     SERVER_BAT_ALM_OUT      = 18;                     // Low battery output for the CNC controllerthat
    #else //SERVER_HW_REVISION_3_0
      //Following Data is valid for the Sensor Basestation PCB hardware version 1.0 and 2.0
      //ESP32 Server specific LEDs  
      const uint8_t     SERVER_POWER_LED        = 22;                     // Attention: Power LED, is not used in ESP32 PCB versions
      const uint8_t     SERVER_WLAN_LED         = 5;                      // LED to inducate the current WLAN state
      const uint8_t     SERVER_TOUCH_LED        = 17;                     // LED to indicate a touch of the 3D sense
      const uint8_t     SERVER_ERROR_LED        = 23;                     // ERROR output that can hold the cnc controller, e.g.critical battery or no more alive msg from client
      const uint8_t     SERVER_SLEEP_IN         = 33;                     // Input pin for the CNC controller to indicate that the slave can go to sleep
      const uint8_t     SERVER_SLEEP_LED        = 19;                     // Sleep Output for LED
      const uint8_t     SERVER_BAT_ALM_LED      = 21;                     // server battery is low
    #endif //SERVER_HW_REVISION_3_0
    const uint8_t     SERVER_HW_REVISION_0      = 25;                     // The basestation/server hardware PCB revision is coded into 3 input bit, this is bit 0
    const uint8_t     SERVER_HW_REVISION_1      = 26;                     // The basestation/server hardware PCB revision is coded into 3 input bit, this is bit 1
    const uint8_t     SERVER_HW_REVISION_2      = 27;                     // The basestation/server hardware PCB revision is coded into 3 input bit, this is bit 2

  #else  //#ifdef SERVER_ESP32
    //ESP8266 Server specific LEDs
    const uint8_t     SERVER_POWER_LED            = 22;                   // Power LED 
    const uint8_t     SERVER_WLAN_LED             = 5;                    // LED to inducate the current WLAN state
    const uint8_t     SERVER_TOUCH_LED            = 17;                   // LED and output to indicate a touch of the 3D sense
    const uint8_t     SERVER_ERROR_LED            = 23;                   // Error output that can hold the cnc controller, e.g.critical battery or no more alive msg from client
    const uint8_t     SERVER_SLEEP_IN             = 33;                   // Input pin for the CNC controller to indicate that the slave can go to sleep
    const uint8_t     SERVER_SLEEP_LED            = 19;                   // Sleep output for LED
    const uint8_t     SERVER_BAT_ALM_LED          = 21;                   // client battery is low
  #endif //#ifdef SERVER_ESP32

  #ifdef CLIENT_ESP32
    //ESP32Client specific LEDs
    const uint8_t     CLIENT_RGB_LED_OUT          = 27;                   // client rgb led instead of multiple leds
    const uint8_t     CLIENT_TOUCH_IN             = 33;                   // digital input pin to listen
    const uint8_t     CLIENT_SLEEP_OUT            = 4;                    // controls external sleep hardware
    const uint8_t     CLIENT_ANALOG_CHANNEL       = 38;                   // Analog In channel for reading the battery voltage
    const uint8_t     CLIENT_CHARGE_IN            = 25;                   // Battery Charging Input
    const uint8_t     CLIENT_HW_REVISION_0        = 19;                   // (Supported by client PCB version 2.0 and later) The client hardware PCB revision is coded into 3 input bit, this is bit 0
    const uint8_t     CLIENT_HW_REVISION_1        = 22;                   // (Supported by client PCB version 2.0 and later)The client hardware PCB revision is coded into 3 input bit, this is bit 1
    const uint8_t     CLIENT_HW_REVISION_2        = 21;                   // (Supported by client PCB version 2.0 and later)The client hardware PCB revision is coded into 3 input bit, this is bit 2

  #else //#ifdef CLIENT_ESP32
    //ESP8266 Client specific LEDs
    const uint8_t     CLIENT_RGB_LED_OUT          = 21;                   // client rgb led
    const uint8_t     CLIENT_TOUCH_IN             = 12;                   // (D6) digital input pin to listen
    const uint8_t     CLIENT_SLEEP_OUT            = 14;                   // (D5) controls external sleep hardware
    const uint8_t     CLIENT_CHARGE_IN            = 13;                   // (D7) Battery Charging Input
  #endif //ifdef CLIENT_ESP32
}  // namespace CncSensor
#endif