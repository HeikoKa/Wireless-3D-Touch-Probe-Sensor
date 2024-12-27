/**  Header
  *   
  *  @author Heiko Kalte  
  *  @date 29.11.2024 
  * 
  *  @version 0.2
  */
  // 0.1:   Time measurement for directive CYCLETIME
  //        Renamed Client BAT_LED to CLIENT_ERROR_OUT
  //        Reduced waiting times for "alive"
  //        New message CLIENT_CYCLE_MSG added 
  // 0.2:   Changed function due to new sleep hardware (former deep sleep consumes to much power)


//TODOs
//is NANOSEC_PER_TICK for ESP32 the same as for ESP8266?


#ifndef CNCSENSOR_H_
#define CNCSENSOR_H_


//###################### change defines start
#define DEBUG       // enable debug output via serial interface on client and server
#define CYCLETIME   // enable measuring the round trip delay from client to server back to client
#define WEBSERVER   // enable webserver

// following defines can not be mixed at the same time, either both ESP32 or both undefined, otherwise a compile error occures
// if you have mixed hardware configurations then you have to change defines between compiling client and server
//#define SERVER_ESP32    // defines server as a ESP32 (Waveshare ESP32-S3-Zero) instead of a ESP8266, comment out to switch to ESP8266
#define SERVER_RGB_LED  // do you have multiple LEDs or just ohne RGB LED
//#define CLIENT_ESP32 // defines client as a ESP32 (Waveshare ESP32-S3-Zero)instead of a ESP8266, comment out to switch to ESP8266
#define CLIENT_RGB_LED  // ###############################################do you have multiple LEDs or just ohne RGB LED
//###################### change defines end



#if defined SERVER_ESP32 && defined CLIENT_ESP32 //ESP32
    #include <WiFi.h>
  #else //ESP8266
    #include <ESP8266WiFi.h>
#endif


namespace CncSensor{
 
  const int          BAUD_RATE              = 9600;                     // Baud rate for serial outputs/debug
  const char*        SSID                   = "3D-Touch-WIFI";          // WLAN network name
  const char*        password               = "123456789";              // WLAN password

  //server consts
  const char*        SERVER_SLEEP_MSG       = "sleep";                  // UDP message from server to client to send client asleep
  const char*        SERVER_HELLO_MSG       = "hello";                  // UDP message from server to client to say hello
  const char*        SERVER_REPLY_MSG       = "reply";                  // UDP message from server to client to reply to hello msg
  const char*        SERVER_ALIVE_MSG       = "alive";                  // UDP message from server to show server is alive  
  const int          CLIENT_ALIVE_CNT_MAX   = 2;                        // maximum client alive counter value
  const long         WIFI_RSSI_REPORT_LEVEL = -80;                      // RSSI level, WLAN signal strength e.g. -40 is better -70 is worse (best was -44)
  const int          SERVER_TICKS_ARRAY_SIZE= 10;                       // Array Size of measured ticks
  const float        NANOSEC_PER_TICK       = 12.5;                     // number of nanosecond per measured tick in CCOUNT register
  //client consts
  const char*        CLIENT_HELLO_MSG       = "hello";                  // UDP message from client to server to say hello
  const char*        CLIENT_REPLY_MSG       = "reply";                  // UDP message from client to server to reply to hello msg
  const char*        CLIENT_TOUCH_HIGH_MSG  = "high";                   // UDP message from client if 3D touch probe is high
  const char*        CLIENT_TOUCH_LOW_MSG   = "low";                    // UDP message from client if 3D touch probe is low
  const char*        CLIENT_ALIVE_MSG       = "alive";                  // UDP message from client alive
  const char*        CLIENT_RSSI_MSG        = "rssi";                   // UDP message from client with rssi (WLAN signal strength)
  const char*        CLIENT_CYCLE_MSG       = "CYC:";                   // UDP message from client to server to transmit the last measured cycle time ticks
  const int          SERVER_ALIVE_CNT_MAX   = 1;                        // maximum server alive counter value, try to reconnect
  const int          SERVER_ALIVE_CNT_DEAD  = 2;                        // maximum server alive counter value, server seems to be dead
  const int          SERVER_AQUN_CNT_MAX    = 2000;                     // number of loop cycles before the server must acknowledge the high/low messages
  const int          TOUCH_PIN_DEBOUNCE     = 700;                      // debounce time in µs for touch input pin


  //client and server consts
  const uint32_t     SERVICE_INTERVALL       = 4000000;                  // ESP8266 timer ticks for service interrupt (ESP8266 max 8388607) 
  const uint32_t     SERVICE_INTERVALL_ESP32 = 30;                       // ESP32 Time in sec between service interrupt calls
  const int          UDP_PACKET_MAX_SIZE     = 16;                       // UDP buffer size
  const uint8_t      SLOW_BLINK              = 1000;                     // delay for slow blinking LED
  const uint8_t      FAST_BLINK              = 500;                      // delay for fast blinking LED
  const uint8_t      SERVER_RGB_BRIGHTNESS   = 5;                        // brightness of the RGB LED
  const uint8_t      CLIENT_RGB_BRIGHTNESS   = 5;                        // brightness of the RGB LED
 
  //IP addresses
  IPAddress         serverIpAddr(192,168,  2,1);                        // local IP address of server
  IPAddress         clientIpAddr(192,168,  2,2);                        // local IP address of client
  IPAddress         gateway     (192,168,  2,0);                        // gateway of server, need to have a value, but does not exist
  IPAddress         subnet      (255,255,255,0);                        // subnet mask of WLAN network

  //global variables
  unsigned int      serverUdpPort           = 4211;                     // server port
  unsigned int      clientUdpPort           = 4210;                     // client port

  //Battery consts
  const float       BAT_LOW_VOLT            = 3.50;                     // battery voltage below this, is indicated as low battery state
  const float       BAT_CRIT_VOLT           = 3.43;                     // battery voltage below this, is indicated as critical battery state
  const char*       CLIENT_BAT_OK_MSG       = "bat ok";                 // UDP message for battery voltage is ok
  const int         CLIENT_BAT_OK           = 1;                        // internal coding for battery is ok
  const char*       CLIENT_BAT_LOW_MSG      = "bat low";                // UDP message for battery is low 
  const int         CLIENT_BAT_LOW          = 2;                        // internal coding for battery is low
  const char*       CLIENT_BAT_CRITICAL_MSG = "bat critical";           // UDP message for battery is critical
  const int         CLIENT_BAT_CRITICAL     = 3;                        // internal coding for battery is critical



  #ifdef SERVER_ESP32
    //Server specific LEDs
    const uint8_t     SERVER_POWER_LED        = 3;                    // Power LED 
    const uint8_t     SERVER_WLAN_LED         = 21;                   // LED to inducate the current WLAN state
    const uint8_t     SERVER_TOUCH_OUT        = 1;                    // LED to indicate a touch of the 3D sense
    const uint8_t     SERVER_ERROR_OUT        = 2;                    // ERROR output that can hold the cnc controller, e.g.critical battery or no more alive msg from client
    const uint8_t     SERVER_SLEEP_IN         = 3;                    // Input pin for the CNC controller to indicate that the slave can go to sleep
    const uint8_t     SERVER_BAT_ALM_OUT      = 4;                    // server battery is low
    const uint8_t     SERVER_RGB_LED_OUT      = 7;                    // Server rgb led
  #else
    //Server specific LEDs
    const uint8_t     SERVER_POWER_LED        = 2;                    // (D4) Power LED 
    const uint8_t     SERVER_WLAN_LED         = 0;                    // (D3) LED to inducate the current WLAN state
    const uint8_t     SERVER_TOUCH_OUT        = 5;                    // (D1) LED to indicate a touch of the 3D sense
    const uint8_t     SERVER_ERROR_OUT        = 14;                   // (D5) ERROR output that can hold the cnc controller, e.g.critical battery or no more alive msg from client
    const uint8_t     SERVER_SLEEP_IN         = 12;                   // (D6) Input pin for the CNC controller to indicate that the slave can go to sleep
    const uint8_t     SERVER_BAT_ALM_OUT      = 4;                    // (D2) client battery is low
    const uint8_t     SERVER_RGB_LED_OUT      = 7;                    // Server rgb led

  #endif

  #ifdef CLIENT_ESP32
    //Client specific LEDs
    const uint8_t     CLIENT_POWER_LED        = 3;                    // Power LED 
    const uint8_t     CLIENT_WLAN_LED         = 21;                   // LED to inducate the current WLAN state
    const uint8_t     CLIENT_ERROR_OUT        = 1;                    // LED to indicate error
    const uint8_t     CLIENT_TOUCH_LED        = 4;                    // LED to indicate a touch of the 3D sensor
    const uint8_t     CLIENT_TOUCH_IN         = 5;                    // digital input pin to listen
    const uint8_t     CLIENT_SLEEP_OUT        = 6;                    // controls external sleep hardware
    const uint8_t     CLIENT_ANALOG_CHANNEL   = 2;                    // Analog In channel for reading the battery voltage
    const uint8_t     CLIENT_CHARGE_IN        = 7;                    // Battery Charging Input
    const uint8_t     CLIENT_RGB_LED_OUT      = 8;                    // client rgb led

  #else //ESP8266
    //Client specific LEDs
    const uint8_t     CLIENT_POWER_LED        = 2;                    // (D4) Power LED 
    const uint8_t     CLIENT_WLAN_LED         = 0;                    // (D3) LED to inducate the current WLAN state
    const uint8_t     CLIENT_ERROR_OUT        = 5;                    // (D1) LED to indicate error
    const uint8_t     CLIENT_TOUCH_LED        = 4;                    // (D2) LED to indicate a touch of the 3D sensor
    const uint8_t     CLIENT_TOUCH_IN         = 12;                   // (D6) digital input pin to listen
    const uint8_t     CLIENT_SLEEP_OUT        = 14;                   // (D5) controls external sleep hardware
    const uint8_t     CLIENT_CHARGE_IN        = 7;                    // Battery Charging Input
    const uint8_t     CLIENT_RGB_LED_OUT      = 7;                    // client rgb led
     //const uint8_t     CLIENT_PREVENT_RESET_OUT       = 14;                       // (D5) prevents reset by touch pin when client is alive

  #endif
}
#endif