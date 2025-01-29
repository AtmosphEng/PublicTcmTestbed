//
// myConfig.h
//
#ifndef MY_CONFIG_H
#define MY_CONFIG_H

#define DEF_TCM_OPERATIONAL // for pin_config file(s)

// EXTRA ENCODER(S)
// ENABLE THE FOLLOWING AS REQUIRED:
//#define CONF_EXTRA_ENC2_TCW // TCWRotaryEncoder2 - NOT defined in TcmDesigner
//#define CONF_EXTRA_ENC3_CCW // CCWRotaryEncoder3 - NOT defined in TcmDesigner

// NETWORK COMMS ***************************************************************************************

// BLUETOOTH
const bool SERIALBT_TRANSPARENT_BRIDGE_FOR_SERIALZ = false;

#define WIFI_BUILD

#define STREAM_1 myCom0com // NOTE: myCom0com needs alternative_ prefixes to available, read, and write methods.
#define STREAM_2 tcpClient
#define STREAM_1_VIRT
//#define STREAM_2_VIRT
#define STREAM_1_STREAM_2_BRIDGE

//#define DEF_BYTE_BY_BYTE // else its buffered
//#define DEF_SERVER_PORT 80
#define DEF_SERVER_PORT 3333 // for tcMenu embedControl tcpip

// ENABLE THE FOLLOWING AS REQUIRED: **************************************************************
#if(0)
#define TCP_SERVER_BUILD
#define DEF_WIFI_AP
//#define DEF_WIFI_STA
#define TARGET_NUM_SERVER		1 //
//#define TARGET_NUM_SERVER		17 // was 13. only if NOT using DEF_WIFI_AP
WiFiServer tcpServer(DEF_SERVER_PORT);

#else
#define TCP_CLIENT_BUILD
#define TARGET_NUM_CLIENT		14
#define DEF_WIFI_STA
#endif

#ifdef WIFI_BUILD

// ENABLE ONE OR NONE OF THE FOLLOWING: ***********************************************************
#define NETWORK_AP1
//#define NETWORK_NETCOMM
//#define NETWORK_HOTSPOT2

#ifdef NETWORK_HOTSPOT2
#define NET_ADDR_BYTE_1			10 // Linux Ubuntu hotspot.
#define NET_ADDR_BYTE_2		 	42 // Linux Ubuntu hotspot.
#else
#define NET_ADDR_BYTE_1			192
#define NET_ADDR_BYTE_2		 	168
#endif

#endif // WIFI_BUILD

#ifdef NETWORK_NETCOMM
#define NET_ADDR_SUBNET_BYTE 20
char ssid[] = netcomm24_wifi_ssid;
char pw[] = netcomm24_wifi_pw;
#endif

#ifdef NETWORK_AP1 // embedded target hosts its own access point.
#define NET_ADDR_SUBNET_BYTE 66 // for tcmenu serial xover embedded remote control.
#define TARGET_NUM_SERVER		1
char ssid[] = ap1_wifi_ssid;
char pw[] = ap1_wifi_pw;
#endif

#ifdef NETWORK_HOTSPOT2
#define NET_ADDR_SUBNET_BYTE 0 // to suit Ubuntu wifi hotspot.
//#define NET_ADDR_SUBNET_BYTE 33 // for linear encoder testing.
//#define NET_ADDR_SUBNET_BYTE 66 // for tcmenu serial xover embedded remote control.
//#define NET_ADDR_SUBNET_BYTE 137 // to suit Windows wifi hotspot.
char ssid[] = hotspot2_wifi_ssid;
char pw[] = hotspot2_wifi_pw;
#endif

#ifdef TCP_SERVER_BUILD
#define DEF_WIFI_IP 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#define TARGET_NUM TARGET_NUM_SERVER
#define TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIAL
#endif

#ifdef TCP_CLIENT_BUILD
#define DEF_WIFI_IP 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_CLIENT
#define TARGET_NUM TARGET_NUM_CLIENT
#define TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIAL
#endif

#define DEF_WIFI_GWAY NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,1  // default gateway. macro concatenation.
IPAddress netmask(255, 255, 255, 0); 				// default

//const unsigned int WIFI_RADIO_CHANNEL = 11; // For WIFI-AP or ESP-NOW: choose a fixed Wifi channel between 1 and 13.
//const unsigned int WIFI_RADIO_CHANNEL = 13; // For WIFI-AP or ESP-NOW: choose a fixed Wifi channel between 1 and 13.


#ifdef INIDEF_MEGA2560_ETHERNET // wired Ethernet
#include <Ethernet.h>
#undef ETH_CLK_MODE // From Lilygo github example.
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN       5
#endif


#if(0) // SERIALBT BLUETOOTH

#define SERIALBT_CLASSIC
#if(1)
#define SERIALBT_MASTER
#else
#define SERIALBT_SLAVE
#endif

#endif // SERIALBT BLUETOOTH


#endif // ifndef for include only once.

// END_OF_FILE
