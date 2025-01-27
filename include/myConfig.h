//
// myConfig.h
//
#ifndef MY_CONFIG_H
#define MY_CONFIG_H

#define DEF_TCM_OPERATIONAL // for pin_config file(s)

// EXTRA ENCODER(S)
//#if(0)
//#define CONF_EXTRA_ENC2_TCW // TCWRotaryEncoder2 - NOT defined in TcmDesigner
//#define CONF_EXTRA_ENC3_CCW // CCWRotaryEncoder3 - NOT defined in TcmDesigner
//#endif

// NETWORK COMMS ***************************************************************************************

// BLUETOOTH
const bool SERIALBT_TRANSPARENT_BRIDGE_FOR_SERIALZ = false;


//#define DEF_BYTE_BY_BYTE // else its buffered

// ENABLE ONLY ONE OF THE FOLLOWING BUILD TYPES
//
#define TCP_SERVER_BUILD
//#define TCP_CLIENT_BUILD

// ENABLE ONE OR NONE OF THE FOLLOWING NETWORK SERVERS
//
#define NETWORK_NETCOMM
//#define NETWORK_HOTSPOT

#define NONWIFI_BUILD
#define DEF_WIFI_STA
//#define DEF_WIFI_AP



//const unsigned int WIFI_RADIO_CHANNEL = 11; // For WIFI-AP or ESP-NOW: choose a fixed Wifi channel between 1 and 13.
//const unsigned int WIFI_RADIO_CHANNEL = 13; // For WIFI-AP or ESP-NOW: choose a fixed Wifi channel between 1 and 13.

#define NET_ADDR_BYTE_1			192
#define NET_ADDR_BYTE_2		 	168
//#define NET_ADDR_BYTE_1			10
//#define NET_ADDR_BYTE_2		 	42

#ifdef NETWORK_NETCOMM
#define NET_ADDR_SUBNET_BYTE 20
char ssid[] = netcomm24_wifi_ssid;
char pw[] = netcomm24_wifi_pw;
#endif

#ifdef NETWORK_HOTSPOT
#define NET_ADDR_SUBNET_BYTE 0 // to suit Ubuntu wifi hotspot.
//#define NET_ADDR_SUBNET_BYTE 33 // for linear encoder testing.
//#define NET_ADDR_SUBNET_BYTE 66 // for tcmenu serial xover embedded remote control.
//#define NET_ADDR_SUBNET_BYTE 137 // to suit Windows wifi hotspot.
char ssid[] = hotspot2_wifi_ssid;
char pw[] = hotspot2_wifi_pw;
#endif

#ifdef DEF_WIFI_AP
#define TARGET_NUM_SERVER		1 // only if WIFI-AP
#else
#define TARGET_NUM_SERVER		17 // was 13
#endif

//#define TARGET_NUM_AP 			TARGET_NUM_SERVER
#define TARGET_NUM_CLIENT		14

#ifdef NONWIFI_BUILD
//const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIALV = false;
//const bool TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIALV = false;
#define TCP_INIT
//#define DEF_WIFI_STA

#if(1) // AAAFIXME
#define INIDEF_WIFI_IP2 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#define TARGET_NUM TARGET_NUM_SERVER
#else
#define INIDEF_WIFI_IP2 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_CLIENT
#define TARGET_NUM TARGET_NUM_CLIENT
#endif
#endif

#ifdef TCP_SERVER_BUILD
const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIALV = true;
//const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIALV = false;
const bool TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIALV = false;
#define TCP_INIT
#define TARGET_NUM TARGET_NUM_SERVER
#define INIDEF_WIFI_IP2 NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER


#endif // TCP_SERVER_BUILD

#ifdef TCP_CLIENT_BUILD
const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIALV = false;
const bool TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIALV = true;
//#define TCP_INIT
#define TARGET_NUM TARGET_NUM_CLIENT
#define INIDEF_WIFI_IP2 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_CLIENT
#define INIDEF_WIFI_IP_SERVER NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#endif

#ifdef TCP_INIT
#define INIDEF_WIFI_GWAY NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,1 	// default gateway
IPAddress netmask(255, 255, 255, 0); 				// default
//#define DEF_SERVER_PORT 80
#define DEF_SERVER_PORT 3333 // for tcMenu embedControl tcpip
WiFiServer tcpServer(DEF_SERVER_PORT);
#endif


#ifdef INIDEF_MEGA2560_ETHERNET // wired Ethernet
#include <Ethernet.h>
#undef ETH_CLK_MODE // From Lilygo github example.
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN       5
#endif


#if(0) // its SERIALBT BLUETOOTH
#define SERIALBT_CLASSIC
#if(1)
#define SERIALBT_MASTER
#else
#define SERIALBT_SLAVE
#endif
#endif


#endif // ifndef for include only once.

// END_OF_FILE
