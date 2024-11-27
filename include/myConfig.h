//
// myConfig.h
//
#ifndef MY_CONFIG_H
#define MY_CONFIG_H


// EXTRA ENCODER(S)
#define CONF_EXTRA_ENC2_TCW // TCWRotaryEncoder2 - NOT defined in TcmDesigner
#define CONF_EXTRA_ENC3_CCW // CCWRotaryEncoder3 - NOT defined in TcmDesigner

const bool SERIALBT_TRANSPARENT_BRIDGE_FOR_SERIAL2 = false;


// NETWORK COMMS ***************************************************************************************

#define DEF_BYTE_BY_BYTE // else its buffered

#if(0)
#define TCP_SERVER_BUILD
#else
#define TCP_CLIENT_BUILD
#endif
#define NETWORK_NETCOMM			// else its esp32AP

#define TARGET_NUM_SERVER		13
#define TARGET_NUM_AP 			TARGET_NUM_SERVER
#define TARGET_NUM_CLIENT		14


#ifdef TCP_SERVER_BUILD
const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIAL2 = true;
const bool TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIAL2 = false;
#define TCP_INIT
#define TARGET_NUM TARGET_NUM_SERVER
#define INIDEF_WIFI_IP2 NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#define INIDEF_WIFI_STA
#endif

#ifdef TCP_CLIENT_BUILD
const bool TCP_SERVER_TRANSPARENT_BRIDGE_FOR_SERIAL2 = false;
const bool TCP_CLIENT_TRANSPARENT_BRIDGE_FOR_SERIAL2 = true;
#define TCP_INIT
#define TARGET_NUM TARGET_NUM_CLIENT
#define INIDEF_WIFI_STA
#undef INIDEF_WIFI_AP // belt and braces
#define INIDEF_WIFI_IP2 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_CLIENT
#define INIDEF_WIFI_IP_SERVER NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#endif

#ifdef NETWORK_NETCOMM
#define NET_ADDR_SUBNET_BYTE 20
char ssid[] = netcomm24_wifi_ssid;
char pw[] = netcomm24_wifi_pw;
#else
#define NET_ADDR_SUBNET_BYTE 66
char ssid[] = esp32AP_wifi_ssid;
char pw[] = esp32AP_wifi_pw;
#endif

#ifdef TCP_INIT

#define NET_ADDR_BYTE_1			192
#define NET_ADDR_BYTE_2		 	168

#define INIDEF_WIFI_GWAY NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,1 	// default
IPAddress netmask(255, 255, 255, 0); 				// default

#ifndef NETWORK_NETCOMM
#define INIDEF_WIFI_AP
#undef INIDEF_WIFI_STA // belt and braces
#define INIDEF_WIFI_IP2 NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_AP
#endif

#define DEF_SERVER_PORT 80
WiFiServer tcpServer(DEF_SERVER_PORT);

#endif

#if(0) // its SERIALBT
#define SERIALBT_CLASSIC
#if(1)
#define SERIALBT_MASTER
#else
#define SERIALBT_SLAVE
#endif
#endif


#ifdef INIDEF_MEGA2560_ETHERNET // wired Ethernet
#include <Ethernet.h>
#undef ETH_CLK_MODE // From Lilygo github example.
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN       5
#endif


#endif // ifndef for include only once.

// END_OF_FILE
