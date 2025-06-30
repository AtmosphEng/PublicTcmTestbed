// #include "generated/PublicTcmSimple_menu.h"
/*
// DESCRIPTION ********************************************************************************************************
//
// PublicSimple_main.cpp
// IDE: PIO.
// TARGET(s)	: VARIOUS
//
// WIFI : Available on ESP32 targets.
//
// UI : TCMENU BY DAVE CHERRY et.al.
//
//	SUMMARY:
//		Simple tcMenu example available to run on  several target boards
//		e.g. esp32-wroom, esp32-s3, mega2560. Mainly Lilygo esp32 boards. Refer to the following repos:
//		https://github.com/AtmosphEng/PublicTcmSimple.git
//		https://github.com/AtmosphEng/boards.git
//
//	FEATURES:
//		tcMenu local user input via rotary encoder, or 2-buttons.
//		tcMenu local user output on LCD.
//		tcMenu remote UI via esp32 wifi
//		tcMenu remote UI via 3-wire SerialX port
//		ARDUINOOTA
//
//
// KNOWN ISSUES:
//
//  TARGET(s): lilygo-t7-s3
//		T7-S3 ESP32-S3 USB CDC mode for Serial : -D ARDUINO_USB_MODE=1 : -D ARDUINO_USB_CDC_ON_BOOT=1 :
//		PIO Serial Terminal does not show any Serial.print but RealTerm is OK
//		(apart from T7-S3 h/w reset causing Windows temp removal of the corrsponding COMx port).
//
//	TARGET(s): lilygo-t-display-s3
//		Fix for blank TFT_eSPI display - check the following 2x custom PIO project files
//   		1) PROJDIR/.pio/libdeps/lilygo-t-display-s3/TFT_eSPI # for custom: User_Setup_Select.h
//   		2) PROJDIR/.pio/libdeps/lilygo-t-display-s3/TFT_eSPI/User_Setups for custom: Setup206_LilyGo_T_Display_S3.h
//
//		T-Display-S3 Compile Warning :
//		Compiling .pio\build\lilygo-t-display-s3\src\PublicTcmSimple_menu.cpp.o
//		In file included from .pio/libdeps/lilygo-t-display-s3/TFT_eSPI/TFT_eSPI.h:96,
//	             from src/tcMenuTfteSpi.h:17,
//               from src/PublicTcmSimple_menu.h:16,
//               from src/src_menu.h:4,
//               from src/PublicTcmSimple_main.cpp:1:
//		.pio/libdeps/lilygo-t-display-s3/TFT_eSPI/Processors/TFT_eSPI_ESP32_S3.h:110:3: warning:
//		#warning >>>>------>> DMA is not supported in parallel mode [-Wcpp]
//		#warning >>>>------>> DMA is not supported in parallel mode
//
//		Compile Warning : lilygo-t-display-s3 AND lilygo-t-embed-s3
//		In file included from .pio/libdeps/lilygo-t-display-s3/TFT_eSPI/TFT_eSPI.cpp:16:
//		.pio/libdeps/lilygo-t-display-s3/TFT_eSPI/TFT_eSPI.h:970:8: warning: #warning >>>>------>>
//		TOUCH_CS pin not defined, TFT_eSPI touch functions will not be available! [-Wcpp]
//		#warning >>>>------>> TOUCH_CS pin not defined, TFT_eSPI touch functions will not be available!
//
//	TARGET(s): lilygo-t-embed-s3
//		The t-embed serial port on GROVE conn. is NOT currently working w. embedCONTROL. However, (USB) Serial is OK.
//
//	TARGET(s): esp32dev_t_internet_com
//		tcMenu embedCONTROL: all ports working (Serial, Serial1, Serial2, Wifi) except wired Ethernet.
//
//	TARGET(s): ALL esp32-s3
//		TCMENU RC w. embedCONTROL not working on Serial with USB CDC mode e.g. 1x USB cable only for prog and debug.
//		platformio.ini: build_flags: -D ARDUINO_USB_MODE=1
//		platformio.ini: build_flags: -D ARDUINO_USB_CDC_ON_BOOT=1
//		WORKAROUND : 2024-04-03 - 2024-10-10
//		1) pin_config-t-display-esp32-s3.h : add : #define	MYSERIAL_BEGIN Serial.begin(BAUD_SERIAL, SERIAL_8N1, 43, 44);
//		2) PublicTcmSimple_menu.cpp:25 : add cast : SerialTagValueTransport serialTransport( (HardwareSerial*) &Serial);
//		3) Use Realterm (optional) to check TCMENU RC HMI heartbeat messages on Serial at 115200 8-n-1 baud settings.
//		4) Close Realterm (optional), reset target, run embedCONTROL and select Serial port.
//
// DESCRIPTION ********************************************************************************************************
*/

#include "src_menu.h" // for tcMenu
#include <Arduino.h>
#include <SPI.h>
#include <SwitchInput.h> // tcMenu-switches

#ifndef ARDUINO_ARCH_AVR
#include <WiFi.h>
#include <esp_wifi.h> // for esp_read_mac() etc.
#endif

#ifdef SERIALBT_CLASSIC
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

#ifdef INIDEF_ETHERMEGA2560
#include "Ethernet.h"
#endif

#ifdef INIDEF_LILYGO_T_INTERNET_COM
#include <ETH.h>
#endif

#ifndef ARDUINO_ARCH_AVR
#include "../../projects/Credentials/Credentials.h"
#endif

#include <WiFi.h>

// enum class WifiMode {off, sta, ap, multi};
// enum class ConnType {server, client};
// WifiMode myWifiMode;
// ConnType myConnType;

// #ifdef WIFI_BUILD
#include "../../libraries/CommsHelper/src/commsHelper.h"
CommsHelper myCommsHelper;

#include "../../libraries/CommsHelper/src/configNetwork.h" // now located at CommsHelper

// #endif

#if (0)
#define MYSERIALX Serial
#else
#define MYSERIALX Serial1
// #define MYSERIALX Serial2
#endif
#include "../../libraries/MyDebug/src/myDebugPrint.h"

#define BAUD_SERIAL			 (115200)
#define BAUD_SERIAL0		 (115200)
#define BAUD_SERIAL1		 (115200)
#define BAUD_SERIAL2		 (115200)

#define DEF_LED_DELAY		 500
#define DEF_SERIAL_DELAY 100
// #define NETWORK_AP_ONBOARD_LINEAR_ENC_APP_RESET_COUNT_FINE

#ifdef ARDUINO_ARCH_AVR
#include <Servo.h>
#endif

#ifdef ARDUINO_ARCH_ESP32
#include <ESP32Servo.h>
#endif

#ifdef INIDEF_UNO
#include "pin_config-avr-uno.h" // SPI data LCD interface
#endif

#ifdef INIDEF_MEGA2560
#include "pin_config-avr-mega2560.h" // SPI data LCD interface
#endif

#ifdef INIDEF_ETHERMEGA2560
#include "pin_config-avr-ethermega2560.h" // SPI data LCD interface
#endif

#ifdef INIDEF_KEYESTUDIO_KS0413
#include "pin_config-keyestudio-ESP32.h" // SPI data LCD interface
#endif

#ifdef INIDEF_CYD
#include "pin_config-cyd.h" // SPI data LCD interface
#endif

#ifdef INIDEF_LILYGO_T_INTERNET_COM
#include "pin_config-t-internet-com-ESP32.h"

#include <Adafruit_NeoPixel.h>
#define LED_COUNT			 1
#define LED_BRIGHTNESS 1 // about 1/5 brightness (max = 255)
Adafruit_NeoPixel pixels(LED_COUNT, PIN_LED_NEOPIXEL);
uint32_t colors[] = {0x000000, 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF}; // Off, Red, Green, Blue, white
#define NUM_COLORS (sizeof colors / sizeof colors[0])
#endif

#ifdef INIDEF_LILYGO_T_DISPLAY_S3
#include "pin_config-t-display-esp32-s3.h" // 8-bit parallel data LCD interface
#endif

#ifdef INIDEF_LILYGO_T_EMBED_S3
#include "pin_config-t-embed-esp32-s3.h" // SPI data LCD interface

#include "APA102.h"
APA102<PIN_APA102_DI, PIN_APA102_CLK> ledStrip; // t-embed builtin RGB_LED
#endif

#ifdef INIDEF_LILYGO_T7_S3
#include "pin_config-t7-s3-esp32-s3.h"
#endif

#ifdef CONF_EXTRA_ENCODER2
HardwareRotaryEncoder *RotaryEncoder2;
#define DEF_ENCODER2					 menuTcmBaseTCW // ASSIGN ENCODER2 TO ANALOGMENUITEM.
#define DEF_TCM_INDEX_ENCODER2 1							// Note - 1st encoder is tcmenu indexed as 0.
#endif

#ifdef CONF_EXTRA_ENCODER3
HardwareRotaryEncoder *RotaryEncoder3;
#define DEF_ENCODER3					 menuTcmLinearEncFine // ASSIGN ENCODER3 TO ANALOGMENUITEM.
#define DEF_TCM_INDEX_ENCODER3 2										// Note - 1st encoder is tcmenu indexed as 0.
#endif

Servo myServoValve; // create servo object to control a servo

#ifdef WIFI_BUILD
// #define BUF_SIZE 80
#define BUF_SIZE 1024

// STATIC(s)
static uint8_t STREAM_2_TO_STREAM_1_Buf[BUF_SIZE];

static int lastCount = 0;
static int currentCount = 0;
static int lastCountMax = 0;
static int lastCountMin = 0;
static int speedCountsPerSec = 0;
static int clientLoops = 0;

static bool currentDirectionTCW = true;
static bool lastDirectionTCW = true;

// WiFiClient tcpClient;
#endif

#ifdef INIDEF_ARDUINOOTA
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#endif

#if defined(ARDUINO_ARCH_AVR) && !defined(INIDEF_UNO)
#define PIN_DUMMY_GROUND_D20 20 // For Mega wiring convenience of Serial1 cable.
#define PIN_DUMMY_GROUND_D21 21 // For Mega wiring convenience of Serial2 cable.

// encoder2 - for direct control of BaseTCW
#define PIN_ENCODER2_CLK_D22 22
#define PIN_ENCODER2_DAT_D24 24
// encoder2 pushbutton on D26 is not used here.
#define PIN_DUMMY_5V0_D28		 28 // For Mega wiring convenience of rotary encoder.
#define PIN_DUMMY_GROUND_D30 30 // For Mega wiring convenience of rotary encoder.

// encoder1 - for direct control of BaseCCW
#define PIN_ENCODER1_CLK_D38 38
#define PIN_ENCODER1_DAT_D40 40
// encoder1 pushbutton on D42 is not used here.
#define PIN_DUMMY_5V0_D44		 44 // For Mega wiring convenience of rotary encoder.
#define PIN_DUMMY_GROUND_D46 46 // For Mega wiring convenience of rotary encoder.

// ANALOG PINS ************************************************************************************
// A0 used for LCD 16x2 shield for polling / reading all operational buttons via resistor network.
// A5 used for LCD 16x2 shield (NB Custom Freetronics pcb mod to relocate D9 to A5)

// encoder0 - slot 0 - for menu navigation
#define PIN_DUMMY_GROUND_A11 A11 // For Mega wiring convenience of rotary encoder.
#define PIN_DUMMY_5V0_A12		 A12 // For Mega wiring convenience of rotary encoder.
// #define PIN_ENCODER0_PBSW_A13 A13 // Assigned in tcmenu Designer Code Generator.
// #define PIN_ENCODER0_CLK_A14	A14 // Assigned in tcmenu Designer Code Generator.
// #define PIN_ENCODER0_DAT_A15	A15 // Assigned in tcmenu Designer Code Generator.
#endif // ARDUINO_ARCH_AVR

#define DEF_BTSERIAL_DELAY_MS				10000

#define DEF_TCM_TASK_SCHEDULE_MS		1000
#define DEF_DISPLAY_LINE_CHAR_COUNT 16
#define DEF_NUMBER_BASE							10

int myCount1 = 0;
int myCount2 = 0;

// *** FUNCTION(S) **********************************************
void refreshMenu(void) {

	// menuTcmCount1.setSendRemoteNeededAll();
	// menuTcmCount2.setSendRemoteNeededAll();
	// menuTcmBaseTCW.setSendRemoteNeededAll();
	// menuTcmBaseCCW.setSendRemoteNeededAll();
	// menuTcmDebugLED.setSendRemoteNeededAll();
}

#ifdef INIDEF_LILYGO_T_EMBED_S3
/* Converts a color from HSV to RGB.
 * h is hue, as a number between 0 and 360.
 * s is the saturation, as a number between 0 and 255.
 * v is the value, as a number between 0 and 255. */
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
	uint8_t f = (h % 60) * 255 / 60;
	uint8_t p = (255 - s) * (uint16_t)v / 255;
	uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
	uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
	uint8_t r = 0, g = 0, b = 0;
	switch ((h / 60) % 6) {
	case 0:
		r = v;
		g = t;
		b = p;
		break;
	case 1:
		r = q;
		g = v;
		b = p;
		break;
	case 2:
		r = p;
		g = v;
		b = t;
		break;
	case 3:
		r = p;
		g = q;
		b = v;
		break;
	case 4:
		r = t;
		g = p;
		b = v;
		break;
	case 5:
		r = v;
		g = p;
		b = q;
		break;
	}
	return rgb_color(r, g, b);
}
#endif

void debugLED(bool state) {

#if (0) // from github lilygo t-embed examples/led/led.ino
	const uint8_t ledSort[7] = {2, 1, 0, 6, 5, 4, 3};
	// Set the number of LEDs to control.
	const uint16_t ledCount = 7;

	for (int i = 0; i < 1000; i++) {
		// Create a buffer for holding the colors (3 bytes per color).
		rgb_color colors[ledCount];
		// Set the brightness to use (the maximum is 31).
		uint8_t brightness = 1;
		static uint64_t time;
		time++;
		for (uint16_t i = 0; i < ledCount; i++) {
			colors[i] = hsvToRgb((uint32_t)time * 359 / 256, 255, 255);
		}
		ledStrip.write(colors, ledCount, brightness);
		delay(10);
	}
#endif

#ifdef INIDEF_LILYGO_T_EMBED_S3 // APA102 RGBLEDs
	const uint16_t ledCount = 1;
	// Create a buffer for holding the colors (3 bytes per color).
	rgb_color colors[ledCount];
	if (state) {
		colors[0] = {64, 64, 64};
	} else {
		colors[0] = {0, 0, 0};
	}
	// Set the brightness to use (the maximum is 31).
	uint8_t brightness = 1;
	ledStrip.write(colors, ledCount, brightness);

#else

#ifdef INIDEF_LILYGO_T_INTERNET_COM // NeoPixel RGBLED

	if (state) {
		pixels.setPixelColor(0, colors[3]); // blue (on).
	} else {
		pixels.setPixelColor(0, colors[0]); // black (off).
	}
	pixels.show();

#else
	digitalWrite(PIN_DEBUG_LED, state); // Menu selection default toggles this boolean value.
#endif

#endif
}

#ifdef INIDEF_LILYGO_T_INTERNET_COM
#define TCP_SERVER_PORT 3333 // For tcmenu embedControl.

// WiFiServer myServer(TCP_SERVER_PORT);
static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event) {
	switch (event) {
	case ARDUINO_EVENT_ETH_START:
		MYSERIALX.println("ETH Started");
		// set eth hostname here
		ETH.setHostname("esp32-ethernet");
		break;
	case ARDUINO_EVENT_ETH_CONNECTED:
		MYSERIALX.println("ETH Connected");
		break;
	case ARDUINO_EVENT_ETH_GOT_IP:
		MYSERIALX.print("ETH MAC: ");
		MYSERIALX.print(ETH.macAddress());
		MYSERIALX.print(", IPv4: ");
		MYSERIALX.print(ETH.localIP());
		if (ETH.fullDuplex()) {
			MYSERIALX.print(", FULL_DUPLEX");
		}
		MYSERIALX.print(", ");
		MYSERIALX.print(ETH.linkSpeed());
		MYSERIALX.println("Mbps");
		eth_connected = true;
		break;
	case ARDUINO_EVENT_ETH_DISCONNECTED:
		MYSERIALX.println("ETH Disconnected");
		eth_connected = false;
		break;
	case ARDUINO_EVENT_ETH_STOP:
		MYSERIALX.println("ETH Stopped");
		eth_connected = false;
		break;
	default:
		break;
	}
}

#endif

#ifdef DEF_TCM_OPERATIONAL // AAAFIXME here this means if SERVER with time source?
// #ifdef MY_TCMENU_SWITCH_ONBOARD_LINEAR_ENCODER

void CALLBACK_FUNCTION onActivationTcmSwitch(uint8_t pin, bool heldDown) {
	// tcMenu-switches if heldDown false, then action is once only on active to non-active edge trasition ?
#if (0)
	if (heldDown == false) { // i.e. 'pressed' i.e. one activation regardless of pressing time. works as logic toggle ?
		// if (heldDown == true) { // i.e. 'pressed' i.e. one activation regardless of pressing time. works as logic toggle
		// ?
		menuTcmLinearEncLaps.setCurrentValue(menuTcmLinearEncLaps.getCurrentValue() + 1,
																				 false); // NOT HardwareRotaryEncoder

#ifdef NETWORK_AP_ONBOARD_LINEAR_ENC_APP_RESET_COUNT_FINE
		// if(1){
		// if((menuTcmLinearEncLaps.getCurrentValue() % 2)){ // Use modulo two to only reset lap position if lap number is
		// ODD.
		if (heldDown == false) { // i.e. 'pressed' i.e. one activation regardless of pressing time. works as logic toggle ?
			// menuTcmLastCountCCW.setCurrentValue(menuTcmLinearEncFine.getCurrentValue(), false); // disp last MAX position
			// count

			menuTcmLinearEncFine.setCurrentValue(-menuTcmLinearEncFine.getOffset(),
																					 false);													// reset count of incr linear encoder
			RotaryEncoder3->setCurrentReading(-menuTcmLinearEncFine.getOffset()); // reset count of incr linear encoder to '0'

		} else {
			// menuTcmLastCountTCW.setCurrentValue(menuTcmLinearEncFine.getCurrentValue()); // display last MINimum position
			// count
		}
#endif
		// delay(1000);
		// vTaskDelay(1000/portTICK_PERIOD_MS); // TaskDelay of x millisec. AAAMAGIC

	} // if (heldDown == true)
#endif
}
#endif

//
void setup() {
#if (0)
	int incomingByte = 0; // For dumping unwanted initial ESP serial boot message.
	Serial0.begin(BAUD_SERIAL0);
	delay(100); // Need time here?
	if (Serial0.available() > 0) {
		// read and DUMP the incoming byte:
		incomingByte = Serial.read();
	}
#endif

	//	Serial.println("Serial begin ok. line 1");
	//	Serial.println("Serial begin ok. line 2");

#ifdef INIDEF_LILYGO_T_INTERNET_COM
	// Neopixel WS2812 RGB LED init.
	pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
	pixels.show();	// Turn OFF all pixels ASAP
	pixels.setBrightness(LED_BRIGHTNESS);
#else
	pinMode(PIN_DEBUG_LED, OUTPUT); // Common LED name for all targets / controllers. Assign I/O mode to pin.
#endif

#if (1)
	debugLED(true); // simple setup routine to provide a double flash of debugLED type installed on target board.
	delay(DEF_LED_DELAY);
	debugLED(false);
	delay(DEF_LED_DELAY);
	debugLED(true);
	delay(DEF_LED_DELAY);
	debugLED(false);
	delay(DEF_LED_DELAY);
#endif

	Serial.begin(BAUD_SERIAL1);
	delay(DEF_SERIAL_DELAY); // Need time here?

#ifdef MYSERIAL0_BEGIN
													 // MYSERIAL0_BEGIN;
	Serial0.begin(BAUD_SERIAL1);
	delay(DEF_SERIAL_DELAY); // Need time here?
#endif

#ifdef MYSERIAL1_BEGIN
	MYSERIAL1_BEGIN;
	delay(DEF_SERIAL_DELAY); // Need time here?
#endif

#ifdef MYSERIAL2_BEGIN
	MYSERIAL2_BEGIN;
	delay(DEF_SERIAL_DELAY); // Need time here?
#if (1)
	Serial2.println("Hello from Serial2");
	Serial2.println("Hello from Serial2");
	Serial2.println("Hello from Serial2");
#endif
#endif

#ifdef MYSERIAL3_BEGIN
	MYSERIAL3_BEGIN;
	delay(DEF_SERIAL_DELAY); // Need time here?
#endif

#if (0)
	Serial.println("hello from Serial"); // ESP32-WROOM UART0 is named Serial, and is Tx common with USB serial.
#endif

#ifdef SERIALBT_CLASSIC
	int incomingByteBT = 0; // for incoming BT serial data

#ifdef DEF_BT_MASTER
	// DEF_SERIALBT.setPin(pin);
	// DEF_SERIALBT.begin("MYESP32MASTER", true);
	// DEF_SERIALBT.setPin(pin);
	// MYSERIAL.println("The device started in master mode, make sure remote BT
	// device is on!");

	// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30
	// secs max) as it needs to resolve name to address first, but it allows to
	// connect to different devices with the same name. Set CoreDebugLevel to Info
	// to view devices bluetooth address and device names connected =
	// DEF_SERIALBT.connect(name); connected = DEF_SERIALBT.connect(address);

	if (connected) {
		// MYSERIAL.println("Connected Succesfully!");
	} else {
		// while(!DEF_SERIALBT.connected(10000)) {
		// if(!DEF_SERIALBT.connected(10000)) {
		if (!DEF_SERIALBT.connected(5000)) {
			// MYSERIAL.println("Failed to connect. Make sure remote device is
			// available and in range, then restart app.");
		}
	}
	// disconnect() may take upto 10 secs max
	if (DEF_SERIALBT.disconnect()) {
		// MYSERIAL.println("Disconnected Succesfully!");
	}
	// this would reconnect to the name(will use address, if resolved) or address
	// used with connect(name/address).
	if (DEF_SERIALBT.connect()) {
		// MYSERIAL.println("Re-connected Succesfully!");
	}
#endif

#ifdef SERIALBT_MASTER
	// const char *pin = "1234"; //<- standard pin would be provided by default
	bool btConnected;

	SerialBT.begin("MYESP32MASTER", true); // Bluetooth MASTER device name
	// SerialBT.setPin(pin);
	Serial.println("The BT device started in master mode, make sure remote BT device was already on first!");

	btConnected = SerialBT.connect(address); // address[] is the MAC of the required BT_SLAVE to connect / pair with.
	if (btConnected) {
		Serial.println("setup(): BT Master Connected Succesfully");
	} else {
		while (!SerialBT.connected(DEF_BTSERIAL_DELAY_MS)) {
			Serial.println(
					"BT Master failed to connect. Make sure remote device is available and in range, then restart app.");
		}
	}

	// disconnect() may take upto 10 secs max
	if (SerialBT.disconnect()) {
		Serial.println("BT Master Disconnected Succesfully!");
	}
	// this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
	// if ( SerialBT.connect() ) {
	if (SerialBT.connect(address)) {
		Serial.println("BT Master re-connected Succesfully!");
	};

#endif

#ifdef SERIALBT_SLAVE
	SerialBT.begin("MYESP32SLAVE", false); // Bluetooth SLAVE device name
	Serial.println("This BT slave MYESP32SLAVE device started, now you can pair it with a bluetooth master!");
	delay(DEF_BTSERIAL_DELAY_MS);
#endif

#if (0) // For Bluetooth binding / pairing with Windows - TEMP
	while (1) {
		delay(DEF_BTSERIAL_DELAY_MS);
	}; // INFINITE LOOP
#endif

#endif // SERIALBT_CLASSIC

#ifdef INIDEF_LILYGO_T_EMBED_S3
	pinMode(PIN_POWER_ON, OUTPUT);
	digitalWrite(PIN_POWER_ON, HIGH);

	pinMode(PIN_LCD_BL, OUTPUT);
	digitalWrite(PIN_LCD_BL, HIGH);
#endif

	myCommsHelper.setupWifi();

#ifdef INIDEF_LILYGO_T_DISPLAY_S3
	pinMode(PIN_POWER_ON, OUTPUT);
	digitalWrite(PIN_POWER_ON, HIGH);

	pinMode(PIN_LCD_BL, OUTPUT);
	digitalWrite(PIN_LCD_BL, HIGH);
#endif

	setupMenu(); // for tcMenu ********************************************************************************

#ifdef DEF_TCM_OPERATIONAL
	// #ifdef MY_TCMENU_SWITCH_ONBOARD_LINEAR_ENCODER
	switches.init(asIoRef(internalDigitalDevice()), SWITCHES_NO_POLLING, true); // use intr?(IOtype, read via, pull-up)
	switches.addSwitch(PIN_ENC2_B, onActivationTcmSwitch, NO_REPEAT, false); // tcMenu-switches tcMenu lib ENC2 NOT used.
#endif

#ifdef WIFI_BUILD
	char ascii_id[DEF_DISPLAY_LINE_CHAR_COUNT]; // allocate null-terminated string storage for answer.
	itoa(TARGET_NUM, ascii_id, DEF_NUMBER_BASE);
	menuTcmTargetNum.setTextValue(ascii_id, false); // false means no callback.
#endif

// #ifdef INIDEF_LILYGO_T_EMBED_S3
#ifdef TCM_UI_LCD
	renderer.turnOffResetLogic(); // Turn off tcmenu cursor reset interval to prevent reset to root position.
#endif

#if defined(ARDUINO_ARCH_AVR) && !defined(INIDEF_UNO)
																// digitalWrite(IO_PIN_53, OUTPUT); // Mega Slave Select (SS)

	digitalWrite(PIN_DUMMY_GROUND_D20, LOW); // For wiring convenience on Mega for Serial1 cable with adjacent ground
	pinMode(PIN_DUMMY_GROUND_D20, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_D20, LOW); // Belt and braces.

	digitalWrite(PIN_DUMMY_GROUND_D21, LOW); // For wiring convenience on Mega for Serial2 cable with adjacent ground
	pinMode(PIN_DUMMY_GROUND_D21, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_D21, LOW); // Belt and braces.

	// first encoder - enc1 - encoder0 - slot 0 - for menu navigation
	digitalWrite(PIN_DUMMY_GROUND_A11, LOW); // For wiring convenience on Mega for rotary enc power 0V on 5-pin header
	pinMode(PIN_DUMMY_GROUND_A11, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_A11, LOW); // Belt and braces.

	digitalWrite(PIN_DUMMY_5V0_A12, HIGH); // For wiring convenience on Mega for rotary enc power 5V on 5-pin header
	pinMode(PIN_DUMMY_5V0_A12, OUTPUT);
	digitalWrite(PIN_DUMMY_5V0_A12, HIGH); // Belt and braces.

#if (0)
	// for mega2560 hardware. ease of use via consecutive pin assignments on the connector(s).

	// first encoder - enc1 - encoder0 - slot 0 - for menu navigation
	digitalWrite(PIN_DUMMY_5V0_A12, HIGH); // For wiring convenience on Mega for rotary encoder VCC 5V0 on 5-pin header
	pinMode(PIN_DUMMY_5V0_A12, OUTPUT);
	digitalWrite(PIN_DUMMY_5V0_A12, HIGH); // Belt and braces.

	digitalWrite(PIN_DUMMY_GROUND_A11, LOW); // For wiring convenience on Mega for rotary encoder ground on 5-pin header
	pinMode(PIN_DUMMY_GROUND_A11, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_A11, LOW); // Belt and braces.

	// second encoder - enc2 - encoder1
	digitalWrite(PIN_DUMMY_5V0_D44, HIGH); // For wiring convenience on Mega for rotary encoder VCC 5V0 on 5-pin header
	pinMode(PIN_DUMMY_5V0_D44, OUTPUT);
	digitalWrite(PIN_DUMMY_5V0_D44, HIGH); // Belt and braces.

	digitalWrite(PIN_DUMMY_GROUND_D46, LOW); // For wiring convenience on Mega for rotary encoder ground on 5-pin header
	pinMode(PIN_DUMMY_GROUND_D46, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_D46, LOW); // Belt and braces.

	// third encoder - enc3 - encoder2
	digitalWrite(PIN_DUMMY_5V0_D28, HIGH); // For wiring convenience on Mega for rotary encoder VCC 5V0 on 5-pin header
	pinMode(PIN_DUMMY_5V0_D28, OUTPUT);
	digitalWrite(PIN_DUMMY_5V0_D28, HIGH); // Belt and braces.

	digitalWrite(PIN_DUMMY_GROUND_D30, LOW); // For wiring convenience on Mega for rotary encoder ground on 5-pin header
	pinMode(PIN_DUMMY_GROUND_D30, OUTPUT);
	digitalWrite(PIN_DUMMY_GROUND_D30, LOW); // Belt and braces.
#endif

#ifdef UI_ROTARY_ENC
	// enc1 is index 0. enc2 is index 1, etc.
	rotaryEncoder1 = new HardwareRotaryEncoder(PIN_ENCODER2_DAT_D40, PIN_ENCODER2_CLK_D38,
																						 [](int encoderValue) { menuTcmBaseCCW.setCurrentValue(encoderValue); });

	rotaryEncoder1->changePrecision(menuTcmBaseCCW.getMaximumValue(), menuTcmBaseCCW.getCurrentValue()); // max, curr
	switches.setEncoder(1, rotaryEncoder1);

	rotaryEncoder2 = new HardwareRotaryEncoder(PIN_ENCODER3_DAT_D24, PIN_ENCODER3_CLK_D22,
																						 [](int encoderValue) { menuTcmBaseTCW.setCurrentValue(encoderValue); });

	rotaryEncoder2->changePrecision(menuTcmBaseTCW.getMaximumValue(),
																	menuTcmBaseTCW.getCurrentValue()); // max value, current value
	switches.setEncoder(2, rotaryEncoder2);
#endif

	myCount1 = 1;
	menuTcmCount1.setCurrentValue(myCount1);

#endif // defined(ARDUINO_ARCH_AVR) && !defined(INIDEF_UNO)

#ifdef DEF_TCM_OPERATIONAL // SERVER - do TCM display updates here ONLY - more runtime efficient
	taskManager.scheduleFixedRate(DEF_PROCESS_UPDATE_MSEC, [] { //  SCHEDULE CODE DURING LOOPING
		currentCount = menuTcmLinearEncFine.getCurrentValue() + menuTcmLinearEncFine.getOffset(); // Update count
		MYDEBUGPRINT(lastCountMax);
		MYDEBUGPRINT(",  ");
		MYDEBUGPRINT(lastCountMin);
		MYDEBUGPRINT(",  ");
		MYDEBUGPRINT(currentCount);
		MYDEBUGPRINT(",  ");
		MYDEBUGPRINT(lastDirectionTCW);
		MYDEBUGPRINT(",  ");
		MYDEBUGPRINTLN(currentDirectionTCW);

		debugLED(menuTcmDebugLED.getBoolean());
		RotaryEncoder3->setCurrentReading(menuTcmLinearEncFine.getCurrentValue());		// Keep val synced w. diff UIs
		menuTcmLinearEncFine.setCurrentValue(menuTcmLinearEncFine.getCurrentValue()); // is this line redundant?
		menuTcmTimeSec.setCurrentValue(menuTcmTimeSec.getCurrentValue() + (DEF_PROCESS_UPDATE_MSEC / 100)); // conv sec/10

		speedCountsPerSec = (currentCount - lastCount) * (1000 / DEF_PROCESS_UPDATE_MSEC); // conv to PerSec.
		menuTcmSpeed.setCurrentValue(speedCountsPerSec - menuTcmSpeed.getOffset()); // tcm offset for neg integer display.

		if ((lastCount < currentCount)) { // does lastCountMax need updating?
			lastCountMax = currentCount;		// update max value found.
			currentDirectionTCW = true;
			MYDEBUGPRINTLN("currentDirectionTCW is true");
		} else {
			if ((lastCount > currentCount)) {
				lastCountMin = currentCount; // AAAMAGIC tcm offset, update min value found.
				currentDirectionTCW = false;
				MYDEBUGPRINTLN("currentDirectionTCW is false");
			}
		}
		lastCount = currentCount; // update

#if (1)
		if (lastDirectionTCW != currentDirectionTCW) { // update UI display(s) once per (half)-lap i.e. TCW or CCW
			lastDirectionTCW = currentDirectionTCW;			 // update
			menuTcmLinearEncLaps.setCurrentValue(menuTcmLinearEncLaps.getCurrentValue() +
																					 1); // NOT HardwareRotaryEncoder ENC2

			if (currentDirectionTCW) {
				menuTcmLastCountCCW.setCurrentValue(
						lastCountMin - menuTcmLastCountCCW.getOffset()); // update OPPOSITE direction count to UI display

				lastCountMin = 0; // thack is taped BLACK at this end, so FORCE reset OPPOSITE direction count for next lap.
				menuTcmLinearEncFine.setCurrentValue(-menuTcmLinearEncFine.getOffset(),
																						 false); // reset count of incr linear encoder
				RotaryEncoder3->setCurrentReading(
						-menuTcmLinearEncFine.getOffset()); // reset count of incr linear encoder to '0'

			} else { // current direction is CCW
				menuTcmLastCountTCW.setCurrentValue(
						lastCountMax - menuTcmLastCountCCW.getOffset()); // update OPPOSITE direction count to UI display
				// lastCountMax = 0; //  reset OPPOSITE direction count for next lap.
			}
		}
#endif

	}); // taskManager

#else // CLIENT

	taskManager.scheduleFixedRate(DEF_PROCESS_UPDATE_MSEC, [] { //  SCHEDULE CODE DURING LOOPING
		// Serial.print("Min:-10, Max:70, ");
		Serial.print(
				"-10, 60, "); // always plot min and max constants to prevent serial plotter from auto scaling in y-axis
		Serial.print((menuTcmLinearEncFine.getCurrentValue() + menuTcmLinearEncFine.getOffset()));

		// Serial.print(",");
		// Serial.print(clientLoops % 70); // for serial print and plot testing. Linear ramp indicates no missing data.

		Serial.println();
		clientLoops++;

	}); // taskManager

#endif // DEF_TCM_OPERATIONAL

#if (0)
#define DEF_TCM_SERIAL_XOVER_SYNC_ONCE
	// #define DEF_TCM_SERIAL_XOVER_SYNC_REP

#ifdef DEF_TCM_SERIAL_XOVER_SYNC_ONCE
	refreshMenu();
#endif
#endif

#ifdef CONF_EXTRA_ENCODER2
	pinMode(PIN_ENC2_A, INPUT_PULLUP); // for RotaryEncoder2
	pinMode(PIN_ENC2_B, INPUT_PULLUP);
	// pinMode(PIN_ENC2_OK, INPUT_PULLUP); // Not required.

	RotaryEncoder2 = new HardwareRotaryEncoder(PIN_ENC2_B, PIN_ENC2_A, [](int valueEncoder2) {
		// no current action on the pushbutton switch press, change the menu using encoder rotation only.
		DEF_ENCODER2.setCurrentValue(valueEncoder2);
	});
	RotaryEncoder2->changePrecision(DEF_ENCODER2.getMaximumValue(), DEF_ENCODER2.getCurrentValue());
	switches.setEncoder(DEF_TCM_INDEX_ENCODER2, RotaryEncoder2); // Do not relocate this line.
#endif

#ifdef CONF_EXTRA_ENCODER3
	pinMode(PIN_ENC3_A, INPUT_PULLUP); // for RotaryEncoder2
	pinMode(PIN_ENC3_B, INPUT_PULLUP);
	// pinMode(PIN_ENC3_OK, INPUT_PULLUP); // Not required.

	RotaryEncoder3 = new HardwareRotaryEncoder(
			PIN_ENC3_B, PIN_ENC3_A,
			[](int valueEncoder3) {
				// no current action on the pushbutton switch press, change the menu using encoder rotation only.
				// AAATEMP DEF_ENCODER3.setCurrentValue(valueEncoder3);
				DEF_ENCODER3.setCurrentValue(valueEncoder3, true); // do NOT update the LCD.
			},
			HWACCEL_NONE); // last param assigns rotary encoder acceleration mode.
	RotaryEncoder3->changePrecision(DEF_ENCODER3.getMaximumValue(), DEF_ENCODER3.getCurrentValue());
	switches.setEncoder(DEF_TCM_INDEX_ENCODER3, RotaryEncoder3); // Do not relocate this line.
#endif

#ifdef INIDEF_ETHERMEGA2560
	// You can use Ethernet.init(pin) to configure the CS pin
	Ethernet.init(PIN_ETHERNET_CS); // Most Arduino shields
	// Ethernet.init(5);   // MKR ETH shield
	// Ethernet.init(0);   // Teensy 2.0
	// Ethernet.init(20);  // Teensy++ 2.0
	// Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
	// Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

	// initialize the ethernet device
	// Ethernet.begin(myMAC, myETHWiredStaticIP, myDNS, myGateway, mySubnet);
	Ethernet.begin(myMAC, myETHWiredStaticIP);

	// Check for Ethernet hardware present
	if (Ethernet.hardwareStatus() == EthernetNoHardware) {
		MYSERIALX.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");

		while (true) {
			delay(1); // do nothing, no point running without Ethernet hardware
		}
	}

	if (Ethernet.linkStatus() == LinkOFF) {
		MYSERIALX.println("Ethernet cable is not connected.");
	}
	// start listening for clients

	MYSERIALX.println(Ethernet.localIP());

	MYSERIALX.println(Ethernet.localIP());

#endif // INIDEF_ETHERMEGA2560

#ifdef INIDEF_LILYGO_T_INTERNET_COM
	WiFi.onEvent(WiFiEvent);

	ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

	if (ETH.config(myETHWiredStaticIP, myGateway, mySubnet, myDNS, myDNS) == false) {
		MYSERIALX.println("Configuration FAILED.");
	} else {
		MYSERIALX.println("Configuration passed.");
	}
#endif

	// myServer.begin();

	// MYSERIALX.print("Chat myServer address:");

	myServoValve.attach(PIN_SERVO_AIR_VALVE);

	myCommsHelper.setupConnection();

#ifdef INIDEF_ARDUINOOTA
	// Port defaults to 3232
	// ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	// ArduinoOTA.setHostname("myesp32");

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA
			.onStart([]() {
				String type;
				if (ArduinoOTA.getCommand() == U_FLASH)
					type = "sketch";
				else // U_SPIFFS
					type = "filesystem";

				// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
				MYSERIALX.println("Start updating " + type);
			})
			.onEnd([]() { MYSERIALX.println("\nEnd"); })
			.onProgress([](unsigned int progress, unsigned int total) {
				MYSERIALX.printf("Progress: %u%%\r", (progress / (total / 100)));
			})
			.onError([](ota_error_t error) {
				MYSERIALX.printf("Error[%u]: ", error);
				if (error == OTA_AUTH_ERROR)
					MYSERIALX.println("Auth Failed");
				else if (error == OTA_BEGIN_ERROR)
					MYSERIALX.println("Begin Failed");
				else if (error == OTA_CONNECT_ERROR)
					MYSERIALX.println("Connect Failed");
				else if (error == OTA_RECEIVE_ERROR)
					MYSERIALX.println("Receive Failed");
				else if (error == OTA_END_ERROR)
					MYSERIALX.println("End Failed");
			});

	ArduinoOTA.begin();
#endif

} // setup

//
void loop() {
	taskManager.runLoop();

	//	delay(10); // time allowance for background processing.

#if (0)
	if (TCP_SERVER_CONNECTIONV) {

		tcpServer.begin();

		WiFiClient tcpClient = tcpServer.available(); // listen for incoming clients

		if (tcpClient) {									// if you get a tcpClient,
			MYDEBUGPRINTLN("New Client.");	// print a message out the serial port
			String currentLine = "";				// make a String to hold incoming data from the tcpClient
			while (tcpClient.connected()) { // loop while the tcpClient's connected
				if (tcpClient.available()) {	// if there's bytes to read from the tcpClient,
					char c = tcpClient.read();	// read a byte, then
					MYDEBUGWRITE(c);						// print it out the serial monitor
					if (c == '\n') {						// if the byte is a newline character

						// if the current line is blank, you got two newline characters in a row.
						// that's the end of the tcpClient HTTP request, so send a response:
						if (currentLine.length() == 0) {
							// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
							// and a content-type so the tcpClient knows what's coming, then a blank line:
							tcpClient.println("HTTP/1.1 200 OK");
							tcpClient.println("Content-type:text/html");
							tcpClient.println();

							// the content of the HTTP response follows the header:
							tcpClient.print("Click <a href=\"/H\">here</a> to turn the LED on pin 19 on.<br>");
							tcpClient.print("Click <a href=\"/L\">here</a> to turn the LED on pin 19 off.<br>");

							// The HTTP response ends with another blank line:
							tcpClient.println();
							// break out of the while loop:
							break;
						} else { // if you got a newline, then clear currentLine:
							currentLine = "";
						}
					} else if (c != '\r') { // if you got anything else but a carriage return character,
						currentLine += c;			// add it to the end of the currentLine
					}

					// Check to see if the tcpClient request was "GET /H" or "GET /L":
					if (currentLine.endsWith("GET /H")) {
						digitalWrite(19, HIGH); // GET /H turns the LED on
					}
					if (currentLine.endsWith("GET /L")) {
						digitalWrite(19, LOW); // GET /L turns the LED off
					}
				}
			}
			// close the connection:
			tcpClient.stop();
			MYDEBUGPRINTLN("Client Disconnected.");
		}
	}
#endif

#ifdef INIDEF_ARDUINOOTA
	ArduinoOTA.handle();
#endif

} // loop
//

// CALLBACK FUNCTION(s) *************************************************************************************

void CALLBACK_FUNCTION onChangeTcmDebugLED(int id) { /* AAATEMP debugLED(menuTcmDebugLED.getBoolean()); */ }

void CALLBACK_FUNCTION onChangeTcmLinearEncLaps(int id) {
#ifdef DEF_TCM_OPERATIONAL
#ifdef CONF_EXTRA_ENCODER2
	RotaryEncoder2->setCurrentReading(menuTcmLinearEncLaps.getCurrentValue()); // Keep val synced w. diff UIs
#endif
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmLinearEncFine(int id) {
#ifdef DEF_TCM_OPERATIONAL
#ifdef CONF_EXTRA_ENCODER3
	// AAATEMP RotaryEncoder3->setCurrentReading(menuTcmLinearEncFine.getCurrentValue()); // Keep val synced w. diff UIs
#endif
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmCount1(int id) {
#ifdef DEF_TCM_OPERATIONAL
	// myCount1++;
	// myServoValve.write(menuTcmCount1.getIntValueIncludingOffset());
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmCount2(int id) {
#ifdef DEF_TCM_OPERATIONAL
	// myCount2++;
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmBaseCCW(int id) {
#ifdef DEF_TCM_OPERATIONAL
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmBaseTCW(int id) {
#ifdef DEF_TCM_OPERATIONAL
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmTimeSec(int id) {
#ifdef DEF_TCM_OPERATIONAL
#endif // ifndef DEF_TCM_OPERATIONAL
}

void CALLBACK_FUNCTION onChangeTcmRefreshMenu(int id) { /* refreshMenu(); */ }

void (*resetFunc)(void) = 0; // Arduino Forum alto777

void CALLBACK_FUNCTION onChangeTcmRestart(int id) {

	// #ifndef INIDEF_ETHERMEGA2560
	//	ESP.restart();
	// #else
	// resetFunc(); if used, uncomment this line.
	// #endif
}

#ifdef INIDEF_LILYGO_T_INTERNET_COM

void CALLBACK_FUNCTION onChangeTcmNeoPixelLedOff(int id) {
	// pixels.setPixelColor(0,0); // Equivalent to the following line.
	pixels.setPixelColor(0, colors[0]); // black (off).
	pixels.show();
}

void CALLBACK_FUNCTION onChangeTcmNeoPixelLedRed(int id) {
	pixels.setPixelColor(0, colors[1]); // red.
	pixels.show();
}

void CALLBACK_FUNCTION onChangeTcmNeoPixelLedGreen(int id) {
	pixels.setPixelColor(0, colors[2]); // green.
	pixels.show();
}

void CALLBACK_FUNCTION onChangeTcmNeoPixelLedBlue(int id) {
	pixels.setPixelColor(0, colors[3]); // blue.
	pixels.show();
}

void CALLBACK_FUNCTION onChangeTcmNeoPixelLedWhite(int id) {
	pixels.setPixelColor(0, colors[4]); // white.
	pixels.show();
}

#endif

void CALLBACK_FUNCTION onChangeTcmNPixLedWhite(int id) {
	// TODO - your menu change code
}

void CALLBACK_FUNCTION onChangeTcmNPixLedRed(int id) {
	// TODO - your menu change code
}

void CALLBACK_FUNCTION onChangeTcmNPixLedOff(int id) {
	// TODO - your menu change code
}

void CALLBACK_FUNCTION onChangeTcmNPixLedGreen(int id) {
	// TODO - your menu change code
}

void CALLBACK_FUNCTION onChangeTcmNPixLedBlue(int id) {
	// TODO - your menu change code
}

#if (0)
// This callback needs to be implemented by you, see the below docs:
//  1. List Docs - https://tcmenu.github.io/documentation/arduino-libraries/tc-menu/menu-item-types/list-menu-item/
//  2. ScrollChoice Docs -
//  https://tcmenu.github.io/documentation/arduino-libraries/tc-menu/menu-item-types/scrollchoice-menu-item/
int CALLBACK_FUNCTION fnNewRuntimeListRtCall(RuntimeMenuItem *item, uint8_t row, RenderFnMode mode, char *buffer,
																						 int bufferSize) {
	switch (mode) {
	default:
		return defaultRtListCallback(item, row, mode, buffer, bufferSize);
	}
}
#endif

//
// END_OF_FILE
//
