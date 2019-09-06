#include <HardwareSerial.h>
#include "SPI.h"
#include "Update.h"
#include "WebServer.h"
#include "DNSServer.h"
#include "FS.h"
#include "ESPmDNS.h"
#include "ArduinoOTA.h"
#include "WiFiManager.h"
#include "WiFiUdp.h"
#include "WiFiMulti.h"
#include "mavlink.h"
#include "TinyGPS++.h"

#include "mavlink.h"
#include "Wire.h"
#define LED_PIN 2

WiFiMulti wifi;
TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);

/* The default UART header for your MCU */ 
int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
int compid = 158;                ///< The component sending the message
int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
uint8_t targetSysId = 1;
 
HardwareSerial serial2(1);

void mavlink_open();
void mavlink_send(const uint8_t *, int);
int mavlink_read(uint8_t *, int);

WiFiUDP udp;
const char *udpHost = "192.168.4.100";
//const char *udpHost = "192.168.43.42";
int udpPort = 7890;

#ifdef ARDUINO_HELTEC_WIFI_KIT_32
#define SCREEN
#define MAVLINK_END
#include <U8g2lib.h>
#include <U8x8lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

class EggTimer {
	uint64_t last;
	int interval; 
public:
	EggTimer(int ms) : interval(ms), last(0) {}
	bool tick() { 
		uint64_t now = millis();
		if (now - last > interval) { 
			last = now;
			return true;
		} 
		return false;
	}
};

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(21, INPUT);
#ifdef SCREEN
	u8g2.begin();
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println("Searching for WiFi");
	u8g2.sendBuffer();
#endif
	
#ifdef MAVLINK_END
	//wifi.setSTAStaticIPConfig(IPAddress(udpHost), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
#endif

	WiFi.mode(WIFI_STA);
	wifi.addAP("Ping-582B", "");
	wifi.addAP("ChloeNet", "niftyprairie7");
	
	digitalWrite(LED_PIN, 1);
	while (WiFi.status() != WL_CONNECTED) {
		wifi.run();
	}

	ArduinoOTA.begin();
	udp.begin(udpPort);
	mavlink_open();
	Serial.begin(9600);
	Serial.setTimeout(1);

	serial2.begin(9600, SERIAL_8N1, 21, 17);
	serial2.setTimeout(1);

}




EggTimer blinkTimer(1000);

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
static uint8_t buf[BUFFER_LENGTH];

void loop() {
//	wifi.run(1);
	ArduinoOTA.handle();

	mavlink_message_t msg;
	uint16_t len;
	static int count = 0;
	static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;

#ifdef SCREEN
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println(WiFi.localIP());
	u8g2.setCursor(0,20);
	u8g2.printf("TRK: %03d   CRS: %03d", 52, 180);
	u8g2.setCursor(0,30);				
	u8g2.printf("UDP: %04d  SER: %04d", udpBytes % 1000, serBytes % 1000);
	u8g2.setCursor(0,40);					
	u8g2.printf("FIX: %04d  APU: %04d", gpsFixes % 1000, apUpdates % 1000);
	u8g2.sendBuffer();
#endif


	

	if (blinkTimer.tick()) 
		count++;
	digitalWrite(LED_PIN, (count & 0x1) ^ (gpsFixes & 0x1) ^ (serBytes & 0x1));
	
	delay(1);
	yield();
	
	if (Serial.available()) {
		int l = Serial.readBytes(buf, sizeof(buf));
		serBytes += l;
		udp.beginPacket(udpHost,udpPort);
		udp.write((uint8_t *)buf, l);
		udp.endPacket();
	}

	// TODO Serial1 have it's own separate gps parser 
	if (serial2.available()) {
		int l = serial2.readBytes(buf, sizeof(buf));
		serBytes += l;
		udp.beginPacket(udpHost,udpPort);
		udp.write((uint8_t *)buf, l);
		udp.endPacket();
	}

	if (count % 50 == 0) { 
/*		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, 
			MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
		
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		// for debugging, set mode so we can see from MAVproxy that ESP32 is connected 
		int new_mode = 7;
		mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len); 
		apUpdates++;
*/
	}
	
	
	int recsize = 0;
	do {
		recsize = 0;
		int avail = udp.parsePacket();
		while(avail > 0) { 
			//Serial.printf("parse %d\n", avail);	Serial.flush();
			recsize = udp.read(buf, min(avail,(int)sizeof(buf)));
			//Serial.printf("read %d/%d\n", recsize, avail); Serial.flush();
			avail -= recsize;
			for (int i = 0; i < recsize; i++) {  
				yield();
				udpBytes++;
				gps.encode(buf[i]);
				if (gps.location.isUpdated()) {
					gpsFixes++;
					//printf("Lat lon: %f %f\n", gps.location.lat(), gps.location.lng());
					uint64_t time_usec = gps.time.value();
					uint8_t gps_id = 1;
					uint16_t ignore_flags = 0; 
					uint32_t time_week_ms = 0; 
					uint16_t time_week = 0;
					uint8_t fix_type = 4;
					int32_t lat = gps.location.lat() * 10000000;
				
					int32_t lon = gps.location.lng() * 10000000; 
					float alt = gps.altitude.meters();;
					float hdop = gps.hdop.value(); 
					float vdop = 0;

					float crs = gps.course.deg();
					float v = gps.speed.mps();
					float vn = cos(radians(crs)) * v;
					float ve = sin(radians(crs)) * v;
					float vd = 0;
					float speed_accuracy = 0; 
					float horiz_accuracy = 0; 
					float vert_accuracy = 0;
					uint8_t satellites_visible = 10;

					mavlink_msg_gps_input_pack(1, 100, &msg, time_usec, 
					gps_id, ignore_flags, time_week_ms, time_week,  fix_type, lat, lon,  alt, hdop, vdop, vn, ve, vd,
					speed_accuracy,  horiz_accuracy, vert_accuracy, satellites_visible);
					uint8_t mavbuf[1028];
					len = mavlink_msg_to_send_buffer(mavbuf, &msg);
					mavlink_send(mavbuf, len);
				}
				if (desiredHeading.isUpdated()) { 
					apUpdates++;
					int new_mode = 7;
					mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
					len = mavlink_msg_to_send_buffer(buf, &msg);
					mavlink_send(buf, len); 

					float h = 0.01 * gps.parseDecimal(desiredHeading.value());
					mavlink_msg_param_set_pack(1, 100, &msg, 0, 0, "CRUISE_HEADING", h, MAV_VAR_FLOAT);
					uint8_t mavbuf[1028];
					len = mavlink_msg_to_send_buffer(mavbuf, &msg);
					mavlink_send(mavbuf, len);
				}
			}
		}			
	} while(recsize > 0);
}

void mavlink_open() {
	//Serial.begin(9600);
}
void mavlink_send(const uint8_t *buf, int len) { 
	Serial.write(buf, len);
}
int mavlink_read(uint8_t *buf, int len) { 
	return 0;
}
