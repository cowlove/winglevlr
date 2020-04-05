#include <HardwareSerial.h>
#include <SD.h>
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
#include "G90Parser.h"

#include "mavlink.h"
#include "Wire.h"
#include "jimlib.h"

#define BUTTON_PIN 17
#define LED_PIN 2

WiFiMulti wifi;
TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);

#define GDL90 1
GDL90Parser gdl90;
GDL90Parser::State state;

/* The default UART header for your MCU */ 
//int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
//int compid = 158;                ///< The component sending the message
//int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
uint8_t targetSysId = 1;
 
void mavlink_open();
void mavlink_send(const uint8_t *, int);
int mavlink_read(uint8_t *, int);

#define MAVLINK_PORT 14450

static void IRAM_ATTR resetModule() {
   //esp_restart();
}

class WatchDogTimer {
	int timeout;
	hw_timer_t *timer = NULL;
public: 
	WatchDogTimer(int ms = 10000) : timeout(ms) {}
	void begin(int ms = 0) {
		if (ms > 0) 
			timeout = ms;
		timer = timerBegin(0, 80, true);                  //timer 0, div 80
		timerAttachInterrupt(timer, &resetModule, true);  //attach callback
		timerAlarmWrite(timer, timeout, false); //set time in us
		timerAlarmEnable(timer);                          //enable interrupt
	}
	void feed() { timerWrite(timer, 0); }
};

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
WiFiUDP udpMAV;


#ifdef ARDUINO_HELTEC_WIFI_KIT_32
#define SCREEN
#define MAVLINK_END
#include <U8g2lib.h>
#include <U8x8lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

static IPAddress mavRemoteIp;
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
static uint8_t buf[BUFFER_LENGTH];
EggTimer screenTimer(100), blinkTimer(1000), udpDebugTimer(1000), mavTimer(300);

RotaryEncoder re(5,18,23);


DigitalButton button(BUTTON_PIN);
DigitalButton button2(0);
LongShortFilter butFilt(1500,600);
LongShortFilter butFilt2(1500,600);
void buttonISR() { 
	button.check();
	button2.check();
	butFilt.check(button.duration());
	butFilt2.check(button2.duration());
}



void screenMsg(const char *msg) { 
#ifdef SCREEN
	u8g2.begin();
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println(msg);
	u8g2.sendBuffer();
#endif
}	

WatchDogTimer wdt(10000);

void setup() {
	//wdt.begin();  // doesn't work yet
	re.begin([]{re.ISR();});
	attachInterrupt(digitalPinToInterrupt(button.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button2.pin), buttonISR, CHANGE);

	mavRemoteIp.fromString("192.168.43.166");
	pinMode(LED_PIN, OUTPUT);
	//pinMode(17, INPUT_PULLUP);
	pinMode(BUTTON_PIN, INPUT_PULLUP);

	screenMsg("Searching for WIFI...");
	//WiFi.mode(WIFI_STA);
	//WiFi.setSleep(false);
	//WiFi.begin("ChloeNet", "niftyprairie7");
	wifi.addAP("Ping-582B", "");
	wifi.addAP("ChloeNet", "niftyprairie7");

	uint64_t startms = millis();
	while (WiFi.status() != WL_CONNECTED) {
		wifi.run();
		delay(10);
		if (millis() - startms > 15000)
			break;
	}
	screenMsg("Setup Complete");

	ArduinoOTA.begin();
	udpSL30.begin(7891);
	udpG90.begin(4000);
	udpMAV.begin(MAVLINK_PORT);
	udpNMEA.begin(7892);
	mavlink_open();
	
	pinMode(22, INPUT);
	pinMode(19, OUTPUT);
	Serial.begin(57600, SERIAL_8N1, 22, 19);
	Serial.setTimeout(1);

	screenMsg("Setup Complete");
}



void mav_gps_msg(float lat, float lon, float crs, float speed, float alt) {
	
	uint64_t time_usec = 0;
	uint8_t gps_id = 12;
	uint16_t ignore_flags = 0; 
	uint32_t time_week_ms = 0; 
	uint16_t time_week = 0;
	uint8_t fix_type = 4;
	float hdop = 1.23; 
	float vdop = 2.34;

	float vn = cos(radians(crs)) * speed;
	float ve = sin(radians(crs)) * speed;
	float vd = 0;
	float speed_accuracy = 10.321; 
	float horiz_accuracy = 10.210; 
	float vert_accuracy = 10.321;
	uint8_t satellites_visible = 11;

	mavlink_message_t msg;
	mavlink_msg_gps_input_pack(1, 100, &msg, time_usec, 
	gps_id, ignore_flags, time_week_ms, time_week,  fix_type, lat * 10000000, lon * 10000000,  alt, hdop, vdop, vn, ve, vd,
	speed_accuracy,  horiz_accuracy, vert_accuracy, satellites_visible);
	
	uint8_t mavbuf[1028];
	int len = mavlink_msg_to_send_buffer(mavbuf, &msg);
	mavlink_send(mavbuf, len);
}	

void loop() {
	//wifi.run(1);
	//ArduinoOTA.handle();
	//wdt.feed();
	
	mavlink_message_t msg;
	uint16_t len;
	static int count = 0;
	static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
	static int buildNumber = 12;
	static int mavBytesIn = 0;
	static mavlink_message_t mavMsgIn;
	static mavlink_status_t mavStatusIn;
	static char lastParam[64];
	static int lastHdg;
	static int mavHeartbeats;
	static int desiredTrk = 180;
	static int apMode = 1;
	static float roll = 0;
	static int gpsUseGDL90 = 0;
	static int obs = 0;
	static int navDTK = 0;
	
	buttonISR();
	if (butFilt.newEvent()) {
		if (butFilt.wasLong == true) { // long press, hold current track
			re.value = lastHdg;
			apMode = 3;
		}
		if (butFilt.wasLong == false) { // short press, set new mode
			apMode = butFilt.wasCount;
		}
		mavTimer.alarmNow();
	}
	
	if (butFilt2.newEvent())
		gpsUseGDL90 = !gpsUseGDL90;
	
#ifdef SCREEN
	if (screenTimer.tick()) { 
		u8g2.clearBuffer();					// clear the internal memory
		u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
		u8g2.setCursor(0,10);				// set write position
		u8g2.printf("WIFI: "); u8g2.println(WiFi.localIP());
		u8g2.setCursor(0,20);
		u8g2.printf(" DTK: %03d   HDG: %03d", (int)desiredTrk, (int)lastHdg);
		u8g2.setCursor(0,30);				
		u8g2.printf("NAVT: %03d   OBS: %03d", navDTK, obs);
		u8g2.setCursor(0,40);					
		u8g2.printf("KNOB: %03d  MODE: %d %d", re.value, apMode, gpsUseGDL90);
		u8g2.setCursor(0,50);					
		u8g2.printf(" UDP: %03d   SER: %03d", udpBytes % 1000, serBytes % 1000);
		u8g2.setCursor(0,60);
		u8g2.printf(" MAV: %03d  ROLL:%+05.1f",  mavHeartbeats % 1000, roll);
		u8g2.sendBuffer();
	}
#endif

	char udpHost[64];
	IPAddress ip = WiFi.localIP();
	sprintf(udpHost, "%d.%d.%d.255", ip[0], ip[1], ip[2]);
	
	if (blinkTimer.tick()) 
		count++;
	digitalWrite(LED_PIN, (count & 0x1) ^ (gpsFixes & 0x1) ^ (serBytes & 0x1));
	
	//delay(10);
	yield();
	
	if (Serial.available()) {
		int l = Serial.readBytes(buf, sizeof(buf));
		serBytes += l + random(0,2);
		udpMAV.beginPacket(mavRemoteIp ,MAVLINK_PORT); // todo - could send to the last ip we received from instead of bcast addr 
		udpMAV.write((uint8_t *)buf, l);
		udpMAV.endPacket();
		
		for (int n = 0; n < l; n++) {
			if (mavlink_parse_char(0, buf[n], &mavMsgIn, &mavStatusIn)) {
				switch(mavMsgIn.msgid) { 
				case MAVLINK_MSG_ID_ATTITUDE:
					mavlink_attitude_t attitude;
					mavlink_msg_attitude_decode(&mavMsgIn, &attitude);
					roll = attitude.roll * 180 / 3.1415;
					break;

				case MAVLINK_MSG_ID_VFR_HUD:
					mavlink_vfr_hud_t vfr_hud;
					mavlink_msg_vfr_hud_decode(&mavMsgIn, &vfr_hud);
					lastHdg = vfr_hud.heading;
					// Debug - make stupid fake GPS signal that follows magnetic heading
					//mav_gps_msg(47.2, 121.1, lastHdg, 50.0, 666);
					break;
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavHeartbeats++;
					break;
				case MAVLINK_MSG_ID_PARAM_VALUE:
					mavlink_param_value_t packet;
					mavlink_msg_param_value_decode(&mavMsgIn, &packet);
					if (strcmp("CRUISE_HEADING", packet.param_id) == 0) { 
						//lastHdg = packet.param_value;
					}
					break;
				}
			}
		}
	}
	
	
	int avail = udpMAV.parsePacket();
	while (avail > 0) { 
		mavRemoteIp = udpMAV.remoteIP();	
		int n = udpMAV.read(buf, min(avail,(int)sizeof(buf)));
		mavBytesIn += n + random(0,2);
		if (n > 0) {
			Serial.write(buf, n);
			avail -= n;
		}
	}

	if (mavTimer.tick()) {
		/*
		int mode_flags = MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED |   MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 
		  |  MAV_MODE_FLAG_SAFETY_ARMED; 
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, 
			0, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
		*/
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
		
		mavlink_msg_param_request_read_pack(1, 200, &msg, 0, 0, "CRUISE_HEADING", -1);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_request_data_stream_pack(1, 200, &msg , 0, 0, MAVLINK_MSG_ID_VFR_HUD , 1 , 1 );
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_request_data_stream_pack(1, 200, &msg , 0, 0, MAVLINK_MSG_ID_ATTITUDE , 1 , 1 );
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
 
		if (apMode == 1) { 
			int new_mode = 0; /* 0 == MANUAL, 2 == STABILIZE, 7 = CRUISE */
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 2) {
			int new_mode = 2;
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 3 || apMode == 4 || apMode == 5) {
			int new_mode = 7;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

			if (apMode == 3) 
				desiredTrk = (int)re.value;
			if (apMode == 4) 
				desiredTrk = (int)(obs + 15.5 + 360) % 360;
			if (apMode == 5) 
				desiredTrk = navDTK;
	
	
			mavlink_msg_param_set_pack(1, 200, &msg, 0, 0, "CRUISE_HEADING", desiredTrk, MAV_VAR_FLOAT);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len);
		}
	}

	int recsize = 0;
	do {
		recsize = 0;
		int avail = udpG90.parsePacket();
		while(avail > 0) { 
			recsize = udpG90.read(buf, min(avail,(int)sizeof(buf)));
			//Serial.printf("read %d/%d\n", recsize, avail); Serial.flush();
			avail -= recsize;
			udpBytes += recsize + random(0,2);
			for (int i = 0; i < recsize; i++) {  
				yield();
				gdl90.add(buf[i]);
				GDL90Parser::State s = gdl90.getState();
				if (gpsUseGDL90 && s.valid && s.updated) { 
					gpsFixes++;
					mav_gps_msg(s.lat, s.lon, s.track, s.hvel * 0.51444, s.palt * 25 - 1000);
				}
			}
		}
	}
	while(recsize > 0);

	avail = udpSL30.parsePacket();
	while(avail > 0) { 
		static char line[128];
		static int index;
		
		int n = udpSL30.read(buf, sizeof(buf));
		avail -= n;
		udpBytes += n + random(0,2);
		for (int i = 0; i < n; i++) {
			if (index >= sizeof(line))
				index = 0;
			if (buf[i] != '\r')
				line[index++] = buf[i];
			if (buf[i] == '\n' || buf[i] == '\r') {
				// 0123456789012
				// $PMRRV34nnnXX
				line[11] = 0; 
				sscanf(line, "$PMRRV34%d", &obs);
				index = 0;
			}

			gps.encode(buf[i]);
			if (!gpsUseGDL90 && gps.location.isUpdated()) {
				gpsFixes++;
				mav_gps_msg(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.course.deg(), gps.speed.mps());
			}
			if (desiredHeading.isUpdated()) { 
				if (apMode == 4) 
					apMode = 5;
				navDTK = 0.01 * gps.parseDecimal(desiredHeading.value());
			}
		}
	}
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
