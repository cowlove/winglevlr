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
#include "G90Parser.h"
#include <MPU9250_asukiaaa.h>
#include <SparkFunMPU9250-DMP.h>
#include <RunningLeastSquares.h>



#include "mavlink.h"
#include "Wire.h"
#include <mySD.h>
#include "jimlib.h"


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

/*

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
 */
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
#define U8G2
#define BUTTON_PIN 17
#define LED_PIN 2
#include <U8g2lib.h>
#include <U8x8lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
RotaryEncoder re(5,18,23);
DigitalButton button(17);
DigitalButton button2(0);
#define SCREEN u8g2
#endif


#define TTGO
#ifdef TTGO
#include <MPU9250_asukiaaa.h>
#include <Adafruit_GFX.h>               // Core graphics library
#include <Adafruit_ST7735.h>            // Hardware-specific library
#include <Kalman.h> 					// Source: https://github.com/TKJElectronics/KalmanFilter
#define LED_PIN 22
RotaryEncoder re(5,18,23);
DigitalButton button(34);
DigitalButton button2(35);
DigitalButton button3(39);
#define TFT_CS 16
#define TFT_RST 9  
#define TFT_DC 17
#define TFT_SCLK 5   
#define TFT_MOSI 23  
#define ST7735
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#define SCREEN tft
#endif // TTGO 

static IPAddress mavRemoteIp;
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
static uint8_t buf[BUFFER_LENGTH];
EggTimer screenTimer(100), blinkTimer(1000), udpDebugTimer(1000), mavTimer(300);

LongShortFilter butFilt(1500,600);
LongShortFilter butFilt2(1500,600);
LongShortFilter butFilt3(1500,600);
void buttonISR() { 
	button.check();
	button2.check();
	button3.check();
	butFilt.check(button.duration());
	butFilt2.check(button2.duration());
	butFilt3.check(button3.duration());
}


void screenMsg(const char *msg) {
#ifdef U8G2
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println(msg);
	u8g2.sendBuffer();
#endif
#ifdef ST7735
	tft.setCursor(0,10);				// set write position
	tft.println(msg);
#endif
}
	
static int screenY = 0;
void screenClear() { 
	screenY = 10;
#ifdef U8G2
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
#endif
#ifdef ST7735
	tft.fillScreen(ST7735_BLACK);                            // CLEAR
#endif
}

void screenInit(const char *msg) { 
#ifdef U8G2
	u8g2.begin();
	screenMsg(msg);
#endif
#ifdef ST7735
	pinMode(27,INPUT);//Backlight:27
	digitalWrite(27,HIGH);//New version added to backlight control
	tft.initR(INITR_18GREENTAB);                             // 1.44 v2.1
	tft.fillScreen(ST7735_BLACK);                            // CLEAR
	tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);           // GREEN
	tft.setRotation(1);                                      // 
	screenY = 10;
	screenMsg("OK");
#endif
}	

void screenNextLine() {
	screenY += 10;
#ifdef U8G2
	u8g2.setCursor(0,screenY);				// set write position
#endif
#ifdef ST7735
	tft.setCursor(5, screenY);
#endif
}
	
#define SCREENLINE screenNextLine(),SCREEN

void screenSend() { 
#ifdef U8G2
	u8g2.sendBuffer();
	#endif
#ifdef ST7735
#endif
}	
WatchDogTimer wdt(10000);

MPU9250_DMP imu;
#define IMU_INT_PIN 4

void imuPrint(); 
void imuInit() { 
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(100);
    }
  }
  
  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  
  imu.setSampleRate(1000);
  imu.dmpSetInterruptMode(DMP_INT_CONTINUOUS);
  imu.setIntLevel(INT_ACTIVE_LOW);
  imu.enableInterrupt(1);
  imu.setIntLatched(INT_50US_PULSE);
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              50); // Set DMP FIFO rate to 10 Hz
  
  attachInterrupt(digitalPinToInterrupt(button.pin), imuPrint, FALLING);
}

struct LogItem { 
	float sec, hdg, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, q4;
	String toString() { 
		char buf[1024];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f", 
		sec, hdg, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, q4);
		return String(buf);	
	 }
};

SDCardBufferedLog<LogItem>  logFile("log1.txt", 200, 100, 1000);
msdFile logFileF;

void printIMUData(void);
int imuInt = 0;
float imuHdg, imuAlt;
void imuPrint() 
{
  imuInt++;
  // Check for new data in the FIFO
  while ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      imu.updateAccel();
      imu.updateGyro();
      imu.updateCompass();
      
      LogItem x;
      x.sec = millis() / 1000.0;
      x.hdg = imuHdg;
      x.alt = imuAlt;
      x.ax = imu.calcAccel(imu.ax);
      x.ay = imu.calcAccel(imu.ay);
      x.az = imu.calcAccel(imu.az);
      x.gx = imu.calcGyro(imu.gx);
      x.gy = imu.calcGyro(imu.gy);
      x.gz = imu.calcGyro(imu.gz);
      x.mx = imu.calcMag(imu.mx);
      x.my = imu.calcMag(imu.my);
      x.mz = imu.calcMag(imu.mz);
      x.q1 = imu.calcQuat(imu.qw);
      x.q2 = imu.calcQuat(imu.qx);
      x.q3 = imu.calcQuat(imu.qy);
      x.q4 = imu.calcQuat(imu.qz);
      x.p = imu.pitch;
      x.r = imu.roll;
      x.y = imu.yaw;
      //Serial.println(x.toString());
      //logFile.add(&x, 0/*timeout*/);
      logFileF.println(x.toString());
    }
  }
}

uint64_t lastTime = 0;
void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  uint64_t now = imu.time;
  //Serial.printf("Q: %+05.4f %+05.4f %+05.4f %+05.4f   ", q0, q1, q2, q3);
  Serial.printf("%+06.2f %+06.2f %+06.2f ", imu.pitch, imu.roll, imu.yaw);
  Serial.printf("ms: %.3f ms\n", (now - lastTime) / 1000.0);
  lastTime = now;
}

//HardwareSerial SerialMav;

#define SerialMav Serial1 


void setup() {
	screenInit("");
	//wdt.begin();  // doesn't work yet

	//re.begin([]{re.ISR();});

	attachInterrupt(digitalPinToInterrupt(button.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button2.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button3.pin), buttonISR, CHANGE);

	mavRemoteIp.fromString("192.168.43.166");
	pinMode(LED_PIN, OUTPUT);
	//pinMode(17, INPUT_PULLUP);
	pinMode(button.pin, INPUT_PULLUP);
	pinMode(button2.pin, INPUT_PULLUP);
	pinMode(button3.pin, INPUT_PULLUP);

	SCREENLINE.println("Initializing IMU...");
	imuInit();

	SCREENLINE.println("Opening SD card...");
	open_TTGOTS_SD();

	SCREENLINE.println("Initializing logfile...");
	//logFile.begin();
	
	SCREENLINE.println("Connecting to WiFi...");
	if (true) { 
		//WiFi.mode(WIFI_STA);
		//WiFi.setSleep(false);
		//WiFi.begin("ChloeNet", "niftyprairie7");
		wifi.addAP("Ping-582B", "");
		wifi.addAP("ChloeNet", "niftyprairie7");

		uint64_t startms = millis();
		while (WiFi.status() != WL_CONNECTED && digitalRead(button.pin) != 0) {
			wifi.run();
			delay(10);
			if (millis() - startms > 11000)
				break;
		}
		udpSL30.begin(7891);
		udpG90.begin(4000);
		udpMAV.begin(MAVLINK_PORT);
		udpNMEA.begin(7892);
	}
	
	SCREENLINE.println("WiFi connected");
	mavlink_open();

#ifdef U8G2
	pinMode(22, INPUT);
	pinMode(19, OUTPUT);
	SerialMav.begin(57600, SERIAL_8N1, 22, 19);
	SerialMav.setTimeout(1);
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(1);
#endif

#ifdef TTGO
	pinMode(21, INPUT);
	pinMode(22, OUTPUT);
	SerialMav.begin(57600, SERIAL_8N1, 21, 22);
	SerialMav.setTimeout(1);
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(1);
#endif

	SCREENLINE.println("Init complete");
	screenClear();
}




void mav_gps_msg(float lat, float lon, float crs, float speed, float alt, float hdop, float vdop) {
	
	uint64_t time_usec = 0;
	uint8_t gps_id = 12;
	uint16_t ignore_flags = 0; 
	uint32_t time_week_ms = 0; 
	uint16_t time_week = 0;
	uint8_t fix_type = 4;
	uint8_t system_type = MAV_TYPE_GENERIC;
	uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
	uint8_t targetSysId = 1;

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
	delay(1);
	
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
	static int gpsUseGDL90 = 1;
	static int obs = 0;
	static int navDTK = 0;
	static bool phSafetySwitch = false; //debug start off logging
	static SDCardBufferedLog<LogItem> *logFile = NULL;
	static bool screenEnabled = true;
	
	Serial.printf("imu int count %d\n", imuInt);
	
	
	if (phSafetySwitch == true) 
		imuPrint();

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
	
	if (butFilt2.newEvent()) { 
		if (butFilt2.wasLong) {
			if (butFilt2.wasCount == 1) 
				phSafetySwitch = !phSafetySwitch;
			if (butFilt2.wasCount == 2)  {
				mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 4/*param5*/, 0, 0);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				mavlink_send(buf, len);
			}	
		} else { 
			apMode = butFilt2.wasCount;
		}
		mavTimer.alarmNow();
	}
	if (butFilt3.newEvent()) { 
		screenEnabled = !screenEnabled;
		if (screenEnabled == false) 
			screenClear();
	}
	
	
	if (phSafetySwitch == false && logFileF) { 
		logFileF.flush();
		logFileF.close();
	} 
	if (phSafetySwitch == true && !logFileF) {
		char fname[128];
		for(int n = 0; n < 999; n++) {
			sprintf(fname, "LOGFILE.%03d", n);
			msdFile f = SD.open(fname, F_READ);
			if (!f) {
				break;
			}
			f.close();
		}
		logFileF = SD.open(fname, (F_READ | F_WRITE | F_CREAT));
	}
		
	/*
	if (phSafetySwitch == false && logFile == NULL) {
		logFile = new SDCardBufferedLog<LogItem>("log.txt", 200, 100, 1000);
		logFile->begin();
	} 
	if (phSafetySwitch == true && logFile != NULL) { 
		logFile->exit();
		delete logFile;
	}*/
	
	
	if (screenEnabled && screenTimer.tick()) { 
		screenY = 10;
		String s = WiFi.localIP().toString();
		SCREENLINE.printf("WIFI: %s", s.c_str());
		SCREENLINE.printf(" DTK: %03d   HDG: %03d", (int)desiredTrk, (int)lastHdg);
		SCREENLINE.printf("NAVT: %03d   OBS: %03d", navDTK, obs);
		SCREENLINE.printf("KNOB: %03d  MODE: %d%d%d", re.value, apMode, gpsUseGDL90, (int)phSafetySwitch);
		SCREENLINE.printf(" UDP: %03d   SER: %03d", udpBytes % 1000, serBytes % 1000);
		SCREENLINE.printf(" MAV: %03d  ROLL:%+05.1f",  mavHeartbeats % 1000, roll);
		screenSend();
	}
	
	if (blinkTimer.tick()) 
		count++;
	digitalWrite(LED_PIN, (count & 0x1) ^ (gpsFixes & 0x1) ^ (serBytes & 0x1));
	
	yield();
	
	if (SerialMav.available()) {
		int l = SerialMav.readBytes(buf, sizeof(buf));
		serBytes += l + random(0,2);
		if (WiFi.status() == WL_CONNECTED) { 
			udpMAV.beginPacket(mavRemoteIp ,MAVLINK_PORT); // todo - could send to the last ip we received from instead of bcast addr 
			udpMAV.write((uint8_t *)buf, l);
			udpMAV.endPacket();
		}
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
			SerialMav.write(buf, n);
			avail -= n;
		}
	}

	if (mavTimer.tick()) {
		uint8_t system_type = MAV_TYPE_GENERIC;
		uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
		uint8_t targetSysId = 1;
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
 
		mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_set_mode_pack(1, 2, &msg, 0, MAV_MODE_FLAG_DECODE_POSITION_SAFETY, (int)(!phSafetySwitch));
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
	 
		if (apMode == 1) { 
			int new_mode = 0; // 0 == MANUAL  
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 2) {
			int new_mode = 2;  //2 == STABILIZE
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 3 || apMode == 4 || apMode == 5) {
			int new_mode = 7; // 7 = CRUISE
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

			if (apMode == 3) 
				desiredTrk = (int)re.value;
			if (apMode == 4) 
				desiredTrk = (int)(obs + 15.5 /*mag variation*/ + 360) % 360;
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
			avail -= recsize;
			udpBytes += recsize + random(0,2);
			for (int i = 0; i < recsize; i++) {  
				yield();
				gdl90.add(buf[i]);
				GDL90Parser::State s = gdl90.getState();
				if (gpsUseGDL90 && s.valid && s.updated) { 
					gpsFixes++;
					int alt = s.palt * 25 - 1000;
					mav_gps_msg(s.lat, s.lon, s.track, s.hvel * 0.51444, s.palt * 25 - 1000, 1.23, 2.34);
					imuAlt = alt;
					imuHdg = s.track;
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
				mav_gps_msg(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.course.deg(), gps.speed.mps(), gps.hdop.hdop(), 2.34);
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
}

void mavlink_send(const uint8_t *buf, int len) { 
	SerialMav.write(buf, len);
}
int mavlink_read(uint8_t *buf, int len) { 
	return 0;
}
