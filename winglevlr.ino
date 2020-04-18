#ifndef UBUNTU
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
#include <MPU9250_asukiaaa.h>
#include <SparkFunMPU9250-DMP.h>
#include <RunningLeastSquares.h>
#include <mySD.h>
#include "Wire.h"
#include <MPU9250_asukiaaa.h>
#else
#include "ESP32sim_ubuntu.h"
#endif




#include "jimlib.h"
#include "RollAHRS.h"
#include "PidControl.h"
#include "mavlink.h"
#include "TinyGPS++.h"
#include "G90Parser.h"

WiFiMulti wifi;

TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);
TinyGPSCustom vtgCourse(gps, "GPVTG", 1);

GDL90Parser gdl90;
GDL90Parser::State state;

RollAHRS ahrs;
PidControl rollPID(30) /*200Hz*/, navPID(50); /*20Hz*/
PidControl *knobPID = &rollPID;
static int servoTrim = 1500;

#define MAVLINK_PORT 14450
void mavlink_open();
void mavlink_send(const uint8_t *, int);

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
WiFiUDP udpMAV;

#define LED_PIN 22
DigitalButton button(34); // middle
DigitalButton button2(35); // left
DigitalButton button3(39); // top
DigitalButton button4(21); // knob press

static IPAddress mavRemoteIp;
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
static uint8_t buf[BUFFER_LENGTH];
EggTimer screenTimer(200), blinkTimer(1000), udpDebugTimer(1000), mavTimer(300);

LongShortFilter butFilt(1500,600);
LongShortFilter butFilt2(1500,600);
LongShortFilter butFilt3(1500,600);
LongShortFilter butFilt4(1500,600);

void buttonISR() { 
	button.check();
	button2.check();
	button3.check();
	butFilt.check(button.duration());
	butFilt2.check(button2.duration());
	butFilt3.check(button3.duration());
	butFilt4.check(button4.duration());
}

namespace Display {
	JDisplay jd;
	int y = 0;
	JDisplayItem<const char *>  ip(&jd,10,y+=10,"WIFI:", "%s ");
	JDisplayItem<float>  dtk(&jd,10,y+=10," DTK:", "%05.1f ");  JDisplayItem<float>  trk(&jd,70,y,    " TRK:", "%05.1f ");
	JDisplayItem<float> navt(&jd,10,y+=10,"NAVT:", "%05.1f ");    JDisplayItem<int>    obs(&jd,70,y,    " OBS:", "%03d ");
	JDisplayItem<int>   knob(&jd,10,y+=10,"KNOB:", "%03d ");    JDisplayItem<int>   mode(&jd,70,y,    "MODE:", "%03d ");
	JDisplayItem<float>  gdl(&jd,10,y+=10," GDL:", "%05.1f ");  JDisplayItem<float>  vtg(&jd,70,y,    " VTG:", "%05.1f ");
	JDisplayItem<float>  rmc(&jd,10,y+=10," RMC:", "%05.1f");   JDisplayItem<float> roll(&jd,70,y,    "ROLL:", "%+05.1f ");
	JDisplayItem<const char *>  log(&jd,10,y+=10," LOG:", "%s  ");

    //JDisplayItem<float> pidc(&jd,10,y+=20,"PIDC:", "%05.1f ");JDisplayItem<int>   serv(&jd,70,y,    "SERV:", "%04d ");
	
	JDisplayItem<float> pidp(&jd,10,y+=10,"   P:", "%05.2f "); JDisplayItem<float> pidg(&jd,70,y,    "GAIN:", "%04.1f ");
	JDisplayItem<float> pidi(&jd,10,y+=10,"   I:", "%05.3f "); JDisplayItem<float> maxb(&jd,70,y,    "MAXB:", "%04.1f ");
	JDisplayItem<float> pidd(&jd,10,y+=10,"   D:", "%05.2f "); JDisplayItem<float> navg(&jd,70,y,    "NAVG:", "%05.1f ");
}

MPU9250_DMP imu;
#define IMU_INT_PIN 4

void imuLog(); 
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
  
  //pinMode(IMU_INT_PIN, INPUT_PULLUP);
  
  imu.setGyroFSR(250/*deg per sec*/);
  imu.setAccelFSR(4/*G*/);
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
 
  // FIFO seems slower than just sampling 
  //imu.setCompassSampleRate(100); // Set mag rate to 10Hz
  //imu.setSampleRate(1000);
  //imu.configureFifo(INV_XYZ_GYRO |INV_XYZ_ACCEL);  

  //imu.dmpSetInterruptMode(DMP_INT_CONTINUOUS);
  //imu.setIntLevel(INT_ACTIVE_LOW);
  //imu.enableInterrupt(1);
  //imu.setIntLatched(INT_50US_PULSE);
  /*imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  */
  //attachInterrupt(digitalPinToInterrupt(button.pin), imuPrint, FALLING);
}

static AhrsInput ahrsInput;
bool imuRead() { 
	
	//if (imu.fifoAvailable() && imu.updateFifo() == INV_SUCCESS) { // FIFO is slow
	if(true){
		imu.updateAccel();
		imu.updateGyro();
		imu.updateCompass();

		AhrsInput &x = ahrsInput;
		x.sec = micros() / 1000000.0;
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
		// remaining items set (alt, hdg, speed) set by main loop
		return true;
	}
	return false;
}

LogItem logItem;
SDCardBufferedLog<LogItem>  *logFile = NULL;

void sdLog()  {
	logItem.ai = ahrsInput;
	//Serial.println(x.toString());
	if (logFile != NULL)
		logFile->add(&logItem, 0/*timeout*/);
}

void printMag() {
      //imu.updateCompass();
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.calcGyro(imu.gx), (float)imu.calcGyro(imu.gy), (float)imu.calcGyro(imu.gz) );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.calcMag(imu.mx), (float)imu.calcMag(imu.my), (float)imu.calcMag(imu.mz) );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.calcAccel(imu.ax), (float)imu.calcAccel(imu.ay), (float)imu.calcAccel(imu.az) );
      Serial.println("");
}


class MyEditor : public JDisplayEditor {
public:
	JDisplayEditableItem pidp = JDisplayEditableItem(Display::pidp, .01);
	JDisplayEditableItem pidi = JDisplayEditableItem(Display::pidi, .001);
	JDisplayEditableItem pidd = JDisplayEditableItem(Display::pidd, .01);
	JDisplayEditableItem pidg = JDisplayEditableItem(Display::pidg, .1);
	JDisplayEditableItem maxb = JDisplayEditableItem(Display::maxb, 1);
	JDisplayEditableItem navg = JDisplayEditableItem(Display::navg, .1);
	
	MyEditor() : JDisplayEditor(0, 27) {
		add(&pidp);	
		add(&pidi);	
		add(&pidd);	
		add(&pidg);	
		add(&maxb);
		add(&navg);
	}
} ed;

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, 1);

	pinMode(32, INPUT);
	pinMode(26, OUTPUT);
	Serial1.begin(57600, SERIAL_8N1, 32, 26);
	Serial1.setTimeout(1);
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(1);

	Display::jd.begin();
	Display::jd.clear();
	
	//wdt.begin();  // doesn't work yet

	pinMode(button.pin, INPUT_PULLUP);
	pinMode(button2.pin, INPUT_PULLUP);
	pinMode(button3.pin, INPUT_PULLUP);
	pinMode(button4.pin, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(button.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button2.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button3.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button4.pin), buttonISR, CHANGE);

	//SCREENLINE.println("Initializing IMU...");
	imuInit();	
	
	if (digitalRead(button4.pin) != 0) { // skip long setup stuff if we're debugging
		//SCREENLINE.println("Opening SD card...");
		//open_TTGOTS_SD();
		//printSD();

		//SCREENLINE.println("Connecting to WiFi...");
		//WiFi.mode(WIFI_STA);
		//WiFi.setSleep(false);
		//WiFi.begin("ChloeNet", "niftyprairie7");
		wifi.addAP("Ping-582B", "");
		wifi.addAP("ChloeNet", "niftyprairie7");
		wifi.addAP("Team America", "51a52b5354");

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
	
	//SCREENLINE.println("WiFi connected");
	mavlink_open();
	mavRemoteIp.fromString("192.168.43.166");

	rollPID.setGains(7.52, 0, 0.11);
	rollPID.finalGain = 16.8;
	navPID.setGains(0.5, 0, 0.1);
	navPID.finalGain = 0.9;
	navPID.hiGain.p = 0; 
	navPID.hiGainTrans.p = 0;
	
	ed.begin();
#ifndef UBUNTU
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
#endif
	ed.pidp.value = knobPID->gain.p;
	ed.pidi.value = knobPID->gain.i;
	ed.pidd.value = knobPID->gain.d;
	ed.pidg.value = knobPID->finalGain;
	ed.maxb.value = 16;
	ed.navg.value = navPID.finalGain;
	
	pinMode(33, OUTPUT);
	ledcSetup(1, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(33, 1);   // GPIO 33 assigned to channel 1
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

template<class T> 
class StaleData {
	uint64_t timeout, lastUpdate;
	T value, invalidValue;
public:
	StaleData(int t, T i) : lastUpdate(0), timeout(t), invalidValue(i) {} 
	bool isValid() { return millis() - lastUpdate < timeout; }
	operator T&() { return isValid() ? value : invalidValue; }
	StaleData<T>& operator =(const T&v) {
		value = v;
		lastUpdate = millis();
		return *this;
	}
	T getValue() { return value; } 
};
 
static StaleData<float> gpsTrackGDL90(5000,-1), gpsTrackRMC(5000,-1), gpsTrackVTG(5000,-1);
static float desiredTrk = -1;
float desRoll = 0;		

void ESP32sim_set_gpsTrackGDL90(float v) { 
	gpsTrackGDL90 = v;
}
void ESP32sim_set_desiredTrk(float v) {
	desiredTrk = v;
}

void loop() {
	mavlink_message_t msg;
	uint16_t len;
	static float mavRoll = 0;
	static int ledOn = 0;
	static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
	static int buildNumber = 12;
	static int mavBytesIn = 0;
	static mavlink_message_t mavMsgIn;
	static mavlink_status_t mavStatusIn;
	static char lastParam[64];
	static int lastHdg;
	static int mavHeartbeats;
	static int apMode = 1;
	static int gpsUseGDL90 = 1; // 0- use VTG sentence, 1 use GDL90 data, 2 use average of both 
	static int obs = 0, lastObs = 0;
	static int navDTK = 0;
	static bool phSafetySwitch = true;
	static bool screenEnabled = true;
	static uint64_t lastLoop = micros();
	static int armServo = 0;
	static RollingAverage<int,1000> loopTime;
	static EggTimer serialReportTimer(1000), navPIDTimer(50);
	static bool selEditing = false;
	static int pwmOutput = 0, servoOutput = 0;
	static float roll = 0;
	static String logFilename("none");
	
	delayMicroseconds(10);
	uint64_t now = micros();
	loopTime.add(now - lastLoop);
	lastLoop = now;
	if (serialReportTimer.tick()) { 
		Serial.printf("roll %+05.1f servo %04d buttons %d%d%d%d Loop time min/avg/max %d/%d/%d\n", roll, servoOutput, 
		digitalRead(button.pin), digitalRead(button2.pin), digitalRead(button3.pin), digitalRead(button4.pin), 
		loopTime.min(), loopTime.average(), loopTime.max());
	}
	
	//Serial.printf("%d\n", (int)(micros() - lastLoop));
	//lastLoop = micros();
	//printMag();
	

	buttonISR();
	if (butFilt.newEvent()) { // MIDDLE BUTTON
		if (!butFilt.wasLong) {
			if (butFilt.wasCount == 1) {
				screenEnabled = true;
				Display::jd.begin();
				Display::jd.forceUpdate();
			} else { 
				gpsUseGDL90 = (gpsUseGDL90 + 1) % 3;
			}
				
		} else { 
			screenEnabled = false;
			Display::jd.clear();
		}
		
	}
	if (butFilt2.newEvent()) { // BOTTOM or LEFT button
		if (butFilt2.wasCount == 1) {
			armServo = !armServo; 
			rollPID.reset();
			navPID.reset();
			// send mavlink message for use in debuggin mavlink connections 
			int new_mode = armServo;
			mavlink_msg_set_mode_pack(1, 200, &msg, 1, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 
		}
		if (butFilt2.wasLong && butFilt2.wasCount == 2)  {
			mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 4/*param5*/, 0, 0);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len);
		}	
	}
	if (butFilt3.newEvent()) { // TOP or RIGHT button 
		phSafetySwitch = !phSafetySwitch;
		if (phSafetySwitch == false) {
			screenEnabled = false;
			Display::jd.clear();
		} else {
			screenEnabled = true;
			Display::jd.begin();
			Display::jd.forceUpdate();
		}
		mavTimer.alarmNow();
	}
	
	if (butFilt4.newEvent()) { 	// main knob button
		if (butFilt4.wasCount == 1) { 
			ed.buttonPress(butFilt4.wasLong);
		}
		if (butFilt4.wasLong && butFilt4.wasCount == 2) {
			ed.negateSelectedValue();
		}
		if (butFilt4.wasLong && butFilt4.wasCount == 3) {
			screenEnabled = !screenEnabled;
			if (!screenEnabled)
				Display::jd.clear();
			else {
				Display::jd.begin();
				Display::jd.forceUpdate();
			}
		}
	}
	
	if (phSafetySwitch == false && logFile == NULL) {
		logFile = new SDCardBufferedLog<LogItem>("AHRSD%03d.DAT", 200/*q size*/, 100/*timeout*/, 1000/*flushInterval*/, false/*textMode*/);
		logFilename = logFile->currentFile;
		Serial.printf("Opened log file '%s'\n", logFile->currentFile.c_str());
	} 
	if (phSafetySwitch == true && logFile != NULL) {
		Serial.printf("Closing log file '%s'\n", logFile->currentFile.c_str());
		delete logFile;
		logFile = NULL;
		//printSD();
	}

	ahrsInput.gpsTrackGDL90 = gpsTrackGDL90;
	ahrsInput.gpsTrackVTG = gpsTrackVTG;
	ahrsInput.gpsTrackRMC = gpsTrackRMC;
	if (gpsUseGDL90 == 0) ahrsInput.gpsTrack = gpsTrackVTG;
	if (gpsUseGDL90 == 1) ahrsInput.gpsTrack = gpsTrackGDL90;
	if (gpsUseGDL90 == 2) {
		if (!gpsTrackVTG.isValid()) {
			ahrsInput.gpsTrack = gpsTrackGDL90;
		} else if (!gpsTrackGDL90.isValid()) {
			ahrsInput.gpsTrack = gpsTrackVTG;
		} else { 
			float diff = gpsTrackVTG - gpsTrackGDL90; 
			if (diff < -180) diff += 360;
			if (diff > 180) diff -= 360;
			ahrsInput.gpsTrack = gpsTrackVTG - diff / 2;
		}
	}
	
	if (imuRead()) {
		roll = ahrs.add(ahrsInput);
		pwmOutput = 0;
		if (ahrs.valid() || digitalRead(button4.pin) == 0) { // hold down button to override and make servo work  
			if (desiredTrk != -1) {
				double hdgErr = ahrsInput.gpsTrack - desiredTrk;
				if(hdgErr < -180) hdgErr += 360;
				if(hdgErr > 180) hdgErr -= 360;
				if (navPIDTimer.tick()) {
					desRoll = -navPID.add(hdgErr, ahrsInput.gpsTrack, ahrsInput.sec);
					desRoll = max(-ed.maxb.value, min(+ed.maxb.value, desRoll));
				}
			}
			// TODO: desRoll may be stale, but leave this bug so serial cmdline can set desRoll 
			rollPID.add(roll - desRoll, roll, ahrsInput.sec);
			//Serial.printf("%05.2f %05.2f %04d\n", desRoll, roll);
			if (armServo) {  
				servoOutput = servoTrim + rollPID.corr;
				pwmOutput = max(1550, min(7300, servoOutput * 4915 / 1500));
			}
		}
		
		ledcWrite(1, pwmOutput); // scale PWM output to 1500-7300 
		logItem.pidP = rollPID.err.p;
		logItem.pidI = rollPID.err.i;
		logItem.pidD = rollPID.err.d;
		logItem.finalGain = rollPID.finalGain;
		logItem.gainP = rollPID.gain.p;
		logItem.gainI = rollPID.gain.i;
		logItem.gainD = rollPID.gain.d;
		logItem.pwmOutput = pwmOutput;
		logItem.servoTrim = servoTrim;
		logItem.desRoll = desRoll;
		logItem.roll = roll;
		logItem.dtk = desiredTrk;
		if (logFile != NULL) {
			sdLog();
		}
	}

	if (screenEnabled) { 
		ed.update();
		knobPID->gain.p = ed.pidp.value;
		knobPID->gain.i = ed.pidi.value;
		knobPID->gain.d = ed.pidd.value;
		knobPID->finalGain = ed.pidg.value;
		navPID.finalGain = ed.navg.value;
	}
	
	if (screenEnabled && screenTimer.tick()) {
		Display::mode.color.vb = (apMode == 4) ? ST7735_RED : ST7735_GREEN;
		Display::mode.color.vf = ST7735_BLACK;
		Display::roll.color.vf = ST7735_RED;

		Display::ip = WiFi.localIP().toString().c_str(); 
		Display::dtk = desiredTrk; 
		Display::trk = ahrsInput.gpsTrack; 
		Display::navt = navDTK; 
		Display::obs = obs; 
		Display::knob = ed.re.value; 
		Display::mode = armServo * 100 + gpsUseGDL90 * 10 + (int)phSafetySwitch; 
		Display::gdl = (float)gpsTrackGDL90;
		Display::vtg = (float)gpsTrackVTG;
		Display::rmc = (float)gpsTrackRMC; 
		Display::roll = roll; Display::log = logFilename.c_str();
		Display::roll.setInverse(false, (logFile != NULL));
		
		//Display::pidc = pid.corr;
		//Display::serv = pwmOutput;
	}
	
	if (blinkTimer.tick()) 
		ledOn ^= 1;
	digitalWrite(LED_PIN, (ledOn & 0x1) ^ (gpsFixes & 0x1) ^ (serBytes & 0x1));
	
	if (Serial1.available()) {
		int l = Serial1.readBytes(buf, sizeof(buf));
		serBytes += l;// + random(0,2);
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
					mavRoll = attitude.roll * 180 / 3.1415;
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
		mavBytesIn += n;// + random(0,2);
		if (n > 0) {
			Serial1.write(buf, n);
			avail -= n;
		} else { 
			break;
		}
	}

	if (mavTimer.tick()) {
		uint8_t system_type = MAV_TYPE_GENERIC;
		uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
		uint8_t targetSysId = 1;

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

		mavlink_msg_set_mode_pack(1, 2, &msg, 0, MAV_MODE_FLAG_DECODE_POSITION_SAFETY, (int)(phSafetySwitch));
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
	 
/*		if (apMode == 1) { 
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
				desiredTrk = (int)(obs + 15.5  + 360) % 360;
			if (apMode == 5) 
				desiredTrk = navDTK;

			mavlink_msg_param_set_pack(1, 200, &msg, 0, 0, "CRUISE_HEADING", desiredTrk, MAV_VAR_FLOAT);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len);
		}
		*/
	}

	int recsize = 0;
	do {
		recsize = 0;
		int avail = udpG90.parsePacket();
		while(avail > 0) { 
			recsize = udpG90.read(buf, min(avail,(int)sizeof(buf)));
			if (recsize <= 0)
				break;
			avail -= recsize;
			udpBytes += recsize; //+ random(0,2);
			for (int i = 0; i < recsize; i++) {  
				gdl90.add(buf[i]);
				GDL90Parser::State s = gdl90.getState();
				if (s.valid && s.updated) {
					gpsTrackGDL90 = s.track;
					if (gpsUseGDL90 == 1 || gpsUseGDL90 == 2) {
						gpsFixes++;
						ahrsInput.alt = s.alt;
						ahrsInput.gspeed = s.hvel;
						mav_gps_msg(s.lat, s.lon, s.track, s.hvel * 0.51444, s.alt, 1.23, 2.34);
					}
				}
			}
		}
	}
	while(recsize > 0);

	if (Serial.available()) {
		static char line[128]; // TODO make a line parser class with a lambda/closure
		static int index;
		int n = Serial.readBytes(buf, sizeof(buf));
		for (int i = 0; i < n; i++) {
			if (index >= sizeof(line))
				index = 0;
			if (buf[i] != '\r')
				line[index++] = buf[i];
			if (buf[i] == '\n' || buf[i] == '\r') {
				line[index] = '\0';
				Serial.printf("RECEIVED COMMAND: %s", line);
				index = 0;
				float f;
				if (sscanf(line, "navhi=%f", &f) == 1) { navPID.hiGain.p = f; }
				else if (sscanf(line, "navtr=%f", &f) == 1) { navPID.hiGainTrans.p = f; }
				else if (sscanf(line, "maxb=%f", &f) == 1) { ed.maxb.value = f; }
				else if (sscanf(line, "roll=%f", &f) == 1) { desRoll = f; }
				else if (sscanf(line, "dtrk=%f", &f) == 1) { desiredTrk = f; }
				else if (sscanf(line, "knob=%f", &f) == 1) {
					if (f == 1) {
						knobPID = &rollPID;
					} else if (f == 2) { 
						knobPID = &navPID;
					}
					ed.pidp.value = knobPID->gain.p;
					ed.pidi.value = knobPID->gain.i;
					ed.pidd.value = knobPID->gain.d;
					ed.pidg.value = knobPID->finalGain;
				} else {
					Serial.printf("UNKNOWN COMMAND: %s", line);
				}
			}
		}
	}

	avail = udpSL30.parsePacket();
	while(avail > 0) { 
		static char line[128];
		static int index;
		
		int n = udpSL30.read(buf, sizeof(buf));
		if (n <= 0)
			break;
		avail -= n;
		udpBytes += n; // + random(0,2);
		for (int i = 0; i < n; i++) {
			if (index >= sizeof(line))
				index = 0;
			if (buf[i] != '\r')
				line[index++] = buf[i];
			if (buf[i] == '\n' || buf[i] == '\r') {
				// 0123456789012
				// $PMRRV34nnnXX
				line[11] = 0; // TODO calculate the checksum 
				sscanf(line, "$PMRRV34%d", &obs);
				index = 0;
				if (obs != lastObs)
					desiredTrk = ((int)(obs + 15.5 + 360)) % 360;
				lastObs = obs;
			}
			gps.encode(buf[i]);
			// Use only VTG course so as to only use G5 data
			if (vtgCourse.isUpdated()) {  
				gpsTrackVTG = gps.parseDecimal(vtgCourse.value()) * 0.01; //gps.course.deg();
			}
			if (gps.course.isUpdated()) { 
				gpsTrackRMC = gps.course.deg();
			}
			if (gps.location.isUpdated() && gpsUseGDL90 == 2) {
				//mav_gps_msg(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.course.deg(), gps.speed.mps(), gps.hdop.hdop(), 2.34);
				ahrsInput.alt = gps.altitude.meters() * 3.2808;
				ahrsInput.gspeed = gps.speed.knots();
				gpsFixes++;
			}
			if (desiredHeading.isUpdated()) { 
				if (apMode == 4) 
					apMode = 5;
				navDTK = 0.01 * gps.parseDecimal(desiredHeading.value());
				desiredTrk = navDTK;
			}
		}
	}
}

void mavlink_open() {}
void mavlink_send(const uint8_t *buf, int len) { Serial1.write(buf, len); }


// TODO:
// add min/max/avg loop cycle time monitoring
// 
