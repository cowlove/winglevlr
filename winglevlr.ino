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
#include "RollAHRS.h"
#include "PidControl.h"


WiFiMulti wifi;
TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);

GDL90Parser gdl90;
GDL90Parser::State state;

RollAHRS ahrs;
PidControl pid(30);
static int servoTrim = 1500;


#define MAVLINK_PORT 14450
void mavlink_open();
void mavlink_send(const uint8_t *, int);

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

#include <MPU9250_asukiaaa.h>
#include <Kalman.h> 					// Source: https://github.com/TKJElectronics/KalmanFilter
#define LED_PIN 22
//RotaryEncoder re(27,33);
DigitalButton button(34);
DigitalButton button2(35);
DigitalButton button3(39);
DigitalButton button4(21);

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
	JDisplayItem<const char *>  ip(&jd,10,y+=10,"WIFI:", "%s");
	JDisplayItem<int>    dtk(&jd,10,y+=10," DTK:", "%03d");  JDisplayItem<int>    hdg(&jd,70,y,    " HDG:", "%03d");
	JDisplayItem<int>   navt(&jd,10,y+=10,"NAVT:", "%03d");  JDisplayItem<int>    obs(&jd,70,y,    " OBS:", "%03d");
	JDisplayItem<int>   knob(&jd,10,y+=10,"KNOB:", "%03d");  JDisplayItem<int>   mode(&jd,70,y,    "MODE:", "%03d");
	JDisplayItem<int>    udp(&jd,10,y+=10," UDP:", "%03d");  JDisplayItem<int>    ser(&jd,70,y,    " SER:", "%03d");
	JDisplayItem<int>    mav(&jd,10,y+=10," MAV:", "%03d");  JDisplayItem<float> roll(&jd,70,y,    "ROLL:", "%+05.1f");
	JDisplayItem<const char *>  log(&jd,10,y+=10," LOG:", "%s");

    //JDisplayItem<float> pidc(&jd,10,y+=20,"PIDC:", "%05.1f");JDisplayItem<int>   serv(&jd,70,y,    "SERV:", "%04d");
	
	JDisplayItem<float> pidp(&jd,10,y+=10,"   P:", "%05.2f"); JDisplayItem<float> pidg(&jd,70,y,    "GAIN:", "%04.1f");
	JDisplayItem<float> pidi(&jd,10,y+=10,"   I:", "%05.2f"); JDisplayItem<float> srvt(&jd,70,y,    "SRVT:", "%04.0f");
	JDisplayItem<float> pidd(&jd,10,y+=10,"   D:", "%05.2f");JDisplayItem<float> xxxx(&jd,70,y,    "XXXX:", "%05.1f");
}

WatchDogTimer wdt(10000);

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
  
  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  
  imu.setSampleRate(1000);
  imu.setGyroFSR(250/*deg per sec*/);
  imu.setAccelFSR(4/*G*/);
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
	//if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {
	if (1) {
		//imu.computeEulerAngles();
		imu.updateAccel();
		imu.updateGyro();
		imu.updateCompass();

		AhrsInput &x = ahrsInput;
		x.sec = millis() / 1000.0;
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
      imu.updateCompass();
      Serial.printf("%+09.4f %+09.4f %+09.4f\n", (float)imu.calcGyro(imu.gx), (float)imu.calcGyro(imu.gy), (float)imu.calcGyro(imu.gz) );
      //Serial.printf("%+09.4f %+09.4f %+09.4f\n", (float)imu.calcMag(imu.mx), (float)imu.calcMag(imu.my), (float)imu.calcMag(imu.mz) );
}

void printDirectory(msdFile dir, int numTabs) {
  while(true) {
     msdFile entry =  dir.openNextFile();
     if (! entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');   // we'll have a nice indentation
     }
     // Print the name
     Serial.print(entry.name());
     /* Recurse for directories, otherwise print the file size */
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       /* files have sizes, directories do not */
       Serial.print("\t\t");
       Serial.println(entry.size());
     }
     entry.close();
	}
}

void printSD() { 
	msdFile root = SD.open("/");
	if (root) {
		printDirectory(root, 0);
		root.close();
	}
}



class JDisplayEditableItem;

class JDisplayEditor {
	std::vector<JDisplayEditableItem *> items;
	bool editing;
	int selectedItem;
public:
	RotaryEncoder re;
	JDisplayEditor(int p1, int p2) : re(p1, p2) {}
	void add(JDisplayEditableItem *i) { 
		items.push_back(i);
	}
	void begin() {
		re.limMin = 0;
		re.limMax = items.size() - 1;
		editing = false;
		re.wrap = false;
		re.value = 0;
		selectedItem = 0;
		//re.begin( [this]()->void{ this->re.ISR(); });
	}
	inline void update(); 
	inline void buttonPress(bool longpress);			
		
};

class JDisplayEditableItem { 
protected:
	JDisplayItemBase &di; 
public:
	float value, newValue, increment;
	enum { UNSELECTED, SELECTED, EDITING } state;
	JDisplayEditableItem(JDisplayItemBase &i, float inc) : di(i), increment(inc) {
	}
	void update() { 
		if (state == EDITING) {
			di.setValue(newValue);
			di.setInverse(false, true);
		} else { 
			di.setValue(value);
			di.setInverse(state == SELECTED, false);
		}
		di.update(false);
	};
};

inline void JDisplayEditor::update() { 
	if (!editing) { 
		if (selectedItem != re.value) { 
			selectedItem = re.value;
			for(int n = 0; n < items.size(); n++) {
				items[n]->state = (n == selectedItem) ? JDisplayEditableItem::SELECTED : 
					JDisplayEditableItem::UNSELECTED;
				items[n]->update();
			}
		}
	} else { 
		items[selectedItem]->newValue = items[selectedItem]->value + re.value * items[selectedItem]->increment;
		items[selectedItem]->update();
	}
}
			
inline void JDisplayEditor::buttonPress(bool longpress) { 
	if (!editing) { 
		editing = true;
		selectedItem = re.value;
		items[selectedItem]->state = JDisplayEditableItem::EDITING;
		items[selectedItem]->newValue = items[selectedItem]->value;
		items[selectedItem]->update();
		re.limMin = -10000;
		re.limMax = +10000;
		re.wrap = false;
		re.value = 0;
	} else { 
		editing = false;
		if (longpress == false) 
			items[selectedItem]->value = items[selectedItem]->newValue;
		items[selectedItem]->state = JDisplayEditableItem::SELECTED;
		re.limMin = 0;
		re.limMax = items.size() - 1;
		re.wrap = false;
		re.value = selectedItem;
	}
	items[selectedItem]->update();
}


class MyEditor : public JDisplayEditor {
public:
	JDisplayEditableItem pidp = JDisplayEditableItem(Display::pidp, .01);
	JDisplayEditableItem pidi = JDisplayEditableItem(Display::pidi, .01);
	JDisplayEditableItem pidd = JDisplayEditableItem(Display::pidd, .01);
	JDisplayEditableItem pidg = JDisplayEditableItem(Display::pidg, .1);
	JDisplayEditableItem srvt = JDisplayEditableItem(Display::srvt, 1);
	
	MyEditor() : JDisplayEditor(33, 27) {
		add(&pidp);	
		add(&pidi);	
		add(&pidd);	
		add(&pidg);	
		add(&srvt);
	}
} ed;

void setup() {
	pinMode(32, INPUT);
	pinMode(26, OUTPUT);
	Serial1.begin(57600, SERIAL_8N1, 32, 26);
	Serial1.setTimeout(1);
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(1);

	Display::jd.begin();
	//wdt.begin();  // doesn't work yet

	pinMode(LED_PIN, OUTPUT);
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
		open_TTGOTS_SD();
		printSD();

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

	pid.setGains(7.52, 0, 0.11);
	pid.finalGain = 16.8;

	ed.begin();
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
	ed.pidp.value = pid.gain.p;
	ed.pidi.value = pid.gain.i;
	ed.pidd.value = pid.gain.d;
	ed.pidg.value = pid.finalGain;
	ed.srvt.value = servoTrim;
	
	pinMode(0, OUTPUT);
	ledcSetup(1, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(0, 1);   // GPIO 0 assigned to channel 1
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
	static int desiredTrk = 180;
	static int apMode = 1;
	static int gpsUseGDL90 = 1;
	static int obs = 0;
	static int navDTK = 0;
	static bool phSafetySwitch = true;
	static bool screenEnabled = true;
	static uint64_t lastLoop = micros();
	static int armServo = 0;
	static RollingAverage<int,1000> loopTime;
	static EggTimer serialReportTimer(500);
	static bool selEditing = false;
	static int pwmOutput = 0, servoOutput = 0;
	static float roll = 0;
	static String logFilename("none");

	delayMicroseconds(10);
	uint64_t now = micros();
	loopTime.add(now - lastLoop);
	lastLoop = now;
	if (serialReportTimer.tick()) { 
		Serial.printf("roll %+05.1f servo %04d Loop time min/avg/max %d/%d/%d\n", roll, servoOutput, loopTime.min(), loopTime.average(), loopTime.max());
	}
	
	//Serial.printf("%d\n", (int)(micros() - lastLoop));
	//lastLoop = micros();
	//printMag();
	

	buttonISR();
	if (butFilt.newEvent()) {
		/*
		if (butFilt.wasLong == true) { // long press, hold current track
			re.value = lastHdg;
			apMode = 3;
		}
		if (butFilt.wasLong == false) { // short press, set new mode
			apMode = butFilt.wasCount;
		}*/
		screenEnabled = true;
		Display::jd.begin();
		Display::jd.forceUpdate();
	}
	if (butFilt2.newEvent()) { 
		if (butFilt2.wasLong) {
			if (butFilt2.wasCount == 2)  {
				mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 4/*param5*/, 0, 0);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				mavlink_send(buf, len);
			}	
		} else { 
			apMode = apMode == 4 ? 1 : 4;
		}
		mavTimer.alarmNow();
	}
	if (butFilt3.newEvent()) {
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
	
	// main knob button
	if (butFilt4.newEvent()) {
		if (butFilt4.wasCount == 1) { 
			ed.buttonPress(butFilt4.wasLong);
		}
		if (butFilt4.wasLong && butFilt4.wasCount == 2) {
			armServo = !armServo;
			if (armServo)
				pid.reset();
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
		logFile = new SDCardBufferedLog<LogItem>("AHRSA%03d.DAT", 200/*q size*/, 100/*timeout*/, 1000/*flushInterval*/, false/*textMode*/);
		logFilename = logFile->currentFile;
		Serial.printf("Opened log file '%s'\n", logFile->currentFile.c_str());
	} 
	if (phSafetySwitch == true && logFile != NULL) {
		Serial.printf("Closing log file '%s'\n", logFile->currentFile.c_str());
		delete logFile;
		logFile = NULL;
		printSD();
	}

	
	if (imuRead()) {
		roll = ahrs.add(ahrsInput);
		pid.add(roll, millis()/1000.0);
		if (armServo) {  
			servoOutput = servoTrim + pid.corr;
			pwmOutput = max(1500, min(7300, servoOutput * 4915 / 1500));
			//Serial.printf("%05.2f %04d %04d\n", pid.corr, servoOutput, pwmOutput);
		} else {
			pwmOutput = 0;
		}
		ledcWrite(1, pwmOutput); // scale PWM output to 1500-7300 
		logItem.pidP = pid.err.p;
		logItem.pidI = pid.err.i;
		logItem.pidD = pid.err.d;
		logItem.finalGain = pid.finalGain;
		logItem.gainP = pid.gain.p;
		logItem.gainI = pid.gain.i;
		logItem.gainD = pid.gain.d;
		logItem.pwmOutput = pwmOutput;
		logItem.servoTrim = servoTrim;
		if (logFile != NULL) {
			sdLog();
		}
	}


	if (screenEnabled) { 
		ed.update();
		pid.gain.p = ed.pidp.value;
		pid.gain.i = ed.pidi.value;
		pid.gain.d = ed.pidd.value;
		pid.finalGain = ed.pidg.value;
		servoTrim = ed.srvt.value;
	}
	if (screenEnabled && screenTimer.tick()) {
		Display::mode.color.vb = (apMode == 4) ? ST7735_RED : ST7735_GREEN;
		Display::mode.color.vf = ST7735_BLACK;
		Display::roll.color.vf = ST7735_RED;

		Display::ip = WiFi.localIP().toString().c_str(); 
		Display::dtk = (int)desiredTrk; 
		Display::hdg = (int)ahrsInput.hdg; 
		Display::navt = navDTK; Display::obs = obs; 
		Display::knob = ed.re.value; 
		Display::mode = armServo * 100 + gpsUseGDL90 * 10 + (int)phSafetySwitch; Display::udp = udpBytes % 1000; 
		Display::ser = serBytes % 1000; 
		Display::mav = mavHeartbeats % 1000; 
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
		mavBytesIn += n + random(0,2);
		if (n > 0) {
			Serial1.write(buf, n);
			avail -= n;
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
/*			int new_mode = 7; // 7 = CRUISE
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

			if (apMode == 3) 
				desiredTrk = (int)re.value;
			if (apMode == 4) 
				desiredTrk = (int)(obs + 15.5  + 360) % 360;
			if (apMode == 5) 
				desiredTrk = navDTK;
*/
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
					mav_gps_msg(s.lat, s.lon, s.track, s.hvel * 0.51444, s.alt, 1.23, 2.34);
					ahrsInput.alt = s.alt;
					ahrsInput.hdg = s.track;
					ahrsInput.gspeed = s.hvel;
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

void mavlink_open() {}
void mavlink_send(const uint8_t *buf, int len) { Serial1.write(buf, len); }


// TODO:
// add min/max/avg loop cycle time monitoring
// 
