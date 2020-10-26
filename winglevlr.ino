
/*
 * TTGO TS 1.4 PINOUTS
 * 
 * 	VN			VP
 * 	33 (PWM)	RST
 * 	27 			32  (KNOB)
 * 	GND(GND)	26  (ROT)
 * 	0			GND 		
 * 	GND			3.3V
 * 	RXD			21  (ROT)
 * 	TXD			22	
 * 	VBAT		5V
 * 
 * 
 * 
 * 
 */





#ifndef UBUNTU
#include <HardwareSerial.h>
#include "SPI.h"
#include "Update.h"
#include "WebServer.h"
#include "DNSServer.h"
#include "FS.h"
#include "ESPmDNS.h"
#include "ArduinoOTA.h"
//#include "WiFiManager.h"
#include "WiFiUdp.h"
#include "WiFiMulti.h"
#include <MPU9250_asukiaaa.h>
#include <SparkFunMPU9250-DMP.h>
#include <RunningLeastSquares.h>
#include <mySD.h>
#include "Wire.h"
#include <MPU9250_asukiaaa.h>
#if defined(ESP32)
//#include "esp_system.h"
#include <esp_task_wdt.h>
#endif
#else
#include "ESP32sim_ubuntu.h"
#endif




#include "jimlib.h"
#include "PidControl.h"
#include "TinyGPS++.h"
#include "G90Parser.h"
#include "RollAHRS.h"

WiFiMulti wifi;

TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);
TinyGPSCustom xte(gps, "GPRMB", 2);
TinyGPSCustom xteLR(gps, "GPRMB", 3);
TinyGPSCustom vtgCourse(gps, "GPVTG", 1);

GDL90Parser gdl90;
GDL90Parser::State state;

RollAHRS ahrs;
PidControl rollPID(30) /*200Hz*/, pitchPID(10,6), navPID(50); /*20Hz*/
PidControl *knobPID = &pitchPID;
static int servoTrim = 1325;

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
WiFiUDP udpMAV;

#define LED_PIN 22
DigitalButton button3(39); // top
DigitalButton button(34); // middle
DigitalButton button2(35); // bottom
DigitalButton button4(32); // knob press

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
	JDisplayItem<float> navt(&jd,10,y+=10,"NAVT:", "%05.1f ");    JDisplayItem<float>    obs(&jd,70,y,    " OBS:", "%05.1f ");
	JDisplayItem<float>  rmc(&jd,10,y+=10," RMC:", "%05.1f");    JDisplayItem<int>   mode(&jd,70,y,    "MODE:", "%03d ");
	JDisplayItem<float>  gdl(&jd,10,y+=10," GDL:", "%05.1f ");  JDisplayItem<float>  vtg(&jd,70,y,    " VTG:", "%05.1f ");
	JDisplayItem<float> pitc(&jd,10,y+=10,"PITC:", "%+05.1f "); JDisplayItem<float> roll(&jd,70,y,    " RLL:", "%+05.1f ");
	JDisplayItem<const char *>  log(&jd,10,y+=10," LOG:", "%s  ");

    //JDisplayItem<float> pidc(&jd,10,y+=20,"PIDC:", "%05.1f ");JDisplayItem<int>   serv(&jd,70,y,    "SERV:", "%04d ");
	
	JDisplayItem<float> pidp(&jd,10,y+=10,"   P:", "%05.2f "); JDisplayItem<float> ttsc(&jd,70,y,    "TTSC:", "%04.0f ");
	JDisplayItem<float> pidi(&jd,10,y+=10,"   I:", "%05.3f "); JDisplayItem<float> ttde(&jd,70,y,    "TTDE:", "%04.0f ");;
	JDisplayItem<float> pidd(&jd,10,y+=10,"   D:", "%04.2f "); JDisplayItem<float> maxb(&jd,70,y,    "MAXB:", "%04.1f ");
	JDisplayItem<float> pidl(&jd,10,y+=10,"   L:", "%04.2f "); JDisplayItem<float> mtin(&jd,70,y,    "MTIN:", "%03.1f ");
}

void ESP32sim_JDisplay_forceUpdate() { 
	Display::jd.forceUpdate();
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
		//x.dtk = imu.calcQuat(imu.qw);
		//x.q2 = imu.calcQuat(imu.qx);
		//x.q3x = imu.calcQuat(imu.qy);
		//x.q4 = imu.calcQuat(imu.qz);
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
const char *logFileName = "AHRSD%03d.DAT";


#ifdef UBUNTU
void ESP32sim_setLogFile(const char *p) { logFileName = p; } 
#endif


void sdLog()  {
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
	JDisplayEditableItem pidp = JDisplayEditableItem(&Display::pidp, .01);
	JDisplayEditableItem pidi = JDisplayEditableItem(&Display::pidi, .001);
	JDisplayEditableItem pidd = JDisplayEditableItem(&Display::pidd, .01);
	JDisplayEditableItem pidl = JDisplayEditableItem(&Display::pidl, .01);
	JDisplayEditableItem maxb = JDisplayEditableItem(&Display::maxb, .1);
	JDisplayEditableItem ttsc = JDisplayEditableItem(&Display::ttsc, 1);
	JDisplayEditableItem ttde = JDisplayEditableItem(&Display::ttde, 1);
	JDisplayEditableItem tzer = JDisplayEditableItem(NULL, 1);;
	JDisplayEditableItem pidg = JDisplayEditableItem(NULL, .1);
	JDisplayEditableItem mtin = JDisplayEditableItem(&Display::mtin, .1);
	
	MyEditor() : JDisplayEditor(26, 21) { // add in correct knob selection order
		add(&pidp);	
		add(&pidi);	
		add(&pidd);	
		add(&pidl);	
		add(&ttsc);
		add(&ttde);	
		add(&maxb);
		add(&mtin);
	}
} ed;

#if 0 
#include "esp_freertos_hooks.h"
bool bApplicationIdleHook() {
	return true;
}
#endif

void setup() {	
	esp_task_wdt_init(15, true);
	esp_err_t err = esp_task_wdt_add(NULL);

	//esp_register_freertos_idle_hook(bApplicationIdleHook);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, 1);
	pinMode(27, OUTPUT);
	digitalWrite(27, 1);

//	pinMode(32, INPUT);
//	pinMode(26, OUTPUT);
//	Serial1.begin(57600, SERIAL_8N1, 32, 26);
//	Serial1.setTimeout(1);
	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(1);

	Display::jd.begin();
	Display::jd.clear();
	
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
	
	if (true || digitalRead(button4.pin) != 0) { // skip long setup stuff if we're debugging
		WiFi.disconnect(true);
		WiFi.mode(WIFI_STA);
		WiFi.setSleep(false);

		wifi.addAP("Ping-582B", "");
		wifi.addAP("Flora_2GEXT", "maynards");
		wifi.addAP("Team America", "51a52b5354");
		wifi.addAP("ChloeNet", "niftyprairie7");
		wifi.addAP("TUK-PUBLIC", "");

		uint64_t startms = millis();
		while (WiFi.status() != WL_CONNECTED /*&& digitalRead(button.pin) != 0*/) {
			wifi.run();
			delay(10);
		}
		
		udpSL30.begin(7891);
		udpG90.begin(4000);
		udpNMEA.begin(7892);
	}
	
	//SCREENLINE.println("WiFi connected");

	rollPID.setGains(7.52, 0.05, 0.11);
	rollPID.finalGain = 16.8;
	rollPID.maxerr.i = 20;
	navPID.setGains(0.5, 0.00, 0.1);
	navPID.maxerr.i = 20;
	navPID.finalGain = 2.2;
	pitchPID.setGains(20.0, 0.0, 2.0, 0, .8);
	pitchPID.finalGain = 0.2;
	pitchPID.maxerr.i = .5;

	ed.begin();
#ifndef UBUNTU
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
#endif
	ed.pidp.value = knobPID->gain.p;
	ed.pidi.value = knobPID->gain.i;
	ed.pidd.value = knobPID->gain.d;
	ed.pidl.value = knobPID->gain.l;
	ed.pidg.value = knobPID->finalGain;
	ed.maxb.value = 12;
	ed.ttsc.value = 45; // seconds to make each test turn 
	ed.ttde.value = 40; // degrees of each test turn 
	ed.tzer.value = 1000;
	ed.mtin.value = 10;
	
	//ed.rlhz.value = 3; // period for relay activation, in seconds
	//ed.mnrl.value = 70;
	//ed.pmin.value = 0.5; // PID total error that triggers relay minimum actuation
	//ed.pmax.value = 2.5; // PID total error that triggers relay maximum actuation 
	pinMode(33, OUTPUT);
	ledcSetup(1, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(33, 1);   // GPIO 33 assigned to channel 1

	ArduinoOTA.begin();
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
 
static StaleData<float> gpsTrackGDL90(3000,-1), gpsTrackRMC(6000,-1), gpsTrackVTG(5000,-1);
static float desiredTrk = -1;
float desRoll = 0;		

#ifdef UBUNTU

void ESP32sim_set_gpsTrackGDL90(float v) { 
	gpsTrackGDL90 = v;
	ahrsInput.g5Hdg = v;
}

/*void ESP32sim_set_g5(float p, float r, float h) { 
	ahrsInput.g5Hdg = h;
	ahrsInput.g5Pitch = p;
	ahrsInput.g5Roll = r;
}
*/

void ESP32sim_set_desiredTrk(float v) {
	desiredTrk = v;
}


bool ESP32sim_replayLogItem(ifstream &i) {
	LogItem l; 
	static uint64_t logfileMicrosOffset = 0;
	
	if (i.read((char *)&l, sizeof(l))) {
		if (logfileMicrosOffset == 0) 
			logfileMicrosOffset = (l.ai.sec * 1000000 - _micros);
		_micros = l.ai.sec * 1000000 - logfileMicrosOffset;
		imu.ax = l.ai.ax;
		imu.ay = l.ai.ay;
		imu.az = l.ai.az;
		imu.gx = l.ai.gx;
		imu.gy = l.ai.gy;
		imu.gz = l.ai.gz;
		imu.mx = l.ai.mx;
		imu.my = l.ai.my;
		imu.mz = l.ai.mz;
		
		ahrsInput = l.ai;
		//g5.hdg = l.ai.g5Hdg * M_PI / 180;
		//g5.roll = l.ai.g5Roll * M_PI / 180;
		//g5.pitch = l.ai.g5Pitch * M_PI / 180;
		return true;
	} 
	return false;
}
#endif

static int serialLogFlags = 0;


void udpSendString(const char *b) { 
	for (int repeat = 0; repeat < 3; repeat++) { 
		udpG90.beginPacket("255.255.255.255", 7892);
		udpG90.write((uint8_t *)b, strlen(b));
		udpG90.endPacket();
		for (int n = 100; n < 103; n++) { 
			char ip[32];
			snprintf(ip, sizeof(ip), "192.168.4.%d", n);
			udpG90.beginPacket(ip, 7892);
			udpG90.write((const uint8_t *)b, strlen(b));
			udpG90.endPacket();
		}
	}
}
	
void pitchTrimSet(float p) { 
	char l[60];
	static int seq = 5;
	snprintf(l, sizeof(l), "trim %f %d\n", p, seq++);
	udpSendString(l);
	logItem.pitchCmd = p;
}

void pitchTrimRelay(int relay, int ms) { 
	char l[60];
	static int seq = 5;
	logItem.flags |= ((1 << relay) | (ms << 8));
	serialLogFlags |= ((1 << relay) | (ms << 8));
	snprintf(l, sizeof(l), "pin %d 0 %d %d\n", relay, ms, seq++);
	udpSendString(l);
}

	
static int servoOverride = 0, pitchTrimOverride = -1;
static bool testTurnActive = false;
static bool testTurnAlternate = false;
static float testTurnLastTurnTime = 0;
static RollingAverage<float,5> crossTrackError;
static RollingAverage<float,10> windCorrectionAngle;

Windup360 currentHdg;
ChangeTimer g5HdgChangeTimer;

void loop() {
	uint16_t len;
	static int ledOn = 0;
	static int manualRelayMs = 60;
	static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
	static int buildNumber = 12;
	static int mavBytesIn = 0;
	static char lastParam[64];
	static int lastHdg;
	static int apMode = 1; // apMode == 4 means follow NMEA HDG and XTE sentences, anything else tracks OBS
	static int hdgSelect = 0; // 0- use g5 hdg, 1 use g5 track, 2 use GDL90 data 
	static float obs = -1, lastObs = -1;
	static float navDTK = -1;
	static bool logActive = false;
	static bool screenEnabled = true;
	static uint64_t lastLoop = micros();
	static int armServo = 0;
	static TwoStageRollingAverage<int,40,40> loopTime;
	static EggTimer serialReportTimer(200), navPIDTimer(50), buttonCheckTimer(10);
	static bool selEditing = false;
	static int pwmOutput = 0, servoOutput = 0;
	static float roll = 0, pitch = 0;
	static String logFilename("none");
	static AhrsInput lastAhrsInput, lastAhrsGoodG5; 
	
	esp_task_wdt_reset();
	ArduinoOTA.handle();

	if (0) {  // debugging memory leak from xTaskCreate/vTaskDelete in log implementation
		static EggTimer t(20000);
		if (t.tick()) {
			static int logtestc = 0;
			if (logtestc++ % 2 == 0) 
				logActive = true;
			else
				logActive = false;
		}
	}
	
	//vTaskDelay(1);
	delayMicroseconds(10);
	//yield();
	
	uint64_t now = micros();
	double nowSec = millis() / 1000.0;
	
	loopTime.add(now - lastLoop);
	lastLoop = now;
	PidControl *pid = &rollPID;
	if (serialReportTimer.tick()) { 
		Serial.printf("%06.3f R %+05.2f P %+05.2f g5 %+05.2f %+05.2f mDip %+05.2f %+05.2f %+05.2f %+05.2f %+05.1f %+05.1f pcmd %06.1f srv %04d xte %3.2f but %d%d%d%d loop %d/%d/%d heap %d\n", 
			millis()/1000.0, roll, pitch, ahrsInput.g5Roll, ahrsInput.g5Pitch, ahrs.magXFit.slope(), ahrs.magYFit.slope(), ahrs.magZFit.slope(), ahrs.magStability, 0.0, 0.0, logItem.pitchCmd, servoOutput, 
			crossTrackError.average()
			,digitalRead(button.pin), digitalRead(button2.pin), digitalRead(button3.pin), digitalRead(button4.pin), 
			(int)loopTime.min(), (int)loopTime.average(), (int)loopTime.max(), ESP.getFreeHeap()
		);
		serialLogFlags = 0;
	}

	if (buttonCheckTimer.tick()) { 
		//printMag(); 
		buttonISR();
		if (butFilt.newEvent()) { // MIDDLE BUTTON
			if (!butFilt.wasLong) {
				if (butFilt.wasCount == 1) {
					hdgSelect = (hdgSelect + 1) % 3;
				} else { 
					screenEnabled = true;
					Display::jd.begin();
					Display::jd.forceUpdate();
				}
					
			} else { 
				testTurnActive = !testTurnActive;
				testTurnAlternate = false;
			}
			
		}
		if (butFilt2.newEvent()) { // BOTTOM or LEFT button
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == true) {	// LONG: zero AHRS sensors
				ahrs.zeroSensors();
			}
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == false) {	// SHORT: arm servo
				armServo = !armServo; 
				rollPID.reset();
				navPID.reset();
				pitchPID.reset();
				ahrs.reset();
			}
		}
		if (butFilt3.newEvent()) { // TOP or RIGHT button 
			if (butFilt3.wasCount == 1 && butFilt3.wasLong == false) {		// SHORT: Stop tracking NMEA dest, toggle desired track between -1/current heading
				apMode = 1;
				if (desiredTrk == -1) 
					desiredTrk = ahrsInput.gpsTrack;
				else 
					desiredTrk = -1;
			}
			if (butFilt3.wasCount == 1 && butFilt3.wasLong == true) {		// LONG: stop/start logging
				logActive = !logActive;
				if (logActive == true) {
					screenEnabled = false;
					Display::jd.clear();
				} else {
					screenEnabled = true;
					Display::jd.begin();
					Display::jd.forceUpdate();
				}
			}
		}
		if (butFilt4.newEvent()) { 	// main knob button
			if (butFilt4.wasCount == 1 && butFilt4.wasLong != true) { 
				ed.buttonPress(butFilt4.wasLong);
			}
			if (butFilt4.wasLong && butFilt4.wasCount == 1) {			// LONG: arm servos
				armServo = !armServo; 
				rollPID.reset();
				navPID.reset();
				pitchPID.reset();
				ahrs.reset();
			}
			if (butFilt4.wasLong && butFilt4.wasCount == 2) {
				ed.negateSelectedValue();
			}
		}
	}
	
	if (testTurnActive && nowSec - testTurnLastTurnTime > (testTurnAlternate ? ed.ttsc.value * 2 : ed.ttsc.value + 10)) {
		testTurnLastTurnTime = nowSec;
		const int deg = ed.ttde.value;
		desiredTrk += testTurnAlternate ? -deg : deg * 2;
		testTurnAlternate = !testTurnAlternate;
		if (desiredTrk <= 0)
			desiredTrk += 360;
		if (desiredTrk > 360) 
			desiredTrk -= 360;
	}
	
	
	if (logActive == true && logFile == NULL) {
		logFile = new SDCardBufferedLog<LogItem>(logFileName, 100/*q size*/, 100/*timeout*/, 1000/*flushInterval*/, false/*textMode*/);
		logFilename = logFile->currentFile;
		Serial.printf("Opened log file '%s'\n", logFile->currentFile.c_str());
	} 
	if (logActive == false && logFile != NULL) {
		Serial.printf("Closing log file '%s'\n", logFile->currentFile.c_str());
		delete logFile;
		logFile = NULL;
		Serial.printf("Closed\n");
	}

	ahrsInput.gpsTrackGDL90 = gpsTrackGDL90;
	ahrsInput.gpsTrackVTG = gpsTrackVTG;
	ahrsInput.gpsTrackRMC = gpsTrackRMC;
	ahrsInput.dtk = desiredTrk;

	if (imuRead()) {
		roll = ahrs.add(ahrsInput);
		pitch = ahrs.pitchCompDriftCorrected;

		if (hdgSelect == 0) { // hybrid G5/GDL90 data 
			if (g5HdgChangeTimer.unchanged(ahrsInput.g5Hdg) < 2.0) { // use g5 data if it's not stale 
				ahrsInput.gpsTrack = ahrsInput.g5Hdg;
				lastAhrsGoodG5 = ahrsInput;
			} else if (ahrsInput.gpsTrackGDL90 != -1) { // otherwise use change in GDL90 data 
				ahrsInput.gpsTrack = lastAhrsGoodG5.gpsTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90); 
			} else if (ahrsInput.gpsTrackRMC != -1) { // otherwise use change in VTG data 
				ahrsInput.gpsTrack = lastAhrsGoodG5.gpsTrack + angularDiff(ahrsInput.gpsTrackRMC - lastAhrsInput.gpsTrackRMC); 
			} else { // otherwise, no available heading/track data 
				ahrsInput.gpsTrack = -1;
			}
		}
		else if (hdgSelect == 1) ahrsInput.gpsTrack = ahrsInput.g5Track;
		else if (hdgSelect == 2) ahrsInput.gpsTrack = gpsTrackGDL90;
		
		if (floor(ahrsInput.sec / 0.05) != floor(lastAhrsInput.sec / 0.05)) { // 20HZ
			float pset = 0;
			float pCmd = pitchPID.add(ahrs.pitchCompDriftCorrected - pset, ahrs.pitchCompDriftCorrected, ahrsInput.sec);
			float trimCmd = ed.tzer.value - pCmd;
			if (pitchTrimOverride != -1) {
				trimCmd = pitchTrimOverride;
			}
			if (armServo == false) { 
				trimCmd = -1;
			}
			pitchTrimSet(trimCmd); 
			logItem.pitchCmd = trimCmd;
		}

		pwmOutput = 0;
		if (1 /*ahrs.valid() || digitalRead(button4.pin) == 0 || servoOverride > 0*/) { // hold down button to override and make servo work  
			if (ahrsInput.dtk != -1) {
				float xteCorrection = 0;
				if (apMode == 4) {
					xteCorrection = max(-20.0, min(20.0, crossTrackError.average() * -50.0));
				} 
				double hdgErr = 0;
				if (ahrs.valid() != false && ahrsInput.gpsTrack != -1) {
					hdgErr = angularDiff(ahrsInput.gpsTrack - ahrsInput.dtk + xteCorrection);
					currentHdg = ahrsInput.gpsTrack;
				}
				if (navPIDTimer.tick()) {
					desRoll = -navPID.add(hdgErr, currentHdg, ahrsInput.sec);
					desRoll = max(-ed.maxb.value, min(+ed.maxb.value, desRoll));
				}
			}
			if (ahrs.valid() == false || ahrsInput.dtk == -1 || ahrsInput.gpsTrack == -1) {
				desRoll = 0.0; // TODO: this breaks roll commands received over the serial bus, add rollOverride variable or something 
			}

			rollPID.add(roll - desRoll, roll, ahrsInput.sec);
			//Serial.printf("%05.2f %05.2f %04d\n", desRoll, roll);
			if (armServo) {  
				servoOutput = servoTrim + rollPID.corr;
				if (servoOverride > 0)
					servoOutput = servoOverride;
				servoOutput = max(550, min(2100, servoOutput));
				pwmOutput = servoOutput * 4915 / 1500;
			}			
		}
		
		ledcWrite(1, pwmOutput); // scale PWM output to 1500-7300 
		logItem.pidP = pitchPID.err.p;
		logItem.pidI = pitchPID.err.i;
		logItem.pidD = pitchPID.err.d;
		logItem.finalGain = pitchPID.finalGain;
		logItem.gainP = pitchPID.gain.p;
		logItem.gainI = pitchPID.gain.i;
		logItem.gainD = pitchPID.gain.d;
		logItem.pwmOutput = pwmOutput;
		logItem.desRoll = desRoll;
		logItem.roll = roll;
		logItem.ai = ahrsInput;
		logItem.ai.q3 = 
			//ahrs.magStability; 
			//ahrs.bankAngle; 
			ahrs.lastGz;
			//ahrs.gyrZOffsetFit.average();
			//-ahrs.zeroAverages.gz.average();
			0;
			
#ifdef UBUNTU
		cout << logItem.toString().c_str() << " " << 
/*44*/	ahrs.compYH <<" "<< servoOutput <<" "<< ahrs.pitchCompDriftCorrected <<" "<< ahrs.gpsPitch  <<" "<<  ahrs.magHdg << " " << 0 <<" "<< 
/*49*/  ahrs.pitchDrift <<" "<< ahrs.accelPitch <<" "<< ahrs.gyroTurnBank <<" "<< ahrs.pG <<" "<<
		"LOG" << endl;
#endif
		if (logFile != NULL) {
			sdLog();
		}
		logItem.flags = 0;
		lastAhrsInput = ahrsInput;
	}

	if (screenEnabled) { 
		ed.update();
		knobPID->gain.p = ed.pidp.value;
		knobPID->gain.i = ed.pidi.value;
		knobPID->gain.d = ed.pidd.value;
		knobPID->gain.l = ed.pidl.value;
		knobPID->finalGain = ed.pidg.value;
	}
	
	if (screenEnabled && screenTimer.tick()) {
		Display::mode.color.vb = (apMode == 4) ? ST7735_RED : ST7735_GREEN;
		Display::mode.color.vf = ST7735_BLACK;
		//Display::roll.color.vf = ST7735_RED;
		//Display::pitc.color.vf = ST7735_RED;

		Display::ip = WiFi.localIP().toString().c_str(); 
		Display::dtk = desiredTrk; 
		Display::trk = ahrsInput.gpsTrack; 
		Display::navt = navDTK; 
		Display::obs = obs; 
		Display::mode = apMode * 1000 + armServo * 100 + hdgSelect * 10 + (int)logActive; 
		Display::gdl = (float)gpsTrackGDL90;
		Display::vtg = (float)gpsTrackVTG;
		Display::rmc = (float)gpsTrackRMC; 
		Display::roll = roll; 
		Display::pitc = pitch; 
		Display::log = logFilename.c_str();
		Display::roll.setInverse(false, (logFile != NULL));
		
		//Display::pidc = pid.corr;
		//Display::serv = pwmOutput;
	}
	
	if (blinkTimer.tick()) 
		ledOn ^= 1;
	digitalWrite(LED_PIN, (ledOn & 0x1) ^ (gpsFixes & 0x1) ^ (serBytes & 0x1));
		
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
					if (s.track > 0 && ahrsInput.g5Hdg > 0) {
						windCorrectionAngle.add(angularDiff(s.track - ahrsInput.g5Hdg));
					}
					if (hdgSelect == 2) {
						gpsFixes++;
						ahrsInput.alt = s.alt;
						ahrsInput.palt = s.palt;
						ahrsInput.gspeed = s.hvel;
					}
				}
			}
		}
	}
	while(recsize > 0);

	if (Serial.available()) {
		static char line[32]; // TODO make a line parser class with a lambda/closure
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
				int relay, ms;
				if (sscanf(line, "navhi=%f", &f) == 1) { navPID.hiGain.p = f; }
				else if (sscanf(line, "navtr=%f", &f) == 1) { navPID.hiGainTrans.p = f; }
				else if (sscanf(line, "maxb=%f", &f) == 1) { ed.maxb.value = f; }
				else if (sscanf(line, "roll=%f", &f) == 1) { desRoll = f; }
				else if (sscanf(line, "pidp=%f", &f) == 1) { pitchPID.gain.p = f; }
				else if (sscanf(line, "pidi=%f", &f) == 1) { pitchPID.gain.i = f; }
				else if (sscanf(line, "pidd=%f", &f) == 1) { pitchPID.gain.d = f; }
				else if (sscanf(line, "pidl=%f", &f) == 1) { pitchPID.gain.l = f; }
				//else if (sscanf(line, "pitch=%f", &f) == 1) { ed.pset.value = f; }
				else if (sscanf(line, "ptrim=%f", &f) == 1) { ed.tzer.value = f; }
				else if (sscanf(line, "mtin=%f", &f) == 1) { ed.mtin.value = f; }
				else if (sscanf(line, "ptman=%f", &f) == 1) { pitchTrimOverride = f; }
				else if (strstr(line, "zeroimu") != NULL) { ahrs.zeroSensors(); }
				else if (sscanf(line, "dtrk=%f", &f) == 1) { desiredTrk = f; }
				else if (sscanf(line, "servo=%f", &f) == 1) { servoOverride = f; }
				else if (sscanf(line, "knob=%f", &f) == 1) {
					if (f == 1) {
						knobPID = &pitchPID;
					} else if (f == 2) { 
						knobPID = &rollPID;
					} else if (f == 3) { 
						knobPID = &navPID;
					}
				} else {
					Serial.printf("UNKNOWN COMMAND: %s", line);
				}
				// in case we changed the PID values or the knob selection 
				ed.pidp.value = knobPID->gain.p;
				ed.pidi.value = knobPID->gain.i;
				ed.pidd.value = knobPID->gain.d;
				ed.pidl.value = knobPID->gain.l;
				ed.pidg.value = knobPID->finalGain;

				//Serial.printf("PID %.2f %.2f %.2f pitch %.2f trim %.2f\n", pitchPID.gain.p, pitchPID.gain.i, pitchPID.gain.d, ed.pset.value, ed.tzer.value);
				//printMag();
			}
		}
	}

	int avail = udpSL30.parsePacket();
	while(avail > 0) { 
		static char line[200];
		static int index;
		
		int n = udpSL30.read(buf, sizeof(buf));
		if (n <= 0)
			break;
		buf[n] = '\0';
		//printf("READ: %s", buf);
		avail -= n;
		udpBytes += n; // + random(0,2);
		for (int i = 0; i < n; i++) {
			if (index >= sizeof(line) - 1)
				index = 0;
			if (buf[i] != '\r')
				line[index++] = buf[i];
			if (buf[i] == '\n' || buf[i] == '\r') {
				line[index++] = '\0';
				index = 0;
				float pit, roll, magHdg, magTrack, knobSel, knobVal, ias, tas, palt, age;
				int mode = 0;
				//Serial.printf("LINE %s\n", line);
				if (strstr(line, " CAN") != NULL && sscanf(line, "%f %f %f %f %f %f %f %f %f %f %d CAN", 
				&pit, &roll, &magHdg, &magTrack, &ias, &tas, &palt,  &knobSel, &knobVal, &age, &mode) == 11
					&& (pit > -2 && pit < 2) && (roll > -2 && roll < 2) && (magHdg > -7 && magHdg < 7) 
					&& (magTrack > -7 && magTrack < 7) && (knobSel >=0 && knobSel < 6)) {
					printf("CAN: %s", line);
					ahrsInput.g5Pitch = pit * 180 / M_PI;
					ahrsInput.g5Roll = roll * 180 / M_PI;
					ahrsInput.g5Hdg = magHdg * 180 / M_PI;
					ahrsInput.g5Track = magTrack * 180 / M_PI;
					ahrsInput.g5Palt = palt / 3.2808;
					ahrsInput.g5Ias = ias / 0.5144;
					ahrsInput.g5Tas = tas / 0.5144;
					ahrsInput.g5TimeStamp = (millis() - (uint64_t)age) / 1000.0;
					apMode = mode;
					if (knobSel == 1 || knobSel == 4) {
						obs = knobVal * 180.0 / M_PI;
						if (obs <= 0) obs += 360;
						if (apMode != 4 && obs != lastObs) {
							desiredTrk = obs;
							crossTrackError.reset();
						}
						lastObs = obs;
					}
				}
			}
			gps.encode(buf[i]);
			// Use only VTG course so as to only use G5 data
			if (vtgCourse.isUpdated()) {  
				gpsTrackVTG = gps.parseDecimal(vtgCourse.value()) * 0.01; //gps.course.deg();
			}
			if (gps.course.isUpdated()) { 
				gpsTrackRMC = gps.course.deg();
			}
			if (gps.location.isUpdated() && hdgSelect == 2) {
				ahrsInput.alt = gps.altitude.meters() * 3.2808;
				ahrsInput.gspeed = gps.speed.knots();
				gpsFixes++;
			}
			if (desiredHeading.isUpdated()) { 
				navDTK = 0.01 * gps.parseDecimal(desiredHeading.value()) - windCorrectionAngle.average();
				if (navDTK < 0)
					navDTK += 360;
				if (apMode == 4) 
					desiredTrk = navDTK;
			}
			if (xte.isUpdated() && xteLR.isUpdated()) {
				float err = 0.01 * gps.parseDecimal(xte.value());
				if (strcmp(xteLR.value(), "L") == 0)
					err *= -1;
				crossTrackError.add(err);
			}
		}
	}
}

float ESP32sim_getPitchCmd() { 
	return logItem.pitchCmd;
}
