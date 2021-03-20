
/*
 * TTGO TS 1.4 PINOUTS
 * 
 * 	VN			VP
 * 	33 (PWM)	RST
 * 	27 			32  (KNOB)
 * 	GND(GND)	26  (ROT)
 * 	0 (ROT)		GND 		
 * 	GND			3.3V
 * 	RXD			21  (used by I2c?)
 * 	TXD			22	(used by I2c?)
 * 	VBAT		5V
 * 
 * 
 * 
 * 
 */

#ifdef UBUNTU
#include "ESP32sim_ubuntu.h"
#else // #ifndef UBUNTU

#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Update.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <MPU9250_asukiaaa.h>
#include <mySD.h>
//#include <FS.h>
//#include <SPIFFS.h>

#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif // #else // UBUNTU

#include <TinyGPS++.h>

#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include "jimlib.h"
#include "RunningLeastSquares.h"
#include "PidControl.h"
#include "G90Parser.h"
#include "RollAHRS.h"

WiFiMulti wifi;

std::string strfmt(const char *, ...); 
//SPIFFSVariable<int> logFileNumber("/winglevlr.logFileNumber", 1);
int logFileNumber = 0;

TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);
TinyGPSCustom xte(gps, "GPRMB", 2);
TinyGPSCustom xteLR(gps, "GPRMB", 3);
TinyGPSCustom vtgCourse(gps, "GPVTG", 1);

GDL90Parser gdl90;
GDL90Parser::State state;

RollAHRS ahrs;
PidControl rollPID(30) /*200Hz*/, pitchPID(10,6), hdgPID(50)/*20Hz*/, xtePID(200); /*20Hz*/
PidControl *knobPID = &hdgPID;
static int servoTrim = 1325;

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
//WiFiUDP udpMAV;

#define LED_PIN 22
/* Old hardware pins: I2C pins/variant seems to determine layout 
struct {
	int topButton = 39;
	int midButton = 34;
	int botButton = 35;
	int knobButton = 32;
	int pwm = 33;
} pins;
*/

struct {
	int topButton = 39;
	int midButton = 37;
	int botButton = 36;
	int knobButton = 32;
	int pwm = 33;
} pins;



DigitalButton button3(39); // top
DigitalButton button(37); // middle
DigitalButton button2(36); // bottom
DigitalButton button4(32); // knob press

//static IPAddress mavRemoteIp;
//#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
//`static uint8_t buf[BUFFER_LENGTH];
EggTimer screenTimer(200);

LongShortFilter butFilt(1500,600);
LongShortFilter butFilt2(1500,600);
LongShortFilter butFilt3(1500,600);
LongShortFilter butFilt4(1500,600);

void setKnobPid(int f); 

void buttonISR() { 
	button.check();
	button2.check();
	button3.check();
	butFilt.check(button.duration());
	butFilt2.check(button2.duration());
	
	butFilt3.check(button3.duration());
	butFilt4.check(button4.duration());
}

namespace LogFlags {
	static const int g5Nav =  0x01;
	static const int g5Ins =  0x02;
	static const int g5Ps =   0x04;
	static const int HdgRMC = 0x08;
	static const int HdgGDL = 0x10;
}

namespace Display {
	JDisplay jd;
	int y = 0;
	const int c2x = 70;
	JDisplayItem<const char *>  ip(&jd,10,y,"WIFI:", "%s ");
	JDisplayItem<float>  dtk(&jd,10,y+=10," DTK:", "%05.1f ");  JDisplayItem<float>    trk(&jd,c2x,y,  " TRK:", "%05.1f ");
	JDisplayItem<float> navt(&jd,10,y+=10,"NAVT:", "%05.1f ");  JDisplayItem<float>    obs(&jd,c2x,y,  " OBS:", "%05.1f ");
	JDisplayItem<float> roll(&jd,10,y+=10,"ROLL:", "%+03.1f");   JDisplayItem<int>    mode(&jd,c2x,y,  "MODE:", "%05d ");
	JDisplayItem<float>  gdl(&jd,10,y+=10," GDL:", "%05.1f ");  JDisplayItem<float> maghdg(&jd,c2x,y,  " MAG:", "%05.1f ");
	//JDisplayItem<float> xtec(&jd,10,y+=10,"XTEC:", "%+05.1f "); JDisplayItem<float> roll(&jd,c2x,y,    " RLL:", "%+05.1f ");
	JDisplayItem<const char *> log(&jd,10,y+=10," LOG:", "%s-"); JDisplayItem<int>   drop(&jd,c2x+30,y,    "", "%03d ");
    //JDisplayItem<float> pidc(&jd,10,y+=20,"PIDC:", "%05.1f ");JDisplayItem<int>   serv(&jd,c2x,y,    "SERV:", "%04d ");
	
	JDisplayItem<float> pidpl(&jd,00,y+=20,"PL:", "%03.2f "); JDisplayItem<float> tttt(&jd,c2x,y,    " TT1:", "%04.1f ");
	JDisplayItem<float> pidph(&jd,00,y+=10,"PH:", "%03.2f "); JDisplayItem<float> ttlt(&jd,c2x,y,    " TT2:", "%04.1f ");;
	JDisplayItem<float>  pidi(&jd,00,y+=10," I:", "%03.2f "); JDisplayItem<float> maxb(&jd,c2x,y,    "MAXB:", "%04.1f ");
	JDisplayItem<float>  pidd(&jd,00,y+=10," D:", "%03.2f "); JDisplayItem<float> maxi(&jd,c2x,y,    "MAXI:", "%04.1f ");
	JDisplayItem<float>  pidg(&jd,00,y+=10," G:", "%03.2f "); 	
	JDisplayItem<float>  dead(&jd,00,y+=10,"DZ:", "%03.1f "); JDisplayItem<float> pidsel(&jd,c2x,y,  " PID:", "%1.0f");
}

class MyEditor : public JDisplayEditor {
public:
	JDisplayEditableItem pidpl = JDisplayEditableItem(&Display::pidpl, .01);
	JDisplayEditableItem pidph = JDisplayEditableItem(&Display::pidph, .01);
	JDisplayEditableItem pidi = JDisplayEditableItem(&Display::pidi, .001);
	JDisplayEditableItem pidd = JDisplayEditableItem(&Display::pidd, .01);
	JDisplayEditableItem pidg = JDisplayEditableItem(&Display::pidg, .1);
	JDisplayEditableItem pidl = JDisplayEditableItem(NULL, .1);
	JDisplayEditableItem maxb = JDisplayEditableItem(&Display::maxb, .1);
	JDisplayEditableItem maxi = JDisplayEditableItem(&Display::maxi, .1);
	JDisplayEditableItem tttt = JDisplayEditableItem(&Display::tttt, 1);
	JDisplayEditableItem ttlt = JDisplayEditableItem(&Display::ttlt, 1);
	JDisplayEditableItem tzer = JDisplayEditableItem(NULL, 1);
	JDisplayEditableItem pidsel = JDisplayEditableItem(&Display::pidsel, 1, 0, 3);
	JDisplayEditableItem dead = JDisplayEditableItem(&Display::dead, .1);
	
	MyEditor() : JDisplayEditor(26, 0) { // add in correct knob selection order
		add(&pidpl);	
		add(&pidph);	
		add(&pidi);	
		add(&pidd);	
		add(&pidg);	
		add(&dead);
		add(&tttt);
		add(&ttlt);	
		add(&maxb);
		add(&maxi);
		add(&pidsel);
	}
} ed;


//MPU9250_DMP imu;
MPU9250_asukiaaa imu(0x68);
#define IMU_INT_PIN 4

int scanI2c() { 
	int count = 0;
	for (byte i = 8; i < 120; i++)
	{
			Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
			if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
			{
					Serial.print ("Found address: ");
					Serial.print (i, DEC);
					Serial.print (" (0x");
					Serial.print (i, HEX);     // PCF8574 7 bit address
					Serial.println (")");
					count++;
			}
	}
	Serial.print ("Found ");      
	Serial.print (count, DEC);        // numbers of devices
	Serial.println (" device(s).");

	return count;
}

void imuLog(); 
void imuInit() { 
	Wire.begin(21,22);
	Serial.println("Scanning I2C bus on pins 21,22");
	if (scanI2c() == 0) { 
		Wire.begin(19,18);
		Serial.println("Scanning I2C bus on pins 19,18");
		scanI2c();
	}
	imu.setWire(&Wire);

	for(int addr = 0x68; addr <= 0x69; addr++) { 
		//imu.address = addr;
		uint8_t sensorId;
		Serial.printf("Checking MPU addr 0x%x: ", (int)imu.address);
		if (imu.readId(&sensorId) == 0) {
			Serial.printf("Found MPU sensor id: 0x%x\n", (int)sensorId);
			break;
		} else {
			Serial.println("Cannot read sensorId");
		}
	}

	//while(1) { delay(1000); printPins(); } 
	imu.beginAccel(ACC_FULL_SCALE_4_G);
	imu.beginGyro(GYRO_FULL_SCALE_250_DPS);
	imu.beginMag(MAG_MODE_CONTINUOUS_100HZ);
}

static AhrsInput ahrsInput;

bool imuRead() { 
	AhrsInput &x = ahrsInput;
	x.sec = millis() / 1000.0;
#ifdef USE_ACCEL
	imu.accelUpdate();
	x.ax = imu.accelX();
	x.ay = imu.accelY();
	x.az = imu.accelZ();
#endif
	imu.gyroUpdate();
	x.gx = imu.gyroX();
	x.gy = imu.gyroY();
	x.gz = imu.gyroZ();

	// limit magnometer update to 100Hz 
	static uint64_t lastUsec = 0;
	if (lastUsec / 10000 != micros() / 10000) { 
		imu.magUpdate();
		x.mx = imu.magX();
		x.my = imu.magY();
		x.mz = imu.magZ();
	}
	lastUsec = micros();
	// remaining items set (alt, hdg, speed) set by main loop
	return true;
}

LogItem logItem;
SDCardBufferedLog<LogItem>  *logFile = NULL;
bool logChanging = false;
const char *logFileName = "AHRSD%03d.DAT";

void sdLog()  {
	//Serial.println(x.toString());
	if (logFile != NULL)
		logFile->add(&logItem, 0/*timeout*/);
	logItem.flags = 0;
}

void printMag() {
      //imu.updateCompass();
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.gyroX(), (float)imu.gyroY(), (float)imu.gyroZ() );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.magX(), (float)imu.magY(), (float)imu.magZ() );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu.accelX(), (float)imu.accelY(), (float)imu.accelZ() );
      Serial.println("");
}

static AhrsInput lastAhrsInput, lastAhrsGoodG5; 

void setup() {	
	//SPIFFS.begin();
	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(1);
	Serial.printf("Reading log file number\n");
	int l = logFileNumber;
	Serial.printf("Log file number %d\n", l);
	logFileNumber = l + 1;

	esp_task_wdt_init(15, true);
	esp_err_t err = esp_task_wdt_add(NULL);

    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector   
 
	//esp_register_freertos_idle_hook(bApplicationIdleHook);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, 1);
	pinMode(27, OUTPUT);
	digitalWrite(27, 1);

	Display::jd.begin();
	Display::jd.clear();
	
	
	attachInterrupt(digitalPinToInterrupt(button.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button2.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button3.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button4.pin), buttonISR, CHANGE);
	
	imuInit();	
	
	WiFi.disconnect(true);
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);

	wifi.addAP("Ping-582B", "");
	wifi.addAP("Flora_2GEXT", "maynards");
	wifi.addAP("Team America", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");

	if (digitalRead(button4.pin) != 0 || digitalRead(button3.pin) != 0) { // skip long setup stuff if we're debugging
		uint64_t startms = millis();
		while (WiFi.status() != WL_CONNECTED /*&& digitalRead(button.pin) != 0*/) {
			wifi.run();
			delay(10);
		}
		
	}

	udpSL30.begin(7891);
	udpG90.begin(4000);
	udpNMEA.begin(7892);
	
	lastAhrsGoodG5.g5Hdg = 0;
	lastAhrsGoodG5.gpsTrackGDL90 = 17;
	lastAhrsGoodG5.gpsTrackRMC = 17;
	
	// Set up PID gains 
	rollPID.setGains(7.52, 0.05, 0.11);
	rollPID.hiGain.p = 1;
	rollPID.hiGainTrans.p = 5;
	rollPID.finalGain = 16.8;
	rollPID.maxerr.i = 20;

	hdgPID.setGains(0.25, 0.02, 0.02);
	hdgPID.hiGain.p = 0.50;
	hdgPID.hiGainTrans.p = 3.0;
	hdgPID.maxerr.i = 20;
	hdgPID.finalGain = 2.2;

	xtePID.setGains(8.0, 0.00, 0.05);
	xtePID.maxerr.i = 1.0;
	xtePID.finalGain = 10.0;
	
	pitchPID.setGains(20.0, 0.0, 2.0, 0, .8);
	pitchPID.finalGain = 0.2;
	pitchPID.maxerr.i = .5;

    // make PID select knob display text from array instead of 0-3	
	Display::pidsel.toString = [](float v){ return String((const char *[]){"HDG ", "ROLL", "XTE ", "PIT "}[(v >=0 && v <= 3) ? (int)v : 0]); };		
	ed.begin();

#ifndef UBUNTU
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
#endif
	ed.maxb.setValue(12);
	ed.tttt.setValue(20); // seconds to make each test turn 
	ed.ttlt.setValue(20); // seconds betweeen test turn  
	ed.tzer.setValue(1000);
	ed.pidsel.setValue(0);
	setKnobPid(ed.pidsel.value);
	ed.update();
	
	//ed.rlhz.value = 3; // period for relay activation, in seconds
	//ed.mnrl.value = 70;
	//ed.pmin.value = 0.5; // PID total error that triggers relay minimum actuation
	//ed.pmax.value = 2.5; // PID total error that triggers relay maximum actuation 
	pinMode(pins.pwm, OUTPUT);
	ledcSetup(1, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(pins.pwm, 1);   // GPIO 33 assigned to channel 1

	ArduinoOTA.begin();
}

 
static StaleData<float> gpsTrackGDL90(3000,-1), gpsTrackRMC(5000,-1), gpsTrackVTG(5000,-1);
static StaleData<int> canMsgCount(3000,-1);
static float desiredTrk = -1;
float desRoll = 0;		
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
	//logItem.pitchCmd = p;
}

void pitchTrimRelay(int relay, int ms) { 
	char l[60];
	static int seq = 5;
	logItem.flags |= ((1 << relay) | (ms << 8));
	serialLogFlags |= ((1 << relay) | (ms << 8));
	snprintf(l, sizeof(l), "pin %d 0 %d %d\n", relay, ms, seq++);
	udpSendString(l);
}

void setKnobPid(int f) { 
	Serial.printf("Knob PID %d\n", f);
	if      (f == 0) { knobPID = &hdgPID; }
	else if (f == 1) { knobPID = &rollPID; }
	else if (f == 2) { knobPID = &xtePID; }
	else if (f == 3) { knobPID = &pitchPID; }
	
	ed.pidpl.setValue(knobPID->gain.p);
	ed.pidph.setValue(knobPID->hiGain.p);
	ed.pidi.setValue(knobPID->gain.i);
	ed.pidd.setValue(knobPID->gain.d);
	ed.pidl.setValue(knobPID->gain.l);
	ed.pidg.setValue(knobPID->finalGain);
	ed.maxi.setValue(knobPID->maxerr.i);
	ed.dead.setValue(knobPID->hiGainTrans.p);
}	
	
static int servoOverride = 0, pitchTrimOverride = -1;
static bool testTurnActive = false;
static int testTurnAlternate = 0;
static float testTurnLastTurnTime = 0;
static RollingAverage<float,5> crossTrackError;
static float xteCorrection = 0;
static float magVar = +15.5; 
static float navDTK = -1;
static bool logActive = false;
static float roll = 0, pitch = 0;
static String logFilename("none");
static int pwmOutput = 0, servoOutput = 0;
static TwoStageRollingAverage<int,40,40> loopTime;
static EggTimer serialReportTimer(200), hdgPIDTimer(50), loopTimer(5), buttonCheckTimer(10);
static int armServo = 0;
static int apMode = 1; // apMode == 4 means follow NMEA HDG and XTE sentences, anything else tracks OBS
static int hdgSelect = 0; //  0 use GDL90 but switch to mode 1 on first can msg. 1- use g5 hdg, 2 use GDL90 data 
static float obs = -1, lastObs = -1;
static bool screenReset = false, screenEnabled = true;
double totalRollErr = 0.0, totalHdgError = 0.0;
//static int ledOn = 0;
static int manualRelayMs = 60;
static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
static uint64_t lastLoop = micros();
static bool selEditing = false;

Windup360 currentHdg;
ChangeTimer g5HdgChangeTimer;

void setObsKnob(float knobSel, float v) { 
	if (knobSel == 1 || knobSel == 4) {
		obs = v * 180.0 / M_PI;
		if (obs <= 0) obs += 360;
		if (apMode != 4 && obs != lastObs) {
			desiredTrk = obs;
			crossTrackError.reset();
			testTurnActive = false;
		}
		if (apMode == 4 && obs != lastObs) { 
			xtePID.reset();
		}
		lastObs = obs;
	}	
}


void loop() {	
	esp_task_wdt_reset();
	ArduinoOTA.handle();	
	delayMicroseconds(100);

#ifndef UBUNTU
	if (!loopTimer.tick())
		return;
#endif

	
	uint64_t now = micros();
	double nowSec = millis() / 1000.0;
	
	loopTime.add(now - lastLoop);
	lastLoop = now;
	PidControl *pid = &rollPID;
	if (true && serialReportTimer.tick()) {
		Serial.printf(
			"%06.3f "
			//"R %+05.2f BA %+05.2f GZA %+05.2f ZC %03d MFA %+05.2f"
			"R %+05.2f P %+05.2f g5 %+05.2f %+05.2f %+05.2f  "
			//"%+05.2f %+05.2f %+05.2f %+05.1f srv %04d xte %3.2f "
			"PID %+06.2f %+06.2f %+06.2f %+06.2f " 
			"but %d%d%d%d loop %d/%d/%d heap %d re.count %d logdrop %d maxwait %d\n", 
			millis()/1000.0,
			//roll, ahrs.bankAngle, ahrs.gyrZOffsetFit.average(), ahrs.zeroSampleCount, ahrs.magStabFit.average(),   
			roll, pitch, ahrsInput.g5Roll, ahrsInput.g5Pitch, ahrsInput.g5Hdg,
			//0.0, 0.0, 0.0, 0.0, servoOutput, crossTrackError.average(),
			knobPID->err.p, knobPID->err.i, knobPID->err.d, knobPID->corr, 
			digitalRead(button.pin), digitalRead(button2.pin), digitalRead(button3.pin), digitalRead(button4.pin), (int)loopTime.min(), (int)loopTime.average(), (int)loopTime.max(), ESP.getFreeHeap(), ed.re.count, logFile != NULL ? logFile->dropped : 0, logFile != NULL ? logFile->maxWaiting : 0,
			0
		);
		if (logFile != NULL) {
			logFile->maxWaiting =  0;
		}
		serialLogFlags = 0;
	}

	/////////////////////////////////////////////////////////////////////////////
	// KNOB/BUTTON INTERFACE
	// 
	// TOP:    short   - apMode = 1, toggle between wing level and hdg hold 
	//         long    - arm servo
	//         double  - zero sensors
	// MIDDLE: short   - left 10 degrees
	//         double  - hdg select mode
	//         long    - start/stop log
	// BOTTOM: short   - right 10 degrees
	//         long    - active test turn sequence 
	// KNOB    long    - arm servo
	//         triple  - servo test mode
	           
	//ed.re.check();
	if (buttonCheckTimer.tick()) { 
		buttonISR();
		if (butFilt3.newEvent()) { // TOP or RIGHT button 
			if (butFilt3.wasCount == 1 && butFilt3.wasLong == false) {		// SHORT: Stop tracking NMEA dest, toggle desired track between -1/current heading
				apMode = 1;
				if (desiredTrk == -1) 
					desiredTrk = ahrsInput.selTrack;
				else 
					desiredTrk = -1;
			}
			if (butFilt3.wasCount == 1 && butFilt3.wasLong == true) { 
				armServo = !armServo; 
			}
			if (butFilt3.wasCount == 3) {
				ahrs.zeroSensors();
			}
		}
		if (butFilt.newEvent()) { // MIDDLE BUTTON
			if (!butFilt.wasLong) {
				if (butFilt.wasCount == 1) {
					desiredTrk = angularDiff(desiredTrk - 10);
					apMode = 1;
				} else { 
					hdgSelect = (hdgSelect + 1) % 4;
				}
					
			} else { 
				if (!logChanging) {
					logChanging = true;
					if (logFile == NULL) {	
							logFile = new SDCardBufferedLog<LogItem>(logFileName, 800/*q size*/, 0/*timeout*/, 5000/*flushInterval*/, false/*textMode*/);
							logFilename = logFile->currentFile;
							logChanging = false;
					} else {
						delete logFile;
						logFile = NULL;
						logChanging = false;
					}
				}
			}
			
		}
		if (butFilt2.newEvent()) { // BOTTOM or LfffEFT button
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == true) {	// LONG: zero AHRS sensors
				testTurnActive = !testTurnActive;
				testTurnAlternate = 0;
			}
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == false) {	// SHORT: arm servo
				desiredTrk = angularDiff(desiredTrk + 10);
				apMode = 1;
			}
		}
								
		if (butFilt4.newEvent()) { 	// main knob button
			if (butFilt4.wasCount == 1 && butFilt4.wasLong != true) { 
				ed.buttonPress(butFilt4.wasLong);
			}
			if (butFilt4.wasLong && butFilt4.wasCount == 1) {			// LONG: arm servos
				armServo = !armServo; 
				//rollPID.reset();
				//hdgPID.reset();
				//pitchPID.reset();
				//ahrs.reset();
			}
			if (butFilt4.wasLong && butFilt4.wasCount == 2) {
				ed.negateSelectedValue();
			}
			if (butFilt4.wasCount == 3) {		
				ed.tttt.setValue(.1); 
				ed.ttlt.setValue(.1);
				armServo = testTurnActive = true;
				 
			}
		}
	}
	
	if (testTurnActive) {
		if (desiredTrk == -1) {
			// roll mode, test turns are fixed time at maxb degrees 
			if (nowSec - testTurnLastTurnTime > ed.tttt.value + ed.ttlt.value) { 
				testTurnLastTurnTime = nowSec;
				testTurnAlternate  = (testTurnAlternate + 1) % 3;
			}
			if (nowSec - testTurnLastTurnTime <= ed.tttt.value)
				desRoll = ed.maxb.value * (testTurnAlternate == 0 ? -1 : 1);
			else 
				desRoll = 0;
		} else {
			// hdg mode, test turn is fixed number of degrees 
			if (nowSec - testTurnLastTurnTime > ed.tttt.value) { 
				testTurnLastTurnTime = nowSec;
				testTurnAlternate  = (testTurnAlternate + 1) % 3;
				desiredTrk = desiredTrk + ed.ttlt.value * (testTurnAlternate == 0 ? -1 : 1);
			}
		}
			
	}

	int recsize = 0;
	do {
		recsize = 0;
		int avail = udpG90.parsePacket();
		while(avail > 0) {
			unsigned char buf[1024]; 
			recsize = udpG90.read(buf, min(avail,(int)sizeof(buf)));
			if (recsize <= 0)
				break;
			avail -= recsize;
			udpBytes += recsize; //+ random(0,2);
			for (int i = 0; i < recsize; i++) {  
				gdl90.add(buf[i]);
				GDL90Parser::State s = gdl90.getState();
				if (s.valid && s.updated) {
					gpsTrackGDL90 = constrain360(s.track - magVar);
					ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.03);
					logItem.flags |= LogFlags::HdgGDL; 
					if (hdgSelect == 2) {
						gpsFixes++;
						ahrsInput.alt = s.alt;
						ahrsInput.palt = s.palt;
						ahrsInput.gspeed = s.hvel;
					}
				}
			}
		}
	} while(recsize > 0);

	if (Serial.available()) {
		static char line[32];
		static unsigned char buf[128]; // TODO make a line parser class with a lambda/closure
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
				if (sscanf(line, "navhi=%f", &f) == 1) { hdgPID.hiGain.p = f; }
				else if (sscanf(line, "navtr=%f", &f) == 1) { hdgPID.hiGainTrans.p = f; }
				else if (sscanf(line, "maxb=%f", &f) == 1) { ed.maxb.value = f; }
				else if (sscanf(line, "roll=%f", &f) == 1) { desRoll = f; }
				else if (sscanf(line, "pidp=%f", &f) == 1) { pitchPID.gain.p = f; }
				else if (sscanf(line, "pidi=%f", &f) == 1) { pitchPID.gain.i = f; }
				else if (sscanf(line, "pidd=%f", &f) == 1) { pitchPID.gain.d = f; }
				else if (sscanf(line, "pidl=%f", &f) == 1) { pitchPID.gain.l = f; }
				//else if (sscanf(line, "pitch=%f", &f) == 1) { ed.pset.value = f; }
				else if (sscanf(line, "ptrim=%f", &f) == 1) { ed.tzer.value = f; }
				//else if (sscanf(line, "mtin=%f", &f) == 1) { ed.mtin.value = f; }
				else if (sscanf(line, "ptman=%f", &f) == 1) { pitchTrimOverride = f; }
				else if (strstr(line, "zeroimu") != NULL) { ahrs.zeroSensors(); }
				else if (sscanf(line, "dtrk=%f", &f) == 1) { desiredTrk = f; }
				else if (sscanf(line, "servo=%f", &f) == 1) { servoOverride = f; }
				else if (sscanf(line, "knob=%f", &f) == 1) { setKnobPid(f); }
				else {
					Serial.printf("UNKNOWN COMMAND: %s", line);
				}
			}
		}
	}

	int avail = 0;
	while((avail = udpSL30.parsePacket()) > 0) { 
		static char line[200];
		static unsigned char buf[256];
		static int index;
		float knobSel = -1;
		int n = udpSL30.read(buf, sizeof(buf));
		//printf("READ: %s\n", buf);
		if (n <= 0)
			break;
		buf[n] = '\0';
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

				vector<string> l = split(string(line), ' ');
				float v;
				for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
					if      (sscanf(it->c_str(), "P=%f",   &v) == 1) { ahrsInput.g5Pitch = v; canMsgCount = canMsgCount + 1; logItem.flags |= LogFlags::g5Ins; }  
					else if (sscanf(it->c_str(), "R=%f",   &v) == 1) { ahrsInput.g5Roll = v; logItem.flags |= LogFlags::g5Ins; } 
					else if (sscanf(it->c_str(), "IAS=%f", &v) == 1) { ahrsInput.g5Ias = v; logItem.flags |= LogFlags::g5Ps;} 
					else if (sscanf(it->c_str(), "TAS=%f", &v) == 1) { ahrsInput.g5Tas = v; logItem.flags |= LogFlags::g5Ps;} 
					else if (sscanf(it->c_str(), "PALT=%f", &v) == 1) { ahrsInput.g5Palt = v; logItem.flags |= LogFlags::g5Ps;} 
					else if (sscanf(it->c_str(), "HDG=%f", &v) == 1) { ahrsInput.g5Hdg = v; logItem.flags |= LogFlags::g5Nav;} 
					else if (sscanf(it->c_str(), "TRK=%f", &v) == 1) { ahrsInput.g5Track = v; logItem.flags |= LogFlags::g5Nav; } 
					else if (sscanf(it->c_str(), "MODE=%f", &v) == 1) { apMode = v; } 
					else if (sscanf(it->c_str(), "KSEL=%f", &v) == 1) { knobSel = v; } 
					else if (sscanf(it->c_str(), "KVAL=%f", &v) == 1) { setObsKnob(knobSel, v); } 
				}
				
				float pit, roll, magHdg, magTrack, knobSel, knobVal, ias, tas, palt, age;
				int mode = 0;
				//printf("LINE %s\n", line);
				if (strstr(line, " CAN") != NULL && sscanf(line, "%f %f %f %f %f %f %f %f %f %f %d CAN", 
				&pit, &roll, &magHdg, &magTrack, &ias, &tas, &palt,  &knobSel, &knobVal, &age, &mode) == 11
					&& (pit > -2 && pit < 2) && (roll > -2 && roll < 2) && (magHdg > -7 && magHdg < 7) 
					&& (magTrack > -7 && magTrack < 7) && (knobSel >=0 && knobSel < 6)) {
					Serial.printf("CAN: %s\n", line);
					ahrsInput.g5Pitch = pit * 180 / M_PI;
					ahrsInput.g5Roll = roll * 180 / M_PI;
					ahrsInput.g5Hdg = magHdg * 180 / M_PI;
					ahrsInput.g5Track = magTrack * 180 / M_PI;
					ahrsInput.g5Palt = palt / 3.2808;
					ahrsInput.g5Ias = ias / 0.5144;
					ahrsInput.g5Tas = tas / 0.5144;
					ahrsInput.g5TimeStamp = (millis() - (uint64_t)age) / 1000.0;
					apMode = mode;
					ahrs.mComp.addAux(ahrsInput.g5Hdg, 1, 0.02);
					logItem.flags |= (LogFlags::g5Nav | LogFlags::g5Ps | LogFlags::g5Ins);
					setObsKnob(knobSel, knobVal);
					Serial.printf("knob sel %f, knob val %f\n", knobSel, knobVal);
					canMsgCount = canMsgCount + 1;
				}
			}

			gps.encode(buf[i]);
			if (buf[i] == '\r' || buf[i] == '\n') { 
				if (vtgCourse.isUpdated()) {  // VTG typically from G5 NMEA serial output
					gpsTrackVTG = constrain360(gps.parseDecimal(vtgCourse.value()) * 0.01 - magVar);
				}
				if (gps.course.isUpdated()) { // RMC typically from ifly NMEA output
					gpsTrackRMC = constrain360(gps.course.deg() - magVar);
					ahrs.mComp.addAux(gpsTrackRMC, 5, 0.07); 	
					logItem.flags |= LogFlags::HdgRMC;
				}
				if (gps.location.isUpdated() && hdgSelect == 2) {
					ahrsInput.alt = gps.altitude.meters() * 3.2808;
					ahrsInput.gspeed = gps.speed.knots();
					gpsFixes++;
				}
				if (desiredHeading.isUpdated()) { 
					navDTK = constrain360(0.01 * gps.parseDecimal(desiredHeading.value()) - magVar);
					if (navDTK < 0)
						navDTK += 360;
					if (canMsgCount.isValid() == false) {
						apMode = 4;
					}
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

	if (imuRead()) { 
		bool tick1HZ = floor(ahrsInput.sec) != floor(lastAhrsInput.sec);

		ahrsInput.gpsTrackGDL90 = gpsTrackGDL90;
		ahrsInput.gpsTrackVTG = gpsTrackVTG;
		ahrsInput.gpsTrackRMC = gpsTrackRMC;
		ahrsInput.dtk = desiredTrk;

		roll = ahrs.add(ahrsInput);
		pitch = ahrs.pitchCompDriftCorrected;

		if (hdgSelect == 0) { // mode 0, use GDL90 until first can message, then switch to G5
			ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;
			if (canMsgCount.isValid() == true) // switch to G5 on first CAN msg
				hdgSelect = 1;  
		}
		if (hdgSelect == 1) { // hybrid G5/GDL90 data 
			if (g5HdgChangeTimer.unchanged(ahrsInput.g5Hdg) < 2.0) { // use g5 data if it's not stale
				if (ahrsInput.gpsTrackGDL90 != -1 || ahrsInput.gpsTrackRMC != -1) { 
					ahrsInput.selTrack = ahrsInput.g5Hdg;
				}
				lastAhrsGoodG5 = ahrsInput;
			} else if (ahrsInput.gpsTrackGDL90 != -1 && ahrsInput.gpsTrackRMC != -1) { 
				ahrsInput.selTrack = lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90) / 2 + 
				 angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC) / 2;
			} else if (ahrsInput.gpsTrackGDL90 != -1) { // otherwise use change in GDL90 data 
				ahrsInput.selTrack = lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90); 
			} else if (ahrsInput.gpsTrackRMC != -1) { // otherwise use change in VTG data 
				ahrsInput.selTrack = lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC); 
			} else { // otherwise, no available heading/track data 
				ahrsInput.selTrack = -1;
			}
		}
		else if (hdgSelect == 2) ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;
		else if (hdgSelect == 3) ahrsInput.selTrack = ahrs.magHdg;
		
		if (false && floor(ahrsInput.sec / 0.05) != floor(lastAhrsInput.sec / 0.05)) { // 20HZ
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
			//logItem.pitchCmd = trimCmd;
		}

		pwmOutput = 0;
		if (true || ahrs.valid() || digitalRead(button4.pin) == 0 || servoOverride > 0) { // hold down button to override and make servo work  
			if (hdgPIDTimer.tick()) {
				if (ahrsInput.dtk != -1) { 
					if (apMode == 4) {
						xteCorrection = -xtePID.add(crossTrackError.average(), crossTrackError.average(), ahrsInput.sec);					
						xteCorrection = max(-40.0, min(40.0, (double)xteCorrection));
						desiredTrk = navDTK + xteCorrection;
						ahrsInput.dtk = desiredTrk;
					} else { 
						xteCorrection = 0;
					}
					double hdgErr = 0;
					if (ahrs.valid() != false && ahrsInput.selTrack != -1) { 
						hdgErr = angularDiff(ahrsInput.selTrack - ahrsInput.dtk);
						currentHdg = ahrsInput.selTrack;
					} else { 
						// lost course guidance, just keep wings level by leaving currentHdg unchanged and no error 
						hdgErr = 0;
					}
					if (abs(hdgErr) > 10.0) {
						hdgPID.resetI();
					}
					desRoll = -hdgPID.add(hdgErr, currentHdg, ahrsInput.sec);
					desRoll = max(-ed.maxb.value, min(+ed.maxb.value, desRoll));
				} else if (!testTurnActive) {
					desRoll = 0.0; // TODO: this breaks roll commands received over the serial bus, add rollOverride variable or something 
				}				
			} 

			rollPID.add(roll - desRoll, roll, ahrsInput.sec);

			if (armServo) {  
				servoOutput = servoTrim + rollPID.corr;
				if (servoOverride > 0)
					servoOutput = servoOverride;
				servoOutput = max(550, min(2100, servoOutput));
				pwmOutput = servoOutput * 4915 / 1500;
			}			
		}	
		
		ledcWrite(1, pwmOutput); // scale PWM output to 1500-7300 
		logItem.pwmOutput = pwmOutput;
		logItem.desRoll = desRoll;
		logItem.roll = roll;
		logItem.ai = ahrsInput;
		//logItem.ai.q3 = ahrs.magCorr; 

		totalRollErr += abs(roll + ahrsInput.g5Roll);
		totalHdgError += abs(angularDiff(ahrs.magHdg - ahrsInput.g5Hdg));
	
#ifdef UBUNTU
		if (millis() < 1000) // don't count error during the first second while stuff initializes 
			totalRollErr = totalHdgError = 0;

		// special logfile name "+", write out log with computed values from the current simulation 			
		if (strcmp(logFilename.c_str(), "+") == 0) { 
			cout << logItem.toString().c_str() << " " <<  ahrs.magHdg << " " << ahrs.bankAngle << " " << ahrs.magBank/*33*/  << " LOG U" << endl;
		}
#endif
		logItem.flags = 0;
		lastAhrsInput = ahrsInput;
	}

	if (ed.pidsel.changed()) {
		setKnobPid(ed.pidsel.value);
	}
	
	if (screenTimer.tick() && screenEnabled) { 
		ed.update();
		knobPID->gain.p = ed.pidpl.value;
		knobPID->hiGain.p = ed.pidph.value;
		knobPID->gain.i = ed.pidi.value;
		knobPID->gain.d = ed.pidd.value;
		knobPID->gain.l = ed.pidl.value;
		knobPID->maxerr.i = ed.maxi.value;
		knobPID->finalGain = ed.pidg.value;
		knobPID->hiGainTrans.p = ed.dead.value;
		
		Display::ip = WiFi.localIP().toString().c_str(); 
		Display::dtk = desiredTrk; 
		Display::trk = ahrsInput.selTrack; 
		Display::navt = navDTK; 
		Display::obs = obs; 
		Display::mode = (canMsgCount.isValid() ? 10000 : 0) + apMode * 1000 + armServo * 100 + hdgSelect * 10 + (int)testTurnActive; 
		Display::gdl = (float)gpsTrackGDL90;
		Display::maghdg = (float)ahrs.magHdg;
		//Display::zsc = ahrs.getGyroQuality(); 
		Display::roll = roll; 
		Display::drop = logFile != NULL ? logFile->dropped : -1;
		//Display::pitch = pitch;
		//Display::xtec = xteCorrection; 
		Display::log = (logFile != NULL) ? logFile->currentFile.c_str() : "none";
		Display::log.setInverse(false, (logFile != NULL));
	}
			
	if (logFile != NULL) {
		sdLog();
	}
}

#ifdef UBUNTU
///////////////////////////////////////////////////////////////////////////////
// Code below this point is used in compiling/running ESP32sim simulation

bool ESP32sim_replayLogItem(ifstream &);
float ESP32sim_pitchCmd = 940.0;
void ESP32sim_set_gpsTrackGDL90(float v);

class FlightSim { 
public:
	ifstream ifile;
	const char *replayFile = NULL;
	int logSkip = 0;
	int logEntries = 0;
	float bank = 0, track = 0, pitch = 0, roll = 0, yaw = 0, simPitch = 0, hdg = 0;
	RollingAverage<float,250> rollCmd;
	uint64_t lastMillis = 0;
	float cmdPitch;
	std::queue<float> gxDelay, pitchDelay;
	
	uint64_t lastMicros = 0;
	
	void flightSim(MPU9250_DMP &imu) { 
		_micros += 3500;
		const float servoTrim = 4915.0;

		float rawCmd = ESP32sim_pitchCmd;
		cmdPitch = rawCmd > 0 ? (940 - rawCmd) / 13 : 0;
		float ngx = (cmdPitch - simPitch) * 1.3;
		ngx = max((float)-5.0,min((float)5.0, ngx));
		simPitch += (cmdPitch - simPitch) * .0015;
		gxDelay.push(ngx);
		pitchDelay.push(simPitch);
				
		// Simulate simple airplane roll/bank/track turn response to 
		// servooutput read from ESP32sim_currentPwm; 
		rollCmd.add((ESP32sim_currentPwm - servoTrim) / servoTrim);
		imu.gy = 0.0 + rollCmd.average() * 10.0;
		bank += imu.gy * (3500.0 / 1000000.0);
		bank = max(-20.0, min(20.0, (double)bank));
		if (floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f\n", (float)millis()/1000.0, 
			ESP32sim_currentPwm, track, 0.0, bank, imu.gy);
		}		
		imu.gz = +1.5 + tan(bank * M_PI/180) / 100 * 1091;
		
		uint64_t now = millis();
		const float bper = .05;
		if (floor(lastMillis * bper) != floor(now * bper)) { // 10hz
			track -= tan(bank * M_PI / 180) * 9.8 / 40 * 25 * bper;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;
		}

		hdg = track - 35.555; // simluate mag var and WCA 
		if (hdg < 0) hdg += 360;	
		ESP32sim_set_gpsTrackGDL90(track);

		while(gxDelay.size() > 400) {
			gxDelay.pop();
			pitchDelay.pop();
			imu.gx = gxDelay.front();
			pitch = pitchDelay.front();
		}
		//printf("SIM %08.3f %+05.2f %+05.2f %+05.2f\n", millis()/1000.0, gx, pitch, cmdPitch);
		imu.az = cos(pitch * M_PI / 180) * 1.0;
		imu.ay = sin(pitch * M_PI / 180) * 1.0;
		imu.ax = 0;

		// simulate meaningless mag readings that stabilize when bank == 0 
		imu.mx = imu.my = bank * 1.4;
		imu.mz += imu.mx;
		
		lastMillis = now;
	}
	
	bool firstLoop = true;
	void ESP32sim_run(MPU9250_DMP &imu) { 
		static float lastTime = 0;
		float now = _micros / 1000000.0;
		
		if (replayFile == NULL) { 
			if (floor(now / .1) != floor(lastTime / .1)) {
				float g5hdg = hdg * M_PI / 180;
				float g5roll = -bank * M_PI / 180;
				float g5pitch = pitch * M_PI / 180;
				ESP32sim_udpInput(7891, strfmt("IAS=%f TAS=%f PALT=%f\n", 90, 100, 1000));
				ESP32sim_udpInput(7891, strfmt("P=%f R=%f\n", g5pitch, g5roll));
				ESP32sim_udpInput(7891, strfmt("HDG=%f\n", g5hdg));
			}
			flightSim(imu);
		} else {
			if (firstLoop == true) { 
				ifile = ifstream(replayFile, ios_base::in | ios::binary);
			}
			while(logSkip > 0 && logSkip-- > 0) {
				ESP32sim_replayLogItem(ifile);
			}
			if (ESP32sim_replayLogItem(ifile) == false) { 
				ESP32sim_done();
				exit(0);
			}
			logEntries++;
		}
	
		//if (now >= 500 && lastTime < 500) {	Serial.inputLine = "pitch=10\n"; }
		//if (now >= 100 && lastTime < 100) {	Serial.inputLine = "zeroimu\n"; }
		firstLoop = false;
		lastTime = now;
	}	
} fsim;

void ESP32sim_parseArg(char **&a, char **endA) {
	if (strcmp(*a, "--replay") == 0) fsim.replayFile = *(++a);
	else if (strcmp(*a, "--replaySkip") == 0) fsim.logSkip = atoi(*(++a));
	else if (strcmp(*a, "--log") == 0) { 
		bm.addPress(pins.midButton, 1, 1, true);  // long press bottom button - start log 1 second in  
		logFileName = (*(++a));
	}	
	else if (strcmp(*a, "--logConvert") == 0) {
		ifstream i = ifstream(*(++a), ios_base::in | ios::binary);
		ofstream o = ofstream(*(++a), ios_base::out | ios::binary);			
		ESP32sim_convertLogCtoD(i, o);
		exit(0);
	}
}

void ESP32sim_setup() { 
	bm.addPress(pins.topButton, 1, 1, true); // knob long press - arm servo
	bm.addPress(pins.botButton, 250, 1, true); // bottom long press - test turn activate 
	bm.addPress(pins.topButton, 500, 1, false); // top short press - hdg hold 
	ahrsInput.dtk = desiredTrk = 90;
}

void ESP32sim_loop() { 
	fsim.ESP32sim_run(imu);
}

void ESP32sim_done() { 
	printf("# %f %f avg roll/hdg errors, %d log entries, %.1f real time seconds\n", totalRollErr / fsim.logEntries, totalHdgError / fsim.logEntries,  fsim.logEntries, millis() / 1000.0);
	exit(0);
}

void ESP32sim_setDebug(const char *s) { 
	vector<string> l = split(string(s), ',');
	float v;
	for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
		if (sscanf(it->c_str(), "zeros.mx=%f", &v) == 1) { ahrs.magOffX = v; } 
		else if (sscanf(it->c_str(), "zeros.my=%f", &v) == 1) { ahrs.magOffY = v; } 
		else if (sscanf(it->c_str(), "zeros.mz=%f", &v) == 1) { ahrs.magOffZ = v; } 
		else if (sscanf(it->c_str(), "zeros.gx=%f", &v) == 1) { ahrs.gyrOffX = v; } 
		else if (sscanf(it->c_str(), "zeros.gy=%f", &v) == 1) { ahrs.gyrOffY = v; } 
		else if (sscanf(it->c_str(), "zeros.gz=%f", &v) == 1) { ahrs.gyrOffZ = v; } 
		else if (sscanf(it->c_str(), "cr1=%f", &v) == 1) { ahrs.compRatio1 = v; } 
		else if (sscanf(it->c_str(), "dc1=%f", &v) == 1) { ahrs.driftCorrCoeff1 = v; } 
		else if (sscanf(it->c_str(), "cr2=%f", &v) == 1) { ahrs.hdgCompRatio = v; } 
		else if (sscanf(it->c_str(), "mbt.cr=%f", &v) == 1) { ahrs.magBankTrimCr = v; } 
		else if (sscanf(it->c_str(), "mbt.maxerr=%f", &v) == 1) { ahrs.magBankTrimMaxBankErr = v; } 
		else if (sscanf(it->c_str(), "dipconstant=%f", &v) == 1) { ahrs.magDipConstant = v; } 
		else if (strlen(it->c_str()) > 0) { 
			printf("Unknown debug parameter '%s'\n", it->c_str()); 
			exit(-1);
		}
	}
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
	
		// Feed logged G5,GPS,NAV data back into the simulation via spoofed UDP network inputs 
		if ((l.flags & LogFlags::g5Ps) || l.ai.g5Ias != ahrsInput.g5Ias || l.ai.g5Tas != ahrsInput.g5Tas || l.ai.g5Palt != ahrsInput.g5Palt) { 
			ESP32sim_udpInput(7891, strfmt("IAS=%f TAS=%f PALT=%f\n", l.ai.g5Ias, l.ai.g5Tas, l.ai.g5Palt)); 
		}
		if ((l.flags & LogFlags::g5Nav) || l.ai.g5Hdg != ahrsInput.g5Hdg || l.ai.g5Track != ahrsInput.g5Track) { 
			ESP32sim_udpInput(7891, strfmt("HDG=%f TRK=%f\n", l.ai.g5Hdg, l.ai.g5Track)); 
		}
		if ((l.flags & LogFlags::g5Ins) || l.ai.g5Roll != ahrsInput.g5Roll || l.ai.g5Pitch != ahrsInput.g5Pitch) { 
			ESP32sim_udpInput(7891, strfmt("R=%f P=%f\n", l.ai.g5Roll, l.ai.g5Pitch)); 
		}
		if (abs(angularDiff(ahrsInput.gpsTrackRMC - l.ai.gpsTrackRMC)) > .1 || (l.flags & LogFlags::HdgRMC) != 0) { 
			char buf[128];
			sprintf(buf, "GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,%.2f,130495,003.8,E", l.ai.gpsTrackRMC + magVar);
			ESP32sim_udpInput(7891, String(nmeaChecksum(std::string(buf))));
		}
		if (abs(angularDiff(ahrsInput.gpsTrackGDL90 - l.ai.gpsTrackGDL90)) > .1 || (l.flags & LogFlags::HdgGDL) != 0) { 
			unsigned char buf[64];
			GDL90Parser::State s;
			s.track = l.ai.gpsTrackGDL90 + magVar;
			int len = gdl90.packMsg10(buf, s);
			ESP32sim_udpInput(4000, String((char *)buf, len));
		}
			
		// special logfile name "-", just replay existing log back out to stdout 
		if (strcmp(logFilename.c_str(), "-") == 0 && l.ai.sec != 0) {
			l.ai.sec = _micros / 1000000.0; 
			cout << l.toString().c_str() << " -1 LOG" << endl;
		}		
		return true;
	} 
	return false;
}

void ESP32sim_set_gpsTrackGDL90(float v) { 
	gpsTrackGDL90 = v; // TODO - pass this through ESP32sim_udpInput()
	ahrsInput.g5Hdg = v;
}

void ESP32sim_JDisplay_forceUpdate() { 
	Display::jd.forceUpdate();
}


#include "TinyGPS++.h"
#include "TinyGPS++.cpp"

#endif
