#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#ifdef UBUNTU
#include "ESP32sim_ubuntu.h"
#else // #ifndef UBUNTU
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <MPU9250_asukiaaa.h>
#endif // #else // UBUNTU

#include <TinyGPS++.h>

#include "jimlib.h"
#include "RollingLeastSquares.h"
#include "PidControl.h"
#include "RollAHRS.h"
#include "G90Parser.h"
#include "WaypointNav.h"
using WaypointNav::trueToMag;
using WaypointNav::magToTrue;

WiFiMulti wifi;

//SPIFFSVariable<int> logFileNumber("/winglevlr.logFileNumber", 1);
int logFileNumber = 0;
int auxMpuPacketCount = 0;
TinyGPSPlus gps;
TinyGPSCustom desiredHeading(gps, "GPRMB", 11);
TinyGPSCustom xte(gps, "GPRMB", 2);
TinyGPSCustom xteLR(gps, "GPRMB", 3);
TinyGPSCustom vtgCourse(gps, "GPVTG", 1);

GDL90Parser gdl90;
GDL90Parser::State state;

RollAHRS ahrs;
PidControl rollPID(30) /*200Hz*/, pitchPID(10,6), hdgPID(50)/*20Hz*/, xtePID(100)/*5hz*/, altPID(100); /*5Hz*/
PidControl *knobPID = &altPID;

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
WiFiUDP udpCmd;
//WiFiUDP udpMAV;

#define LED_PIN 22
/* Old hardware pins: I2C pins/variant seems to determine layout 
struct {
	int topButton = 35;
	int midButton = 34;
	int botButton = 39;
	int knobButton = 32;
	int pwm_roll = 33;
	int pwm_pitch = 18;
} pins;
*/


struct {
	int topButton = 36;
	int midButton = 37;
	int botButton = 39;
	int knobButton = 32;
	int pwm_pitch = 33;
	int pwm_roll = 18;
	//int servo_enable = 18;
	int sda = 21; 
	int scl = 22; 
//	int led = 19;
	int tft_backlight = 27;
	int serial2_tx = 27;
	int serial2_rx = 19;
} pins;

MPU9250_asukiaaa *imu = NULL;

float pitchTrim = -.3, rollTrim = 0;
void halInit() { 
	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(1);

	Wire.begin(21,22);
	Serial.println("Scanning I2C bus on pins 21,22");
	if (scanI2c() == 0) { 
		Wire.begin(19,18);
		Serial.println("Scanning I2C bus on pins 19,18");
		if (scanI2c() == 0) {
			Serial.println("No I2C devices found, rebooting...");
			ESP.restart();
		}
		Serial.println("Older TTGO-TS board, changing pin assignments, IMU likely oriented wrong");
		pins.sda = 19;
		pins.scl = 22;
		pins.midButton = 34;
		pins.topButton = 35;
		//pins.servo_enable = 36;
		//pins.led = 21; // dont know 
		// TODO: IMU is inverted on these boards 
	}
	
	for(int addr = 0x68; addr <= 0x69; addr++) { 
		imu = new MPU9250_asukiaaa(addr);
		//imu.address = addr;
		uint8_t sensorId;
		Serial.printf("Checking MPU addr 0x%x: ", (int)imu->address);
		if (imu->readId(&sensorId) == 0) {
			Serial.printf("Found MPU sensor id: 0x%x\n", (int)sensorId);
			break;
		} else {
			Serial.println("Cannot read sensorId");
			delete imu;
		}
	}

	if (0) {
		pinMode(pins.tft_backlight, OUTPUT); // TFT backlight
		digitalWrite(pins.tft_backlight, 1);
	} else { 
		Serial2.begin(57600, SERIAL_8N1, pins.serial2_rx, pins.serial2_tx);
		Serial2.setTimeout(1);
	}

	while(0) { // basic hardware testing - halt here and debug pins 
		static int alternate = 0;
		delay(100); 
		printPins();
		pinMode(pins.tft_backlight, OUTPUT);
		//pinMode(pins.led, OUTPUT);
		alternate = !alternate;
		//digitalWrite(pins.led, alternate);
		digitalWrite(pins.tft_backlight, !alternate);
	} 
	
	imu->beginAccel(ACC_FULL_SCALE_4_G);
	imu->beginGyro(GYRO_FULL_SCALE_250_DPS);
	imu->beginMag(MAG_MODE_CONTINUOUS_100HZ);
}

DigitalButton buttonTop(pins.topButton); // top
DigitalButton buttonMid(pins.midButton); // middle
DigitalButton buttonBot(pins.botButton); // bottom
DigitalButton buttonKnob(pins.knobButton); // knob press

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
	buttonMid.check();
	buttonBot.check();
	buttonTop.check();
	butFilt.check(buttonMid.duration());
	butFilt2.check(buttonBot.duration());
	
	butFilt3.check(buttonTop.duration());
	butFilt4.check(buttonKnob.duration());
}

void serialOutput(const String &s) {
	for (int n = 0; n < 30; n++) { 
		udpCmd.beginPacket("255.255.255.255", 9000);
		udpCmd.write((uint8_t *)s.c_str(), s.length());
		udpCmd.endPacket();
		delay(1);
	}
	Serial.write((uint8_t *)s.c_str(), s.length()); 
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
    JDisplayItem<float> logw(&jd,10,y+=10,"LOGW:", "%05.0f ");
	
	JDisplayItem<float> pidpl(&jd,00,y+=10,"PL:", "%03.2f "); JDisplayItem<float> tttt(&jd,c2x,y,    " TT1:", "%04.1f ");
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

void imuLog(); 


static AhrsInput ahrsInput;

bool imuRead() { 
	AhrsInput &x = ahrsInput;
	x.sec = millis() / 1000.0;
#ifdef USE_ACCEL
	imu->accelUpdate();
	x.ax = imu->accelX();
	x.ay = imu->accelY();
	x.az = imu->accelZ();
#endif
	imu->gyroUpdate();
	x.gx = imu->gyroX();
	x.gy = imu->gyroY();
	x.gz = imu->gyroZ();

	// limit magnometer update to 100Hz 
	static uint64_t lastUsec = 0;
	if (lastUsec / 10000 != micros() / 10000) { 
		imu->magUpdate();
		x.mx = imu->magX();
		x.my = imu->magY();
		x.mz = imu->magZ();
	}
	// disable mpu logging 
	if (false && lastUsec / 20000 != micros() / 20000) { 
		AuxMpuData a;
		a.ax = x.ax; a.ay = x.ay; a.az = x.az;
		a.gx = x.gx; a.gy = x.gy; a.gz = x.gz;
		a.mx = x.mx; a.my = x.my; a.mz = x.mz;
		String ad = a.toString();
		if (Serial2.availableForWrite() > 120) { 
			Serial2.println(a.toString());
		}
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
	//Serial.println(logItem.toString());
	logItem.flags = 0;
}

void printMag() {
      //imu->updateCompass();
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->gyroX(), (float)imu->gyroY(), (float)imu->gyroZ() );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->magX(), (float)imu->magY(), (float)imu->magZ() );
      Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->accelX(), (float)imu->accelY(), (float)imu->accelZ() );
      Serial.println("");
}

static AhrsInput lastAhrsInput, lastAhrsGoodG5; 

void setup() {	
	//SPIFFS.begin();
	
	halInit();
	// ugh: redo pin assignments, hal may have changed them
	buttonTop.pin = pins.topButton; 
	buttonBot.pin = pins.botButton;
	buttonMid.pin = pins.midButton;
	
	buttonTop.read(); buttonBot.read(); buttonMid.read(); buttonKnob.read();
	
	Serial.printf("Reading log file number\n");
	int l = logFileNumber;
	Serial.printf("Log file number %d\n", l);
	logFileNumber = l + 1;

	esp_task_wdt_init(15, true);
	esp_err_t err = esp_task_wdt_add(NULL);

    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector   
 
	//esp_register_freertos_idle_hook(bApplicationIdleHook);
	//pinMode(LED_PIN, OUTPUT);
	//digitalWrite(LED_PIN, 1);
	pinMode(pins.tft_backlight, OUTPUT); // TFT backlight
	digitalWrite(pins.tft_backlight, 1);
	//pinMode(pins.servo_enable, OUTPUT);
	//digitalWrite(pins.servo_enable, 1);

	Display::jd.begin();
	Display::jd.setRotation(3);
	Display::jd.clear();
	
	attachInterrupt(digitalPinToInterrupt(buttonMid.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonBot.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonTop.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonKnob.pin), buttonISR, CHANGE);
		
	WiFi.disconnect(true);
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);

	wifi.addAP("Ping-582B", "");
	wifi.addAP("Flora_2GEXT", "maynards");
	wifi.addAP("Team America", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");

	if (!buttonKnob.read() && !buttonTop.read()) { // skip long setup stuff if we're debugging
		uint64_t startms = millis();
		while (WiFi.status() != WL_CONNECTED /*&& digitalRead(button.pin) != 0*/) {
			wifi.run();
			delay(10);
		}	
	}

	udpSL30.begin(7891);
	udpG90.begin(4000);
	udpNMEA.begin(7892);
	udpCmd.begin(7895);
	
	lastAhrsGoodG5.g5Hdg  = lastAhrsGoodG5.gpsTrackGDL90 = lastAhrsGoodG5.gpsTrackRMC = -1;
	ahrsInput.g5Hdg = ahrsInput.gpsTrackGDL90 = ahrsInput.gpsTrackRMC = -1;
	
	// Set up PID gains 
	rollPID.setGains(7.52, 0.05, 0.11);
	rollPID.hiGain.p = 1;
	rollPID.hiGainTrans.p = 5;
	rollPID.finalGain = 16.8;
	rollPID.maxerr.i = 20;

	hdgPID.setGains(0.25, 0.02, 0.02);
	hdgPID.hiGain.p = 25.00;
	hdgPID.hiGainTrans.p = 15.0;
	hdgPID.maxerr.i = 20;
	hdgPID.finalGain = 0.5;

	xtePID.setGains(8.0, 0.00, 0.05);
	xtePID.maxerr.i = 1.0;
	xtePID.finalGain = 10.0;
	
	pitchPID.setGains(20.0, 0.0, 2.0, 0, .8);
	pitchPID.finalGain = 5.0;
	pitchPID.maxerr.i = .5;

	altPID.setGains(1.0, 0.0, 0.1);
	altPID.finalGain = 1.0;

    // make PID select knob display text from array instead of 0-3	
	Display::pidsel.toString = [](float v){ return String((const char *[]){"PIT ", "ALT ", "ROLL", "XTE ", "HDG "}[(v >=0 && v <= 3) ? (int)v : 0]); };		
	ed.begin();

#ifndef UBUNTU
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
#endif
	ed.maxb.setValue(12);
	ed.tttt.setValue(20); // seconds to make each test turn 
	ed.ttlt.setValue(20); // seconds betweeen test turn  
	ed.tzer.setValue(1000);
	ed.pidsel.setValue(1);
	setKnobPid(ed.pidsel.value);
	ed.update();
	
	//ed.rlhz.value = 3; // period for relay activation, in seconds
	//ed.mnrl.value = 70;
	//ed.pmin.value = 0.5; // PID total error that triggers relay minimum actuation
	//ed.pmax.value = 2.5; // PID total error that triggers relay maximum actuation 
	pinMode(pins.pwm_pitch, OUTPUT);
	pinMode(pins.pwm_roll, OUTPUT);
	ledcSetup(1, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcSetup(0, 50, 16); // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(pins.pwm_pitch, 0);   // GPIO 33 assigned to channel 1
	ledcAttachPin(pins.pwm_roll, 1);   // GPIO 33 assigned to channel 1

	ArduinoOTA.begin();
}

 
static StaleData<float> gpsTrackGDL90(3000,-1), gpsTrackRMC(5000,-1), gpsTrackVTG(5000,-1);
static StaleData<int> canMsgCount(3000,-1);
static float desiredTrk = -1;
float desRoll = 0, desPitch = -15, desAlt = 0;		
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
	if      (f == 0) { knobPID = &pitchPID; }
	else if (f == 1) { knobPID = &altPID;}
	else if (f == 1) { knobPID = &rollPID; }
	else if (f == 2) { knobPID = &xtePID; }
	else if (f == 3) { knobPID = &hdgPID; }
	
	ed.pidpl.setValue(knobPID->gain.p);
	ed.pidph.setValue(knobPID->hiGain.p);
	ed.pidi.setValue(knobPID->gain.i);
	ed.pidd.setValue(knobPID->gain.d);
	ed.pidl.setValue(knobPID->gain.l);
	ed.pidg.setValue(knobPID->finalGain);
	ed.maxi.setValue(knobPID->maxerr.i);
	ed.dead.setValue(knobPID->hiGainTrans.p);
}	
	
static bool testTurnActive = false;
static int testTurnAlternate = 0;
static float testTurnLastTurnTime = 0;
static RollingAverage<float,5> crossTrackError;
static float xteCorrection = 0;
static float navDTK = -1;
static bool logActive = false;
static float roll = 0, pitch = 0;
static String logFilename("none");
static int servoOutput[2], servoTrim[2] = {1500, 1500};
static TwoStageRollingAverage<int,40,40> loopTime;
static EggTimer serialReportTimer(200), hz5(200), loopTimer(5), buttonCheckTimer(10);
static int armServo = 0;
static int servoSetupMode = 0; // Referenced when servos not armed.  0: servos left alone, 1: both servos neutral + trim, 2: both servos full in, 3: both servos full out
static int apMode = 1; // apMode == 4 means follow NMEA HDG and XTE sentences, anything else tracks OBS
static int hdgSelect = 3; //  0 use GDL90 but switch to mode 1 on first can msg. 1- use g5 hdg, 2 use GDL90 data 
static float obs = -1, lastObs = -1;
static bool screenReset = false, screenEnabled = true;
struct {
	double hdg, roll, pitch; 
	void clear() { hdg = roll = pitch = 0; }
} totalError;
//static int ledOn = 0;
static int manualRelayMs = 60;
static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
static uint64_t lastLoop = micros();
static bool selEditing = false;

std::string waypointList;
WaypointsSequencerString *wpNav = NULL;
GDL90Parser::State gdl90State;

Windup360 currentHdg;
ChangeTimer g5HdgChangeTimer;

void parseSerialCommandInput(const char *buf, int n) { 
	static LineBuffer lb;
	lb.add(buf, n, [](const char *line) { 
		//Serial.printf("RECEIVED COMMAND: %s", line);
		float f, f2;
		int relay, ms;
		if (sscanf(line, "navhi=%f", &f) == 1) { hdgPID.hiGain.p = f; }
		else if (sscanf(line, "navtr %f", &f) == 1) { hdgPID.hiGainTrans.p = f; }
		else if (sscanf(line, "maxb %f", &f) == 1) { ed.maxb.value = f; }
		else if (sscanf(line, "roll %f", &f) == 1) { desRoll = f; }
		else if (sscanf(line, "pidp %f", &f) == 1) { pitchPID.gain.p = f; }
		else if (sscanf(line, "pidi %f", &f) == 1) { pitchPID.gain.i = f; }
		else if (sscanf(line, "pidd %f", &f) == 1) { pitchPID.gain.d = f; }
		else if (sscanf(line, "pidl %f", &f) == 1) { pitchPID.gain.l = f; }
		else if (sscanf(line, "pidl %f", &f) == 1) { pitchPID.gain.l = f; }
		//else if (sscanf(line, "pitch=%f", &f) == 1) { ed.pset.value = f; }
		else if (sscanf(line, "ptrim=%f", &f) == 1) { ed.tzer.value = f; }
		//else if (sscanf(line, "mtin=%f", &f) == 1) { ed.mtin.value = f; }
		else if (strstr(line, "zeroimu") == line) { ahrs.zeroSensors(); }
		else if (sscanf(line, "dtrk=%f", &f) == 1) { desiredTrk = f; }
		else if (sscanf(line, "s %f %f", &f, &f2) == 2) { servoOutput[0] = f; servoOutput[1] = f2; }
		else if (sscanf(line, "trim %f %f", &f, &f2) == 2) { rollTrim = f; pitchTrim = f2; }
		else if (sscanf(line, "dpitch %f", &f) == 1) { desPitch = f; }
		else if (sscanf(line, "knob=%f", &f) == 1) { setKnobPid(f); }
		else if (strstr(line, "wpclear") == line) { waypointList = ""; }
		else if (strstr(line, "wpadd ") == line) { waypointList += (line + 6); waypointList += "\n"; }
		else if (strstr(line, "wpstart") == line && wpNav == NULL) { 
			wpNav = new WaypointsSequencerString(waypointList); 
		}
		else if (strstr(line, "wpstop") == line && wpNav != NULL ) { delete wpNav; wpNav = NULL; }
		else {
			Serial.printf("UNKNOWN COMMAND: %s", line);
		}
	});
}


void setObsKnob(float knobSel, float v) { 
	if (knobSel == 2) {

		desAlt = v * 3.2808;
	}
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


static const float servoThrow = 2.0;

void setServos(float x, float y) { 
	float leftStringX = 14;
	float leftStringY = 7;

	float rightStringX = 11;
	float rightStringY = 7;

	float leftLen = sqrt(leftStringX * leftStringX + leftStringY * leftStringY);
	float rightLen = sqrt(rightStringX * rightStringX + rightStringY * rightStringY);

	float xScale = +1.0;
	float yScale = -1.0;

	float x1 = leftStringX + xScale * x + rollTrim;
	float y1 = leftStringY + yScale * y + pitchTrim;
	float leftNewLen = sqrt(x1 * x1 + y1 * y1);

	x1 = rightStringX - xScale * x;
	y1 = rightStringY + yScale * y;
	float rightNewLen = sqrt(x1 * x1 + y1 * y1);

	float s0 =  +(rightNewLen - rightLen) / servoThrow * 2000 + 1500;
	float s1 =  +(leftNewLen - leftLen) / servoThrow * 2000 + 1500;

	servoOutput[0] = max(450, min(2550, (int)s0));
	servoOutput[1] = max(450, min(2550, (int)s1));
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
			"ALT %04.0f desA %04.0f "
			//"%+05.2f %+05.2f %+05.2f %+05.1f srv %04d xte %3.2f "
			"PID %+06.2f %+06.2f %+06.2f %+06.2f " 
			//"but %d%d%d%d loop %d/%d/%d heap %d re.count %d logdrop %d maxwait %d auxmpu %d"
			"sv %04d %04d "
			"\n",
			millis()/1000.0,
			//roll, ahrs.bankAngle, ahrs.gyrZOffsetFit.average(), ahrs.zeroSampleCount, ahrs.magStabFit.average(),   
			roll, pitch, ahrsInput.g5Roll, ahrsInput.g5Pitch, ahrsInput.g5Hdg,
			ahrsInput.alt, desAlt,
			//0.0, 0.0, 0.0, 0.0, servoOutput, crossTrackError.average(),
			knobPID->err.p, knobPID->err.i, knobPID->err.d, knobPID->corr, 
			//buttonTop.read(), buttonMid.read(), buttonBot.read(), buttonKnob.read(), (int)loopTime.min(), (int)loopTime.average(), (int)loopTime.max(), ESP.getFreeHeap(), ed.re.count, 
			//	logFile != NULL ? logFile->dropped : 0, logFile != NULL ? logFile->maxWaiting : 0, auxMpuPacketCount
			servoOutput[0], servoOutput[1],
			/*dummy*/0
		);
		if (logFile != NULL) {
			logFile->maxWaiting =  0;
		}
		serialLogFlags = 0;
		auxMpuPacketCount = 0;
	}

	/////////////////////////////////////////////////////////////////////////////
	// KNOB/BUTTON INTERFACE
	// 
	// TOP:    short   - apMode = 1, toggle between wing level and hdg hold 
	//         long    - arm servo
	//         triple  - zero sensors
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
				if (armServo) {
					armServo = false;
					servoSetupMode = 0;
				} else { 
					servoSetupMode = (servoSetupMode + 1) % 4; 
				}
				Serial.printf("Servo setup mode %d\n", servoSetupMode);

			}
			if (butFilt3.wasCount == 3) {
				std::string s = ahrs.zeroSensors();
				serialOutput(String(s.c_str()));
			}
		}
		if (butFilt.newEvent()) { // MIDDLE BUTTON
			if (!butFilt.wasLong) {
				if (butFilt.wasCount == 1) {
					desiredTrk = constrain360(desiredTrk - 10);
					apMode = 1;
				} else { 
					hdgSelect = (hdgSelect + 1) % 4;
				}					
			} else { 
				if (!logChanging) {
					logChanging = true;
					if (logFile == NULL) {	
							logFile = new SDCardBufferedLog<LogItem>(logFileName, 400/*q size*/, 0/*timeout*/, 5000/*flushInterval*/, false/*textMode*/);
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
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == true) {
				testTurnActive = !testTurnActive;
				testTurnAlternate = 0;
			}
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == false) {	
				desiredTrk = constrain360(desiredTrk + 10);
				apMode = 1;
			}
		}
								
		if (butFilt4.newEvent()) { 	// main knob button
			if (butFilt4.wasCount == 1 && butFilt4.wasLong != true) { 
				ed.buttonPress(butFilt4.wasLong);
			}
			if (butFilt4.wasLong && butFilt4.wasCount == 1) {			
				armServo = !armServo;
				servoSetupMode = 0;
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
					gpsTrackGDL90 = trueToMag(s.track);
					ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.03);
					logItem.flags |= LogFlags::HdgGDL; 
					gpsFixes++;
					ahrsInput.alt = s.alt * 3.2808;
					ahrsInput.palt = s.palt;
					ahrsInput.gspeed = s.hvel;
					gdl90State = s;
				}
			}
		}
	} while(recsize > 0);

	if (Serial2.available()) {
		static unsigned char buf[128]; // TODO make a line parser class with a lambda/closure
		static LineBuffer lb;
		int n = Serial2.readBytes(buf, min((int)sizeof(buf), (int)Serial2.available()));
		lb.add((char *)buf, n, [](const char *line) { 
			auxMpuPacketCount++;
			if (auxMPU.fromString(line)) {
			}
		});
	}

	if (Serial.available()) {
		char buf[1024];
		int n = Serial.readBytes((uint8_t *)buf, sizeof(buf));
		parseSerialCommandInput(buf, n);
	}
	if (udpCmd.parsePacket() > 0) {
		char buf[1024];
		int n = udpCmd.read((uint8_t *)buf, sizeof(buf));
		parseSerialCommandInput(buf, n);
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
					else if (sscanf(it->c_str(), "HDG=%f", &v) == 1) { 
						ahrsInput.g5Hdg = v; logItem.flags |= LogFlags::g5Nav;
						ahrs.mComp.addAux(ahrsInput.g5Hdg, 1, 0.02);
					} 
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
					//Serial.printf("CAN: %s\n", line);
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
					//Serial.printf("knob sel %f, knob val %f\n", knobSel, knobVal);
					canMsgCount = canMsgCount + 1;
				}
			}

			gps.encode(buf[i]);
			if (buf[i] == '\r' || buf[i] == '\n') { 
				if (vtgCourse.isUpdated()) {  // VTG typically from G5 NMEA serial output
					gpsTrackVTG = trueToMag(gps.parseDecimal(vtgCourse.value()) * 0.01);
				}
				if (gps.course.isUpdated()) { // RMC typically from ifly NMEA output
					gpsTrackRMC = trueToMag(gps.course.deg());
					ahrs.mComp.addAux(gpsTrackRMC, 5, 0.07); 	
					logItem.flags |= LogFlags::HdgRMC;
				}
				//if (gps.location.isUpdated() && hdgSelect == 2) {
				//	ahrsInput.alt = gps.altitude.meters() * 3.2808;
				//	ahrsInput.gspeed = gps.speed.knots();
				//	gpsFixes++;
				//}
				if (desiredHeading.isUpdated()) { 
					navDTK = trueToMag(0.01 * gps.parseDecimal(desiredHeading.value()));
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

	//printMag();
	if (imuRead()) { 
		bool tick1HZ = floor(ahrsInput.sec) != floor(lastAhrsInput.sec);

		if (apMode == 3 && wpNav != NULL) {
			wpNav->wptTracker.curPos.loc.lat = gdl90State.lat;
			wpNav->wptTracker.curPos.loc.lon = gdl90State.lon;
			wpNav->wptTracker.curPos.alt = gdl90State.alt;
			wpNav->wptTracker.curPos.valid = true;
			wpNav->run(ahrsInput.sec - lastAhrsInput.sec);
			desiredTrk = trueToMag(wpNav->wptTracker.track);
		}
		
		ahrsInput.gpsTrackGDL90 = gpsTrackGDL90;
		ahrsInput.gpsTrackVTG = gpsTrackVTG;
		ahrsInput.gpsTrackRMC = gpsTrackRMC;
		ahrsInput.dtk = desiredTrk;

		roll = ahrs.add(ahrsInput);
		pitch = ahrs.pitch;

		if (hdgSelect == 0) { // mode 0, use GDL90 until first can message, then switch to G5
			ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;
			if (ahrsInput.g5Hdg != -1) //canMsgCount.isValid() == true) // switch to G5 on first CAN msg
				hdgSelect = 1;  
		}
		if (hdgSelect == 1) { // hybrid G5/GDL90 data 
			if (ahrsInput.g5Hdg != -1 && g5HdgChangeTimer.unchanged(ahrsInput.g5Hdg) < 2.0) { // use g5 data if it's not stale
				//if (ahrsInput.gpsTrackGDL90 != -1 || ahrsInput.gpsTrackRMC != -1) { 
					ahrsInput.selTrack = ahrsInput.g5Hdg;
				//}
				lastAhrsGoodG5 = ahrsInput;
			} else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackGDL90 != -1 && ahrsInput.gpsTrackRMC != -1
				&& lastAhrsGoodG5.gpsTrackGDL90 != -1 && lastAhrsGoodG5.gpsTrackRMC != - 1) { 
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90) / 2 + 
				 angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC) / 2);
			} else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackGDL90 != -1 && lastAhrsGoodG5.gpsTrackGDL90 != -1) { // otherwise use change in GDL90 data 
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90));
			} else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackRMC != -1 &&  lastAhrsGoodG5.gpsTrackRMC != -1) { // otherwise use change in VTG data 
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC)); 
			} else { // otherwise, no available heading/track data 
				ahrsInput.selTrack = -1;
			}
		}
		else if (hdgSelect == 2) ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;
		else if (hdgSelect == 3) ahrsInput.selTrack = ahrs.magHdg;
		
		if (hz5.tick()) {
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
		 	float altErr = (apMode == 3) ? desAlt - ahrsInput.alt : 0;	
			altPID.add(altErr, ahrsInput.alt, ahrsInput.sec);
		}

		rollPID.add(roll - desRoll, roll, ahrsInput.sec);
		float altCorr = max(min(altPID.corr * 0.01, 5.0), -5.0);
		pitchPID.add(ahrs.pitch - desPitch + altCorr, ahrs.pitch, ahrsInput.sec);

		if (armServo == true) {  
			// TODO: pids were tuned and output results in units of relative uSec servo PWM durations. 
			// hack tmp: convert them back into inches so we can add in inch-specified trim values 
			float x = rollPID.corr / 2000 * servoThrow;
			float y = pitchPID.corr / 2000 * servoThrow;
			setServos(x, y);
		} else switch(servoSetupMode) { 
			case 2: setServos(0, -8); break;
			case 3: setServos(0, +8); break;
			case 0: break; // TODO: no hardware yet to depower the servos
			case 1: setServos(0, 0); break; 
			default: setServos(0, 0);
		}
		

		ledcWrite(0, servoOutput[0] * 4915 / 1500); // scale PWM output to 1500-7300 
		ledcWrite(1, servoOutput[1] * 4915 / 1500); // scale PWM output to 1500-7300 

		logItem.pwmOutputRoll = servoOutput[0];
		logItem.pwmOutputPitch = servoOutput[1];
		logItem.desRoll = desRoll;
		logItem.roll = roll;
		logItem.magHdg = ahrs.magHdg;
		logItem.bankAngle = ahrs.bankAngle;
		logItem.magBank = ahrs.magBank;
		logItem.pitch = ahrs.pitch; 
		logItem.ai = ahrsInput;
		logItem.auxMpu = auxMPU;

		totalError.roll += abs(roll + ahrsInput.g5Roll);
		totalError.hdg += abs(angularDiff(ahrs.magHdg - ahrsInput.g5Hdg));
		totalError.pitch += abs(pitch - ahrsInput.g5Pitch);
	
#ifdef UBUNTU
		static bool errorsCleared = false; 
		if (errorsCleared == false && millis() < 200000) {  // don't count error during the first 200 sec, let AHRS stabilize  
			totalError.clear();
			errorsCleared = true;
		}
	
		// special logfile name "+", write out log with computed values from the current simulation 			
		if (strcmp(logFilename.c_str(), "+") == 0) { 
			cout << logItem.toString().c_str() << strfmt("%+011.5lf %+011.5lf %06.3f %f	LOG U", gdl90State.lat, gdl90State.lon,
				ahrs.accelRoll, ahrs.accelPitch) << endl;
		}
#endif
		logItem.flags = 0;
		lastAhrsInput = ahrsInput;
	}

	//digitalWrite(pins.servo_enable, !armServo);
	
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
		Display::navt = auxMPU.gy; //navDTK; 
		Display::obs = obs; 
		Display::mode = (canMsgCount.isValid() ? 10000 : 0) + apMode * 1000 + armServo * 100 + hdgSelect * 10 + (int)testTurnActive; 
		Display::gdl = (float)gpsTrackGDL90;
		Display::maghdg = (float)ahrs.magHdg;
		//Display::zsc = ahrs.getGyroQuality(); 
		Display::roll = roll; 
		Display::drop = logFile != NULL ? logFile->dropped : -1;
		Display::logw = logFile != NULL ? logFile->written / 200 : -1;
		//Display::pitch = pitch;
		//Display::xtec = xteCorrection; 
		Display::log = (logFile != NULL) ? logFile->currentFile.c_str() : "none";
		Display::log.setInverse(false, (logFile != NULL));
	}
			
	if (logFile != NULL) {
		sdLog();
	}
	
	// Use onboard LED to indicate active logging 
	//pinMode(pins.led, OUTPUT);
	//digitalWrite(pins.led, logFile == NULL);
}

#ifdef UBUNTU
///////////////////////////////////////////////////////////////////////////////
// Code below this point is used in compiling/running ESP32sim simulation


//bool ESP32sim_replayLogItem(ifstream &);
float ESP32sim_pitchCmd = 940.0;
//void ESP32sim_set_gpsTrackGDL90(float v);
void ESP32sim_done();

class ESP32sim_winglevlr : public ESP32sim_Module {
public:
	ifstream trackSimFile;
	//using WaypointNav::TrackSimFileParser;
	WaypointsSequencerFile *tSim = NULL;
	IntervalTimer hz100 = IntervalTimer(100/*msec*/);

	ifstream ifile;
	const char *replayFile = NULL;
	int logSkip = 0;
	int logEntries = 0;
	float bank = 0, track = 0, pitch = 0, roll = 0, yaw = 0, simPitch = 0, hdg = 0;
	RollingAverage<float,100> rollCmd;
	uint64_t lastMillis = 0;
	float cmdPitch;
	float speed = 105;
	WaypointNav::LatLonAlt curPos;
	std::queue<float> gxDelay, pitchDelay;
	
	uint64_t lastMicros = 0;

	float stickX, stickY;
	void pwmToStickPosition() {
	}

	void flightSim(MPU9250_DMP *imu) { 
		//TODO: flightSim is very fragile/unstable.  Poke values into the
		// main loop code to make sure things work. 
		hdgPID.finalGain = 0.5;
		pitchTrim = rollTrim = 0;

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
		
		rollCmd.add((ESP32sim_currentPwm[1] - servoTrim) / servoTrim);
		imu->gy = 0.0 + rollCmd.average() * 1;
		bank += imu->gy * (3500.0 / 1000000.0);
		bank = max(-25.0, min(25.0 , (double)bank));
		if (0 && floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f\n", (float)millis()/1000.0, 
			ESP32sim_currentPwm[0], track, desRoll, bank, imu->gy);
		}		
		imu->gz = 0 + tan(bank * M_PI/180) / 100 * 1091;
		
		uint64_t now = millis();
		const float bper = .05;
		if (floor(lastMillis * bper) != floor(now * bper)) { // 10hz
			track -= tan(bank * M_PI / 180) * 9.8 / 40 * 25 * bper;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;
			set_gpsTrack(track);
		}

		hdg = track - 35.555; // simluate mag var and arbitrary WCA 
		if (hdg < 0) hdg += 360;	

		while(gxDelay.size() > 400) {
			gxDelay.pop();
			pitchDelay.pop();
			imu->gx = gxDelay.front();
			pitch = pitchDelay.front();
		}
		//printf("SIM %08.3f %+05.2f %+05.2f %+05.2f\n", millis()/1000.0, gx, pitch, cmdPitch);
		imu->az = cos(pitch * M_PI / 180) * 1.0;
		imu->ay = sin(pitch * M_PI / 180) * 1.0;
		imu->ax = 0;

		// simulate meaningless mag readings that stabilize when bank == 0 
		imu->mx = imu->my = imu->mz = 0;
		//bank * 1.4;
		//imu->mz += imu->mx;
		
		float dist = speed * 0.51444 * (now - lastMillis) / 1000.0; 
		curPos.loc = WaypointNav::locationBearingDistance(curPos.loc, magToTrue(track), dist);
		curPos.alt = 1000; // TODO 

		lastMillis = now;

	}

	bool ESP32csim_useAuxMpu = false;
	bool ESP32sim_replayLogItem(ifstream &i) {
		LogItem l; 
		static uint64_t logfileMicrosOffset = 0;
		
		if (i.read((char *)&l, sizeof(l))) {
			if (logfileMicrosOffset == 0) 
				logfileMicrosOffset = (l.ai.sec * 1000000 - _micros);
			_micros = l.ai.sec * 1000000 - logfileMicrosOffset;
			imu->ax = l.ai.ax;
			imu->ay = l.ai.ay;
			imu->az = l.ai.az;
			imu->gx = l.ai.gx;
			imu->gy = l.ai.gy;
			imu->gz = l.ai.gz;
			imu->mx = l.ai.mx;
			imu->my = l.ai.my;
			imu->mz = l.ai.mz;
			auxMPU = l.auxMpu;

			if (ESP32csim_useAuxMpu) { 
				float cksum = abs(auxMPU.ax) + abs(auxMPU.ay) + abs(auxMPU.az) +
					abs(auxMPU.gx) + abs(auxMPU.gy) + abs(auxMPU.gz) +
					abs(auxMPU.mx) + abs(auxMPU.my) + abs(auxMPU.mz);
				if (cksum < 1000) { 
					imu->ax = auxMPU.ax; imu->ay = auxMPU.ay; imu->az = auxMPU.az;
					imu->gx = auxMPU.ax; imu->gy = auxMPU.ay; imu->gz = auxMPU.az;
					imu->mx = auxMPU.ax; imu->my = auxMPU.ay; imu->mz = auxMPU.az;
				}
			}

			l.ai.g5Pitch = min(max(-30.0, (double)l.ai.g5Pitch), 30.0);
			l.ai.g5Roll = min(max(-30.0, (double)l.ai.g5Roll), 30.0);
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
				sprintf(buf, "GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,%.2f,130495,003.8,E", 
					magToTrue(l.ai.gpsTrackRMC));
				ESP32sim_udpInput(7891, string(nmeaChecksum(std::string(buf))));
			}
			if (abs(angularDiff(ahrsInput.gpsTrackGDL90 - l.ai.gpsTrackGDL90)) > .1 || ahrsInput.gspeed != l.ai.gspeed ||
				(l.flags & LogFlags::HdgGDL) != 0) { 
				unsigned char buf[64];
				GDL90Parser::State s;
				if (l.ai.gpsTrackGDL90 != -1) { 	
					s.track = magToTrue(l.ai.gpsTrackGDL90);
					s.hvel = l.ai.gspeed;
					int len = gdl90.packMsg10(buf, s);
					ESP32sim_udpInput(4000, string((char *)buf, len));
				}
			}
				
			servoOutput[0] = l.pwmOutputRoll;
			servoOutput[1] = l.pwmOutputPitch;

			// special logfile name "-", just replay existing log back out to stdout 
			if (strcmp(logFilename.c_str(), "-") == 0 && l.ai.sec != 0) {
				l.ai.sec = _micros / 1000000.0; 
				cout << l.toString().c_str() << " -1 LOG" << endl;
			}		
			return true;
		} 
		return false;
	}

	float gpsTrackFuzz = 0.00;
	void set_gpsTrack(float t) { 
		hdgSelect = 0;
		float t1 = -1, t2 = -1;
		if (t != -1) {
			t1 = random01() < gpsTrackFuzz ? -1 : constrain360(t + 0.1 * random01());
			t2 = random01() < gpsTrackFuzz ? -1 : constrain360(t + 0.1 * random01());
		}
		if (1) { 
			// Broken 
			GDL90Parser::State s;
			s.lat = curPos.loc.lat;
			s.lon = curPos.loc.lon;
			s.alt = curPos.alt;
			s.track = t1;
			s.vvel = 0;
			s.hvel = tSim != NULL ? tSim->wptTracker.speed : 0;
			s.palt = (s.alt + 1000) / 25;

			WiFiUDP::InputData buf;
			buf.resize(128);
			int n = gdl90.packMsg11(buf.data(), s);
			buf.resize(n);
			ESP32sim_udpInput(4000, buf);
			buf.resize(128);
			n = gdl90.packMsg10(buf.data(), s);
			buf.resize(n);
			ESP32sim_udpInput(4000, buf);;
			ESP32sim_udpInput(7891, strfmt("HDG=%f TRK=%f\n", t2, t2)); 
		} else {
			// TODO needs both set?  Breaks with only GDL90 
			gpsTrackGDL90 = t1;
			ahrsInput.g5Hdg = t2;;

			// TODO: mComp filter breaks at millis() rollover 
			// make winglevlr_ubuntu && time ./winglevlr_ubuntu --serial --tracksim ./tracksim_KBFI_14R.txt --seconds 10000  | grep -a "TSIM" > /tmp/simplot.txt && gnuplot -e 'f= "/tmp/simplot.txt"; set y2tic; set ytic nomirror; p f u 5 w l, f u 6 w l, f u 7 w l ax x1y2; pause 111'
			ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.03);
			ahrs.mComp.addAux(ahrsInput.g5Hdg, 1, 0.02);					
		}
	}

	std::vector<char> trackSimFileContents;
	//wrap_vector_as_istream tsf; 

	void parseArg(char **&a, char **la) override {
		if (strcmp(*a, "--replay") == 0) replayFile = *(++a);
		else if (strcmp(*a, "--replaySkip") == 0) logSkip = atoi(*(++a));
		else if (strcmp(*a, "--log") == 0) { 
			//bm.addPress(pins.midButton, 1, 1, true);  // long press bottom button - start log 1 second in  
			logFilename = (*(++a));
		} else if (strcmp(*a, "--startpos") == 0) {
			sscanf(*(++a), "%lf,%lf,%f", &curPos.loc.lat, &curPos.loc.lon, &curPos.alt); 
			gdl90State.lat = curPos.loc.lat; gdl90State.lon = curPos.loc.lon; // HACK : stuff the main loops gld90State just so initial data logs have valid looking data 

		} else if (strcmp(*a, "--tracksim") == 0) {
				tSim = new WaypointsSequencerFile(*(++a));
				//tSim = trackSimFile = ifstream(*(++a), ios_base::in | ios_base::binary);
				//trackSimFile = wrap_vector_as_istream(trackSimFileContents);
		} else if (strcmp(*a, "--logConvert") == 0) {
			ifstream i = ifstream(*(++a), ios_base::in | ios::binary);
			ofstream o = ofstream(*(++a), ios_base::out | ios::binary);			
			ESP32sim_convertLogOldToNew(i, o);
			o.flush();
			o.close();
			exit(0);
		} else if (strcmp(*a, "--debug") == 0) {
			vector<string> l = split(string(*(++a)), ',');
			float v;
			for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
				if (sscanf(it->c_str(), "zeros.mx=%f", &v) == 1) { ahrs.magOffX = v; } 
				else if (sscanf(it->c_str(), "zeros.my=%f", &v) == 1) { ahrs.magOffY = v; } 
				else if (sscanf(it->c_str(), "zeros.mz=%f", &v) == 1) { ahrs.magOffZ = v; } 
				else if (sscanf(it->c_str(), "zeros.gx=%f", &v) == 1) { ahrs.gyrOffX = v; } 
				else if (sscanf(it->c_str(), "zeros.gy=%f", &v) == 1) { ahrs.gyrOffY = v; } 
				else if (sscanf(it->c_str(), "zeros.gz=%f", &v) == 1) { ahrs.gyrOffZ = v; } 
				else if (sscanf(it->c_str(), "ahrs.debug=%f", &v) == 1) { ahrs.debugVar = v; } 
				else if (sscanf(it->c_str(), "cr1=%f", &v) == 1) { ahrs.compRatio1 = v; } 
				else if (sscanf(it->c_str(), "dc1=%f", &v) == 1) { ahrs.driftCorrCoeff1 = v; } 
				else if (sscanf(it->c_str(), "cr2=%f", &v) == 1) { ahrs.hdgCompRatio = v; } 
				else if (sscanf(it->c_str(), "mbt.cr=%f", &v) == 1) { ahrs.magBankTrimCr = v; } 
				else if (sscanf(it->c_str(), "mbt.maxerr=%f", &v) == 1) { ahrs.magBankTrimMaxBankErr = v; } 
				else if (sscanf(it->c_str(), "dipconstant=%f", &v) == 1) { ahrs.magDipConstant = v; } 
				else if (sscanf(it->c_str(), "ahrs.crpitch=%f", &v) == 1) { ahrs.compRatioPitch = v; } 
				else if (sscanf(it->c_str(), "ahrs.pitchoffset=%f", &v) == 1) { ahrs.pitchOffset = v; } 
				else if (sscanf(it->c_str(), "ahrs.rolloffset=%f", &v) == 1) { ahrs.rollOffset = v; } 
				else if (sscanf(it->c_str(), "ahrs.useauxmpu=%f", &v) == 1) { ESP32csim_useAuxMpu = v; } 
				else if (sscanf(it->c_str(), "ahrs.gxdecel=%f", &v) == 1) { ahrs.gXdecelCorrelation = v; } 
				else if (sscanf(it->c_str(), "ahrs.bankanglescale=%f", &v) == 1) { ahrs.bankAngleScale = v; }
				else if (strlen(it->c_str()) > 0) { 
					printf("Unknown debug parameter '%s'\n", it->c_str()); 
					exit(-1);
				}
			}
		}
	}	
	void setup() override {
		if (replayFile == NULL) { 
			bm.addPress(pins.knobButton, 1, 1, true); // knob long press - arm servo
			//bm.addPress(pins.botButton, 250, 1, true); // bottom long press - test turn activate 
			//bm.addPress(pins.topButton, 200, 1, false); // top short press - wings level mode  
			//bm.addPress(pins.topButton, 300, 1, false); // top short press - hdg hold
			ahrsInput.dtk = desiredTrk = 135;
			if (tSim != NULL) { 
				tSim->wptTracker.onSteer = [&](float s) { 
					ahrsInput.dtk = desiredTrk = trueToMag(s);
					return magToTrue(track);
				};
			}
		}
	}

	bool firstLoop = true;	
	float now, lastTime = 0;
	bool hz(float hz) { return floor(now * hz) != floor(lastTime * hz); }
	bool at(float t) { return now > t && lastTime < t; }
	void loop() override {
		now = _micros / 1000000.0;		
		if (replayFile == NULL) { 
			if (floor(now / .1) != floor(lastTime / .1)) {
				float g5hdg = hdg * M_PI / 180;
				float g5roll = -bank * M_PI / 180;
				float g5pitch = pitch * M_PI / 180;
				ESP32sim_udpInput(7891, strfmt("IAS=%f TAS=%f PALT=%f\n", 90, 100, 1000));
				ESP32sim_udpInput(7891, strfmt("P=%f R=%f\n", g5pitch, g5roll));
				// now handled by onNavigate();
				//ESP32sim_udpInput(7891, strfmt("HDG=%f\n", g5hdg));
			}
			flightSim(imu);
			if (hz100.tick(micros()/1000.0) && tSim != NULL) {
				tSim->run(hz100.interval / 1000.0);
			}

			if (at(150.0)) { 
				Serial.inputLine =  "wpclear\n"
									"wpadd REPEAT 1\n"
									"wpadd 47.59509212379994, -122.38743386638778 1000\n"
									"wpadd 47.59233901597324, -122.37080179677619 500\n"
									"wpadd 47.53887718258715, -122.30994494011797 50\n"
									"wpadd 47.46106431485166, -122.47652028295599\n"
									"wpstart\n";
				apMode = 3;
			}
			if (false && hz(1.0/6000)) {
				//ahrs.reset();
				//rollPID.reset();
				//hdgPID.reset();
				/*tSim.wptTracker.onNavigate = [](float) { 
					ahrsInput.dtk = desiredTrk = 100; 
					return magToTrue(ahrsInput.g5Hdg); 
				};*/
			}

		} else {
			if (firstLoop == true) { 
				ifile = ifstream(replayFile, ios_base::in | ios::binary);
			}
			while(logSkip > 0 && logSkip-- > 0) {
				ESP32sim_replayLogItem(ifile);
			}
			if (ESP32sim_replayLogItem(ifile) == false) { 
				ESP32sim_exit();
			}
			logEntries++;
		}
	
		//if (now >= 500 && lastTime < 500) {	Serial.inputLine = "pitch=10\n"; }
		//if (now >= 100 && lastTime < 100) {	Serial.inputLine = "zeroimu\n"; }
		firstLoop = false;
		lastTime = now;
	}
	void done() override { 
		printf("# %f %f %f avg roll/pitch/hdg errors, %d log entries, %.1f real time seconds\n", 
		totalError.roll / logEntries, totalError.pitch / logEntries,  totalError.hdg / logEntries, logEntries, millis() / 1000.0);
	}
} espsim;


#include "TinyGPS++.h"
#include "TinyGPS++.cpp"

#endif



/*
 * TTGO TS 1.4 PINOUTS
 * 
 * 	18 (PWM)	19 (LED)
 * 	33 (PWM)	RST
 * 	27 backlight 32  (KNOB)
 * 	GND(GND)	26  (ROT)
 * 	0 (ROT)		GND 		
 * 	GND			3.3V
 * 	RXD			22  (used by I2c?)
 * 	TXD			21	(used by I2c?)
 * 	VBAT		5V
 * 
 * 
 * 
 * 
 */

/*  TTGO TS 1.2 old- buttons 39, 34, 35
 *    39   36
 *    33   RST
 *    27   32
 *    GND  26
 *    0    GND
 *    GND  
 */
