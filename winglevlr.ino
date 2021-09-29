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
#include <rom/rtc.h>
#include <MPU9250_asukiaaa.h>
//#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <SparkFun_Ublox_Arduino_Library.h>
#endif // #else // UBUNTU

#include <TinyGPS++.h>

#include "jimlib.h"
#include "RollingLeastSquares.h"
#include "PidControl.h"
#include "RollAHRS.h"
#include "GDL90Parser.h"
#include "WaypointNav.h"

bool debugFastBoot = false;

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
PidControl rollPID(30) /*200Hz*/, pitchPID(10,6), hdgPID(50)/*20Hz*/, xtePID(100)/*5hz*/, altPID(25); /*5Hz*/
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
	int serial2_rx = 19;
	int serial2_tx = 27;
	int gps_rx = 19;
	int gps_tx = 27;


} pins;

MPU9250_asukiaaa *imu = NULL;


class UbloxGPS {
public: 
	SFE_UBLOX_GPS myGNSS;
	int gpsGood = 0;
	void init() {
		Serial.printf("Trying 115K BPS...\n");
		Serial2.begin(115200, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
		if (myGNSS.begin(Serial2) == false) { 
		Serial.printf("Trying 9.6K BPS...\n");
		Serial2.begin(9600, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
		if (myGNSS.begin(Serial2) == false)  {
			Serial.println("u-blox GNSS not detected. Please check wiring");
			return;
		}
		myGNSS.setSerialRate(115200); //Set UART1 to 57600bps.
		Serial2.begin(115200, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
		myGNSS.saveConfiguration();        //Optional: Save the current settings to flash and BBR

		}

		myGNSS.setUART1Output(COM_TYPE_UBX);
		myGNSS.setNavigationFrequency(10); 
		myGNSS.setAutoPVT(true, true); 
		//myGNSS.setAutoPVTrate(0.10); //Set output to 5 times a second

		gpsGood = 1;
		Serial.printf("Found GPS\n");
	}
	float lat, lon, hdg, hac, gs, siv, alt;
	bool check() { 
		if (gpsGood && myGNSS.getPVT(220)) {
			lon = myGNSS.getLongitude() / 10000000.0;
			hdg = myGNSS.getHeading() / 100000.0;
			lat = myGNSS.getLatitude() / 10000000.0;
			alt = myGNSS.getAltitudeMSL() / 1000.0;
			hac = myGNSS.getHeadingAccEst() / 100000.0;
			gs = myGNSS.getGroundSpeed() / 1000.0 / 0.51444;
			siv = myGNSS.getSIV();
			//Serial.printf("GPS: %+09.4f, %+09.4f %+05.1f %02d\n", lat, lon, hdg, siv);
			return true;
		}
		return false;
	}
} ublox; 


float stickTrimY = 0.00, stickTrimX = 0.10;
void halInit() { 
	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(1);

	Serial.printf("\nReset reason: %d %d\n", (int)rtc_get_reset_reason(0), (int)rtc_get_reset_reason(1));
	Serial.printf(__BASE_FILE__ " ver %s\n", GIT_VERSION);
	Wire.begin(21,22);
	//Wire.setClock(400000);
	Serial.println("Scanning I2C bus on pins 21,22");
	if (scanI2c() == 0) { 
		Wire.begin(19,18);
		//Wire.setClock(400000);
		Serial.println("Scanning I2C bus on pins 19,18");
		if (scanI2c() == 0) {
			Serial.println("No I2C devices found, rebooting...");
			ESP.restart();
		}
		Serial.println("Older TTGO-TS board, changing pin assignments, IMU likely oriented wrong");
		pins.sda = 19;
		pins.scl = 18;
		pins.midButton = 34;
		pins.topButton = 35;
		//pins.knobButton = 32;
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

	ublox.init();
}

float ubloxHdgCr = 0.0023;

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
	static const int ublox = 0x20;
}

namespace Display {
	JDisplay jd;
	int y = 0;
	const int c2x = 70;
	JDisplayItem<const char *>  ip(&jd,10,y,"WIFI:", "%s ");
	JDisplayItem<float>  dtk(&jd,10,y+=10," DTK:", "%05.1f ");  JDisplayItem<float>    trk(&jd,c2x,y,  " TRK:", "%05.1f ");
	JDisplayItem<float> pitc(&jd,10,y+=10,"PITC:", "%+03.1f ");  JDisplayItem<float>    obs(&jd,c2x,y,  " OBS:", "%05.1f ");
	JDisplayItem<float> roll(&jd,10,y+=10,"ROLL:", "%+03.1f");   JDisplayItem<int>    mode(&jd,c2x,y,  "MODE:", "%05d ");
	JDisplayItem<float>  gdl(&jd,10,y+=10," GDL:", "%05.1f ");  JDisplayItem<float> g5hdg(&jd,c2x,y,  "G5HD:", "%05.1f ");
	//JDisplayItem<float> xtec(&jd,10,y+=10,"XTEC:", "%+05.1f "); JDisplayItem<float> roll(&jd,c2x,y,    " RLL:", "%+05.1f ");
	JDisplayItem<const char *> log(&jd,10,y+=10," LOG:", "%s-"); JDisplayItem<int>   drop(&jd,c2x+30,y,    "", "%03d ");
    JDisplayItem<float> logw(&jd,10,y+=10,"LOGW:", "%05.0f ");
	
	JDisplayItem<float> pidpl(&jd,00,y+=10,"PL:", "%03.2f "); JDisplayItem<float> tttt(&jd,c2x,y,    " TT1:", "%04.1f ");
	JDisplayItem<float> pidph(&jd,00,y+=10,"PH:", "%03.2f "); JDisplayItem<float> ttlt(&jd,c2x,y,    " TT2:", "%04.1f ");;
	JDisplayItem<float>  pidi(&jd,00,y+=10," I:", "%03.2f "); JDisplayItem<float> maxb(&jd,c2x,y,    "MAXB:", "%04.1f ");
	JDisplayItem<float>  pidd(&jd,00,y+=10," D:", "%03.2f "); JDisplayItem<float> maxi(&jd,c2x,y,    "MAXI:", "%04.1f ");
	JDisplayItem<float>  pidg(&jd,00,y+=10," G:", "%03.2f "); JDisplayItem<float> ptrim(&jd,c2x,y,  "PTRM:", "%+4.1f");
	JDisplayItem<float>  dead(NULL,00,y+=00,"DZ:", "%03.1f "); 
    JDisplayItem<float>  dalt(&jd,10,y+=10,"DALT:", "%05.0f "); JDisplayItem<float> pidsel(&jd,c2x,y,  " PID:", "%1.0f");		

    //JDisplayItem<float> navt(NULL,10,y+=10,"NAVT:", "%05.1f ");
}


float lastDesAlt = 0.0;
class MyEditor : public JDisplayEditor {
public:
	JDisplayEditableItem dtrk = JDisplayEditableItem(&Display::dtk, 1, 0, 359, true);
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
//	JDisplayEditableItem tzer = JDisplayEditableItem(NULL, 1);
	JDisplayEditableItem pidsel = JDisplayEditableItem(&Display::pidsel, 1, 0, 4);
	JDisplayEditableItem pitchTrim = JDisplayEditableItem(&Display::ptrim, .1);
	JDisplayEditableItem dead = JDisplayEditableItem(&Display::dead, .1);
	JDisplayEditableItem desAlt = JDisplayEditableItem(&Display::dalt, 20);
	
	MyEditor() : JDisplayEditor(26, 0) { // add in correct knob selection order
		add(&dtrk);	
		add(&pidpl);	
		add(&pidph);	
		add(&pidi);	
		add(&pidd);	
		add(&pidg);	
		add(&desAlt);
		add(&tttt);
		add(&ttlt);	
		add(&maxb);
		add(&maxi);
		add(&pitchTrim);
		add(&pidsel);
		//add(&desAlt);
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
	if (false && lastUsec / 10000 != micros() / 10000) { 
		AuxMpuData a;
		a.ax = x.ax; a.ay = x.ay; a.az = x.az;
		a.gx = x.gx; a.gy = x.gy; a.gz = x.gz;
		a.mx = x.mx; a.my = x.my; a.mz = x.mz;
		String ad = a.toString() + "\n";
		udpG90.beginPacket("255.255.255.255", 7892);
		udpG90.write((uint8_t *)ad.c_str(), ad.length());
		udpG90.endPacket();
	}	
	lastUsec = micros();
	// remaining items set (alt, hdg, speed) set by main loop
	
	return true;

}

LogItem logItem;
SDCardBufferedLog<LogItem>  *logFile = NULL;
bool logChanging = false;
const char *logFileName = "AHRSD%03d.DAT";
static AhrsInput lastAhrsInput, lastAhrsGoodG5; 
static StaleData<float> gpsTrackGDL90(3000,-1), gpsTrackRMC(5000,-1), gpsTrackVTG(5000,-1);
static StaleData<int> canMsgCount(3000,-1);
static float desiredTrk = -1;
float desRoll = 0, /*pitchTrim = -8,*/ pitchToStick = 0.15, desPitch = 0;//, desAlt = 0;		
static int serialLogFlags = 0;

void setDesiredTrk(float f) { 
	desiredTrk = round(f);
	ed.dtrk.setValue(desiredTrk);
}

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

	esp_task_wdt_init(22, true);
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

	if (!debugFastBoot && !buttonKnob.read() && !buttonTop.read()) { // skip long setup stuff if we're debugging
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
	rollPID.hiGain.p = 2;
	rollPID.hiGainTrans.p = 5;
	rollPID.finalGain = 16.8;
	rollPID.maxerr.i = 20;

	hdgPID.setGains(0.25, 0.07, 0.02);
	hdgPID.hiGain.p = 10;
	hdgPID.hiGainTrans.p = 8.0;
	hdgPID.maxerr.i = 20;
	hdgPID.finalGain = 1.0;

	xtePID.setGains(8.0, 0.00, 0.10);
	xtePID.maxerr.i = 1.0;
	xtePID.finalGain = 15.0;
	
	pitchPID.setGains(20.0, 0.0, 2.0, 0, .8);
	pitchPID.finalGain = 5.0;
	pitchPID.maxerr.i = .5;

	altPID.setGains(1.0, 0.01, 3.0);
	altPID.finalGain = -.5;
	pitchPID.maxerr.i = 100;

    // make PID select knob display text from array instead of 0-3	
	Display::pidsel.toString = [](float v){ return String((const char *[]){"PIT ", "ALT ", "ROLL", "XTE ", "HDG "}[(v >=0 && v <= 4) ? (int)v : 0]); 
	};		
	ed.begin();

#ifndef UBUNTU
	ed.re.begin([ed]()->void{ ed.re.ISR(); });
#endif
	ed.maxb.setValue(12);
	ed.tttt.setValue(60); // seconds to make each test turn 
	ed.ttlt.setValue(75); // seconds betweeen test turn, ordegrees per turn   
	//ed.tzer.setValue(1000);
	ed.pidsel.setValue(1);
	ed.dtrk.setValue(desiredTrk);
	ed.desAlt.setValue(1000);
	ed.pitchTrim.setValue(0);
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

void setKnobPid(int f) { 
	Serial.printf("Knob PID %d\n", f);
	if      (f == 0) { knobPID = &pitchPID; }
	else if (f == 1) { knobPID = &altPID;}
	else if (f == 2) { knobPID = &rollPID; }
	else if (f == 3) { knobPID = &xtePID; }
	else if (f == 4) { knobPID = &hdgPID; }
	
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
static EggTimer serialReportTimer(200), loopTimer(5), buttonCheckTimer(10);
static int armServo = 0;
static int servoSetupMode = 0; // Referenced when servos not armed.  0: servos left alone, 1: both servos neutral + trim, 2: both servos full in, 3: both servos full out
static int apMode = 1; // apMode == 4 means follow NMEA HDG and XTE sentences, anything else tracks OBS
static int hdgSelect = 3; //  0 use fusion magHdg 
static float obs = -1, lastObs = -1;
static bool screenReset = false, screenEnabled = true;
struct {
	double hdg, roll, pitch, hdgSum, rollSum, pitchSum;
	void clear() { hdg = roll = pitch = hdgSum = rollSum = pitchSum = 0; }
} totalError;
//static int ledOn = 0;
static int manualRelayMs = 60;
static int gpsFixes = 0, udpBytes = 0, serBytes = 0, apUpdates = 0;
static uint64_t lastLoop = micros();
static bool selEditing = false;
float stickX, stickY;
std::string waypointList;
WaypointNav::WaypointSequencer *wpNav = NULL;
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
		//else if (sscanf(line, "ptrim=%f", &f) == 1) { ed.tzer.value = f; }
		//else if (sscanf(line, "mtin=%f", &f) == 1) { ed.mtin.value = f; }
		else if (strstr(line, "zeroimu") == line) { ahrs.zeroSensors(); }
		else if (sscanf(line, "dtrk=%f", &f) == 1) { setDesiredTrk(f); }
		else if (sscanf(line, "s %f %f", &f, &f2) == 2) { servoOutput[0] = f; servoOutput[1] = f2; }
		else if (sscanf(line, "strim %f %f", &f, &f2) == 2) { stickTrimX = f; stickTrimY = f2; }
		else if (sscanf(line, "strimx %f", &f) == 1) { stickTrimX = f; }
		else if (sscanf(line, "strimy %f", &f) == 1) { stickTrimY = f; }
		else if (sscanf(line, "ptrim %f", &f) == 1) { ed.pitchTrim.setValue(f); }
		else if (sscanf(line, "p2stick %f", &f) == 1) { pitchToStick = f; }
		else if (sscanf(line, "mode %f", &f) == 1) { apMode = f; }
		else if (sscanf(line, "knob=%f", &f) == 1) { setKnobPid(f); }
		else if (sscanf(line, "alt %f", &f) == 1) { ed.desAlt.value = f; }
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
		ed.desAlt.value = v * 3.2808;
	}
	if (knobSel == 1 /*|| knobSel == 4*/) {
		obs = v * 180.0 / M_PI;
		if (obs <= 0) obs += 360;
		if (apMode != 4 && obs != lastObs) {
			setDesiredTrk(obs);
			crossTrackError.reset();
			testTurnActive = false;
		}
		if (apMode == 4 && obs != lastObs) { 
			xtePID.reset();
		}
		lastObs = obs;
	}	
}



namespace ServoControlOld { 
	const float servoThrow = 2.0;
	const float leftStringX = 14;
	const float leftStringY = 7;

	const float rightStringX = 11;
	const float rightStringY = 7;

	const float leftLen = sqrt(leftStringX * leftStringX + leftStringY * leftStringY);
	const float rightLen = sqrt(rightStringX * rightStringX + rightStringY * rightStringY);

	float xScale = +1.0;
	float yScale = +1.0;

	pair<int, int> stickToServo(float x, float y) { 
		float x1 = leftStringX + xScale * x;
		float y1 = leftStringY + yScale * y;
		float leftNewLen = sqrt(x1 * x1 + y1 * y1);

		x1 = rightStringX - xScale * x;
		y1 = rightStringY + yScale * y;
		float rightNewLen = sqrt(x1 * x1 + y1 * y1);

		float s0 =  +(rightNewLen - rightLen) / servoThrow * 2000 + 1500;
		float s1 =  +(leftNewLen - leftLen) / servoThrow * 2000 + 1500;

		return pair<int, int>(s0, s1);
	}

	pair<float,float> servoToStick(int s0, int s1) {
		float a = leftStringX + rightStringX;
		float b = (s0 - 1500.0) / 2000 * servoThrow + rightLen; 
		float c = (s1 - 1500.0) / 2000 * servoThrow + leftLen; 

		float area = sqrt( 4 * a * a * b * b - (a * a + b * b - c * c) * (a * a + b * b - c * c)) / 4;
		float Y = area * 2 / a;
		float X = sqrt(c * c - Y * Y);

		float x = (X - leftStringX) / xScale;
		float y = (Y - leftStringY) / yScale;

		return pair<float,float>(x, y);
	}
};

namespace ServoControl { 
	const float servoThrow = +2.0;

	pair<int, int> stickToServo(float x, float y) { 
		float s1 = -x / servoThrow * 2000 + 1500;
		float s0 = -y / servoThrow * 2000 + 1500;
		return pair<int, int>(s0, s1);
	}

	pair<float,float> servoToStick(int s0, int s1) {
		float x = -(s1 - 1500) / 2000.0 * servoThrow;
		float y = -(s0 - 1500) / 2000.0 * servoThrow;
		
		return pair<float,float>(x, y);
	}
};


void setServos(float x, float y) {
	pair<int,int> s = ServoControl::stickToServo(x, y);
	servoOutput[0] =  max(450, min(2550, s.first));
	servoOutput[1] =  max(450, min(2550, s.second));
}

bool firstLoop = true;
bool immediateLogStart = false;

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
		// SERIAL STATUS line output 
		Serial.printf(
			"%06.2f "
			//"R %+05.2f BA %+05.2f GZA %+05.2f ZC %03d MFA %+05.2f"
			"%+05.2f,%+05.2f R%+05.1f P%+05.1f DP%+05.1f "
			"A%04.0f DA%04.0f "
			//"%+05.2f %+05.2f %+05.2f %+05.1f srv %04d xte %3.2f "
			"C %+06.2f %+05.1f %+05.1f %+05.1f " 
			"but %d%d%d%d loop %d/%d/%d heap %d re.count %d logdrop %d maxwait %d "
			"a%d "
			//"s%04d %04d "			
			"\n",
			millis()/1000.0,
			//roll, ahrs.bankAngle, ahrs.gyrZOffsetFit.average(), ahrs.zeroSampleCount, ahrs.magStabFit.average(),   
			stickX, stickY, roll, pitch, desPitch, 
			ahrsInput.alt, ed.desAlt.value,
			//0.0, 0.0, 0.0, 0.0, servoOutput, crossTrackError.average(),
			knobPID->err.p, knobPID->err.i, knobPID->err.d, knobPID->corr, 
			buttonTop.read(), buttonMid.read(), buttonBot.read(), buttonKnob.read(), (int)loopTime.min(), (int)loopTime.average(), (int)loopTime.max(), ESP.getFreeHeap(), ed.re.count, 
				logFile != NULL ? logFile->dropped : 0, logFile != NULL ? logFile->maxWaiting : 0, 
			auxMpuPacketCount, 
			//servoOutput[0], servoOutput[1],
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
	//         long    - servo setup mode 1, 2, 3, 4
	//         triple  - zero sensors
	// MIDDLE: short   - left 10 degrees
	//         double  - hdg select mode
	//         long    - start/stop log
	// BOTTOM: short   - right 10 degrees
	//		   long    - alt hold 
	//         double  - active test turn sequence 
	// KNOB    long    - arm servo
	//         triple  - servo test mode
	           
	//ed.re.check();
	if (firstLoop == true && (digitalRead(buttonMid.pin) == 0  || debugFastBoot)) { 
		logFile = new SDCardBufferedLog<LogItem>(logFileName, 500/*q size*/, 0/*timeout*/, 500/*flushInterval*/, false/*textMode*/);
		logFilename = logFile->currentFile;
		logChanging = false;
		immediateLogStart = true;
	}
	if (buttonCheckTimer.tick()) { 
		buttonISR();
		if (butFilt3.newEvent()) { // TOP or RIGHT button 
			if (butFilt3.wasCount == 1 && butFilt3.wasLong == false) {		// SHORT: Stop tracking NMEA dest, toggle desired track between -1/current heading
				apMode = 1;
				if (desiredTrk == -1) 
					setDesiredTrk(ahrsInput.selTrack);
				else 
					setDesiredTrk(-1);
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
				if (wpNav != NULL) {
					delete wpNav;
					wpNav = NULL;
				} else {
					waypointList = 
"REPEAT 1\n"
"47.47329740698361, -122.60949120308448 1800\n"
"47.42959741100363, -122.53173591135885 1700\n" 
"47.4331127570086, -122.64209866008706  1800\n" 
"47.47560332145362, -122.49804894088612 1900\n"
;

					wpNav = new WaypointsSequencerString(waypointList);
					apMode = 3;	
				}
			}
		}
		if (butFilt.newEvent()) { // MIDDLE BUTTON
			if (!butFilt.wasLong) {
				if (butFilt.wasCount == 1) {
					setDesiredTrk(constrain360(desiredTrk - 10));
					apMode = 1;
				} else { 
					hdgSelect = (hdgSelect + 1) % 4;
				}					
			} else { 
				if (!logChanging && (immediateLogStart != true || millis() > 10000)) {
					logChanging = true;
					if (logFile == NULL) {	
							logFile = new SDCardBufferedLog<LogItem>(logFileName, 200/*q size*/, 0/*timeout*/, 5000/*flushInterval*/, false/*textMode*/);
							logFilename = logFile->currentFile;
							logChanging = false;
					} else {
						delete logFile;
						logFile = NULL;
						logChanging = false;
					}
				}
				immediateLogStart = false;
			}
			
		}
		if (butFilt2.newEvent()) { // BOTTOM or LfffEFT button
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == true) {
				armServo = !armServo;
				servoSetupMode = 0;
				//rollPID.reset();
			}
			if (butFilt2.wasCount == 1 && butFilt2.wasLong == false) {	
				setDesiredTrk(constrain360(desiredTrk + 10));
				apMode = 1;
			}
			if (butFilt2.wasCount == 2) {
				testTurnActive = !testTurnActive;
				testTurnAlternate = 0;
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
				setDesiredTrk(desiredTrk + ed.ttlt.value * (testTurnAlternate == 0 ? -1 : 1));
			}
		}
			
	}

	if (udpG90.parsePacket() > 0) { 
		unsigned char buf[1024]; 
		int n = udpG90.read(buf, sizeof(buf));
		for (int i = 0; i < n; i++) {  
			gdl90.add(buf[i]);
			GDL90Parser::State s = gdl90.getState();
			if (s.valid && s.updated) {
				gpsTrackGDL90 = trueToMag(s.track);
				ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.02);
				logItem.flags |= LogFlags::HdgGDL; 
				gpsFixes++;
				ahrsInput.alt = s.alt * 3.2808;
				ahrsInput.palt = s.palt;// * 25 + 1000;
				ahrsInput.gspeed = s.hvel;
				gdl90State = s;
			}
		}
	}

	if (udpNMEA.parsePacket() > 0) { 
		char buf[1024];
		static LineBuffer lb;
		int n = udpNMEA.read((uint8_t *)buf, sizeof(buf));
		lb.add((char *)buf, n, [](const char *line) { 
			if (auxMPU.fromString(line)) {
				auxMpuPacketCount++;
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

	if (udpSL30.parsePacket() > 0) { 
		uint8_t buf[1024];
		static LineBuffer lb;
		int n = udpSL30.read(buf, sizeof(buf));
		lb.add(buf, n, [](const char *line) {
			vector<string> l = split(string(line), ' ');
			float v, knobSel = 0;
			for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
				if      (sscanf(it->c_str(), "P=%f",   &v) == 1) { ahrsInput.g5Pitch = v; canMsgCount = canMsgCount + 1; logItem.flags |= LogFlags::g5Ins; }  
				else if (sscanf(it->c_str(), "R=%f",   &v) == 1) { ahrsInput.g5Roll = v; logItem.flags |= LogFlags::g5Ins; } 
				else if (sscanf(it->c_str(), "IAS=%f", &v) == 1) { ahrsInput.g5Ias = v; logItem.flags |= LogFlags::g5Ps;} 
				else if (sscanf(it->c_str(), "TAS=%f", &v) == 1) { ahrsInput.g5Tas = v; logItem.flags |= LogFlags::g5Ps;} 
				else if (sscanf(it->c_str(), "PALT=%f", &v) == 1) { ahrsInput.g5Palt = v; logItem.flags |= LogFlags::g5Ps;} 
				else if (sscanf(it->c_str(), "HDG=%f", &v) == 1) { 
					ahrsInput.g5Hdg = v; logItem.flags |= LogFlags::g5Nav;
					ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.03);
				} 
				else if (sscanf(it->c_str(), "TRK=%f", &v) == 1) { ahrsInput.g5Track = v; logItem.flags |= LogFlags::g5Nav; } 
				else if (sscanf(it->c_str(), "MODE=%f", &v) == 1) { apMode = v; } 
				else if (sscanf(it->c_str(), "KSEL=%f", &v) == 1) { knobSel = v; } 
				else if (sscanf(it->c_str(), "KVAL=%f", &v) == 1) { setObsKnob(knobSel, v); } 
			}
			
			float pit, roll, magHdg, magTrack, knobVal, ias, tas, palt, age;
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
				//ahrsInput.g5TimeStamp = (millis() - (uint64_t)age) / 1000.0;
				apMode = mode;
				ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.03);
				logItem.flags |= (LogFlags::g5Nav | LogFlags::g5Ps | LogFlags::g5Ins);
				setObsKnob(knobSel, knobVal);
				//Serial.printf("knob sel %f, knob val %f\n", knobSel, knobVal);
				canMsgCount = canMsgCount + 1;
			}

		});

		// TODO - get rid of TinyGPSPlus, just use pattern matchine 
		for (int i = 0; i < n; i++) {
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

	if (ublox.check()) { 
		logItem.flags |= LogFlags::ublox;
		ahrsInput.ubloxHdg = trueToMag(ublox.hdg);
		ahrsInput.ubloxHdgAcc = ublox.hac;
		ahrsInput.ubloxAlt = ublox.alt * FEET_PER_METER;
		ahrsInput.ubloxGroundSpeed = ublox.gs;
		ahrsInput.lat = ublox.lat;
		ahrsInput.lon = ublox.lon;
		if (ublox.hac < 7) {
			ahrs.mComp.addAux(ahrsInput.ubloxHdg, 10, ubloxHdgCr);
		}
	}

	//printMag();
	if (imuRead()) { 
		bool tick1HZ = floor(ahrsInput.sec) != floor(lastAhrsInput.sec);
		bool tick20HZ = floor(ahrsInput.sec * 20.0) != floor(lastAhrsInput.sec * 20.0);
		bool tick5HZ = floor(ahrsInput.sec * 5.0) != floor(lastAhrsInput.sec * 5.0);

		if (tick5HZ) { 
			if (apMode == 3 && wpNav != NULL) {
				wpNav->wptTracker.curPos.loc.lat = ublox.lat;
				wpNav->wptTracker.curPos.loc.lon = ublox.lon;
				wpNav->wptTracker.curPos.alt = ahrsInput.ubloxAlt / FEET_PER_METER;
				wpNav->wptTracker.speed = ahrsInput.ubloxGroundSpeed;
				wpNav->wptTracker.curPos.valid = true;
				wpNav->run(ahrsInput.sec - lastAhrsInput.sec);
				if (tick1HZ) {
					crossTrackError.add(wpNav->wptTracker.xte * .0005);
				}
				xteCorrection = -xtePID.add(crossTrackError.average(), crossTrackError.average(), ahrsInput.sec);					
				xteCorrection = max(-40.0, min(40.0, (double)xteCorrection));
				setDesiredTrk(trueToMag(wpNav->wptTracker.commandTrack) + xteCorrection);
				ed.desAlt.value = wpNav->wptTracker.commandAlt * 3.2808;
				ahrsInput.dtk = desiredTrk;

			} else if (apMode == 4) {
				xteCorrection = -xtePID.add(crossTrackError.average(), crossTrackError.average(), ahrsInput.sec);					
				xteCorrection = max(-40.0, min(40.0, (double)xteCorrection));
				setDesiredTrk(navDTK + xteCorrection);
				ahrsInput.dtk = desiredTrk;
			} else { 
				xteCorrection = 0;
			}
			if ((apMode == 3 && ed.desAlt.value != -1000)) { 
				float altErr = ed.desAlt.value - ahrsInput.ubloxAlt;
				if (abs(altErr) > 500) {
					altPID.resetI();					
				}	
				altPID.add(altErr, ahrsInput.ubloxAlt, ahrsInput.sec);
			} else { 
				altPID.reset();
			}
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
		
		// TODO:  it seems hdgPID was tuned for 20Hz.   Accidently moved into 5hz loop? 
		if (tick20HZ) {
			if (ahrsInput.dtk != -1) { 
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
		float altCorr = max(min(altPID.corr * 0.01, 3.0), -3.0);
		desPitch = ed.pitchTrim.value + altCorr;
		pitchPID.add(ahrs.pitch - desPitch, ahrs.pitch - desPitch, ahrsInput.sec);

		if (armServo == true) {  
			// TODO: pids were tuned and output results in units of relative uSec servo PWM durations. 
			// hack tmp: convert them back into inches so we can add in inch-specified trim values 
			stickX = stickTrimX + rollPID.corr / 2000 * ServoControl::servoThrow;
			stickY = stickTrimY + pitchPID.corr / 2000 * ServoControl::servoThrow +
				(desPitch - ed.pitchTrim.value) * pitchToStick; 
			//	y = 0; // disable pitch
			stickX += cos(millis() / 100.0) * .04;
			stickY += sin(millis() / 100.0) * .04;
			setServos(stickX, stickY);
		} else switch(servoSetupMode) { 
			case 0:
				// leave servos where they are  
				break;
			case 1: 
				stickX = stickTrimX;
				stickY = stickTrimY;  
				stickX += cos(millis() / 100.0) * .04;
				stickY += sin(millis() / 100.0) * .04;
				setServos(stickX, stickY); 
				break;
			case 2:
				stickX = stickY = +8;  
				setServos(stickX, stickY); 
				break;
			case 3: 
				stickX = stickY = -8;  
				setServos(stickX, stickY); 
				break;
		}
		
		ledcWrite(0, servoOutput[0] * 4915 / 1500); // scale PWM output to 1500-7300 
		ledcWrite(1, servoOutput[1] * 4915 / 1500); // scale PWM output to 1500-7300 

		logItem.pwmOutput0 = servoOutput[0];
		logItem.pwmOutput1 = servoOutput[1];
		logItem.desRoll = desRoll;
		logItem.desAlt = (apMode == 3) ? ed.desAlt.value : -1000;
		logItem.desRoll = desRoll;
		logItem.roll = roll;
		logItem.magHdg = ahrs.magHdg;
		logItem.xte = crossTrackError.average();
		//logItem.bankAngle = ahrs.bankAngle;
		//logItem.magBank = ahrs.magBank;
		logItem.pitch = ahrs.pitch; 
		logItem.ai = ahrsInput;
		//logItem.auxMpu = auxMPU;

		totalError.roll += abs(roll + ahrsInput.g5Roll);
		totalError.rollSum += roll + ahrsInput.g5Roll;
		totalError.hdg += abs(angularDiff(ahrs.magHdg - ahrsInput.ubloxHdg));
		totalError.hdgSum += (angularDiff(ahrs.magHdg - ahrsInput.ubloxHdg));
		totalError.pitch += abs(pitch - ahrsInput.g5Pitch);
		totalError.pitchSum += (pitch - ahrsInput.g5Pitch);
	
#ifdef UBUNTU
		static bool errorsCleared = false; 
		if (errorsCleared == false && millis() < 200000) {  // don't count error during the first 200 sec, let AHRS stabilize  
			totalError.clear();
			errorsCleared = true;
		}
	
		// special logfile name "+", write out log with computed values from the current simulation 			
		if (strcmp(logFilename.c_str(), "+") == 0) {
			pair<float,float> stick = ServoControl::servoToStick(servoOutput[0], servoOutput[1]); 
			cout << logItem.toString().c_str() << strfmt("%+011.5lf %+011.5lf %06.3f %f	LOG U", gdl90State.lat, gdl90State.lon,
				stick.first, stick.second) << endl;
		}
#endif
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
		//Display::dtk = desiredTrk; 
		Display::trk = ahrsInput.selTrack; 
		//Display::navt = auxMPU.gy; //navDTK; 
		Display::obs = obs; 
		Display::mode = (canMsgCount.isValid() ? 10000 : 0) + apMode * 1000 + armServo * 100 + hdgSelect * 10 + (int)testTurnActive; 
		Display::gdl = (float)gpsTrackGDL90;
		Display::g5hdg = ublox.hdg;
		//Display::g5hdg = (float)ahrsInput.g5Hdg;
		//Display::zsc = ahrs.getGyroQuality(); 
		Display::roll = roll; 
		Display::pitc = pitch; 
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
	logItem.flags = 0;
	firstLoop = false;
}

#ifdef UBUNTU
///////////////////////////////////////////////////////////////////////////////
// Code below this point is used in compiling/running ESP32sim simulation


void ESP32sim_done();

class ESP32sim_winglevlr : public ESP32sim_Module {
public:
	IntervalTimer hz100 = IntervalTimer(100/*msec*/);

	string wpFile;
	ifstream ifile;
	const char *replayFile = NULL;
	int logSkip = 0;
	int logEntries = 0;
	float bank = 0, track = 0, pitch = 0, roll = 0, yaw = 0, simPitch = 0, hdg = 0;
	RollingAverage<float,30> rollCmd;
	uint64_t lastMillis = 0;
	float cmdPitch;
	float speed = 80;
	WaypointNav::LatLonAlt curPos;
	std::queue<float> gxDelay, pitchDelay;
	
	uint64_t lastMicros = 0;


	void flightSim(MPU9250_DMP *imu) { 
		//TODO: flightSim is very fragile/unstable.  Poke values into the
		// main loop code to make sure things work. 
		hdgPID.finalGain = 0.5;
		stickTrimY = stickTrimX = 0;
		ahrs.gyrOffZ = 1;

		_micros = (_micros + 5000);
		_micros -= (_micros % 5000);
		//const float servoTrim = 4915.0;

		// Simulate simple airplane roll/bank/track turn response to 
		// servooutput read from ESP32sim_currentPwm;
		pair<float,float> stick = ServoControl::servoToStick(
			ESP32sim_currentPwm[0] * 1500.0 / 4915, 
			ESP32sim_currentPwm[1] * 1500.0 / 4915);
		float stickX = stick.first;
		float stickY = stick.second;

		cmdPitch = stickY * 10.5;
		float ngx = (cmdPitch - simPitch) * 0.15;
		imu->gx = max((float)-15.0,min((float)15.0, ngx));
		simPitch += (cmdPitch - simPitch) * 0.15;
		this->pitch = simPitch;

		rollCmd.add(stickX);
		imu->gy = ahrs.gyrOffY + rollCmd.average() * 2;
		imu->gy *= -1;
		bank += imu->gy * (3500.0 / 1000000.0) * 2.2;
		bank = max(-15.0, min(15.0 , (double)bank));
		if (1 && floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f SIM\n", (float)millis()/1000.0, 
			ESP32sim_currentPwm[0], track, desRoll, bank, imu->gy);
		}		
		imu->gz = ahrs.gyrOffZ + tan(bank * M_PI/180) / speed * 1091;
		imu->gz *= -1;

		uint64_t now = millis();
		const float bper = .04;
		if (floor(lastMillis * bper) != floor(now * bper)) { // 10hz
			track += (tan(((bank + ahrs.rollOffset)) * M_PI / 180) * 9.8 / 40 * 25) * bper * 2.5;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;
			set_gpsTrack(trueToMag(track));
		}

		hdg = track - 35.555; // simluate mag var and arbitrary WCA 
		if (hdg < 0) hdg += 360;	

		if (0) { 
			printf("SIM %08.3f (%+04.1f,%+04.1f) %+05.2f %+05.2f %+05.2f %+05.2f\n", 
				(float)(millis()/1000.0), stickX, stickY, imu->gx, cmdPitch, pitch, logItem.pitch);
		}
		const float simPitchOffset = 0.0;
		imu->az = cos((this->pitch + simPitchOffset) * M_PI / 180) * 1.0;
		imu->ay = sin((this->pitch + simPitchOffset)* M_PI / 180) * 1.0;
		imu->ax = 0;

		// simulate meaningless mag readings that stabilize when bank == 0 
		imu->mx = cos(hdg * M_PI/180) * 50;
		imu->my = sin(hdg * M_PI/180) * 50 + 50;
		imu->mz = -70;
		//imu->mx =imu->my = imu->mz = 0;
		ahrs.magBankTrim = 0; // TODO simulate mag so this doesn't oscillate
		//ahrs.magHdg = hdg;
		
		float dist = speed * 0.51444 * (now - lastMillis) / 1000.0; 
		curPos.loc = WaypointNav::locationBearingDistance(curPos.loc, magToTrue(track), dist);
		curPos.alt += sin((pitch + simPitchOffset) * M_PI/180) * dist; 

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
			//auxMPU = l.auxMpu;

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
			if ((l.flags & LogFlags::ublox) || (l.ai.ubloxHdg != ahrsInput.ubloxHdg)) { 
				ublox.myGNSS.hdg = magToTrue(l.ai.ubloxHdg) * 100000.0;
				ublox.myGNSS.hac = l.ai.ubloxHdgAcc * 100000.0;
				ublox.myGNSS.alt = l.ai.ubloxAlt * 1000.0 / FEET_PER_METER;
				ublox.myGNSS.gs = l.ai.ubloxGroundSpeed * 0.51444 * 1000;
				
#ifdef UBUNTU
				// TMP hack: make up for incorrectly logged ublox grounspeed
				// *********************************************************
				//ublox.myGNSS.gs = l.ai.ubloxGroundSpeed * 1000;
				//assert(l.ai.ubloxGroundSpeed < 75);
				// *********************************************************
#endif

				ublox.myGNSS.fresh = true;
			}
			if (abs(angularDiff(ahrsInput.gpsTrackRMC - l.ai.gpsTrackRMC)) > .1 || (l.flags & LogFlags::HdgRMC) != 0) { 
				char buf[128];
				snprintf(buf, sizeof(buf), "GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,%.2f,130495,003.8,E", 
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
					s.palt = l.ai.palt;
					s.alt = l.ai.alt / FEET_PER_METER;
					s.lat = curPos.loc.lat;
					s.lon = curPos.loc.lon;
					int len = gdl90.packMsg10(buf, sizeof(buf), s);
					ESP32sim_udpInput(4000, string((char *)buf, len));
				}
			}
				
			servoOutput[0] = l.pwmOutput0;
			servoOutput[1] = l.pwmOutput1;
			setDesiredTrk(l.ai.dtk);

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
			s.hvel = speed;
			s.palt = (s.alt + 1000) / 25;

			ublox.myGNSS.hdg = t1 * 100000.0;
			ublox.myGNSS.hac = 5;
			ublox.myGNSS.alt = curPos.alt * 1000.0;
			ublox.myGNSS.lat = curPos.loc.lat * 10000000;
			ublox.myGNSS.lon = curPos.loc.lon * 10000000;
			ublox.myGNSS.gs =  s.hvel * 0.51444 * 1000.0;
			ublox.myGNSS.fresh = true;


			WiFiUDP::InputData buf;
			buf.resize(128);
			int n = gdl90.packMsg11(buf.data(), buf.size(), s);
			buf.resize(n);
			ESP32sim_udpInput(4000, buf);
			buf.resize(128);
			n = gdl90.packMsg10(buf.data(), buf.size(), s);
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
			ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.02);					
		}
	}

	std::vector<char> trackSimFileContents;
	//wrap_vector_as_istream tsf; 
	ifstream gdl90file; 
	void parseArg(char **&a, char **la) override {
		if (strcmp(*a, "--replay") == 0) replayFile = *(++a);
		else if (strcmp(*a, "--replaySkip") == 0) logSkip = atoi(*(++a));
		else if (strcmp(*a, "--log") == 0) { 
			//bm.addPress(pins.midButton, 1, 1, true);  // long press bottom button - start log 1 second in  
			logFilename = (*(++a));
		} else if (strcmp(*a, "--startpos") == 0) {
			sscanf(*(++a), "%lf,%lf,%f,%f,%f", &curPos.loc.lat, &curPos.loc.lon, &curPos.alt, &track, &speed);
			curPos.alt /= FEET_PER_METER;
			gdl90State.lat = curPos.loc.lat; 
			gdl90State.lon = curPos.loc.lon; // HACK : stuff the main loops gld90State just so initial data logs have valid looking data 
			set_gpsTrack(track);
		} else if (strcmp(*a, "--plotcourse") == 0) {
			float hdg, dist, alt, repeat;
			sscanf(*(++a), "%f", &repeat);
			char **fa = a;
			while(repeat-- > 0) {
				a = fa;
				while(sscanf(*(++a), "%f,%f,%f", &hdg, &dist, &alt) == 3) {
					track += hdg;
					curPos.loc = WaypointNav::locationBearingDistance(curPos.loc, track, dist);
					curPos.alt += alt;
					printf("%+011.8f, %+011.8f %05f\n", curPos.loc.lat, curPos.loc.lon, curPos.alt);		
				} 
			}
			exit(0);
		} else if (strcmp(*a, "--tracksim") == 0) {
				wpFile = *(++a);
		} else if (strcmp(*a, "--button") == 0) {
			int pin, clicks, longclick;
			float tim;
			sscanf(*(++a), "%f,%d,%d,%d", &tim, &pin, &clicks, &longclick);
			bm.addPress(pin, tim, clicks, longclick);

		} else if (strcmp(*a, "--logConvert") == 0)	 {
			ifstream i = ifstream(*(++a), ios_base::in | ios::binary);
			ofstream o = ofstream(*(++a), ios_base::out | ios::binary);			
			ESP32sim_convertLogOldToNew(i, o);
			o.flush();
			o.close();
			exit(0);
		} else if (strcmp(*a, "--gdl") == 0) { 
            gdl90file = ifstream(*(++a), ios_base::in | ios_base::binary);
   		} else if (strcmp(*a, "--testStick") == 0) {
			using namespace ServoControl;
			for (float x = -servoThrow; x <= +servoThrow; x += servoThrow / 10) {
				for (float y = -servoThrow; y <= +servoThrow; y += servoThrow / 10) {
					setServos(x, y);
					pair<float,float> r = servoToStick(servoOutput[0], servoOutput[1]);
					printf("%+06.2f, %+06.2f  ->  %04d, %04d  ->  %+06.2f,%+06.2f\n", x, y, servoOutput[0], servoOutput[1], r.first, r.second);
				}

			}
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
				else if (sscanf(it->c_str(), "ahrs.crhdg=%f", &v) == 1) { ahrs.hdgCompRatio = v; } 
				else if (sscanf(it->c_str(), "mbt.cr=%f", &v) == 1) { ahrs.magBankTrimCr = v; } 
				else if (sscanf(it->c_str(), "mbt.maxerr=%f", &v) == 1) { ahrs.magBankTrimMaxBankErr = v; } 
				else if (sscanf(it->c_str(), "dipconstant=%f", &v) == 1) { ahrs.magDipConstant = v; } 
				else if (sscanf(it->c_str(), "ahrs.crpitch=%f", &v) == 1) { ahrs.compRatioPitch = v; } 
				else if (sscanf(it->c_str(), "ahrs.pitchoffset=%f", &v) == 1) { ahrs.pitchOffset = v; } 
				else if (sscanf(it->c_str(), "ahrs.rolloffset=%f", &v) == 1) { ahrs.rollOffset = v; } 
				else if (sscanf(it->c_str(), "ahrs.useauxmpu=%f", &v) == 1) { ESP32csim_useAuxMpu = v; } 
				else if (sscanf(it->c_str(), "ahrs.gxdecel=%f", &v) == 1) { ahrs.gXdecelCorrelation = v; } 
				else if (sscanf(it->c_str(), "ahrs.bankanglescale=%f", &v) == 1) { ahrs.bankAngleScale = v; }
				else if (sscanf(it->c_str(), "ubloxcr=%f", &v) == 1) { ubloxHdgCr = v; }
				else if (strlen(it->c_str()) > 0) { 
					printf("Unknown debug parameter '%s'\n", it->c_str()); 
					exit(-1);
				}
			}
		}
	}	
	void setup() override {
		setServos(1,1);
		ServoControl::servoToStick(servoOutput[0],servoOutput[1]);
		if (replayFile == NULL) { 
			bm.addPress(pins.knobButton, 1, 1, true); // knob long press - arm servo
			//bm.addPress(pins.botButton, 250, 1, true); // bottom long press - test turn activate 
			//bm.addPress(pins.topButton, 200, 1, false); // top short press - wings level mode  
			//bm.addPress(pins.topButton, 300, 1, false); // top short press - hdg hold
			//setDesiredTrk(ahrsInput.dtk = 135);
		}
		
	}

	bool firstLoop = true;	
	float now, lastTime = 0;
	bool hz(float hz) { return floor(now * hz) != floor(lastTime * hz); }
	bool at(float t) { return now > t && lastTime < t; }
	void loop() override {
		now = _micros / 1000000.0;

		if (hz(100) && gdl90file) {
				std::vector<unsigned char> data(300);
				gdl90file.read((char *)data.data(), data.size());       
				int n = gdl90file.gcount();
				if (gdl90file && n > 0) { 
						ESP32sim_udpInput(4000, data);
				}
		}       


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

			if (at(5.0) && wpFile.length()) {
				wpNav = new WaypointsSequencerFile(wpFile.c_str());
				apMode = 3;
				hdgSelect = 0;
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
MIN132 board

GND 		RST  				TXD/IO03  	GND 
NC      	IO36                RXD   		IO27
IO39 		IO26                IO22  		IO25
IO35		IO18 				IO21  		IO32
IO33 		IO19				IO17 		TDI/IO12 
IO34 		IO23 				IO16 		IO04
TMS/IO14 	IO05 				GND 		IO00
NC 			3.3V 				VCC			IO02/LED
SD2			TCK/IO13			TDO/IO15	SD1
CMD			SD3					SDD			CLK

*/
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
