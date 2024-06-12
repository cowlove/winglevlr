#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#ifdef UBUNTU
#define CSIM_PRINTF printf
#include "ESP32sim_ubuntu.h"
#else // #ifndef UBUNTU
void noprintf(const char *, ...) {}
#define CSIM_PRINTF printf
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <rom/rtc.h>
#include <MPU9250_asukiaaa.h>
// #include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#endif // #else // UBUNTU

#include <TinyGPS++.h>

#include "jimlib.h"
#include "TTGO_TS.h"
#include "RollingLeastSquares.h"
#include "PidControl.h"
#include "RollAHRS.h"
#include "GDL90Parser.h"
#include "WaypointNav.h"
#include "ServoVisualizer.h"
#include "espNowMux.h"
#include "reliableStream.h"
#include "confPanel.h"

bool debugFastBoot = false;

using WaypointNav::magToTrue;
using WaypointNav::trueToMag;

// WiFiMulti wifi;
JStuff j;


// SPIFFSVariable<int> logFileNumber("/winglevlr.logFileNumber", 1);
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

struct PIDS {
	PidControl rollPID = PidControl(AHRS_RATE_SCALE(30)) /*200Hz*/,
			   pitchPID = PidControl(AHRS_RATE_SCALE(10), 6),
			   hdgPID = PidControl(50) /*20Hz*/,
			   xtePID = PidControl(100) /*5hz*/,
			   altPID = PidControl(25); /*5Hz*/

	PIDS() {
		// Set up PID gains
		rollPID.setGains(7.52, 0.01, 0.11); // input in degrees bank err, output in stickThrow units
		rollPID.hiGain.p = 2;
		rollPID.hiGainTrans.p = 5;
		rollPID.maxerr.i = 2.0; // degrees bank err
		rollPID.outputTrim = 0.0;
		rollPID.inputTrim = -0.54;
		rollPID.finalGain = 16.8;
		rollPID.finalScale = 0.001;

		hdgPID.setGains(0.5, 0.001, 0.50); // input in degrees hdg err, output in degrees desired bank
		hdgPID.hiGain.p = 10;
		hdgPID.hiGainTrans.p = 8.0;
		hdgPID.maxerr.i = 20; // degrees hdg err
		hdgPID.finalGain = 1.0;

		xtePID.setGains(8.0, 0.001, 0.20); // input in NM xte error, output in degrees desired hdg change
		xtePID.maxerr.i = 1.0;
		xtePID.finalGain = 20.0;

		pitchPID.setGains(20.0, 0.0, 2.0, 0, .8); // input in degrees of pitch err, output in stickthrow units
		pitchPID.finalGain = 5.0;
		pitchPID.maxerr.i = 1; // degrees
		pitchPID.outputTrim = -0.0;
		pitchPID.inputTrim = +0;
		pitchPID.finalScale = 0.001;

		altPID.setGains(1.0, 0.050, 2.0); // input in feet of alt err, output in degrees of pitch change
		altPID.finalGain = -5.00;
		altPID.maxerr.i = 200; // feet
		altPID.outputTrim = 0.0;
		altPID.finalScale = 0.01;
		altPID.iMaxChange = 3.0; /* feet/sec above which I-err wont be accumulated */
	}
} pids;

PidControl *knobPID = &pids.altPID;

WiFiUDP udpSL30;
WiFiUDP udpNMEA;
WiFiUDP udpG90;
WiFiUDP udpCmd;
// WiFiUDP udpMAV;

LogItem logItem;
SDCardBufferedLog<LogItem> *logFile = NULL;
bool logChanging = false;
const char *logFileName = "AHRSD%03d.DAT";
static StaleData<float> gpsTrackGDL90(3000, -1), gpsTrackRMC(5000, -1), gpsTrackVTG(5000, -1);
static StaleData<int> canMsgCount(3000, -1);
static float desiredTrk = -1;
static float cmdRoll = 0, pitchToStick = -0.05 /*WHY negative?*/, desPitch = -4.0, cmdPitch = 0, desAlt = 0;
static float stickXYTransNeg = 0, stickXYTransPos = 0;
static int serialLogFlags = 0;
float tttt = 60; // seconds to make each test turn
float ttlt = 75; // seconds betweeen test turn, ordegrees per turn
float rollToStick = 0.0, rollToPitch = 0.0;
float maxRollRate = 5.0; // deg/sec 
float servoGain = 1.70;
int g5LineCount = 0;
int serialLogMode = 0x0;

#define LED_PIN 22
/* Old hardwarinput pins: I2C pins/variant seems to determine layout
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
	// int servo_enable = 18;
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
		int bps = 38400;
		// myGNSS.enableDebugging(Serial, false);
		Serial.printf("Trying %d BPS...\n", bps);
		Serial2.begin(bps, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
		for(int i = 0; i < 100; i++) {  
			while(Serial2.available()) { 
				int c = Serial2.read();
				Serial.printf("%c", c);
			}
			delay(5);
		}
		if (myGNSS.begin(Serial2) == false) {
			Serial.printf("Trying 9.6K BPS...\n");
			Serial2.begin(9600, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
			if (myGNSS.begin(Serial2) == false) {
				Serial.printf("Trying 115K BPS...\n");
				Serial2.begin(115200, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
				if (myGNSS.begin(Serial2) == false) {
					Serial.println("u-blox GNSS not detected. Please check wiring");
					return;
				}
			}
			myGNSS.setSerialRate(bps); // Set UART1 to 57600bps.
			Serial2.begin(bps, SERIAL_8N1, pins.gps_rx, pins.gps_tx);
			myGNSS.saveConfiguration(); // Optional: Save the current settings to flash and BBR
		}

		int b;
		b = myGNSS.setUART1Output(COM_TYPE_UBX);
		j.out("setUART1Output: %d", b);
		b = myGNSS.setNavigationFrequency(10);
		j.out("setNavigationFrequency: %d", b);
		b = myGNSS.setAutoPVT(true, true, 100);
		j.out("setAutoPVT: %d", b);
		// b = myGNSS.setAutoPVTrate(0.10); //Set output to 5 times a second
		// j.out("setAutoPVTrate: %d", b);

		gpsGood = 1;
		OUT("Found GPS\n");
	}
	double lat, lon;
	float hdg, hac, gs, siv, alt;
	bool fixOk;
	int count = 0;
	bool check() {
		if (gpsGood && myGNSS.getPVT(10)) {
			lon = myGNSS.getLongitude() / 10000000.0;
			hdg = myGNSS.getHeading() / 100000.0;
			lat = myGNSS.getLatitude() / 10000000.0;
			alt = myGNSS.getAltitudeMSL() / 1000.0;
			hac = myGNSS.getHeadingAccEst() / 100000.0;
			gs = myGNSS.getGroundSpeed() / 1000.0 / MPS_PER_KNOT;
			siv = myGNSS.getSIV();
			fixOk = myGNSS.getGnssFixOk();
			if (fixOk) 
				count++;
			//Serial.printf("GPS %+13.8f %+13.8f %+05.1f %.0f %.2f %d\n", lat, lon, hdg, siv, 
			//	gs, (int)fixOk);
			return fixOk;
		}
		return false;
	}
#ifdef UBUNTU
	void fakeValues(double lat, double lon, float hdg, float hac, float gs, float alt, float siv) {
		myGNSS.lon = lon * 10000000.0;
		myGNSS.lat = lat * 10000000.0;
		myGNSS.hdg = hdg * 100000;
		myGNSS.alt = alt * 1000;
		myGNSS.hac = hac * 100000;
		myGNSS.gs = gs * 1000.0 * MPS_PER_KNOT;
		myGNSS.siv = siv;
		myGNSS.fresh = 1;
	}
#endif
} ublox;

void halInit() {
	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(1);

	Serial.printf("\nReset reason: %d %d\n", (int)rtc_get_reset_reason(0), (int)rtc_get_reset_reason(1));
	Serial.printf(__BASE_FILE__ " ver %s\n", GIT_VERSION);
	Wire.begin(21, 22);
	// Wire.setClock(400000);
	Serial.println("Scanning I2C bus on pins 21,22");
	if (scanI2c() == 0) {
		Wire.begin(19, 18);
		// Wire.setClock(400000);
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
		// pins.knobButton = 32;
		// pins.servo_enable = 36;
		// pins.led = 21; // dont know
		//  TODO: IMU is inverted on these boards
	}

	for (int addr = 0x68; addr <= 0x69; addr++) {
		imu = new MPU9250_asukiaaa(addr);
		// imu.address = addr;
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

	while (0) { // basic hardware testing - halt here and debug pins
		static int alternate = 0;
		delay(100);
		printPins();
		pinMode(pins.tft_backlight, OUTPUT);
		// pinMode(pins.led, OUTPUT);
		alternate = !alternate;
		// digitalWrite(pins.led, alternate);
		digitalWrite(pins.tft_backlight, !alternate);
	}

	if (ahrs.rotate180 == false) {
		int temp = pins.botButton;
		pins.botButton = pins.topButton;
		pins.topButton = temp;
	}

	imu->beginAccel(ACC_FULL_SCALE_4_G);
	imu->beginGyro(GYRO_FULL_SCALE_250_DPS);
	imu->beginMag(MAG_MODE_CONTINUOUS_100HZ);
}

float ubloxHdgCr = 0.0023;

DigitalButton buttonTop(pins.topButton);   // top
DigitalButton buttonMid(pins.midButton);   // middle
DigitalButton buttonBot(pins.botButton);   // bottom
DigitalButton buttonKnob(pins.knobButton); // knob press

// static IPAddress mavRemoteIp;
// #define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
//`static uint8_t buf[BUFFER_LENGTH];
EggTimer screenTimer(200);

LongShortFilter butFilt(1500, 600);
LongShortFilter butFilt2(1500, 600);
LongShortFilter butFilt3(1500, 600);
LongShortFilter butFilt4(1500, 600);

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
	static const int g5Nav = 0x01;
	static const int g5Ins = 0x02;
	static const int g5Ps = 0x04;
	static const int HdgRMC = 0x08;
	static const int HdgGDL = 0x10;
	static const int ublox = 0x20;
	static const int wptNav = 0x40;
	static const int ttaNav = 0x80;
}

namespace ServoControlString {
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

		float s0 = +(rightNewLen - rightLen) / servoThrow * 2000 + 1500;
		float s1 = +(leftNewLen - leftLen) / servoThrow * 2000 + 1500;

		return pair<int, int>(s0, s1);
	}

	pair<float, float> servoToStick(int s0, int s1) {
		float a = leftStringX + rightStringX;
		float b = (s0 - 1500.0) / 2000 * servoThrow + rightLen;
		float c = (s1 - 1500.0) / 2000 * servoThrow + leftLen;

		float area = sqrt(4 * a * a * b * b - (a * a + b * b - c * c) * (a * a + b * b - c * c)) / 4;
		float Y = area * 2 / a;
		float X = sqrt(c * c - Y * Y);

		float x = (X - leftStringX) / xScale;
		float y = (Y - leftStringY) / yScale;

		return pair<float, float>(x, y);
	}
};

ServoVisualizer *svis = nullptr;
struct XY {
	XY(float x1, float y1) : x(x1), y(y1) {}
	float x;
	float y;
};
namespace ServoControlElbow {
	// 1) arm angles start with 0 degrees is forward, so sin/cos usage may seem reversed
	// 2) arm[0].angle is absolute angle of the first arm
	// 3) arm[1].angle is the relative angle of the second arm to the first
	// 4) (0,0) is stick neutral position
	// 5) anchorPos is the x/y of the arm[0] hinge point

	XY trim(0, -0.95), strim(30, -30);
	const float servoThrow = +1.2;
	const float hinge = 15;
	float maxChange = .12;

	struct ArmInfo {
		float length;
		float angle;
		// TODO: broken, only works if anchorPos.x == 0, arms are equal
	} arms[] = {{2.1, DEG2RAD(225 + hinge)}, {2.1, DEG2RAD(90 - hinge * 2)}};

	XY anchorPos(0, -sqrt(
						arms[0].length *arms[0].length +
						arms[1].length * arms[1].length -
						2 * arms[0].length * arms[1].length * cos(arms[1].angle)));

	XY oldPos(0, 0);
	float srvPerDeg = -6.2;
	float ang2servo(float a) {
		return 1500.0 + a * srvPerDeg;
	}
	float servo2ang(float s) {
		return (s - 1500.0) / srvPerDeg;
	}

	pair<float, float> servoToStick(float s0, float s1);
	pair<int, int> stickToServo(float ox, float oy) {
		// x  = .05;
		// y = 0;
		float x = ox + trim.x;
		float y = oy + trim.y;

		// move stickXYTrans here 
		y += abs(x) * (x < 0 ? stickXYTransNeg : stickXYTransPos);

		x = min(servoThrow, max(-servoThrow, x));
		y = min(servoThrow, max(-servoThrow, y));

		x = min(oldPos.x + maxChange, max(oldPos.x - maxChange, x));
		y = min(oldPos.y + maxChange, max(oldPos.y - maxChange, y));
		oldPos = XY(x, y);

		assert(anchorPos.x == 0);
		assert(arms[0].length == arms[1].length);
		double anchOffX = x - anchorPos.x;
		double anchOffY = y - anchorPos.y;
		double armLen = sqrt(anchOffX * anchOffX + anchOffY * anchOffY);
		double ang1 = RAD2DEG(acos(
								  (arms[0].length * arms[0].length +
								   arms[1].length * arms[1].length -
								   armLen * armLen) /
								  (2 * arms[0].length * arms[1].length)) -
							  arms[1].angle);
		// need to calculate the offset in ang0 caused by change in ang1
		double arm1NeutralAbsAng = arms[0].angle - M_PI + arms[1].angle;
		double arm1AbsAng = arm1NeutralAbsAng + DEG2RAD(ang1);
		double ang1OffsetX = arms[1].length * (sin(arm1AbsAng) - sin(arm1NeutralAbsAng));
		double ang1OffsetY = -arms[1].length * (cos(arm1AbsAng) - cos(arm1NeutralAbsAng));
		double angOffset = (ang1OffsetX == 0 && ang1OffsetY == 0) ? 0 : RAD2DEG(atan2(ang1OffsetX - anchorPos.x, ang1OffsetY - anchorPos.y));
		// RAD2DEG(atan2(ang1OffsetY - anchorPos.y, ang1OffsetX - anchorPos.x));
		double ang0 = (x == 0 && y == 0) ? 0 : RAD2DEG(atan2(x - anchorPos.x, y - anchorPos.y));
		// RAD2DEG(atan2(y - anchorPos.y, x - anchorPos.x));
		ang0 += angOffset;

		pair<float, float> servo;
		servo.first = ang2servo(ang0) + strim.x;
		servo.second = ang2servo(ang1) + strim.y;
		if (svis != nullptr && (svis->startTime == 0 || millis() / 1000.0 > svis->startTime)) {
			svis->scale = svis->len0 / arms[0].length;
			svis->yoffset = -anchorPos.y * svis->scale;
			vector<pair<float, float>> pts;
			pts.push_back(pair<float, float>(-x, -y));
			pts.push_back(pair<float, float>(0, -anchorPos.y - armLen));
			svis->update(RAD2DEG(arms[0].angle) + ang0,
						 RAD2DEG(arms[0].angle - M_PI + arms[1].angle) + ang0 + ang1, pts);
		}
		pair<float, float> cs = servoToStick(servo.first, servo.second);

		// CSIM_PRINTF("x:%6.2f y:%6.2f aoaa%6.2f, aora %6.2f al:%6.2f a0:%6.2f a1:%6.2f a1ox: %06.2f a1oy: %06.2f ao: %06.2f s0:%06.2f s1:%06.2f cx:%6.2f cy:%6.2f S2S\n",
		//	ox, oy, RAD2DEG(arm1NeutralAbsAng ), RAD2DEG(arm1AbsAng),
		//	armLen, ang0, ang1, ang1OffsetX, ang1OffsetY, angOffset, servo.first, servo.second,
		//	(double)cs.first, (double)cs.second);
		return pair<int, int>(servo.first, servo.second);
	}

	pair<float, float> servoToStick(float s0, float s1) {
		float a0 = DEG2RAD(servo2ang(s0 - strim.x)) + arms[0].angle;
		float a1 = a0 - M_PI + arms[1].angle + DEG2RAD(servo2ang(s1 - strim.y));
		// CSIM_PRINTF("a0:%6.3f a1:%6.3f\n", RAD2DEG(a0), RAD2DEG(a1));

		float x = anchorPos.x - sin(a0) * arms[0].length - sin(a1) * arms[1].length;
		float y = anchorPos.y - cos(a0) * arms[0].length - cos(a1) * arms[1].length;

		return pair<float, float>(x - trim.x, y - trim.y);
	}
};

namespace ServoControlLinear {
	const float servoThrow = +1;
	XY trim(0, 0), strim(0, 0), gain(1.0, 1.0);
	float maxChange = 0; // unimplemented
	pair<int, int> stickToServo(float x, float y) {
		x = min(servoThrow, max(-servoThrow, x * gain.x + trim.x));
		y = min(servoThrow, max(-servoThrow, y * gain.y + trim.y));
		float s1 = x / servoThrow * 450 + 1500 + strim.x;
		float s0 = y / servoThrow * 450 + 1500 + strim.y;
		return pair<int, int>(s0, s1);
	}

	pair<float, float> servoToStick(int s0, int s1) {
		float x = ((s1 - 1500 - strim.x) / 2000.0 * servoThrow - trim.x) / gain.x;
		float y = ((s0 - 1500 - strim.y) / 2000.0 * servoThrow - trim.y) / gain.y;

		return pair<float, float>(x, y);
	}
};

#define ServoControl ServoControlLinear

namespace Display {
	JDisplay jd;
	JDisplayEditor jde(26, 0);

	JDisplayEditor *ed(&jde);

	typedef JDisplayItem<float> F;
	typedef JDisplayEditableItem E;

	int y = 0;
	const int c1x = 00, c2x = 70;
	F ip(&jd, c1x, y, "WIFI:", "%.0f");	F stickX(&jd, c2x, y, "ST:", "%+.2f");F stickY(&jd, c2x + 50, y, "/", "%+.2f");
	E dtrk(&jd, c1x, y += 10, " DTK:", "%05.1f ", ed, 1, 0, 359, true);       F trk(&jd, c2x, y, " TRK:", "%05.1f ");
	F pitc(&jd, c1x, y += 10, "PITC:", "%+03.1f ");  		F obs(&jd, c2x, y, " OBS:", "%05.1f ");
	F roll(&jd, c1x, y += 10, "ROLL:", "%+03.1f");   		F mode(&jd, c2x, y, "MODE:", "%06.0f ");
	F gdl(&jd, c1x, y += 10, " GDL:", "%05.1f ");    		F g5hdg(&jd, c2x, y, " HDG:", "%05.1f ");
	JDisplayItem<const char *> log(&jd, c1x, y += 10, " LOG:", "%s-"); F drop(&jd, c2x + 30, y, "", "%03.0f ");
	E pidpl(&jd, c1x, y += 10, "PL:", "%04.2f ", ed, .01);	E sg(&jd, c2x, y, "  SG:", "%04.2f ", ed, 0.01, &servoGain);
	E pidph(&jd, c1x, y += 10, "PH:", "%04.2f ", ed, .01);	E sty(&jd, c2x, y, " STY:", "%+05.2f ", ed, 0.01, &ServoControl::trim.y);
	E pidi(&jd, c1x, y += 10, " I:", "%05.4f ", ed, .0001);	E stx(&jd, c2x, y, " STX:", "%+05.2f ", ed, 0.01, &ServoControl::trim.x);
	E pidd(&jd, c1x, y += 10, " D:", "%04.2f ", ed, .01);	E maxb(&jd, c2x, y, "MAXB:", "%04.1f ", ed, 0.1);
	E pidg(&jd, c1x, y += 10, " G:", "%04.2f ", ed, .01);	E pidot(&jd, c2x, y, "POTR:", "%+5.2f ", ed, 0.01);	
	E dead(&jd, c1x, y += 10, "DZ:", "%04.1f ", ed, .1);  	E pidit(&jd, c2x, y, "PITR:", "%+5.2f ", ed, 0.01); 	
	E p2s (&jd, c1x, y += 10, "PS:", "%04.2f ", ed, .01, &pitchToStick);
	E pidsel = JDisplayEditableItem(&jd, c2x, y, " PID:", "%1.0f", ed, 1, NULL, 0, 4);

	//E maxi(&jd, c2x, y, "MAXI:", "%04.1f ", ed, 0.1);
	// E dalt(NULL,c1x,y+=0,"DALT:", "%05.0f ", NULL, 20, &desAlt);
	// F navt(NULL,10,y+=10,"NAVT:", "%05.1f ");
	F logw(NULL, c1x, y += 10, "LOGW:", "%05.0f ");
}

float lastDesAlt = 0.0;

void imuLog();

static AhrsInput ahrsInput, lastAhrsInput, lastAhrsGoodG5;

bool imuRead() {
	AhrsInput &x = ahrsInput;
	x.sec = micros() / 1000000.0;
	// TODO fix this in a less gross way.
	while (x.sec < lastAhrsInput.sec) {
		x.sec += 65536.0 * 65536.0 / 1000000.0;
	}
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
		a.ax = x.ax;
		a.ay = x.ay;
		a.az = x.az;
		a.gx = x.gx;
		a.gy = x.gy;
		a.gz = x.gz;
		a.mx = x.mx;
		a.my = x.my;
		a.mz = x.mz;
		String ad = a.toString() + "\n";
		udpG90.beginPacket("255.255.255.255", 7892);
		udpG90.write((uint8_t *)ad.c_str(), ad.length());
		udpG90.endPacket();
	}
	lastUsec = micros();
	// remaining items set (alt, hdg, speed) set by main loop

	return true;
}

void setDesiredTrk(float f) {
	desiredTrk = f; // round(f);
	Display::dtrk.setValue(desiredTrk);
	//cmdRoll = 0;
}

void sdLog() {
	// Serial.println(x.toString());
	if (logFile != NULL)
		logFile->add(&logItem, 0 /*timeout*/);
	// Serial.println(logItem.toString());
	logItem.flags = 0;
}

void printMag() {
	// imu->updateCompass();
	Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->gyroX(), (float)imu->gyroY(), (float)imu->gyroZ());
	Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->magX(), (float)imu->magY(), (float)imu->magZ());
	Serial.printf("%+09.4f %+09.4f %+09.4f ", (float)imu->accelX(), (float)imu->accelY(), (float)imu->accelZ());
	Serial.println("");
}

void parseSerialCommandInput(const char *buf, int n);
void parseSerialLine(const char *buf) {
	parseSerialCommandInput(buf, strlen(buf));
	parseSerialCommandInput("\n", 1);
}
void setupCp();
void parseG5Line(const char *);

class PidControlUI : public ConfPanelClient {
public:
	PidControlUI(ConfPanelTransportEmbedded *s) : ConfPanelClient(s) {}
	vector<PidControl *> pids;
	vector<string> pidNames;
	int selectedIndex = 0, previousIndex = -1;
	PID errs, gains;
	float finalGain, totalErr, inputTrim, outputTrim, iMaxChange;
	void add(PidControl *p, const char *name) { 
		pids.push_back(p);
		pidNames.push_back(string(name));
	}
	void begin() { 
		string names;
		for(auto i = pidNames.begin(); i != pidNames.end(); i++) { 
			if (i != pidNames.begin())
				names += "/";
			names += *i;
		}
		addEnum(&selectedIndex, "Selected PID", names.c_str());
		addFloat(&errs.p, "P Err", 0, "%.2f");
		addFloat(&errs.i, "I Err", 0, "%.3f");
		addFloat(&errs.d, "D Err", 0, "%.2f");
		addFloat(&totalErr, "Total Err", 0, "%.2f");
		addFloat(&gains.p, "P Gain", 0.01, "%.2f");
		addFloat(&gains.i, "I Gain", 0.001, "%.3f");
		addFloat(&gains.d, "D Gain", 0.01, "%.2f");
		addFloat(&finalGain, "Final Gain", 0.01, "%.2f");
		addFloat(&inputTrim, "Input Trim", 0.01, "%.2f");
		addFloat(&outputTrim, "Output Trim", 0.01, "%.2f");
		addFloat(&iMaxChange, "I-Err Max Change", 0.01, "%.2f");
	}
	int index() { 
		return min((int)pids.size() - 1, max(0, selectedIndex));
	}
	void run() { 
		if (selectedIndex != previousIndex) { 
			previousIndex = selectedIndex;
			gains = pids[index()]->gain;
			finalGain = pids[index()]->finalGain;
			inputTrim = pids[index()]->inputTrim;
			outputTrim = pids[index()]->outputTrim;
			iMaxChange = pids[index()]->iMaxChange;
		}
		errs = pids[index()]->err;
		totalErr = pids[index()]->corr;
		pids[index()]->gain = gains;
		pids[index()]->finalGain = finalGain;
		pids[index()]->inputTrim = inputTrim;
		pids[index()]->outputTrim = outputTrim;
		pids[index()]->iMaxChange = iMaxChange;
	}
};	

//ReliableTcpServer server(4444);
ReliableStreamESPNow server("CP");
ReliableStreamESPNow g5("G5");

ConfPanelTransportEmbedded cup(&server);
//ConfPanelUdpTransport cup;
ConfPanelClient cpc(&cup);

ConfPanelClient cpc2(&cup);
PidControlUI cpc3(&cup);
//ReliableTcpClient client("192.168.4.1", 4444);

void setup() {
	Display::jd.begin();
	Display::jd.setRotation(ahrs.rotate180 ? 3 : 1);
	Display::jd.clear();

	halInit();

	// ugh: redo pin assignments, hal may have changed them
	buttonTop.pin = pins.topButton;
	buttonBot.pin = pins.botButton;
	buttonMid.pin = pins.midButton;

	buttonTop.read();
	buttonBot.read();
	buttonMid.read();
	buttonKnob.read();

	debugFastBoot = buttonTop.read();
	Serial.printf(getMacAddress().c_str());
	if (getMacAddress() == PROGMEM "C44F337F8AB9")
		debugFastBoot = true;
	j.jw.enabled = !debugFastBoot;

	if (!debugFastBoot) 
		ublox.init();

	j.mqtt.active = false;
	
	j.begin();
	j.mqtt.active = false;
	j.run();
	j.mqtt.active = false;
	j.cli.on(".*", parseSerialLine);

	Serial.printf("Reading log file number\n");
	int l = logFileNumber;
	Serial.printf("Log file number %d\n", l);
	logFileNumber = l + 1;

	esp_task_wdt_init(22, true);
	esp_err_t err = esp_task_wdt_add(NULL);

	// WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

	// esp_register_freertos_idle_hook(bApplicationIdleHook);
	// pinMode(LED_PIN, OUTPUT);
	// digitalWrite(LED_PIN, 1);
	pinMode(pins.tft_backlight, OUTPUT); // TFT backlight
	digitalWrite(pins.tft_backlight, 1);
	// pinMode(pins.servo_enable, OUTPUT);
	// digitalWrite(pins.servo_enable, 1);

	attachInterrupt(digitalPinToInterrupt(buttonMid.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonBot.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonTop.pin), buttonISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(buttonKnob.pin), buttonISR, CHANGE);

#if 0
	WiFi.disconnect(true);
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);

	wifi.addAP("Ping-582B", "");
	wifi.addAP("Tip of the Spear", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");
#endif
	if (j.jw.enabled && !debugFastBoot && !buttonKnob.read() && !buttonTop.read()) { // skip long setup stuff if we're debugging
		uint64_t startms = millis();
		while (WiFi.status() != WL_CONNECTED /*&& digitalRead(button.pin) != 0*/) {
			j.run();
			delay(10);
		}
	}

	if (j.jw.enabled) { 
		udpSL30.begin(7891);
		udpG90.begin(4000);
		udpNMEA.begin(7892);
		udpCmd.begin(7895);
	}

	lastAhrsGoodG5.g5Hdg = lastAhrsGoodG5.gpsTrackGDL90 = lastAhrsGoodG5.gpsTrackRMC = -1;
	ahrsInput.g5Hdg = ahrsInput.gpsTrackGDL90 = ahrsInput.gpsTrackRMC = -1;

	// make PID select knob display text from array instead of 0-3
	Display::pidsel.toString = [](float v) {
		return String((const char *[]){"PIT ", "ALT ", "ROLL", "XTE ", "HDG "}[(v >= 0 && v <= 4) ? (int)v : 0]);
	};
	Display::jde.begin();
#ifndef UBUNTU
	Display::jde.re.begin([]() -> void
						  { Display::jde.re.ISR(); });
#endif
	Display::maxb.setValue(12);
	// ed.tzer.setValue(1000);
	Display::pidsel.setValue(1);
	Display::dtrk.setValue(desiredTrk);
	setKnobPid(Display::pidsel.value);
	if (debugFastBoot) {
		Display::ip.color.lb = Display::ip.color.vb = ST7735_RED;
	}
	Display::jde.update();

	// ed.rlhz.value = 3; // period for relay activation, in seconds
	// ed.mnrl.value = 70;
	// ed.pmin.value = 0.5; // PID total error that triggers relay minimum actuation
	// ed.pmax.value = 2.5; // PID total error that triggers relay maximum actuation
	pinMode(pins.pwm_pitch, OUTPUT);
	pinMode(pins.pwm_roll, OUTPUT);
	ledcSetup(1, 50, 16);			  // channel 1, 50 Hz, 16-bit width
	ledcSetup(0, 50, 16);			  // channel 1, 50 Hz, 16-bit width
	ledcAttachPin(pins.pwm_pitch, 0); // GPIO 33 assigned to channel 1
	ledcAttachPin(pins.pwm_roll, 1);  // GPIO 33 assigned to channel 1

	// ArduinoOTA.begin();
	setupCp();

	cpc3.add(&pids.rollPID, "ROLL");
	cpc3.add(&pids.hdgPID, "HDG");
	cpc3.add(&pids.xtePID, "XTE");
	cpc3.add(&pids.pitchPID, "PIT");
	cpc3.add(&pids.altPID, "ALT");
	cpc3.begin();
	cpc2.schemaFlags = 0x1;

#if 0 
	espNowMux.registerReadCallback("g5", 
        [](const uint8_t *mac, const uint8_t *data, int len){
			string s;
			s.assign((const char *)data, len);
			//Serial.printf("G5 data: %s\n", s.c_str());
			parseG5Line(s.c_str()); 
    });
#endif
 
}

void udpSendString(const char *b) {
	for (int repeat = 0; repeat < 3; repeat++) {
		udpG90.beginPacket("255.255.255.5", 7892);
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
	if (f == 0) {
		knobPID = &pids.pitchPID;
	} else if (f == 1) {
		knobPID = &pids.altPID;
	} else if (f == 2) {
		knobPID = &pids.rollPID;
	} else if (f == 3) {
		knobPID = &pids.xtePID;
	} else if (f == 4) {
		knobPID = &pids.hdgPID;
	}

	Display::pidpl.setValue(knobPID->gain.p);
	Display::pidph.setValue(knobPID->hiGain.p);
	Display::pidi.setValue(knobPID->gain.i);
	Display::pidd.setValue(knobPID->gain.d);
	// Display::pidl.setValue(knobPID->gain.l);
	Display::pidg.setValue(knobPID->finalGain);
	//Display::maxi.setValue(knobPID->maxerr.i);
	Display::dead.setValue(knobPID->hiGainTrans.p);
	Display::pidot.setValue(knobPID->outputTrim);
	Display::pidit.setValue(knobPID->inputTrim);
}

static bool testTurnActive = false;
static int testTurnAlternate = 0;
static float testTurnLastTurnTime = 0;
static RollingAverage<float, 5> crossTrackError;
static float xteCorrection = 0;
static float navDTK = -1;
static bool logActive = false;
static float roll = 0, pitch = 0;
static String logFilename("none");
static int servoOutput[2], servoTrim[2] = {1500, 1500};
static TwoStageRollingAverage<int, 40, 40> loopTime;
static EggTimer serialReportTimer(200), loopTimer(AHRS_RATE_INV_SCALE(5)), buttonCheckTimer(10);
static int armServo = 1;
static int servoSetupMode = 0; // Referenced when servos not armed.  0: servos left alone, 1: both servos neutral + trim, 2: both servos full in, 3: both servos full out
static int apMode = 1;		   // apMode == 4 means follow NMEA HDG and XTE sentences, anything else tracks OBS
static int hdgSelect = 2;	   //  0 use fusion magHdg, 1 g5 can, 2 ublox, 3 VTG 
static int altSelect = 0;	   //  0 0blox, 1 g5palt 
static float obs = -1, lastObs = -1;
static bool screenReset = false, screenEnabled = true;
static int ahrsSource = 1;

#ifdef UBUNTU
struct ErrorChannel {
	vector<float> hist;
	double sum = 0;
	void add(double a) {
		hist.push_back(a);
		sum += a;
	}
	void clear() {
		sum = 0;
		hist.clear();
	}
	double err() {
		double absSum = 0;
		double avg = sum / hist.size();
		for (vector<float>::iterator it = hist.begin(); it != hist.end(); it++) {
			absSum += abs(*it - avg);
		}
		return absSum / hist.size();
	}
};

struct {
	ErrorChannel roll, pitch, hdg;
	void clear() {
		roll.clear();
		pitch.clear();
		hdg.clear();
	}
} totalError;
#endif

// static int ledOn = 0;
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
void startMakeoutSess();

void parseSerialCommandInput(const char *buf, int n) {
	static LineBuffer lb;
	lb.add(buf, n, [](const char *line) { 
		OUT("RECEIVED COMMAND: %s", line);
		float f, f2;
		int relay, ms;
		if (sscanf(line, "navhi=%f", &f) == 1) { pids.hdgPID.hiGain.p = f; }
		else if (sscanf(line, "navtr %f", &f) == 1) { pids.hdgPID.hiGainTrans.p = f; }
		else if (sscanf(line, "srate %f", &f) == 1) { ServoControl::maxChange = f; }
		else if (sscanf(line, "sgain %f", &f) == 1) { servoGain = f; }
		else if (sscanf(line, "trimx %f", &f) == 1) { ServoControl::trim.x = f; }
		else if (sscanf(line, "trimy %f", &f) == 1) { ServoControl::trim.y = f; }
		else if (sscanf(line, "strimx %f", &f) == 1) { ServoControl::strim.x = f; }
		else if (sscanf(line, "strimy %f", &f) == 1) { ServoControl::strim.y = f; }
		else if (sscanf(line, "sgain %f", &f) == 1) { servoGain = f; }
		else if (sscanf(line, "maxb %f", &f) == 1) { Display::maxb.setValue(f); }
		else if (sscanf(line, "roll %f", &f) == 1) { cmdRoll = f; }
		else if (sscanf(line, "pidp %f", &f) == 1) { pids.pitchPID.gain.p = f; }
		else if (sscanf(line, "pidi %f", &f) == 1) { pids.pitchPID.gain.i = f; }
		else if (sscanf(line, "pidd %f", &f) == 1) { pids.pitchPID.gain.d = f; }
		//else if (sscanf(line, "pidl %f", &f) == 1) { pitchPID.gain.l = f; }
		else if (sscanf(line, "pidl %f", &f) == 1) { pids.pitchPID.gain.l = f; }
		//else if (sscanf(line, "pitch=%f", &f) == 1) { ed.pset.value = f; }
		//else if (sscanf(line, "ptrim=%f", &f) == 1) { ed.tzer.value = f; }
		//else if (sscanf(line, "mtin=%f", &f) == 1) { ed.mtin.value = f; }
		else if (strstr(line, "zeroimu") == line) { Serial.print(ahrs.zeroSensors().c_str()); }
		else if (strstr(line, "makeout") == line) { startMakeoutSess(); }
		else if (sscanf(line, "dtrk=%f", &f) == 1) { setDesiredTrk(f); }
		else if (sscanf(line, "s %f %f", &f, &f2) == 2) { servoOutput[0] = f; servoOutput[1] = f2; }
		else if (sscanf(line, "p2stick %f", &f) == 1) { pitchToStick = f; }
		else if (sscanf(line, "mode %f", &f) == 1) { apMode = f; }
		else if (sscanf(line, "knob=%f", &f) == 1) { setKnobPid(f); }
		else if (sscanf(line, "dalt %f", &f) == 1) { desAlt = f; }
		else if (strstr(line, "wpclear") == line) { waypointList = ""; }
		else if (strstr(line, "wpadd ") == line) { waypointList += (line + 6); waypointList += "\n"; }
		else if (strstr(line, "wpstart") == line && wpNav == NULL) { 
			wpNav = new WaypointsSequencerString(waypointList); 
			logItem.flags |= LogFlags::wptNav;
		} else if (strstr(line, "wpstop") == line && wpNav != NULL ) { delete wpNav; wpNav = NULL; }
		else if (sscanf(line, "knobturn %f", &f) == 1) { 
			Display::jde.re.change((int)f); 
			//serialOutput(Display::jd.dump());
		} else if (sscanf(line, "knobpress %f", &f) == 1) { 
			Display::jde.buttonPress((int)f); 
			//serialOutput(Display::jd.dump());
		} else if (sscanf(line, "smode %f", &f) == 1) { 
			serialLogMode = f;
		} else {
			OUT("UNKNOWN COMMAND: %s", line);
		} });
}

void setObsKnob(float knobSel, float v) {
	if (knobSel == 2) {
		desAlt = v * FEET_PER_METER;
	}
	if (knobSel == 1 /*|| knobSel == 4*/) {
		obs = v * 180.0 / M_PI;
		if (obs <= 0)
			obs += 360;
		if (obs != lastObs) {
			setDesiredTrk(obs);
			crossTrackError.reset();
			testTurnActive = false;
			pids.xtePID.reset();
			if (abs(obs - lastObs) > 5 && wpNav != NULL) {
				delete wpNav;
				wpNav = NULL;
			}
		}
		lastObs = obs;
	}
}

void setServos(float x, float y) {
	pair<int, int> s = ServoControl::stickToServo(x, y);
	servoOutput[0] = max(900, min(2100, s.first));
	servoOutput[1] = max(450, min(2550, s.second));
}

bool firstLoop = true;
int imuReadCount = 0;
bool immediateLogStart = false;

void startMakeoutSess() {
	Serial.printf("makeout sess %x\n", wpNav);
	if (wpNav != NULL) {
		delete wpNav;
		wpNav = NULL;
	} else {
		Display::maxb.setValue(15);
		WaypointNav::LatLon curPos(ublox.lat, ublox.lon);
		WaypointNav::LatLon nextPos =
			WaypointNav::locationBearingDistance(curPos, magToTrue(ahrsInput.selTrack), 6300);

		waypointList = sfmt(
			"REPEAT 1\n"
			"HDG %f\nWAIT 10\nHDG %f\nWAIT 110\n"
			"HDG %f\nWAIT 10\nHDG %f\nWAIT 60\n"
			"%f, %f\n"
			"HDG %f\nWAIT 10\nHDG %f\nWAIT 110\n"
			"HDG %f\nWAIT 10\nHDG %f\nWAIT 60\n"
			"%f, %f\n",
			constrain360(ahrsInput.selTrack + 90), constrain360(ahrsInput.selTrack + 180),
			constrain360(ahrsInput.selTrack + 270), constrain360(ahrsInput.selTrack + 0),
			curPos.lat, curPos.lon,
			constrain360(ahrsInput.selTrack + 270), constrain360(ahrsInput.selTrack + 180),
			constrain360(ahrsInput.selTrack + 90), constrain360(ahrsInput.selTrack + 0),
			curPos.lat, curPos.lon);
		wpNav = new WaypointsSequencerString(waypointList);
		logItem.flags |= LogFlags::wptNav;
	}
}

// Button synopsis:
//	TOP:  1 short: delete wpNav, alternate b/w wings-level and hold current track/pitch
//        1 long : arm servo if needed, step through servoSetupMode
//        2 short: increment hdgSelect
//        3 short: startMakeoutSession();
//		boot hold: debugFastBoot (skip wifi, etc)
// MIDDLE:1 short: hdg -10 degrees
//        1 long : start/stop logging
//		  2 short: pitch - 1 degrees
//      boot hold: immediate log start 
// BOTTOM:1 short: hdg +10 degrees 
//        2 short: pitch + 1 degrees
//        3 short: activate test turns 
//        1 long : arm/disarm servo, reset servoSetupMode to 0  
// KNOB   1 long : arm/disarm servo
//        1 short: GUI select

void doButtons() { 
	const float pitchInc = 1, hdgInc = 10;
	buttonISR();
	if (butFilt3.newEvent()) { // TOP or RIGHT button
		if (butFilt3.wasCount == 1 && butFilt3.wasLong == false) { 
			// SHORT: Stop tracking NMEA dest, toggle desired track between -1/current heading
			apMode = 1;
			if (wpNav != NULL) {
				delete wpNav;
				wpNav = NULL;
			}
			if (desiredTrk == -1) {
				desPitch = ahrs.pitch;
				setDesiredTrk(ahrsInput.selTrack);
			} else {
				setDesiredTrk(-1);
			}
		}
		if (butFilt3.wasCount == 1 && butFilt3.wasLong == true) {
			if (armServo) {
				armServo = false;
				servoSetupMode = 0;
			} else {
				servoSetupMode = (servoSetupMode + 1) % 6;
			}
			Serial.printf("Servo setup mode %d\n", servoSetupMode);
		}
		if (butFilt3.wasCount == 2 && butFilt3.wasLong == false) {
			// double press - hdg select
			hdgSelect = (hdgSelect + 1) % 6;
		}
		if (butFilt3.wasCount == 3) {
			startMakeoutSess();
		}
	}
	if (butFilt.newEvent()) { // MIDDLE BUTTON
		if (!butFilt.wasLong && butFilt.wasCount == 1) {
				setDesiredTrk(constrain360(desiredTrk - hdgInc));
				apMode = 1;
		}
		if (butFilt.wasCount == 2) {
				desPitch -= pitchInc;
		}
		if (butFilt.wasLong) { 
			if (!logChanging && (immediateLogStart != true || millis() > 10000)) {
				logChanging = true;
				if (logFile == NULL) {
					logFile = new SDCardBufferedLog<LogItem>(logFileName, 200 /*q size*/, 0 /*timeout*/, 5000 /*flushInterval*/, false /*textMode*/);
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
	if (butFilt2.newEvent()) { // BOTTOM or LEFT button
		if (butFilt2.wasCount == 1 && butFilt2.wasLong == true) {
			armServo = !armServo;
			servoSetupMode = 0;
		}
		if (butFilt2.wasCount == 1 && butFilt2.wasLong == false) {
			setDesiredTrk(constrain360(desiredTrk + hdgInc));
			apMode = 1;
		} 
		if (butFilt2.wasCount == 2) {
			desPitch += pitchInc;
		}
		if (butFilt2.wasCount == 3) {
			testTurnActive = !testTurnActive;
			testTurnAlternate = 0;
			if (testTurnActive) {
				logItem.flags |= LogFlags::ttaNav;
			}
		}
	}

	if (butFilt4.newEvent()) { // main knob button
		if (butFilt4.wasCount == 1 && butFilt4.wasLong) {
			Display::jde.buttonPress(butFilt4.wasLong);
		}
		if (butFilt4.wasLong && butFilt4.wasCount == 1) {
			armServo = !armServo;
			servoSetupMode = 0;
		}
		if (butFilt4.wasCount == 3)	{
			tttt = .1;
			ttlt = .1;
			armServo = testTurnActive = true;
		}
	}
}
void ParseNMEAChar(const char *, int);
void parseG5Line(const char *line) { 
	g5LineCount++;
	vector<string> l = split(string(line), ' ');
	float v, knobSel = 0;
	for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
		if (sscanf(it->c_str(), "P=%f", &v) == 1) {
			ahrsInput.g5Pitch = v;
			canMsgCount = canMsgCount + 1;
			logItem.flags |= LogFlags::g5Ins;
		} else if (sscanf(it->c_str(), "R=%f", &v) == 1) {
			ahrsInput.g5Roll = v;
			logItem.flags |= LogFlags::g5Ins;
		} else if (sscanf(it->c_str(), "SL=%f", &v) == 1) {
			ahrsInput.g5Slip = v;
			logItem.flags |= LogFlags::g5Ins;
		} else if (sscanf(it->c_str(), "IAS=%f", &v) == 1) {
			ahrsInput.g5Ias = v;
			logItem.flags |= LogFlags::g5Ps;
		} else if (sscanf(it->c_str(), "TAS=%f", &v) == 1) {
			ahrsInput.g5Tas = v;
			logItem.flags |= LogFlags::g5Ps;
		} else if (sscanf(it->c_str(), "PALT=%f", &v) == 1) {
			ahrsInput.g5Palt = v * 3.2808;
			logItem.flags |= LogFlags::g5Ps;
		} else if (sscanf(it->c_str(), "HDG=%f", &v) == 1) {
			ahrsInput.g5Hdg = v;
			logItem.flags |= LogFlags::g5Nav;
			ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.03);
		} else if (sscanf(it->c_str(), "TRK=%f", &v) == 1) {
			ahrsInput.g5Track = v;
			logItem.flags |= LogFlags::g5Nav;
		} else if (sscanf(it->c_str(), "MODE=%f", &v) == 1) {
			Serial.printf("G5 MODE %f\n", v);
			apMode = v;
			if (apMode == 5) {
				startMakeoutSess();
			}
		} else if (sscanf(it->c_str(), "KSEL=%f", &v) == 1) {
			knobSel = v;
		} else if (sscanf(it->c_str(), "KVAL=%f", &v) == 1) {
			setObsKnob(knobSel, v);
		} else if (strstr(it->c_str(), "NMEA=") == it->c_str()) { 
			ParseNMEAChar(it->c_str() + 5, strlen(it->c_str()) - 5);
		}
	}


	float pit, roll, magHdg, magTrack, knobVal, ias, tas, palt, age;
	int mode = 0;
	// printf("LINE %s\n", line);
	if (strstr(line, " CAN") != NULL && sscanf(line, "%f %f %f %f %f %f %f %f %f %f %d CAN", &pit, &roll, &magHdg, &magTrack, &ias, &tas, &palt, &knobSel, &knobVal, &age, &mode) == 11 && (pit > -2 && pit < 2) && (roll > -2 && roll < 2) && (magHdg > -7 && magHdg < 7) && (magTrack > -7 && magTrack < 7) && (knobSel >= 0 && knobSel < 6)) {
		// Serial.printf("CAN: %s\n", line);
		ahrsInput.g5Pitch = pit * 180 / M_PI;
		ahrsInput.g5Roll = roll * 180 / M_PI;
		ahrsInput.g5Hdg = magHdg * 180 / M_PI;
		ahrsInput.g5Track = magTrack * 180 / M_PI;
		ahrsInput.g5Palt = palt;
		ahrsInput.g5Ias = ias / 0.5144;
		ahrsInput.g5Tas = tas / 0.5144;
		// ahrsInput.g5TimeStamp = (millis() - (uint64_t)age) / 1000.0;
		apMode = mode;
		ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.03);
		logItem.flags |= (LogFlags::g5Nav | LogFlags::g5Ps | LogFlags::g5Ins);
		setObsKnob(knobSel, knobVal);
		// Serial.printf("knob sel %f, knob val %f\n", knobSel, knobVal);
		canMsgCount = canMsgCount + 1;
	}
}

void ParseNMEAChar(const char *buf, int n) {
	for (int i = 0; i < n; i++) {  
		gps.encode(buf[i]);
		if (buf[i] == '\r' || buf[i] == '\n') {
			if (vtgCourse.isUpdated()) { 
				// VTG typically from G5 NMEA serial output
				gpsTrackVTG = trueToMag(gps.parseDecimal(vtgCourse.value()) * 0.01);
			}
			if (gps.course.isUpdated()) { 
				// RMC typically from ifly NMEA output
				gpsTrackRMC = trueToMag(gps.course.deg());
				ahrs.mComp.addAux(gpsTrackRMC, 5, 0.07);
				logItem.flags |= LogFlags::HdgRMC;
			}
			if (desiredHeading.isUpdated()) {
				navDTK = trueToMag(0.01 * gps.parseDecimal(desiredHeading.value()));
				if (navDTK < 0)
					navDTK += 360;
				if (canMsgCount.isValid() == false) {
					// if we never got a can message, just hop straight to NAV mode 
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
float loopCount10Hz = 0;

void loop() {
	esp_task_wdt_reset();
	// ArduinoOTA.handle();
	j.run();
	if(j.hz(10)) loopCount10Hz++;
	cup.run();
	//cpc3.run();
	delayMicroseconds(100);

#ifndef UBUNTU
	if (!loopTimer.tick())
		return;
#endif

	uint64_t now = micros();
	double nowSec = millis() / 1000.0;

	if (debugFastBoot && nowSec > 3600) {
		ESP.restart();
	}
	loopTime.add(now - lastLoop);
	lastLoop = now;
	PidControl *pid = &pids.rollPID;
	if (serialReportTimer.tick() && (serialLogMode & 0x1)) {
		// SERIAL STATUS line output
		Serial.printf(
			"%06.2f "
			//"R %+05.2f BA %+05.2f GZA %+05.2f ZC %03d MFA %+05.2f"
			"%+05.2f,%+05.2f R%+05.1f P%+05.1f DR%+05.1f DP%+05.1f "
			"A%04.0f DA%04.0f "
			//"%+05.2f %+05.2f %+05.2f %+05.1f srv %04d xte %3.2f "
			"PIDC %+06.2f %+05.1f %+05.1f %+05.1f "
			//"but %d%d%d%d loop %d/%d/%d heap %d re.count %d logdrop %d maxwait %d "
			//"a%d "
			//"s%04d %04d "
			"%d\n",
			millis() / 1000.0,
			// roll, ahrs.bankAngle, ahrs.gyrZOffsetFit.average(), ahrs.zeroSampleCount, ahrs.magStabFit.average(),
			stickX, stickY, roll, pitch, cmdRoll, desPitch,
			ahrsInput.alt, desAlt,
			// 0.0, 0.0, 0.0, 0.0, servoOutput, crossTrackError.average(),
			knobPID->err.p, knobPID->err.i, knobPID->err.d, knobPID->corr,
			//buttonTop.read(), buttonMid.read(), buttonBot.read(), buttonKnob.read(), (int)loopTime.min(), (int)loopTime.average(), (int)loopTime.max(), ESP.getFreeHeap(), Display::jde.re.count,
			//logFile != NULL ? logFile->dropped : 0, logFile != NULL ? logFile->maxWaiting : 0,
			//ublox.count,
			// servoOutput[0], servoOutput[1],
			0);
		if (logFile != NULL) {
			logFile->maxWaiting = 0;
		}
		serialLogFlags = 0;
	}

	// ed.re.check();
	if (firstLoop == true && (digitalRead(buttonMid.pin) == 0 /* || debugFastBoot*/)) {
		// if (debugFastBoot && nowSec > 60 && logFile == NULL) {
		logFile = new SDCardBufferedLog<LogItem>(logFileName, 200 /*q size*/, 0 /*timeout*/, 500 /*flushInterval*/, false /*textMode*/);
		logFilename = logFile->currentFile;
		logChanging = false;
		immediateLogStart = true;
	}
	firstLoop = false;
	if (buttonCheckTimer.tick()) {
		doButtons();
	}

	if (testTurnActive) {
		if (desiredTrk == -1) {
			// roll mode, test turns are fixed time at maxb degrees
			if (nowSec - testTurnLastTurnTime > tttt + ttlt) {
				testTurnLastTurnTime = nowSec;
				testTurnAlternate = (testTurnAlternate + 1) % 3;
			} 
			if (nowSec - testTurnLastTurnTime <= tttt)
				cmdRoll = Display::maxb.value * (testTurnAlternate == 0 ? -1 : 1);
			else
				cmdRoll = 0;
		} else {
			// hdg mode, test turn is fixed number of degrees
			if (nowSec - testTurnLastTurnTime > tttt) {
				testTurnLastTurnTime = nowSec;
				testTurnAlternate = (testTurnAlternate + 1) % 3;
				setDesiredTrk(desiredTrk + ttlt * (testTurnAlternate == 2 ? -1 : 1));
			}
		}
	}

	string g5input = g5.read();
	if (g5input.length() > 0) { 
		parseG5Line(g5input.c_str()); 
	}
	if (udpG90.parsePacket() > 0) {
		unsigned char buf[1024];
		int n = udpG90.read(buf, sizeof(buf));
		for (int i = 0; i < n; i++) {
			gdl90.add(buf[i]);
			GDL90Parser::State s = gdl90.getState();
			if (s.valid && s.updated) {
				gpsTrackGDL90 = trueToMag(s.track);
				ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.05);
				logItem.flags |= LogFlags::HdgGDL;
				gpsFixes++;
				ahrsInput.alt = s.alt * 3.2808;
				ahrsInput.palt = s.palt; // * 25 + 1000;
				ahrsInput.gspeed = s.hvel;
				gdl90State = s;
			}
		}
	}

	if (udpNMEA.parsePacket() > 0) {
		char buf[1024];
		static LineBuffer lb;
		int n = udpNMEA.read((uint8_t *)buf, sizeof(buf));
		ParseNMEAChar(buf, n);
	}

	if (udpCmd.parsePacket() > 0) {
		char buf[1024];
		int n = udpCmd.read((uint8_t *)buf, sizeof(buf));
		parseSerialCommandInput(buf, n);
		parseSerialCommandInput("\n", 1);
	}

	if (udpSL30.parsePacket() > 0) {
		uint8_t buf[1024];
		static LineBuffer lb;
		int n = udpSL30.read(buf, sizeof(buf));
		lb.add(buf, n, [](const char *line) {
			parseG5Line(line);
		});

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

	// printMag();
	if (imuRead()) {
		bool tick1HZ = floor(ahrsInput.sec) != floor(lastAhrsInput.sec);
		bool tick20HZ = floor(ahrsInput.sec * 20.0) != floor(lastAhrsInput.sec * 20.0);
		bool tick5HZ = floor(ahrsInput.sec * 5.0) != floor(lastAhrsInput.sec * 5.0);

		if (tick5HZ) {
			if (wpNav != NULL) {
				apMode = 3;
				wpNav->wptTracker.curPos.loc.lat = ublox.lat;
				wpNav->wptTracker.curPos.loc.lon = ublox.lon;
				wpNav->wptTracker.curPos.alt = ahrsInput.ubloxAlt / FEET_PER_METER;
				wpNav->wptTracker.speed = ahrsInput.ubloxGroundSpeed;
				wpNav->wptTracker.curPos.valid = true;
				wpNav->run(0.2);
				if (wpNav->didWaypointChange) {
					crossTrackError.reset();
					pids.xtePID.reset();
				}
				if (tick1HZ) {
					crossTrackError.add(wpNav->wptTracker.xte * .0005);
				}
				if (abs(crossTrackError.average()) > 0.2) {
					pids.xtePID.resetI();
				}
				static const double xteCorr = 20;
				xteCorrection = -pids.xtePID.add(crossTrackError.average(), crossTrackError.average(), ahrsInput.sec);
				xteCorrection = max(-xteCorr, min(xteCorr, (double)xteCorrection));

				setDesiredTrk(trueToMag(wpNav->wptTracker.commandTrack) + xteCorrection);
				if (wpNav->wptTracker.commandAlt > -1000) {
					desAlt = wpNav->wptTracker.commandAlt * FEET_PER_METER;
				}
				ahrsInput.dtk = desiredTrk;
				Serial.printf("wptNav %06.1f ct:%06.1f trk:%06.1f %03.1f %010.5f %010.5f\n", wpNav->wptTracker.distToWaypoint,
							  wpNav->wptTracker.commandTrack, magToTrue(ahrsInput.selTrack), wpNav->waitTime, ublox.lat, ublox.lon);
			} else if (apMode == 4) {
				xteCorrection = -pids.xtePID.add(crossTrackError.average(), crossTrackError.average(), ahrsInput.sec);
				xteCorrection = max(-40.0, min(40.0, (double)xteCorrection));
				setDesiredTrk(navDTK + xteCorrection);
				ahrsInput.dtk = desiredTrk;
			} else {
				xteCorrection = 0;
			}
			if (desAlt > 1000) {
				float altErr = desAlt - ((altSelect == 0) ? ahrsInput.ubloxAlt : ahrsInput.g5Palt);
				if (abs(altErr) > 500) {
					pids.altPID.resetI();
				}
				pids.altPID.add(altErr, ahrsInput.ubloxAlt, ahrsInput.sec);
			} else {
				pids.altPID.reset();
			}
		}

		ahrsInput.gpsTrackGDL90 = gpsTrackGDL90;
		ahrsInput.gpsTrackVTG = gpsTrackVTG;
		ahrsInput.gpsTrackRMC = gpsTrackRMC;
		ahrsInput.dtk = desiredTrk;

		if (ahrsSource == 0) {
			roll = ahrs.add(ahrsInput);
			pitch = ahrs.pitch;
		} else { 
			roll = -ahrsInput.g5Roll;
			pitch = ahrsInput.g5Pitch;
		}

		if (hdgSelect == 0) { 
			// mode 0, use GDL90 until first can message, then switch to G5
			ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;
			if (ahrsInput.g5Hdg != -1) // canMsgCount.isValid() == true) // switch to G5 on first CAN msg
				hdgSelect = 1;
		}
		if (hdgSelect == 1) { // hybrid G5/GDL90 data
			if (ahrsInput.g5Hdg != -1 && g5HdgChangeTimer.unchanged(ahrsInput.g5Hdg) < 2.0) { 
				// use g5 data if it's not stale
				// if (ahrsInput.gpsTrackGDL90 != -1 || ahrsInput.gpsTrackRMC != -1) {
				ahrsInput.selTrack = ahrsInput.g5Hdg;
				//}
				lastAhrsGoodG5 = ahrsInput;
			} else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackGDL90 != -1 && ahrsInput.gpsTrackRMC != -1 && lastAhrsGoodG5.gpsTrackGDL90 != -1 && lastAhrsGoodG5.gpsTrackRMC != -1) {
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90) / 2 +
												  angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC) / 2);
			} else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackGDL90 != -1 && lastAhrsGoodG5.gpsTrackGDL90 != -1) { 
				// otherwise use change in GDL90 data
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackGDL90 - lastAhrsGoodG5.gpsTrackGDL90));
			}
			else if (lastAhrsGoodG5.selTrack != -1 && ahrsInput.gpsTrackRMC != -1 && lastAhrsGoodG5.gpsTrackRMC != -1)
			{ // otherwise use change in VTG data
				ahrsInput.selTrack = constrain360(lastAhrsGoodG5.selTrack + angularDiff(ahrsInput.gpsTrackRMC - lastAhrsGoodG5.gpsTrackRMC));
			} else { // otherwise, no available heading/track data
				ahrsInput.selTrack = -1;
			}
		}
		else if (hdgSelect == 2)
			ahrsInput.selTrack = ahrsInput.ubloxHdg;
		else if (hdgSelect == 3)
			ahrsInput.selTrack = ahrsInput.g5Hdg;
		else if (hdgSelect == 4)
			ahrsInput.selTrack = ahrsInput.g5Track;
		else if (hdgSelect == 5)
			ahrsInput.selTrack = ahrsInput.gpsTrackGDL90;

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
				if (abs(hdgErr) > 15.0) {
					pids.hdgPID.resetI();
					pids.xtePID.resetI();
				}
				float newRoll = -pids.hdgPID.add(hdgErr, currentHdg, ahrsInput.sec);
				newRoll = max(-Display::maxb.value, min(+Display::maxb.value, newRoll));
				cmdRoll = max(cmdRoll - maxRollRate / 20, min(cmdRoll + maxRollRate / 20, newRoll));
			} else if (!testTurnActive) {
				//desRoll = 0.0; // TODO: this breaks roll commands received over the serial bus, add rollOverride variable or something
			}
		}

		pids.rollPID.add(roll - cmdRoll, roll, ahrsInput.sec);
		float altCorr = max(min(pids.altPID.corr, 5.0), -5.0);
		cmdPitch = desPitch + altCorr + abs(sin(DEG2RAD(roll - pids.rollPID.inputTrim)) * rollToPitch);
		pids.pitchPID.add(pitch - cmdPitch, pitch - cmdPitch, ahrsInput.sec);

		if (armServo == true) {
			// TODO: pids were tuned and output results in units of relative uSec servo PWM durations.
			// hack tmp: convert them back into inches so we can add in inch-specified trim values
			stickX = pids.rollPID.corr * servoGain;
			float xytrans = abs(stickX) * (stickX < 0 ? stickXYTransNeg : stickXYTransPos);
			stickY = (pids.pitchPID.corr  
				+ abs(sin(DEG2RAD(roll - pids.rollPID.inputTrim))) * rollToStick 
				+ (cmdPitch * pitchToStick) 
				+ xytrans
			) * servoGain;
			// stickY = pids.pitchPID.corr * servoGain;
			// stickY = 0;
			// stickX += cos(millis() / 100.0) * .04;
			// stickY += sin(millis() / 100.0) * .04;
			setServos(stickX, stickY);
		} else
			switch (servoSetupMode) {
			case 0:
				// leave servos where they are
				break;
			case 1: {
				stickX = cos(millis() / 300.0) * ServoControl::servoThrow * .04;
				stickY = sin(millis() / 300.0) * ServoControl::servoThrow * .04;
				setServos(stickX, stickY);
				break;
			}
			case 2: {
				float speed = 500.0;
				stickX = sin(millis() / speed) * ServoControl::servoThrow * 1;
				stickY = sin(millis() / speed) * ServoControl::servoThrow * 1;
				int phase = ((int)(millis() / speed / 2 / M_PI)) % 2;
				if (phase == 0) {
					stickX = 0;
				} else if (phase == 1) {
					stickY = 0;
				} else {
					stickX = stickY = 0;
				}
				setServos(stickX, stickY);
				break;
			}
			case 3: {
				float speed = 500.0;
				stickX = cos(millis() / speed) * ServoControl::servoThrow;
				stickY = cos(millis() / speed) * ServoControl::servoThrow;
				int phase = ((int)(millis() / speed / M_PI)) % 4;
				if (phase == 0) {
					stickY = -ServoControl::servoThrow;
				} else if (phase == 1) {
					stickX = -ServoControl::servoThrow;
				} else if (phase == 2) { 
					stickY = +ServoControl::servoThrow;
					stickX = -stickX;
				} else if (phase == 3) {
					stickX = +ServoControl::servoThrow;
					stickY = -stickY;
				}
				setServos(stickX, stickY);
				break;
			}
			case 4:
				stickX = stickY = -ServoControl::servoThrow;
				setServos(stickX, stickY);
				break;
			case 5:
				stickX = stickY = +ServoControl::servoThrow;
				setServos(stickX, stickY);
				break;
			}

		ledcWrite(0, servoOutput[0] * 4915 / 1500); // scale PWM output to 1500-7300
		ledcWrite(1, servoOutput[1] * 4915 / 1500); // rscale PWM output to 1500-7300

		logItem.pwmOutput0 = servoOutput[0];
		logItem.pwmOutput1 = servoOutput[1];
		logItem.desRoll = cmdRoll;
		logItem.desAlt = (apMode == 3) ? desAlt : -1000;
		logItem.desPitch = desPitch;
		logItem.roll = roll;
		logItem.magHdg = ahrs.magHdg;
		logItem.xte = crossTrackError.average();
		// logItem.bankAngle = ahrs.bankAngle;
		// logItem.magBank = ahrs.magBank;
		logItem.pitch = pitch;
		logItem.ai = ahrsInput;

		lastAhrsInput = ahrsInput;
	}

	// digitalWrite(pins.servo_enable, !armServo);

	if (Display::pidsel.changed()) {
		setKnobPid(Display::pidsel.value);
	}

	if (screenTimer.tick() && screenEnabled) {
		Display::jde.update();
		if (0)  {
			knobPID->gain.p = Display::pidpl.value;
			knobPID->hiGain.p = Display::pidph.value;
			knobPID->gain.i = Display::pidi.value;
			knobPID->gain.d = Display::pidd.value;
			// knobPID->gain.l = Display::pidl.value;
			//knobPID->maxerr.i = Display::maxi.value;
			knobPID->finalGain = Display::pidg.value;
			knobPID->hiGainTrans.p = Display::dead.value;
			knobPID->outputTrim = Display::pidot.value;
			knobPID->inputTrim = Display::pidit.value;
		}

		Display::ip = WiFi.localIP()[3];
		Display::stickX = stickX;
		Display::stickY = stickY;
		// Display::dtk = desiredTrk;
		Display::trk = ahrsInput.selTrack;
		Display::obs = obs;
		Display::obs.setInverse(false, (g5LineCount / 20) % 2 == 0);
		Display::mode = servoSetupMode * 100000 + (canMsgCount.isValid() ? 10000 : 0) + apMode * 1000 
						+ armServo * 100 + hdgSelect * 10 + (int)testTurnActive;
		Display::gdl = (float)gpsTrackGDL90;
		Display::g5hdg = ahrsInput.ubloxHdg;
		Display::g5hdg.setInverse(false, (ublox.count / 10) % 2 == 0);
		// Display::g5hdg = (float)ahrsInput.g5Hdg;
		// Display::zsc = ahrs.getGyroQuality();
		Display::roll = roll;
		Display::pitc = pitch;
		Display::drop = logFile != NULL ? logFile->dropped : -1;
		Display::logw = logFile != NULL ? logFile->written / 200 : -1;
		// Display::pitch = pitch;
		// Display::xtec = xteCorrection;
		Display::log = (logFile != NULL) ? logFile->currentFile.c_str() : "none      ";
		Display::log.setInverse(false, (logFile != NULL));
#ifdef UBUNTU
	//	::printf("%s\n", Display::jd.dump().c_str());
#endif
	}

#if 0
	// stuff sim debugging things into logItem 
	logItem.ai.gpsTrackVTG = imuReadCount++;
	logItem.ai.gpsTrackGDL90 = ahrs.mComp.value;
	logItem.ai.gpsTrackRMC = ahrs.mComp.bestAux;
#endif
#ifdef UBUNTU
	static bool errorsCleared = false;
	if (errorsCleared == false && millis() < 200000) { 
		// don't count error during the first 200 sec, let AHRS stabilize
		totalError.clear();
		errorsCleared = true;
	}
	totalError.roll.add(roll + ahrsInput.g5Roll);
	totalError.hdg.add(angularDiff(ahrs.magHdg - ahrsInput.ubloxHdg));
	totalError.pitch.add(pitch - ahrsInput.g5Pitch);

	// special logfile name "+", write out log with computed values from the current simulation
	if (strcmp(logFilename.c_str(), "+") == 0) {
		pair<float, float> stick = ServoControl::servoToStick(servoOutput[0], servoOutput[1]);
		cout << logItem.toString().c_str() << strfmt("%f %f	LOG U", stick.first, stick.second) << endl;
	}
#endif
	if (logFile != NULL) {
		sdLog();
	}
	logItem.flags = 0;
}


#define ADDPID(x) if (1) { \
	cpc.addFloat(&pids.x.gain.p, #x " P Gain", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.gain.i, #x " I Gain", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.gain.d, #x " D Gain", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.hiGain.p, #x " PH Gain", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.hiGainTrans.p, #x " PH Trans", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.finalGain, #x " Final Gain", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.inputTrim, #x " In Trim", 0.01, "%.2f");\
	cpc.addFloat(&pids.x.outputTrim, #x " Out Trim", 0.01, "%.2f");\
}

int foo = 1;
void setupCp() { 
	cpc2.addFloat(&loopCount10Hz, "10hz Timer Count Client 2", 1, "%.0f");
	cpc.addEnum(&ahrsSource, "AHRS Source", "INS/G5");
	cpc.addFloat(&desiredTrk, "Set Heading", 1, "%03.0f Mag");
	cpc.addFloat(&ahrsInput.selTrack, "Heading", 1, "%.1f");
	//cpc.addFloat(&desAlt, "Set Altitude", 10, "%.0f'");
	cpc.addFloat(&cmdRoll, "Command Roll", 0.1, "%.2f");
	cpc.addFloat(&roll, "Roll", 1, "%.2f");
	cpc.addFloat(&desPitch, "Set Pitch", 1, "%.2f");
	cpc.addFloat(&cmdPitch, "Command Pitch", 1, "%.2f");
	cpc.addFloat(&pitch, "Pitch", 1, "%.2f");
	cpc.addFloat(&servoGain, "Servo Gain", 0.01, "%.2f");
	cpc.addFloat(&maxRollRate, "Max Roll Rate", 0.1, "%.1f");
	cpc.addFloat(&Display::maxb.value, "Max Bank", 0.1, "%.1f");
	cpc.addFloat(&pitchToStick, "Pitch->StickY", 0.01, "%.2f");
	cpc.addFloat(&rollToStick, "Roll->StickY", 0.01, "%.2f");
	cpc.addFloat(&stickXYTransPos, "Stick XY Transfer +", 0.01, "%.2f");
	cpc.addFloat(&stickXYTransNeg, "Stick XY Transfer -", 0.01, "%.2f");
	cpc.addEnum(&hdgSelect, "Heading Source", "AUTO/HY/UBLOX/G5HD/G5TR/GDL90");
	cpc.addEnum(&altSelect, "Altitude Source", "UBLOX/G5");
	cpc.addFloat(&ahrsInput.g5Palt, "G5 Altitude");
	cpc.addFloat(&ahrsInput.ubloxAlt, "UBLOX Altitude");
	cpc.addFloat(&ServoControl::trim.x, "Trim X", 0.01, "%.2f");
	cpc.addFloat(&ServoControl::trim.y, "Trim Y", 0.01, "%.2f");
	cpc.addFloat(&ServoControl::strim.x, "STrim X", 1, "%.0f");
	cpc.addFloat(&ServoControl::strim.y, "STrim Y", 1, "%.0f");
	cpc.addInt(&servoOutput[0], "Servo0");
	cpc.addInt(&servoOutput[1], "Servo1");
	//serialLogMode = 0;
}
#ifdef UBUNTU
///////////////////////////////////////////////////////////////////////////////
// Code below this point is used in compiling/running ESP32sim simulation

void ESP32sim_done();

class ESP32sim_winglevlr : public ESP32sim_Module {
public:
	IntervalTimer hz100 = IntervalTimer(100 /*msec*/);
	string wpFile;
	ifstream ifile;
	const char *replayFile = NULL;
	int logSkip = 0;
	int logEntries = 0;
	float bank = 0, track = 0, pitch = 0, roll = 0, yaw = 0, simPitch = 0;
	uint64_t lastMillis = 0;
	float cmdPitch;
	float speed = 80, windDir = 200, wind, windVel = 20, windGust = 0;
	WaypointNav::LatLonAlt curPos;
	std::queue<float> gxDelay, pitchDelay;

	uint64_t lastMicros = 0;
	int replayReduce = 0;

	RollingAverage<float, (int)AHRS_RATE_SCALE(120)> delayRoll;
	RollingAverage<float, (int)AHRS_RATE_SCALE(40)> delayBank;

	void flightSim(MPU9250_DMP *imu) {
		// TODO: flightSim is very fragile/unstable.  Poke values into the
		//  main loop code to make sure things work.
		// hdgPID.finalGain = 0.5;
		pids.pitchPID.outputTrim = pids.rollPID.outputTrim = 0;
		pids.rollPID.inputTrim = 0;
		ahrs.gyrOffZ = ahrs.gyrOffX = ahrs.gyrOffY = 0;

		int period = AHRS_RATE_INV_SCALE(5000);
		_micros = (_micros + period);
		_micros -= (_micros % period);
		// const float servoTrim = 4915.0;

		// Simulate simple airplane roll/bank/track turn response to
		// servooutput read from ESP32sim_currentPwm;
		pair<float, float> stick = ServoControl::servoToStick(
			ESP32sim_currentPwm[0] * 1500.0 / 4915,
			ESP32sim_currentPwm[1] * 1500.0 / 4915);
		float stickX = stick.first;
		float stickY = stick.second;

		cmdPitch = stickY * 10.5;
		float ngx = (cmdPitch - simPitch) * 0.15;
		imu->gx = max((float)-15.0, min((float)15.0, ngx));
		simPitch += (cmdPitch - simPitch) * 0.15;
		this->pitch = simPitch;

		delayRoll.add(stickX);
		imu->gy = ahrs.gyrOffY + delayRoll.average() * 4.5;
		imu->gy *= -1;
		bank += imu->gy * (AHRS_RATE_INV_SCALE(5000.0) / 1000000.0);
		bank = max(-20.0, min(20.0, (double)bank));
		if (1 && floor(lastMillis / 100) != floor(millis() / 100)) { 
			// 10hz
			printf("%08.3f servo %05d/%05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f SIM\n", (float)millis() / 1000.0,
				   ESP32sim_currentPwm[0], ESP32sim_currentPwm[1], track, cmdRoll, bank, imu->gy);
		}
		imu->gz = ahrs.gyrOffZ + tan(bank * M_PI / 180) / speed * 1091;
		imu->gz *= -1;

		uint64_t now = millis();
		if (j.hz(10)) {
			delayBank.add(max(-0.5, abs(bank - 2.3)) * bank / abs(bank));
			float dbank = delayBank.average();
			// TODO wind effect on ground track change
			track = constrain360(track + tan(dbank * M_PI / 180) * 9.8 * 0.14);
			set_gpsTrack(track);
		}

		// hdg = track - 35.555; // simluate mag var and arbitrary WCA
		// if (hdg < 0) hdg += 360;

		if (0) {
			printf("SIM %08.3f (%+04.1f,%+04.1f) %+05.2f %+05.2f %+05.2f %+05.2f\n",
				   (float)(millis() / 1000.0), stickX, stickY, imu->gx, cmdPitch, pitch, (double)logItem.pitch);
		}
		const float simPitchOffset = 0.0;
		imu->az = cos((this->pitch + simPitchOffset) * M_PI / 180) * 1.0;
		imu->ay = sin((this->pitch + simPitchOffset) * M_PI / 180) * 1.0;
		imu->ax = 0;

		float hdg = track;
		// simulate meaningless mag readings that stabilize when bank == 0
		imu->mx = cos(hdg * M_PI / 180) * 50 + 50;
		imu->my = sin(hdg * M_PI / 180) * 50 + 50;
		imu->mz = -70;
		ahrs.magBankTrim = 0; // TODO simulate mag so this doesn't oscillate
		// ahrs.magHdg = hdg;

		if (j.hz(.01)) {
			wind = windVel;
			if (windGust > 0)
				wind += random() * 1.0 / RAND_MAX * (windGust - windVel);
		}
		// Simluated wind doesn't quite work
		float relWindAngle = angularDiff(track - windDir);
		float gndSpeed = sqrt(wind * wind + speed * speed - 2 * wind * speed * cos(DEG2RAD(relWindAngle)));
		// float gndSpeed = speed;
		float dist = gndSpeed * MPS_PER_KNOT * (now - lastMillis) / 1000.0;
		curPos.loc = WaypointNav::locationBearingDistance(curPos.loc, track, dist);
		curPos.alt += sin((pitch + simPitchOffset) * M_PI / 180) * dist;

		lastMillis = now;

		if (ahrs.rotate180 == false) {
			imu->ax *= -1;
			imu->ay *= -1;
			imu->mx *= -1;
			imu->my *= -1;
			imu->gx *= -1;
			imu->gy *= -1;
		}
	}

	bool ESP32csim_useAuxMpu = false;
	bool ESP32sim_replayLogItem(ifstream &i) {
		LogItem l;
		static uint64_t logfileMicrosOffset = 0;
		int logFlags = 0;
		for (int n = 0; n < replayReduce; n++) {
			i.read((char *)&l, sizeof(l));
			logFlags |= l.flags;
		}

		if (i.read((char *)&l, sizeof(l))) {
			l.flags |= logFlags;
			if (logfileMicrosOffset == 0)
				logfileMicrosOffset = (l.ai.sec * 1000000 - _micros);
			_micros = l.ai.sec * 1000000.0 - logfileMicrosOffset;
			imu->ax = l.ai.ax;
			imu->ay = l.ai.ay;
			imu->az = l.ai.az;
			imu->gx = l.ai.gx;
			imu->gy = l.ai.gy;
			imu->gz = l.ai.gz;
			imu->mx = l.ai.mx;
			imu->my = l.ai.my;
			imu->mz = l.ai.mz;
			ahrsInput = l.ai;

			l.ai.g5Pitch = min(max(-45.0, (double)l.ai.g5Pitch), 45.0);
			l.ai.g5Roll = min(max(-45.0, (double)l.ai.g5Roll), 45.0);
			// Feed logged G5,GPS,NAV data back into the simulation via spoofed UDP network inputs
			if ((l.flags & LogFlags::g5Ps) /*|| l.ai.g5Ias != ahrsInput.g5Ias || l.ai.g5Tas != ahrsInput.g5Tas || l.ai.g5Palt != ahrsInput.g5Palt*/) {
				ESP32sim_udpInput(7891, strfmt("IAS=%f TAS=%f PALT=%f\n", (double)l.ai.g5Ias, (double)l.ai.g5Tas, (double)l.ai.g5Palt));
			}
			if ((l.flags & LogFlags::g5Nav) /* || l.ai.g5Hdg != ahrsInput.g5Hdg || l.ai.g5Track != ahrsInput.g5Track*/) {
				ESP32sim_udpInput(7891, strfmt("HDG=%f TRK=%f\n", (double)l.ai.g5Hdg, (double)l.ai.g5Track));
			}
			if ((l.flags & LogFlags::g5Ins) /* || l.ai.g5Roll != ahrsInput.g5Roll || l.ai.g5Pitch != ahrsInput.g5Pitch*/) {
				ESP32sim_udpInput(7891, strfmt("R=%f P=%f SL=%f\n", (double)l.ai.g5Roll, (double)l.ai.g5Pitch, (double)l.ai.g5Slip));
			}
			if ((l.flags & LogFlags::ublox) || abs(l.ai.ubloxHdg != ahrsInput.ubloxHdg) < .01) {
				ublox.myGNSS.hdg = magToTrue(l.ai.ubloxHdg) * 100000.0;
				ublox.myGNSS.hac = l.ai.ubloxHdgAcc * 100000.0;
				ublox.myGNSS.alt = l.ai.ubloxAlt * 1000.0 / FEET_PER_METER;
				ublox.myGNSS.gs = l.ai.ubloxGroundSpeed * 0.51444 * 1000;
				ublox.myGNSS.lat = l.ai.lat * 10000000.0;
				ublox.myGNSS.lon = l.ai.lon * 10000000.0;
				ublox.myGNSS.fresh = true;
			}
			if (abs(angularDiff(ahrsInput.gpsTrackRMC - l.ai.gpsTrackRMC)) > .1 || (l.flags & LogFlags::HdgRMC) != 0) {
				char buf[128];
				snprintf(buf, sizeof(buf), "GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,%.2f,130495,003.8,E",
						 magToTrue(l.ai.gpsTrackRMC));
				ESP32sim_udpInput(7891, string(nmeaChecksum(std::string(buf))));
			}
			if ( // abs(angularDiff(ahrsInput.gpsTrackGDL90 - l.ai.gpsTrackGDL90)) > .1 || ahrsInput.gspeed != l.ai.gspeed ||
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

			servoOutput[0] = servoOutput[1] = 0;
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
		// hdgSelect = 2;
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
			ublox.myGNSS.gs = s.hvel * 0.51444 * 1000.0;
			ublox.myGNSS.fresh = true;

			WiFiUDP::InputData buf;
			buf.resize(128);
			int n = gdl90.packMsg11(buf.data(), buf.size(), s);
			buf.resize(n);
			ESP32sim_udpInput(4000, buf);
			buf.resize(128);
			n = gdl90.packMsg10(buf.data(), buf.size(), s);
			buf.resize(n);
			ESP32sim_udpInput(4000, buf);
			;
			ESP32sim_udpInput(7891, strfmt("HDG=%f TRK=%f\n", trueToMag(t2), trueToMag(t2)));
		} else {
			// TODO needs both set?  Breaks with only GDL90
			gpsTrackGDL90 = t1;
			ahrsInput.g5Hdg = t2;
			;

			// TODO: mComp filter breaks at millis() rollover
			// make winglevlr_ubuntu && time ./winglevlr_ubuntu --serial --tracksim ./tracksim_KBFI_14R.txt --seconds 10000  | grep -a "TSIM" > /tmp/simplot.txt && gnuplot -e 'f= "/tmp/simplot.txt"; set y2tic; set ytic nomirror; p f u 5 w l, f u 6 w l, f u 7 w l ax x1y2; pause 111'
			ahrs.mComp.addAux(gpsTrackGDL90, 4, 0.03);
			ahrs.mComp.addAux(ahrsInput.g5Hdg, 2, 0.02);
		}
	}

	std::vector<char> trackSimFileContents;
	// wrap_vector_as_istream tsf;
	ifstream gdl90file;
	void parseArg(char **&a, char **la) override {
		if (strcmp(*a, "--replay") == 0)
			replayFile = *(++a);
		else if (strcmp(*a, "--replaySkip") == 0)
			logSkip = atoi(*(++a));
		else if (strcmp(*a, "--replayReduce") == 0)
			replayReduce = atoi(*(++a));
		else if (strcmp(*a, "--log") == 0) {
			// bm.addPress(pins.midButton, 1, 1, true);  // long press bottom button - start log 1 second in
			logFilename = (*(++a));
		} else if (strcmp(*a, "--servovis") == 0) {
			float svisStartTime;
			sscanf(*(++a), "%f", &svisStartTime);
			svis = new ServoVisualizer();
			svis->startTime = svisStartTime;
			servoSetupMode = 3;
		} else if (strcmp(*a, "--wind") == 0) {
			sscanf(*(++a), "%f@%fG%f", &windDir, &windVel, &windGust);
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
			while (repeat-- > 0) {
				a = fa;
				while (sscanf(*(++a), "%f,%f,%f", &hdg, &dist, &alt) == 3) {
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
			ESP32sim_pinManager::manager->addPress(pin, tim, clicks, longclick);
		} else if (strcmp(*a, "--logConvert") == 0) {
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
			while (1) {
				for (float x = 0; x <= +servoThrow; x += servoThrow / 20)
					setServos(x, 0);
				for (float x = +servoThrow; x >= -servoThrow; x -= servoThrow / 20)
					setServos(x, 0);
				for (float x = -servoThrow; x <= 0; x += servoThrow / 20)
					setServos(x, 0);
				for (float x = 0; x <= +servoThrow; x += servoThrow / 20)
					setServos(0, x);
				for (float x = +servoThrow; x >= -servoThrow; x -= servoThrow / 20)
					setServos(0, x);
				for (float x = -servoThrow; x <= 0; x += servoThrow / 20)
					setServos(0, x);
			}
		} else if (strcmp(*a, "--testStick1") == 0) {
			using namespace ServoControl;
			float x, y;
			sscanf(*(++a), "%f,%f", &x, &y);
			setServos(x, y);
			while (1) {
				sleep(1);
			}
		} else if (strcmp(*a, "--debug") == 0) {
			vector<string> l = split(string(*(++a)), ',');
			float v;
			for (vector<string>::iterator it = l.begin(); it != l.end(); it++) {
				if (sscanf(it->c_str(), "zeros.mx=%f", &v) == 1) {
					ahrs.magOffX = v;
				} else if (sscanf(it->c_str(), "zeros.my=%f", &v) == 1) {
					ahrs.magOffY = v;
				} else if (sscanf(it->c_str(), "zeros.mz=%f", &v) == 1) {
					ahrs.magOffZ = v;
				} else if (sscanf(it->c_str(), "zeros.gx=%f", &v) == 1) {
					ahrs.gyrOffX = v;
				} else if (sscanf(it->c_str(), "zeros.gy=%f", &v) == 1) {
					ahrs.gyrOffY = v;
				} else if (sscanf(it->c_str(), "zeros.gz=%f", &v) == 1) {
					ahrs.gyrOffZ = v;
				} else if (sscanf(it->c_str(), "ahrs.debug=%f", &v) == 1) {
					ahrs.debugVar = v;
				} else if (sscanf(it->c_str(), "cr1=%f", &v) == 1) {
					ahrs.compRatio1 = v;
				} else if (sscanf(it->c_str(), "dc1=%f", &v) == 1) {
					ahrs.driftCorrCoeff1 = v;
				} else if (sscanf(it->c_str(), "ahrs.crhdg=%f", &v) == 1) {
					ahrs.hdgCompRatio = v;
				} else if (sscanf(it->c_str(), "mbt.cr=%f", &v) == 1) {
					ahrs.magBankTrimCr = v;
				} else if (sscanf(it->c_str(), "mbt.maxerr=%f", &v) == 1) {
					ahrs.magBankTrimMaxBankErr = v;
				} else if (sscanf(it->c_str(), "dipconstant=%f", &v) == 1) {
					ahrs.magDipConstant = v;
				} else if (sscanf(it->c_str(), "ahrs.crpitch=%f", &v) == 1) {
					ahrs.compRatioPitch = v;
				} else if (sscanf(it->c_str(), "ahrs.useauxmpu=%f", &v) == 1) {
					ESP32csim_useAuxMpu = v;
				} else if (sscanf(it->c_str(), "ahrs.gxdecel=%f", &v) == 1) {
					ahrs.gXdecelCorrelation = v;
				} else if (sscanf(it->c_str(), "ahrs.bankanglescale=%f", &v) == 1) {
					ahrs.bankAngleScale = v;
				} else if (sscanf(it->c_str(), "ubloxcr=%f", &v) == 1) {
					ubloxHdgCr = v;
				} else if (sscanf(it->c_str(), "pids.pitch.itrim=%f", &v) == 1) {
					pids.pitchPID.inputTrim = v;
				} else if (strlen(it->c_str()) > 0) {
					//printf("Unknown debug parameter '%s'\n", it->c_str());
					//exit(-1);
				}
			}
		} else {
			//printf("Unknown debug parameter '%s'\n", *a);
			//exit(-1);
		}
	}
	void setup() override {
		setServos(1, 1);
		ServoControl::servoToStick(servoOutput[0], servoOutput[1]);
		if (replayFile == NULL) {
			ESP32sim_pinManager::manager->addPress(pins.knobButton, 1, 1, true); // knob long press - arm servo
																				 // bm.addPress(pins.botButton, 250, 1, true); // bottom long press - test turn activate
																				 // bm.addPress(pins.topButton, 200, 1, false); // top short press - wings level mode
																				 // bm.addPress(pins.topButton, 300, 1, false); // top short press - hdg hold
																				 // setDesiredTrk(ahrsInput.dtk = 135);
		}
		ESP32sim_pinManager::manager->addPress(pins.knobButton, 1, 1, true); // knob long press - arm servo
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
				float g5hdg = DEG2RAD(track); // TODO mag/true
				float g5roll = DEG2RAD(-bank);
				float g5pitch = DEG2RAD(pitch);
				ESP32sim_udpInput(7891, strfmt("IAS=%f TAS=%f PALT=%f\n", 90, 100, 1000));
				ESP32sim_udpInput(7891, strfmt("P=%f R=%f\n", g5pitch, g5roll));
				// now handled by onNavigate();
				// ESP32sim_udpInput(7891, strfmt("HDG=%f\n", g5hdg));
			}
			flightSim(imu);

			if (at(5.0) && wpFile.length()) {
				wpNav = new WaypointsSequencerFile(wpFile.c_str());
				hdgSelect = 3;
			}
			if (at(1.0)) { // alt bug to ~1200 ft
				ESP32sim_udpInput(7891, "KSEL=2 KVAL=350.0\n");
			}
		} else {
			if (firstLoop == true) {
				ifile = ifstream(replayFile, ios_base::in | ios::binary);
			}
			while (logSkip > 0 && logSkip-- > 0) {
				ESP32sim_replayLogItem(ifile);
			}
			if (ESP32sim_replayLogItem(ifile) == false) {
				ESP32sim_exit();
			}
			logEntries++;
		}

		// if (now >= 500 && lastTime < 500) {	Serial.inputLine = "pitch=10\n"; }
		// if (now >= 100 && lastTime < 100) {	Serial.inputLine = "zeroimu\n"; }
		firstLoop = false;
		lastTime = now;
	}
	void done() override {
		printf("# %f %f %f avg roll/pitch/hdg errors, %d log entries, %.1f real time seconds\n",
			   totalError.roll.err(), totalError.pitch.err(), totalError.hdg.err(), (int)totalError.hdg.hist.size(),
			   millis() / 1000.0);
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
 * 	27 backlght 32  (KNOB)
 * 	GND(GND)	26  (ROT)
 * 	0 (ROT)		GND
 * 	GND			3.3V
 * 	RXD			22  (used by I2c?)
 * 	TXD			21	(used by I2c?)
 * 	VBAT		5V
 */

/*  TTGO TS 1.2 old- buttons 39, 34, 35
 *    39   36
 *    33   RST
 *    27   32
 *    GND  26
 *    0    GND
 *    GND
 */
