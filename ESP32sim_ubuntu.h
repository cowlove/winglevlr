#include <cstdint>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
#include "RunningLeastSquares.h"


typedef int SemaphoreHandle_t;
int xSemaphoreCreateCounting(int, int) { return 0; } 
int xSemaphoreGive(int) { return 0; } 
int xSemaphoreTake(int, int) { return 0; }
#define portMAX_DELAY 0 


using namespace std;
static uint64_t _micros = 0;
uint64_t micros() { return ++_micros & 0xffffffff; }
uint64_t millis() { return ++_micros / 1000; }
void pinMode(int, int) {}
static int ESP32sim_currentPwm = 0;
extern float ESP32sim_getPitchCmd();
extern void ESP32sim_setLogFile(const char *);
extern float ESP32sim_getRollErr();
void ESP32sim_convertLogCtoD(ifstream &i, ofstream &o);
void printFinalReport();

extern void ESP32sim_setDebug(const char *);
extern double totalRollErr, totalHdgError;

typedef int esp_err_t; 
void esp_task_wdt_init(int, int) {}
void esp_task_wdt_reset() {}
esp_err_t esp_task_wdt_add(void *) { return 0; }

class File {
public:
	operator bool() { return false; } 
	int read(uint8_t *, int) { return 0; } 
	int close() { return 0; } 
	int printf(const char *, ...) { return 0; } 
};

struct {
	void begin() {}
	void format() {}
	File open(const char *, const char *) { return File(); } 
} SPIFFS;

struct {
	void begin() {}
	void handle() {}
} ArduinoOTA;

struct {
	int getFreeHeap() { return 0; }
} ESP;

class ButtonManager {
	struct PressInfo { int pin; float start; float duration; };
	std::vector<PressInfo> presses;
public:
	void add(int pin, float start, float duration) {
		PressInfo pi; 
		pi.pin = pin; pi.start = start; pi.duration = duration;
		presses.push_back(pi);
	}
	
	void addPress(int pin, float time, int clicks, bool longPress)  {
		for(int n = 0; n < clicks; n++) {
			float duration = longPress ? 2.5 : .2; 
			add(pin, time,  duration);
			time += duration + .2;
		}
	}
	
	int check(int pin, float time) {
		float now = millis() / 1000.0;
		for (vector<PressInfo>::iterator it = presses.begin(); it != presses.end(); it++) { 
			if (it->pin == pin && now >= it->start && now < it->start + it->duration)
				return 0;
		} 
		return 1;
	} 
} bm;

int digitalRead(int p) {
	// HACK simple proof-of-concept to simulate button push and arm
	// the servos  
	float now = millis()/1000.0;
	
	//if (p == 35 && now >= 1 && now < 3.1) return 0;  // arm servos
	//if (p == 34 && now >= 110 && now < 113.1) return 0;  // activate test turn mode

	return bm.check(p, now);
}
void ledcWrite(int chan, int val) { 
	ESP32sim_currentPwm = val;
	//printf("pwm %d\n", val);
} 
int digitalPinToInterrupt(int) { return 0; }
void digitalWrite(int, int) {};
void attachInterrupt(int, void (*)(), int) {} 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void delayMicroseconds(int m) { _micros += m; }
void delay(int m) { delayMicroseconds(m*1000); }
void yield() {}

#define radians(x) ((x)*M_PI/180)
#define degrees(x) ((x)*180.0/M_PI)
#define sq(x) ((x)*(x))
#define TWO_PI (2*M_PI)
#define INPUT_PULLUP 0 
#define OUTPUT 0 
#define CHANGE 0 
#define INPUT 0 
#define INPUT_PULLDOWN 0
#define SERIAL_8N1 0
#define ST7735_RED 0 
#define ST7735_GREEN 0 
#define ST7735_BLACK 0 
#define ST7735_WHITE 0 
#define ST7735_YELLOW 0 

class String {
	public:
	std::string st;
	String(const char *s) : st(s) {}
	String(std::string s) : st(s) {}
	String(int s) : st(std::to_string(s)) {}
	String() {}
	int length() { return st.length(); } 
	bool operator!=(const String& x) { return st != x.st; } 
	String &operator+(const String& x) { st = st + x.st; return *this; } 
	const char *c_str(void) { return st.c_str(); }
};

class IPAddress {
public:
	void fromString(const char *) {}
	String toString() { return String(); }	
};

class FakeSerial { 
	public:
	String inputLine;
	bool toConsole = false;
	void begin() {}
	void begin(int, int, int, int) {}
	void begin(int, int) {}
	void print(const char *p) { this->printf("%s", p); }
	void println(const char *p= NULL) { this->printf("%s\n", p != NULL ? p : ""); }
	void print(char c) { this->printf("%c", c); } 
	void printf(const char  *f, ...) { 
		va_list args;
		va_start (args, f);
		if (toConsole) 
			vprintf (f, args);
		va_end (args);
	}
	void setTimeout(int) {}
	void flush() {} 
	int available() { return inputLine.length(); }
	int readBytes(uint8_t *b, int l) {
		int rval =  min(inputLine.length(), l);
		strncpy((char *)b, inputLine.c_str(), rval);
		inputLine = String();
		return rval;
	} 
	int write(const uint8_t *, int) { return 0; }
	int write(const char *) { return 0; }	
} Serial, Serial1, Serial2;

#define WL_CONNECTED 0
#define WIFI_STA 0
class FakeWiFi {
	public:
	int status() { return WL_CONNECTED; } 
	IPAddress localIP() { return IPAddress(); } 
	void setSleep(bool) {}
	void mode(int) {}
	void disconnect(bool) {}
} WiFi;

class msdFile {
	public: 
	bool operator!() { return false; } 
	operator bool() { return false; } 
	msdFile openNextFile(void) { return *this; }
	void close(void) {}
	
};

class FakeSD {
	public:
	bool begin(int, int, int, int) { return true; }
	msdFile open(const char *) { return msdFile(); } 
} SD;

class WiFiMulti {
public:
	void addAP(const char *, const char *) {}
	void run() {}
};


void ESP32sim_JDisplay_forceUpdate();
bool ESP32sim_replayLogItem(ifstream &);
int logEntries = 0;
const int ACC_FULL_SCALE_4_G = 0, GYRO_FULL_SCALE_250_DPS = 0, MAG_MODE_CONTINUOUS_100HZ = 0;
float ESP32sim_pitchCmd = 940.0;

class WiFiUDP {
	int port, txPort;
	bool toSerial = false;
public:
	void begin(int p) { port = p; }
	void beginPacket(IPAddress, int p) { beginPacket(NULL, p); }
	void beginPacket(const char *, int p) { txPort = p; }
	void write(const uint8_t *b, int len) {
		float f;
		if (txPort == 7892 && sscanf((const char *)b, "trim %f", &f) == 1) {
			ESP32sim_pitchCmd = f;
		}
	}
	void endPacket() {}

	static std::map<int,String> inputMap;
	int  parsePacket() { 
		if (inputMap.find(port) != inputMap.end()) { 
			return inputMap.find(port)->second.length();
		} else { 
			return 0;
		}
	}
	int read(uint8_t *b, int l) {
		if (inputMap.find(port) != inputMap.end()) { 
			String inputLine = inputMap.find(port)->second;
			int rval =  min(inputLine.length(), l);
			strncpy((char *)b, inputLine.c_str(), rval);
			if (toSerial) { 
				printf("UDP: %s", b);
			}
			inputMap.erase(port);
			return rval;
		} else { 
			return 0;
		}
	}

	IPAddress remoteIP() { return IPAddress(); } 
};

std::map<int,String> WiFiUDP::inputMap;
void udpInput(int p, String s) { 
	WiFiUDP::inputMap[p] = s;
}

#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 1
#define INV_XYZ_COMPASS 0
// TODO: remove these pokes, instead send NMEA or GDL90 data 
// to the simulated UDP ports
void ESP32sim_set_gpsTrackGDL90(float v);
//void ESP32sim_set_g5(float p, float r, float h);
void ESP32sim_set_desiredTrk(float v);
extern float desRoll;

struct { 
	float pitch, roll, hdg, ias, tas, alt, knobVal;
	int knobSel, age; 
} g5;


void ESP32sim_simulateG5Input(float pit, float roll, float hdg, float ias, float tas, float alt, int knobSel, float knobVal, int age) { 
	char buf[128];
	int mode = 0;
	snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %d %.3f %d %d CAN\n",
		pit*M_PI/180, roll*M_PI/180, hdg*M_PI/180, 0.0, ias*0.5144, tas*0.5144, alt*3.2808, knobSel, knobVal, age, mode);
	WiFiUDP::inputMap[7891] = String(buf);
}


static const char *replayFile;
ifstream ifile;
static int logSkip; //log entries to skip

class MPU9250_DMP {
	float bank = 0, track = 0, simPitch = 0;
	RollingAverage<float,500> rollCmd;
	uint64_t lastMillis = 0;
	float cmdPitch;
public:
	int begin(){ 
		if (replayFile != NULL) { 
			ifile = ifstream(replayFile, ios_base::in | ios::binary);
		}
		return true; 
	}
	void setGyroFSR(int) {};
    void setAccelFSR(int) {};
    void setSensors(int) {}
	void updateAccel() {}
	void accelUpdate() { updateAccel(); }
	void magUpdate() { updateCompass(); }
	void gyroUpdate() { updateGyro(); }

	float accelX() { return ax; } 
	float accelY() { return ay; } 
	float accelZ() { return az; } 
	float magX() { return mx; } 
	float magY() { return my; } 
	float magZ() { return mz; } 
	float gyroX() { return gx; } 
	float gyroY() { return gy; } 
	float gyroZ() { return gz; } 
	
	void beginAccel(int) { begin(); }
	void beginGyro(int) {}
	void beginMag(int) {}
	int readId(uint8_t *) { return 0; } 
	
	std::queue<float> gxDelay, pitchDelay;

	void flightSim() { 
		float rawCmd = ESP32sim_pitchCmd;
		cmdPitch = rawCmd > 0 ? (940 - rawCmd) / 13 : 0;
		float ngx = (cmdPitch - simPitch) * 1.3;
		ngx = max((float)-5.0,min((float)5.0, ngx));
		simPitch += (cmdPitch - simPitch) * .0015;
		gxDelay.push(ngx);
		pitchDelay.push(simPitch);
				
		// Simulate simple airplane roll/bank/track turn response to 
		// servooutput read from ESP32sim_currentPwm; 
		_micros += 3500;
		const float servoTrim = 4915.0;
		rollCmd.add((ESP32sim_currentPwm - servoTrim) / servoTrim);
		gy = 0.0 + rollCmd.average() * 10.0;
		bank += gy * (3500.0 / 1000000.0);
		bank = max(-20.0, min(20.0, (double)bank));
		if (floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f\n", (float)millis()/1000.0, 
			ESP32sim_currentPwm, track, desRoll, bank, gy);
		}
		
		gz = +1.5 + tan(bank * M_PI/180) / 100 * 1091;
		
		
		uint64_t now = millis();
		const float bper = .07;
		if (floor(lastMillis * bper) != floor(now * bper)) { // 10hz
			track -= tan(bank * M_PI / 180) * 9.8 / 40 * 25 * bper;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;
		}

		float hdg = track - 35.555; // simluate mag var and WCA 
		if (hdg < 0) hdg += 360;	
		g5.hdg = hdg * M_PI / 180;
		g5.roll = -bank * M_PI / 180;
		g5.pitch = pitch * M_PI / 180;
		ESP32sim_set_gpsTrackGDL90(track);

		while(gxDelay.size() > 400) {
			gxDelay.pop();
			pitchDelay.pop();
			gx = gxDelay.front();
			pitch = pitchDelay.front();
		}
		//printf("SIM %08.3f %+05.2f %+05.2f %+05.2f\n", millis()/1000.0, gx, pitch, cmdPitch);
		az = cos(pitch * M_PI / 180) * 1.0;
		ay = sin(pitch * M_PI / 180) * 1.0;
		ax = 0;

		// simulate meaningless mag readings that stabilize when bank == 0 
		mx = my = bank * 1.4;
		mz += mx;
		
		lastMillis = now;
	}

	void ESP32sim_run() { 
		static float lastTime = 0;
		float now = _micros / 1000000.0;

		if (replayFile == NULL) { 
			if (floor(now / .1) != floor(lastTime / .1)) {
				ESP32sim_simulateG5Input(g5.pitch, g5.roll, g5.hdg, g5.ias, g5.tas, g5.alt, g5.knobSel, g5.knobVal, g5.age);
			}
			flightSim();
		} else {
			while(logSkip > 0 && logSkip-- > 0) {
				ESP32sim_replayLogItem(ifile);
			}
			if (ESP32sim_replayLogItem(ifile) == false) { 
				printFinalReport();
				exit(0);
			}
			logEntries++;
		}

		//if (now >= 500 && lastTime < 500) {	Serial.inputLine = "pitch=10\n"; }
		//if (now >= 100 && lastTime < 100) {	Serial.inputLine = "zeroimu\n"; }

		lastTime = now;
	}

	
	//LogItem l, prevl;
	void updateGyro() {
	}
	void updateCompass() {}
	float calcAccel(float x) { return x; }
	float calcGyro(float x) { return x; }
	float calcQuat(float x) { return x; }
	float calcMag(float x) { return x; }
	float ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,pitch,roll,yaw;
	MPU9250_DMP(int addr = 0x68) { bzero(this, sizeof(this)); } 
};

typedef MPU9250_DMP MPU9250_asukiaaa;

typedef char byte;

#include "TinyGPS++.h"
#include "TinyGPS++.cpp"

void setup(void);
void loop(void);
static void JDisplayToConsole(bool b);

void printFinalReport() { 
	printf("# %f %f avg roll/hdg errors, %d log entries, %.1f real time seconds\n", ESP32sim_getRollErr() / logEntries, totalHdgError / logEntries,  logEntries, millis() / 1000.0);
	exit(0);
}

int main(int argc, char **argv) {
	float seconds = 0;
	for(char **a = argv + 1; a < argv+argc; a++) {
		if (strcmp(*a, "--serial") == 0) Serial.toConsole = true;
		if (strcmp(*a, "--jdisplay") == 0) JDisplayToConsole(true);
		if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%f", &seconds); 
		if (strcmp(*a, "--debug") == 0) {
			ESP32sim_setDebug(*(++a));
		} 
		if (strcmp(*a, "--replay") == 0) replayFile = *(++a);
		if (strcmp(*a, "--replaySkip") == 0) logSkip = atoi(*(++a));
		if (strcmp(*a, "--log") == 0) { 
			bm.addPress(39, 1, 1, true);  // long press top button - start log 1 second in  
			ESP32sim_setLogFile(*(++a));
		}	
		if (strcmp(*a, "--logConvert") == 0) {
			ifstream i = ifstream(*(++a), ios_base::in | ios::binary);
			ofstream o = ofstream(*(++a), ios_base::out | ios::binary);
			
			ESP32sim_convertLogCtoD(i, o);
			exit(0);
		}
	}
	
	bm.addPress(32, 1, 1, true); // knob long press - arm servo
	bm.addPress(37, 250, 1, true); // mid long press - test turn activate 
	bm.addPress(39, 500, 1, false); // top short press - hdg hold 

	ESP32sim_set_desiredTrk(90);

	setup();
	uint64_t lastMillis = 0;
	double totalErr = 0;
	while(seconds <= 0 || _micros / 1000000.0 < seconds) {
		uint64_t now = millis();
		loop();
 

		if (floor(now / 1000) != floor(lastMillis / 1000)) { 
			ESP32sim_JDisplay_forceUpdate();	
		}
		lastMillis = now;
	}
	printFinalReport();
}
