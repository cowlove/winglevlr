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
#include "RunningLeastSquares.h"

using namespace std;
static uint64_t _micros = 0;
uint64_t micros() { return ++_micros; }
uint64_t millis() { return micros() / 1000; }
void pinMode(int, int) {}
static int ESP32sim_currentPwm = 0;
float ESP32sim_getPitchCmd();

typedef int esp_err_t; 
void esp_task_wdt_init(int, int) {}
void esp_task_wdt_reset() {}
esp_err_t esp_task_wdt_add(void *) { return 0; }

struct {
	void begin() {}
	void handle() {}
} ArduinoOTA;

struct {
	int getFreeHeap() { return 0; }
} ESP;

int digitalRead(int p) {
	// HACK simple proof-of-concept to simulate button push and arm
	// the servos  
	
	if (p == 35 && millis() > 1000 && millis() < 3100) return 0;  // arm servos
	//if (p == 39 && millis() > 5000 && millis() < 7100) return 0; // start logging
	
	return 1; 
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
} Serial, Serial1;

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


float ESP32sim_pitchCmd = 940.0;

class WiFiUDP {
	int port, txPort;
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
			printf("UDP: %s", b);
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
void ESP32sim_set_g5(float p, float r, float h);
void ESP32sim_set_desiredTrk(float v);
extern float desRoll;

struct { 
	float pitch, roll, hdg, ias, tas, alt, knobVal;
	int knobSel, age; 
} g5;

void ESP32sim_run() { 
	static float lastTime = 0;
	float now = micros() / 1000000.0;

	if (floor(now / .1) != floor(lastTime / .1)) {
		char buf[128];
		snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %.3f %.3f %d %.3f %d CAN\n",
			g5.pitch, g5.roll, g5.hdg, g5.ias, g5.tas, g5.alt, g5.knobSel, g5.knobVal, g5.age);
		WiFiUDP::inputMap[7891] = String(buf);
	}

	if (now > 001) ESP32sim_set_desiredTrk(90);
	if (now > 100) ESP32sim_set_desiredTrk(185);
	if (now > 300) ESP32sim_set_desiredTrk(90);
	if (now > 500) ESP32sim_set_desiredTrk(175);
		
	if (now >= 100 && lastTime < 100) {	Serial.inputLine = "pitch=10\n"; }
	if (now >= 400 && lastTime < 400) {	Serial.inputLine = "zeroimu\n"; }

	lastTime = now;
}

void ESP32sim_JDisplay_forceUpdate();


class MPU9250_DMP {
	float bank = 0, track = 0, simPitch = 0;
	RollingAverage<float,200> rollCmd;
	uint64_t lastMillis = 0;
	float cmdPitch;
public:
	
	int begin(){ return true; }
	void setGyroFSR(int) {};
    void setAccelFSR(int) {};
    void setSensors(int) {}
	void updateAccel() { 
		ESP32sim_run();
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
	}

	std::queue<float> gxDelay, pitchDelay;
	
	void updateGyro() {
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
		rollCmd.add((ESP32sim_currentPwm - 4915.0) / 4915.0);
		gy = rollCmd.average() * 2.0;
		bank += gy * .01;
		bank = max(-15.0, min(15.0, (double)bank));
		if (floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			//printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f\n", (float)millis()/1000.0, 
			//ESP32sim_currentPwm, track, desRoll, bank, gy);
		}
		uint64_t now = millis();
		if (floor(lastMillis / 1000) != floor(now / 1000)) { // 1hz
			track -= bank * 0.15;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;
			ESP32sim_JDisplay_forceUpdate();	
		}

		g5.hdg = track * M_PI / 180;
		g5.roll = -bank * M_PI / 180;
		g5.pitch = pitch * M_PI / 180;

		lastMillis = now;
	}
	void updateCompass() {}
	float calcAccel(float x) { return x; }
	float calcGyro(float x) { return x; }
	float calcQuat(float x) { return x; }
	float calcMag(float x) { return x; }
	float ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,pitch,roll,yaw;
	MPU9250_DMP() { bzero(this, sizeof(this)); } 
};

typedef char byte;

#include "TinyGPS++.h"
#include "TinyGPS++.cpp"

void setup(void);
void loop(void);
static void JDisplayToConsole(bool b);
int main(int argc, char **argv) {
	float seconds = 0;
	for(char **a = argv + 1; a < argv+argc; a++) {
		if (strcmp(*a, "--serial") == 0) Serial.toConsole = true;
		if (strcmp(*a, "--jdisplay") == 0) JDisplayToConsole(true);
		if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%f", &seconds); 
	}
	
	setup();
	while(seconds <= 0 || _micros / 1000000 < seconds) loop();
}
