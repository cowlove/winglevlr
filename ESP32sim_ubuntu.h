#include <cstdint>
#include <algorithm>
#include <vector>
#include <cstring>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include "RunningLeastSquares.h"

using namespace std;
static uint64_t _micros = 0;
uint64_t micros() { return ++_micros; }
uint64_t millis() { return micros() / 1000; }
void pinMode(int, int) {}
static int ESP32sim_currentPwm = 0;
int digitalRead(int p) {
	// HACK simple proof-of-concept to simulate button push and arm
	// the servos  
	if (p == 35 && millis() > 100 && millis() < 200)
		return 0;
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

class FakeSerial { 
	public:
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
	int available() { return 0; }
	int readBytes(uint8_t *, int) { return 0; } 
	int write(const uint8_t *, int) { return 0; }
} Serial, Serial1;

#define WL_CONNECTED 0
class String {
	public:
	std::string st;
	String(const char *s) : st(s) {}
	String() {}
	bool operator!=(const String& x) { return st != x.st; } 
	const char *c_str(void) { return st.c_str(); }
};
class IPAddress {
public:
	void fromString(const char *) {}
	String toString() { return String(); }	
};
class FakeWiFi {
	public:
	int status() { return WL_CONNECTED; } 
	IPAddress localIP() { return IPAddress(); } 
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

class WiFiUDP {
	ifstream i;
	int delay;
	int nextPacket;
public:
	void begin(int p) {
		char fname[256];
		snprintf(fname, sizeof(fname), "udp%04d.dat", p);
		// TMP HACK - don't read GDL data, let sim do it below 
		//	i = ifstream(fname, ios_base::in | ios::binary);
		nextPacket = _micros;
		delay = 500;
	}
	void beginPacket(IPAddress, int) {}
	void write(uint8_t *, int) {}
	void endPacket() {}
	int parsePacket() { return i.good() &&  _micros > nextPacket; } ;
	int read(uint8_t *buf, int buflen) {
		nextPacket += delay; 
		i.read((char *)buf, 1);
		return i.gcount();
	}
	IPAddress remoteIP() { return IPAddress(); } 
};
#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 0
#define INV_XYZ_COMPASS 0
// TODO: remove these pokes, instead send NMEA or GDL90 data 
// to the simulated UDP ports
void ESP32sim_set_gpsTrackGDL90(float v);
void ESP32sim_set_desiredTrk(float v);
extern float desRoll;


void ESP32sim_JDisplay_forceUpdate();

class MPU9250_DMP {
	float bank = 0, track = 0;
	RollingAverage<float,200> rollCmd;
	uint64_t lastMillis = 0;
public:
	int begin(){ return true; }
	void setGyroFSR(int) {};
    void setAccelFSR(int) {};
    void setSensors(int) {}
	void updateAccel() {}
	void updateGyro() {
		// Simulate simply airplane roll/bank/track turn response to 
		// servooutput read from ESP32sim_currentPwm; 
		_micros += 3500;
		rollCmd.add((ESP32sim_currentPwm - 4915.0) / 4915.0);
		gy = rollCmd.average() * 2.0;
		bank += gy * .01;
		bank = max(-15.0, min(15.0, (double)bank));
		if (floor(lastMillis / 100) != floor(millis() / 100)) { // 10hz
			//printf("%08.3f servo %05d track %05.2f desRoll: %+06.2f bank: %+06.2f gy: %+06.2f\n", (float)millis()/1000.0, currentPwm, track, desRoll, bank, gy);
		}
		uint64_t now = millis();
		if (floor(lastMillis / 1000) != floor(now / 1000)) { // 1hz
			track -= bank * 0.15;
			if (track < 0) track += 360;
			if (track > 360) track -= 360;	
			ESP32sim_set_gpsTrackGDL90(track);
			ESP32sim_JDisplay_forceUpdate();	
		}
		int s = now/1000;
		if (s > 001) ESP32sim_set_desiredTrk(90);
		if (s > 100) ESP32sim_set_desiredTrk(180);
		if (s > 300) ESP32sim_set_desiredTrk(90);
		if (s > 500) ESP32sim_set_desiredTrk(180);

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
