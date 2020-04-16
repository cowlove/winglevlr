
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


using namespace std;
static uint64_t _micros = 0;
uint64_t micros() { return ++_micros; }
uint64_t millis() { return micros() / 1000; }
void pinMode(int, int) {}
int digitalRead(int) { return 1; }
int digitalPinToInterrupt(int) { return 0; }
void digitalWrite(int, int) {};
void attachInterrupt(int, void (*)(), int) {} 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void ledcWrite(int, int) {} 
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
		i = ifstream(fname, ios_base::in | ios::binary);
		nextPacket = _micros;
		delay = 50;
	}
	void beginPacket(IPAddress, int) {}
	void write(uint8_t *, int) {}
	void endPacket() {}
	int parsePacket() { return i.good() &&  _micros > nextPacket; } ;
	int read(uint8_t *buf, int buflen) {
		nextPacket = _micros + delay; 
		i.read((char *)buf, 1);
		return i.gcount();
	}
	IPAddress remoteIP() { return IPAddress(); } 
};
#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 0
#define INV_XYZ_COMPASS 0

class MPU9250_DMP {
public:
	int begin(){ return true; }
	void setGyroFSR(int) {};
    void setAccelFSR(int) {};
    void setSensors(int) {}
	void updateAccel() {}
	void updateGyro() {}
	void updateCompass() {}
	float calcAccel(float) { return 0; }
	float calcGyro(float) { return 0; }
	float calcQuat(float) { return 0; }
	float calcMag(float) { return 0; }
	float ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,pitch,roll,yaw;
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
