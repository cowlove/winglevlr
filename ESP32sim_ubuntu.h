
#include <cstdint>
#include <algorithm>
#include <vector>
#include <cstring>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

using namespace std;

uint64_t millis() { static uint64_t m = 0; return ++m; }
uint64_t micros() { return millis() * 1000; }
void pinMode(int, int) {}
int digitalRead(int) { return 1; }
int digitalPinToInterrupt(int) { return 0; }
void digitalWrite(int, int) {};
void attachInterrupt(int, void (*)(), int) {} 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void ledcWrite(int, int) {} 
void delay(int) {}
void delayMicroseconds(int) {}
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
	void begin() {}
	void begin(int, int, int, int) {}
	void begin(int, int) {}
	int print(const char *p) { return this->printf("%s", p); }
	int println(const char *p= NULL) { return this->printf("%s\n", p != NULL ? p : ""); }
	int print(char c) { return this->printf("%c", c); } 
	int printf(const char  *f, ...) { 
		va_list args;
		va_start (args, f);
		int r = vprintf (f, args);
		va_end (args);
		return r;
	}
	void setTimeout(int) {} 
	int available() { return 0; }
	int readBytes(uint8_t *, int) { return 0; } 
	int write(const uint8_t *, int) { return 0; }
} Serial, Serial1;

#define WL_CONNECTED 0
class String {
	public:
	String(const char *) {}
	String() {}
	bool operator!=(const String&) { return false; } 
	const char *c_str(void) { return "FakeString";  }
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
public:
	void begin(int) {}
	void beginPacket(IPAddress, int) {}
	void write(uint8_t *, int) {}
	void endPacket() {}
	int parsePacket() { return 0; } ;
	int read(const uint8_t *, int) { return 0; }
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
int main() {
	setup();
	while(1) loop();
}
