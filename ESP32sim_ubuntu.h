/* Simple library and simulation environment to compile and run an Arduino sketch as a 
 * standard C command line program. 
 * 
 * Most functionality is unimplemented, and just stubbed out but minimal simluated 
 * Serial/UDP/Interrupts/buttons are sketched in. 
 */

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

static uint64_t _micros = 0;
uint64_t micros() { return ++_micros & 0xffffffff; }
uint64_t millis() { return ++_micros / 1000; }

//#include "RunningLeastSquares.h"
//#include "jimlib.h"

// Stub out FreeRTOS stuff 
typedef int SemaphoreHandle_t;
int xSemaphoreCreateCounting(int, int) { return 0; } 
int xSemaphoreGive(int) { return 0; } 
int xSemaphoreTake(int, int) { return 0; }
#define portMAX_DELAY 0 
#define tskIDLE_PRIORITY 0
void xTaskCreate(void (*)(void *), const char *, int, void *, int, void *) {}

#define WRITE_PERI_REG(a, b) if(0) {}
#define RTC_CNTL_BROWN_OUT_REG 0

std::string strfmt(const char *, ...); 
using namespace std;
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
		float now = millis() / 1000.0; // TODO this is kinda slow 
		for (vector<PressInfo>::iterator it = presses.begin(); it != presses.end(); it++) { 
			if (it->pin == pin && now >= it->start && now < it->start + it->duration)
				return 0;
		} 
		return 1;
	} 
} bm;

int digitalRead(int p) {
	return bm.check(p, millis()/1000.0);
}
void ledcWrite(int chan, int val) { 
	ESP32sim_currentPwm = val;
} 

class InterruptManager { 
	ifstream ifile;
	uint64_t nextInt = 0; 
	int count = 0;
public:
	void (*intFunc)() = NULL;
	void getNext() { 
		int delta;
		ifile >> delta;
		delta = min(100000, delta);
		nextInt += delta;
		ifile >> delta;
		delta = min(50000, delta);
		nextInt += delta;
		count++;
	}
	void setInterruptFile(char const *fn) { 
		ifile = ifstream(fn, ios_base::in);
		getNext();
	}
	void run() {
		if (intFunc != NULL && ifile && micros() >= nextInt) { 
			intFunc();
			getNext();
		}
	}
} intMan;

int digitalPinToInterrupt(int) { return 0; }
void digitalWrite(int, int) {};
void attachInterrupt(int, void (*i)(), int) { intMan.intFunc = i; } 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void delayMicroseconds(int m) { _micros += m; intMan.run(); }
void delay(int m) { delayMicroseconds(m*1000); }
void yield() { intMan.run(); }
void analogSetCycles(int) {}
void adcAttachPin(int) {}
int analogRead(int) { return 0; } 

#define radians(x) ((x)*M_PI/180)
#define degrees(x) ((x)*180.0/M_PI)
#define sq(x) ((x)*(x))
#define TWO_PI (2*M_PI)
#define INPUT_PULLUP 0 
#define OUTPUT 0 
#define CHANGE 0 
#define RISING 0
#define FALLING 0
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
	String(const char *b, int l) { 
		st = std::string();
		for (int n = 0; n < l; n++) {
			st.push_back(b[n]);
		}
	} 
	String(int s) : st(std::to_string(s)) {}
	String() {}
	int length() const { return st.length(); } 
	bool operator!=(const String& x) { return st != x.st; } 
	String &operator+(const String& x) { st = st + x.st; return *this; } 
	const char *c_str(void) const { return st.c_str(); }
};

class IPAddress {
public:
	void fromString(const char *) {}
	String toString() const { return String(); }	
};

class FakeSerial { 
	public:
	String inputLine;
	bool toConsole = false;
	void begin() {}
	void begin(int, int, int, int) {}
	void begin(int, int) {}
	void print(int, int) {}
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
#define DEC 0
#define HEX 0 

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
const int ACC_FULL_SCALE_4_G = 0, GYRO_FULL_SCALE_250_DPS = 0, MAG_MODE_CONTINUOUS_100HZ = 0;

extern float ESP32sim_pitchCmd; 

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
			for (int n = 0; n < rval; n++) { 
				b[n] = inputLine.st[n];
			}
			//strncpy((char *)b, inputLine.c_str(), rval);
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

void ESP32sim_udpInput(int p, String s) { 
	std::map<int,String> &m = WiFiUDP::inputMap;
	if (m.find(p) == m.end())
		m[p] = s;
	else
		m[p] = m[p] + s;
}

class WireC {
public:
	void begin(int, int) {}
	void beginTransmission(int) {}
	bool endTransmission() { return false; }
} Wire;

#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 1
#define INV_XYZ_COMPASS 0

class MPU9250_DMP {
public:
	int address = 0;
	int begin(){ return 1; }
	void setWire(WireC *) {}
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
	
	void updateGyro() {}
	void updateCompass() {}
	float calcAccel(float x) { return x; }
	float calcGyro(float x) { return x; }
	float calcQuat(float x) { return x; }
	float calcMag(float x) { return x; }
	float ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz;
	//pitch,roll,yaw;
	MPU9250_DMP(int addr = 0x68) { bzero(this, sizeof(this)); } 
};

typedef MPU9250_DMP MPU9250_asukiaaa;
typedef char byte;

class WebServer {
	public:
	WebServer(int) {}
};

void setup(void);
void loop(void);
static void JDisplayToConsole(bool b);

void ESP32sim_parseArg(char **&a, char **end);
void ESP32sim_setup();
void ESP32sim_loop();
void ESP32sim_done();

int main(int argc, char **argv) {
	float seconds = 0;
	for(char **a = argv + 1; a < argv+argc; a++) {
		if (strcmp(*a, "--serial") == 0) Serial.toConsole = true;
		else if (strcmp(*a, "--jdisplay") == 0) JDisplayToConsole(true);
		else if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%f", &seconds); 
		else if (strcmp(*a, "--debug") == 0) {
			ESP32sim_setDebug(*(++a));
		} 
		else if (strcmp(*a, "--interruptFile") == 0) {
			intMan.setInterruptFile(*(++a));
		}
		else ESP32sim_parseArg(a, argv + argc);		
	}
	
	ESP32sim_setup();
	setup();

	uint64_t lastMillis = 0;
	double totalErr = 0;
	while(seconds <= 0 || _micros / 1000000.0 < seconds) {
		uint64_t now = millis();
		ESP32sim_loop();
		loop();
		intMan.run();

		if (floor(now / 1000) != floor(lastMillis / 1000)) { 
			ESP32sim_JDisplay_forceUpdate();	
		}
		lastMillis = now;
	}
	ESP32sim_done();
}

/************************************************
 * Add the following stubs to project *.INO file 
 ************************************************
#ifdef UBUNTU
void ESP32sim_setDebug(char const *) {}
void ESP32sim_parseArg(char **&a, char **endA) {}
void ESP32sim_setup() {} // called before arduino setup()
void ESP32sim_loop() {}  // called before each arduino loop()
void ESP32sim_done() {}  // called before exit();
void ESP32sim_JDisplay_forceUpdate() {}
#endif

*/

