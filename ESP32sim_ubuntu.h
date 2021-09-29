/* Simple library and simulation environment to compile and run an Arduino sketch as a 
 * standard C command line program. 
 * 
 * Most functionality is unimplemented, and just stubbed out.  Minimal simluated 
 * Serial/UDP/Interrupts/buttons are sketched in. 
 * 
 * Currently replaces the following block of Arduino include files:
 * 
 *
 */

#include <cstdint>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>
#include <string>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
#include <functional>

using namespace std;
#ifndef GIT_VERSION
#define GIT_VERSION "no-git-version"
#endif

typedef char byte;
static uint64_t _micros = 0;
static uint64_t _microsMax = 0xffffffff;
uint64_t micros() { return _microsMax > 0 ? ++_micros & _microsMax : ++_micros; }
uint64_t millis() { return ++_micros / 1000; }

// Stub out FreeRTOS stuff 
typedef int SemaphoreHandle_t;
int xSemaphoreCreateCounting(int, int) { return 0; } 
int xSemaphoreGive(int) { return 0; } 
int xSemaphoreTake(int, int) { return 0; }
int uxSemaphoreGetCount(int) { return 0; } 
#define portMAX_DELAY 0 
#define tskIDLE_PRIORITY 0
#define pdMS_TO_TICKS(x) (x) 	
void xTaskCreate(void (*)(void *), const char *, int, void *, int, void *) {}
#define WRITE_PERI_REG(a, b) if(0) {}
#define RTC_CNTL_BROWN_OUT_REG 0

typedef int esp_err_t; 
void esp_task_wdt_init(int, int) {}
void esp_task_wdt_reset() {}
esp_err_t esp_task_wdt_add(void *) { return 0; }
esp_err_t esp_task_wdt_delete(const void *) { return 0; }
int rtc_get_reset_reason(int) { return 0; } 



namespace fs { 
class File {
	public: 
	bool operator!() { return false; } 
	operator bool() { return false; } 
	File openNextFile(void) { return *this; }
	void close() {}
        int print(const char *) { return 0; }
	int printf(const char *, ...) { return 0; } 
	int write(const char *, int) { return 0; } 
	int flush() { return 0; }	
	int read(uint8_t *, int) { return 0; } 
};
};
using fs::File;

struct FakeSPIFFS {
	void begin() {}
	void format() {}
	File open(const char *, const char *) { return File(); } 
} SPIFFS;

struct FakeArduinoOTA {
	void begin() {}
	void handle() {}
	int getCommand() { return 0; } 
	void onEnd(function<void(void)>) {}
	void onStart(function<void(void)>) {}
	void onError(function<void(int)>) {}
	void onProgress(function<void(int, int)>) {}
} ArduinoOTA;

typedef int ota_error_t; 
#define OTA_AUTH_ERROR 0 
#define OTA_BEGIN_ERROR 0 
#define OTA_CONNECT_ERROR 0 
#define OTA_RECEIVE_ERROR 0 
#define OTA_END_ERROR 0 

struct FakeESP {
	int getFreeHeap() { return 0; }
	void restart() {}
	int getChipId() { return 0xdeadbeef; }
} ESP;

struct WiFiManager {
};

struct OneWireNg {
	OneWireNg(int, int) {}
	typedef int ErrorCode; 
	typedef int Id[8];
	static const int EC_MORE = 0, EC_DONE = 0;
	void writeByte(int) {}
	void addressSingle(Id) {}
	void touchBytes(const unsigned char *, int) {}
	void searchReset();
	static int crc8(const unsigned char *, int) { return 0; }
	int search(Id) { return 0; }
};
typedef OneWireNg OneWireNg_CurrentPlatform;

class ButtonManager {
	struct PressInfo { int pin; float start; float duration; };
	vector<PressInfo> presses;
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
	return bm.check(p, micros()/1000000.0);
}

static int ESP32sim_currentPwm[16];
void ledcWrite(int chan, int val) {
		ESP32sim_currentPwm[chan] = val;
} 

// Takes an input of a text file with line-delimited usec intervals between 
// interrupts, delivers an interrupt to the sketch-provided ISR
// TODO: only handles one ISR and one interrupt source 
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

void pinMode(int, int) {}
void digitalWrite(int, int) {};
int digitalPinToInterrupt(int) { return 0; }
void attachInterrupt(int, void (*i)(), int) { intMan.intFunc = i; } 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void delayMicroseconds(int m) { _micros += m; intMan.run(); }
void delay(int m) { delayMicroseconds(m*1000); }
void yield() { intMan.run(); }
//void analogSetCycles(int) {}
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
	string st;
	String(const char *s) : st(s) {}
	String(string s) : st(s) {}
	String(const char *b, int l) { 
		st = string();
		for (int n = 0; n < l; n++) {
			st.push_back(b[n]);
		}
	} 
	String(int s) : st(to_string(s)) {}
	String() {}
	int length() const { return st.length(); } 
	bool operator!=(const String& x) { return st != x.st; } 
	String &operator+(const String& x) { st = st + x.st; return *this; } 
	const char *c_str(void) const { return st.c_str(); }
	operator const char *() { return c_str(); } 
};

String operator +(const char *a, String b) { 
	return String(a) + b;
}

class IPAddress {
public:
	void fromString(const char *) {}
	String toString() const { return String(); }	
    int operator [](int) { return 0; }  
};

class FakeSerial { 
	public:
	String inputLine;
	bool toConsole = false;
	void begin(int a = 0, int b = 0, int c = 0, int d = 0) {}
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
	int availableForWrite() { return 1; } 
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
	int begin(const char *, const char *) { return 0; }
	int status() { return WL_CONNECTED; } 
	IPAddress localIP() { return IPAddress(); } 
	void setSleep(bool) {}
	void mode(int) {}
	void disconnect(bool) {}
	int waitForConnectResult() { return 0; }
} WiFi;

class FakeSD {
	public:
	bool begin(int, int, int, int) { return true; }
	fs::File open(const char *) { return fs::File(); } 
} SD;

class WiFiMulti {
public:
	void addAP(const char *, const char *) {}
	void run() {}
};

class WiFiUDP {
	int port, txPort;
	bool toSerial = false;
public:
	void begin(int p) { port = p; }
	int beginPacket(IPAddress, int p) { return beginPacket(NULL, p); }
	int beginPacket(const char *, int p) { txPort = p; return 1; }
	void write(const uint8_t *b, int len) {
		float f;
		if (txPort == 7892 && sscanf((const char *)b, "trim %f", &f) == 1) {
			//ESP32sim_pitchCmd = f;
		}
	}
	int endPacket() { return 1; }

	typedef vector<unsigned char> InputData;
	typedef map<int, InputData> InputMap;
	static InputMap inputMap;
	int  parsePacket() { 
		if (inputMap.find(port) != inputMap.end()) { 
			return inputMap.find(port)->second.size();
		} else { 
			return 0;
		}
	}
	int read(uint8_t *b, int l) {
		if (inputMap.find(port) != inputMap.end()) { 
			InputData in = inputMap.find(port)->second;
			int rval = min((int)in.size(), l);
			for (int n = 0; n < rval; n++) { 
				b[n] = in.at(n);
			}
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

// TODO: extend this to use vector<unsigned char> to handle binary data	
WiFiUDP::InputMap WiFiUDP::inputMap;

void ESP32sim_udpInput(int p, const WiFiUDP::InputData &s) { 
	WiFiUDP::InputMap &m = WiFiUDP::inputMap;
	if (m.find(p) == m.end())
		m[p] = s;
	else
		m[p].insert(m[p].end(), s.begin(), s.end());
}

void ESP32sim_udpInput(int p, const string &s) {
	ESP32sim_udpInput(p, WiFiUDP::InputData(s.begin(), s.end()));
} 

struct NTPClient {
	NTPClient(WiFiUDP &a) {}
	void update() {}
	void begin() {}
	long getEpochTime() { return 50000000 + millis() / 1000.0; }
	int getMinutes() { return millis() % (60 * 60 * 1000) / (60 * 1000); }
	int getHours() { return millis() % (60 * 60 * 24 * 1000) / (60 * 60 * 1000); }
	int getSeconds() { return millis() % (60 * 1000) / (1000); }
	void setUpdateInterval(int) {}
	String getFormattedTime() { return String("TIMESTRING"); }

};

class FakeWire {
public:
	void begin(int, int) {}
	void beginTransmission(int) {}
	bool endTransmission() { return false; }
} Wire;

typedef enum {
    ESP_NOW_SEND_SUCCESS = 0,       /**< Send ESPNOW data successfully */
    ESP_NOW_SEND_FAIL,              /**< Send ESPNOW data fail */
} esp_now_send_status_t;
typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr, const uint8_t *data, int data_len);
typedef void (*esp_now_send_cb_t)(const uint8_t *mac_addr, esp_now_send_status_t status);
void esp_now_init() {}
void esp_now_register_send_cb(esp_now_send_cb_t) {}
void esp_now_register_recv_cb(esp_now_recv_cb_t) {}
int esp_now_send(const uint8_t*, const uint8_t*, size_t) { return 0; }

#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 1
#define INV_XYZ_COMPASS 0
const int ACC_FULL_SCALE_4_G = 0, GYRO_FULL_SCALE_250_DPS = 0, MAG_MODE_CONTINUOUS_100HZ = 0;

class MPU9250_DMP {
public:
	int address = 0;
	int begin(){ return 1; }
	void setWire(FakeWire *) {}
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

#define UPLOAD_FILE_START 0 
#define UPLOAD_FILE_END 0 
#define UPLOAD_FILE_WRITE 0 
#define UPDATE_SIZE_UNKNOWN 0

struct {
	int hasError() { return 0; } 
	int write(const char *, int) { return 0; }
	int begin(int) { return 0; } 
	void printError(FakeSerial &) {}
	int end(int) { return 0; }
} Update;

class HTTPUpload {
public:
	int status, currentSize, totalSize;
	String filename;
	const char *buf;
};

#define HTTP_GET 0 
#define HTTP_POST 0
#define U_FLASH 0  

class WebServer {
	HTTPUpload u;
	public:
	WebServer(int) {}
	void begin() {}
	void on(const char *, int, function<void(void)>, function<void(void)>) {}
	void on(const char *, int, function<void(void)>) {}
	void sendHeader(const char *, const char *) {}
	void send(int, const char *, const char *) {}
	HTTPUpload &upload() { return u; }
	void handleClient() {}
};

class FakeCAN {
public:
  FakeCAN() {}
  int begin(long baudRate) { return 1; }
  void end() {}
  int endPacket() { return 0; }
  int parsePacket() { return 0; }
  int packetId() { return 0; }
  int read() { return 0; }
  int packetRtr()  { return 0; }
  void onReceive(void(*callback)(int)) {}
  int filter(int id, int mask) { return 0; }
  int filterExtended(long id, long mask) { return 0; }
  int setPins(int, int) { return 0; }
  int write(int) { return 0; } 
  int beginExtendedPacket(int) { return 0; } 
} CAN;

struct RTC_DS3231 {
};

#define COM_TYPE_UBX 0
class SFE_UBLOX_GPS {
public:
	double lat, lon;
	float hdg, hac, gs, siv, alt;
	bool fresh = false; 
	bool begin(FakeSerial &) { return true; } 
	bool setUART1Output(int) { return true; } 
	float getHeading() { return hdg; }
	float getHeadingAccEst() { return hac; }
	double getLatitude() { return lat; }
	double getLongitude() { return lon; }
	float getAltitudeMSL() { return alt; }
	float getGroundSpeed() { return gs; }
	float getSIV() { return siv; }
	bool getPVT(int) { 
		bool rval = fresh;
		fresh = false;
		return rval;
	}	
	bool setSerialRate(int) { return 0; }
	bool setAutoPVT(int, int) { return 0; }
	void saveConfiguration() {}
	bool setNavigationFrequency(int) { return 0; } 
};


void setup(void);
void loop(void);

class ESP32sim_Module {
public:
	ESP32sim_Module();
	virtual void parseArg( char **&, char **) {}
	virtual void setup() {};
	virtual void loop() {};
	virtual void done() {};
};

class ESP32sim {
public:
	vector<ESP32sim_Module *> modules;
	struct TimerInfo { 
		uint64_t last;
		uint64_t period;
		function<void(void)> func;
	};
	typedef vector<TimerInfo> timers;
	void main(int argc, char **argv) {
		float seconds = 0;
		for(char **a = argv + 1; a < argv+argc; a++) {
			if (strcmp(*a, "--serial") == 0) Serial.toConsole = true;
			else if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%f", &seconds); 
			else if (strcmp(*a, "--interruptFile") == 0) { 
				intMan.setInterruptFile(*(++a));
			}
			else for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) {
				(*it)->parseArg(a, argv + argc);
			}
		}
		
		for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
			(*it)->setup();
		setup();

		uint64_t lastMillis = 0;
		while(seconds <= 0 || _micros / 1000000.0 < seconds) {
			uint64_t now = millis();
			for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
				(*it)->loop();
			loop();
			intMan.run();

			//if (floor(now / 1000) != floor(lastMillis / 1000)) { 
			//	ESP32sim_JDisplay_forceUpdate();	
			//}
			lastMillis = now;
		}
	}
	void exit() { 
		for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
			(*it)->done();	
		::exit(0);
	}
} esp32sim;

void ESP32sim_exit() { 	esp32sim.exit(); }

inline 	ESP32sim_Module::ESP32sim_Module() { 
	esp32sim.modules.push_back(this);
}

int main(int argc, char **argv) {
	esp32sim.main(argc, argv);
}

