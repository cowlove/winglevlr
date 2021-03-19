#ifndef INC_JIMLIB_H
#define INC_JIMLIB_H
#include <functional>
#include <string>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#ifndef UBUNTU
#endif

#include <stdarg.h>
std::string strfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
	return std::string(buf);
}



void printPins() { 
        for (int n = 0; n <= 1; n++) {
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        for (int n = 2; n <= 5; n++) {
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        for (int n = 12; n <= 39; n++) { 
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        Serial.print("\n");
}



class FakeMutex {
	public:
	void lock() {}
	void unlock() {}
};

class FakeSemaphore { 
 public:
	FakeSemaphore(int max = 1, int init = 1) {}
	bool take(int delay = 0) { return true; } 
	void give() {}
};

#if !defined UBUNTU && defined ESP32 
#include <SPIFFS.h>
#include <esp_task_wdt.h>


class Mutex {
	SemaphoreHandle_t xSem;
 public:
	Mutex() { 
		xSem = xSemaphoreCreateCounting(1,1);
		unlock();
	}
	void lock() { xSemaphoreTake(xSem, portMAX_DELAY); } 
	void unlock() { xSemaphoreGive(xSem); }
};

class Semaphore { 
	SemaphoreHandle_t xSem;
 public:
	Semaphore(int max = 1, int init = 1) { 
		xSem = xSemaphoreCreateCounting(max, init);
	}
	bool take(int delay = portMAX_DELAY) { return xSemaphoreTake(xSem, delay); } 
	void give() { xSemaphoreGive(xSem); }
};
#else
typedef FakeMutex Mutex;
typedef FakeSemaphore Semaphore; 
#endif

class ScopedMutex {
	Mutex *mutex; 
  public:
	ScopedMutex(Mutex &m) : mutex(&m) { mutex->lock(); } 
#if !defined UBUNTU && defined ESP32 
	ScopedMutex(FakeMutex &m) : mutex(NULL) {} 
#endif
	~ScopedMutex() { if (mutex != NULL) mutex->unlock(); } 
};



class LineBuffer {
public:
	char line[1024];
	char len;
	int add(char c) {
		int r = 0; 
		if (c != '\r' && c != '\n');
			line[len++] = c;
		if (len >= sizeof(line) || c == '\n') {
			r = len;
				line[len] = '\0';
			len = 0;
		}
		return r;
	}
};

class EggTimer {
	uint64_t last;
	uint64_t interval; 
	bool first = true;
public:
	EggTimer(float ms) : interval(ms * 1000), last(0) { reset(); }
	bool tick() { 
		uint64_t now = micros();
		if (now - last >= interval) { 
			last += interval;
			// reset last to now if more than one full interval behind 
			if (now - last >= interval) 
				last = now;
			return true;
		} 
		return false;
	}
	void reset() { 
		last = micros();
	}
	void alarmNow() { 
		last = 0;
	}
};


class DigitalDebounce {
	EggTimer timer;
	bool recentlyPressed;
	long startPress;
	int lastDuration;
	int lastVal;
public:
	int duration;
	int count;
	DigitalDebounce(int ms = 50) : timer(ms), recentlyPressed(false),count(0) {}
	bool checkOneshot(bool button) {
		bool rval = false; 
		if (button == true) {
			if (timer.tick() && lastVal == false) 
				recentlyPressed = false;
			rval = !recentlyPressed;
			if (rval) {
				count++;
				startPress = millis();
			}
			recentlyPressed = true;
			timer.reset();
			duration = max(1UL, millis() - startPress);
		} else {
			duration = 0;
			if (timer.tick()) 
				recentlyPressed = false;
		}
		lastVal = button;
		return rval;
	}
	int checkEndPress() {
		int rval = 0; 
		if (duration == 0 && lastDuration > 0) 
			rval = lastDuration;
		lastDuration = duration;
		return rval;
	}
};

#ifdef ESP32
class RotaryEncoder {
public:
	int pin1, pin3;
	DigitalDebounce a,b;
	int limMin, limMax;
	int value;
	bool wrap;
	int count = 0;
	unsigned long lastChange;

#ifndef UBUNTU
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	void IRAM_ATTR ISR() {	
		portENTER_CRITICAL_ISR(&(this->mux));
		check();
		portEXIT_CRITICAL_ISR(&(this->mux));
	}
	void begin(void (*ISR_callback)(void)) {
		attachInterrupt(digitalPinToInterrupt(pin1), ISR_callback, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pin3), ISR_callback, CHANGE);
	}
#else
	void begin(void *) {};
#endif
	
	RotaryEncoder(int p1, int p3, int debounce = 5) : a(debounce), b(debounce), pin1(p1), pin3(p3){
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin3, INPUT_PULLUP);
		limMin = 001;
		limMax = 360;
		value = limMin;
		wrap = true;
		lastChange = 0;
	}
	void setLimits(int mn, int mx, bool wrap) {
		limMin = mn;
		limMax = mx;
		this->wrap = wrap;
	}
	void check() {
		unsigned long now = millis();
		//if (now - lastChange > 500)
		//	Serial.printf("\n\n");
		count++;
		int buta = !digitalRead(pin1);
		int butb = !digitalRead(pin3);
		int oa = a.checkOneshot(buta);
		int ob = b.checkOneshot(butb);
		int delta = 0;
		if (oa && !butb) 
			delta = -1;
		if (ob && !buta)
			delta = +1;
		//Serial.printf("%d/%d %d/%d %d\n", buta, oa, butb, ob, delta );
		
		if (delta != 0) {
			if(lastChange > 0 && now - lastChange < 20)
				delta *= 5;
			lastChange = now;
		}
		value += delta;
		if (value < limMin) value = wrap ? limMax : limMin;
		if (value > limMax) value = wrap ? limMin : limMax;
	}
};
#endif 

class DigitalButton { 
	DigitalDebounce deb;
	bool inverted;
public:
	int pin;
	int count;
	DigitalButton(int p, bool invert = true, int mode = INPUT_PULLUP, int debounceMs = 5) : pin(p), inverted(invert), deb(debounceMs) {
		pinMode(pin, mode);
	}
	bool check() {
		bool in = digitalRead(pin);
		if (inverted) in = !in;
		bool rval = deb.checkOneshot(in);
		if (rval) count++;
		return rval;
	}
	int duration() {
		check();
		return deb.duration;
	}
	int checkEndPress() {
		check();
		return deb.checkEndPress();
	}
};

template <class T>
class Changed {
	T old;
	bool first, cas;
public:
	Changed(int changeAtStart = false) : first(true), cas(changeAtStart) {}
	bool changed(T n) { 
		bool r = first ? cas : (n != old);
		old = n;
		first = false;
		return r;
	}
};

class LongShortFilter { 	
	int longPressMs, resetMs;
	unsigned long lastEndTime;
public:
	bool wasLong;
	int last, wasDuration;
	int count, wasCount;
	int lastDuration;
	int events;
	bool countedLongPress;
	bool finishingLong = false;
	LongShortFilter(int longMs, int resetTimeMs) : longPressMs(longMs), resetMs(resetTimeMs) {
		last = events = lastDuration = count = wasCount = lastEndTime = 0;
		countedLongPress = false;
	}
	
	bool check(int l) { 
		unsigned long now = millis();
		last = l;
		int rval = false;
		if (last == 0) { 
			if (lastDuration > 0) {  // press just ended, button up 
				count++;
				lastEndTime = now;
				wasDuration = lastDuration;
				lastDuration = 0;
			}
			if (lastEndTime > 0 && now - lastEndTime >= resetMs) { // resetMS after last button press 
				if (!countedLongPress) {
					wasCount = count;
					rval = true;
				}
				lastEndTime = count = 0;
				countedLongPress = false;
			} 
		} else { 
			if (last >= longPressMs && !countedLongPress) { // long press just finished, button still down
				countedLongPress = true;
				lastDuration = wasDuration = last;
				wasCount = ++count;
				rval = true;
			} else {
				lastDuration = last;
			}
		}
		if (rval == true) {
			events++;
			wasLong = wasDuration >= longPressMs;
		}
		return false;
	}
	bool inProgress() { return ((last != 0) || (lastEndTime > 0 && millis() - lastEndTime < resetMs)) && (countedLongPress == false); }
	int inProgressCount() { return count + ((last != 0) ? 1 : 0); }
	Changed<int> eventCount;
	bool newEvent() { 
		return eventCount.changed(events);
	}
};

class DigitalButtonLongShort { 
	public:
	LongShortFilter filter;
	DigitalButton button;
	DigitalButtonLongShort(int p, int l = 1000, int d = 250) : filter(l, d), button(p) {}
	bool newEvent() { return filter.newEvent(); } 
	void run() { 
		filter.check(button.duration());
	}
	bool newEventR() { run(); return newEvent(); }
	int count() { return filter.wasCount; } 
	bool inProgress() { return filter.inProgress(); } 
	int inProgressCount() { return filter.count; } 
	int wasLong() { return filter.wasLong; } 
};

template<class T> 
class StaleData {
	uint64_t timeout, lastUpdate, lastChange;
	T value, invalidValue;
	bool chg = false; 
public:
	StaleData(int t, T i) : lastUpdate(0), timeout(t), invalidValue(i) {} 
	bool isValid() { return millis() - lastUpdate < timeout; }
	operator T&() { return getValue(); }
	StaleData<T>& operator =(const T&v) {
		chg = value != v;
		value = v;
		lastUpdate = millis();
		if (chg) 
			lastChange = millis();
		return *this;
	}
	T& getValue() { return isValid() ? value : invalidValue; }
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval && isValid(); 
	}
	uint64_t age() { 
		return millis() - lastUpdate;
	}
	uint64_t timeSinceChange() { 
		return millis() - lastChange;
	}
};

template<class T> 
class ChangedData {
	T value;
	bool chg = false;
	bool first = true;
public:
	ChangedData(T i) {} 
	operator T&() { return value; }
	ChangedData<T>& operator =(const T&v) {
		chg = value != v || first;
		value = v;
		first = false;
		return *this;
	}
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval; }
};

#ifdef ESP32
template <typename Out>
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

// mutex to serialize SD card and TFT writes. 
Mutex mutexSPI;

void open_TTGOTS_SD() { 
	for (int retries = 0; retries < 2; retries++) { 	
		ScopedMutex lock(mutexSPI);
		Serial.print("Initializing SD card...");
		if (SD.begin(13, 15, 2, 14)) { 
			Serial.println("initialization done.");
			return;
		}
		Serial.println("initialization failed!");
		delay(100);
	}
	Serial.println("giving up");
}

#ifndef UBUNTU
void printDirectory(msdFile dir, int numTabs) {
  while(true) {
     msdFile entry =  dir.openNextFile();
     if (! entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');   // we'll have a nice indentation
     }
     // Print the name
     Serial.print(entry.name());
     /* Recurse for directories, otherwise print the file size */
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       /* files have sizes, directories do not */
       Serial.print("\t\t");
       Serial.println(entry.size());
     }
     entry.close();
	}
}

void printSD() { 
	msdFile root = SD.open("/");
	if (root) {
		printDirectory(root, 0);
		root.close();
	}
}
#else
void printSD() {}
#endif
#endif


#ifdef UBUNTU
#include <fcntl.h>
#include <unistd.h>
#endif

#ifdef ESP32
template <class T>
void SDCardBufferedLogThread(void *p);


template<class T> 
class SPIFFSVariable { 
	String filename;
	const T def;
public:
	SPIFFSVariable(const char *f, const T &d) : filename(f), def(d) {}
	operator const T() {
		T val = def;
		File file = SPIFFS.open(filename.c_str(), "r");
		if (file) { 
			uint8_t buf[64];
			int b = file.read(buf, sizeof(buf) - 1);
			Serial.printf("read %d bytes\n");
			file.close();
			if (b > 0) {
				buf[b] = 0; 
				sscanf((char *)buf, "%d", &val);
			}
		}
		return val;
	} 
	SPIFFSVariable & operator=(const T&v) { 
		File file = SPIFFS.open(filename.c_str(), "w");
		if (file) { 
			file.printf("%d\n", v);
			file.close();
		} else { 
			Serial.printf("error writing file %s\n", filename.c_str());
			SPIFFS.format();
		}
		return *this;
	}
};



static bool logFileBusySPI = false;

template <class T>
class SDCardBufferedLog {
public:
	int dropped = 0;
	int maxWaiting = 0;
	int written = 0;
#ifndef UBUNTU
	QueueHandle_t queue;
	int timo;
	int flushInterval;
	bool exitNow = false;
	const char *filename;

	void exit() { 
		exitNow = true;
		Serial.printf("Exiting, qsize %d, free heap %d\n", (int)uxQueueMessagesWaiting(queue), ESP.getFreeHeap());
		T v;
		add(&v);
		while(exitNow) {
			yield();
			delay(1);
		};
	}
	bool textMode;
public:
	String currentFile;
	StaticQueue_t qb;
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true) : 
		filename(fname), 
		flushInterval(flushInt), 
		timo((timeout+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS) {
			
		queue = xQueueCreate(len, sizeof(T));
		this->textMode = textMode;
		xTaskCreate(SDCardBufferedLogThread<T>, "SDCardBufferedLogThread", 8192, (void *)this, tskIDLE_PRIORITY, NULL );   
	}
	~SDCardBufferedLog() { 
		 this->exit();
		 vQueueDelete(queue);
		 printSD();
		 SD.end();
		 logFileBusySPI = false;
	}
	void add(const T *v) {
		add(v, timo);
	}
	void add(const T &v) {
		add(&v, timo);
	}
	void add(const T *v, int t) {
		if (xQueueSend(queue, v, (t+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS) != pdTRUE) 
			dropped++;
		int waiting = uxQueueMessagesWaiting(queue);
		logFileBusySPI = waiting > 20;
		if (waiting > maxWaiting)
			maxWaiting = waiting;
			
	}
	int queueLen() {
		return uxQueueMessagesWaiting(queue);
	}
	void setFilename() { 
		int fileVer = 1;
		char fname[20];
		msdFile f;
		if (strchr(filename, '%')) { // if filename contains '%', scan for existing files  
			for(fileVer = 1; fileVer < 999; fileVer++) {
				snprintf(fname, sizeof(fname), filename, fileVer);
				ScopedMutex lock(mutexSPI);
				if (!(f = SD.open(fname, (F_READ)))) {
					f.close();
					break;
				}
			} 
		}
		sprintf(fname, filename, fileVer);
		currentFile = String(fname);
	}

	void *thread() {
		uint64_t lastWrite, startTime, lastFlush;
		lastFlush = lastWrite = startTime = millis();
		msdFile f;

		open_TTGOTS_SD();
		setFilename();
		{
			ScopedMutex lock(mutexSPI);
			SD.remove((char *)currentFile.c_str());
			f = SD.open((char *)currentFile.c_str(), (F_READ | F_WRITE | F_CREAT));
		}
		Serial.printf("Opened %s\n", currentFile.c_str());
		while(!exitNow) { // TODO: doesn't flush queue before exiting 	
			T v;
			if (xQueueReceive(queue, &v, timo) == pdTRUE) {
				if (!exitNow) {
					uint64_t now = millis();
					if (f) {
						ScopedMutex lock(mutexSPI);
						if (textMode)  
							f.println(v.toString());
						else 
							f.write((uint8_t *)&v, sizeof(v));
						written++;
					}
					lastWrite = now;
					//Serial.printf("WROTE\n");
				}
			}
			uint64_t now = millis();
			if (now - lastFlush >= flushInterval) {
				if (f) {
					ScopedMutex lock(mutexSPI);
					f.flush();
				}
				lastFlush = now;
			}
			int waiting = uxQueueMessagesWaiting(queue);
			logFileBusySPI = waiting > 20;
		}
		if (f) {
			ScopedMutex lock(mutexSPI);
			f.close();
		}
		exitNow = false;
		Serial.printf("task out\n");
		vTaskDelete(NULL);
	}
	void flush() { 
		while(uxQueueMessagesWaiting(queue) > 0) 
			sleep(1);
	}
#else
public:
	String currentFile;
	int fd;
	void add(const T *v, int timo = 0) { 
		int s = write(fd, (void *)v, sizeof(T));
		printf("%s LOG\n", v->toString().c_str()); 
	}
	void add(const T &v, int timo = 0) { 
		add(&v);
	}
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true) {
		currentFile = fname;
		char buf[64];
		snprintf(buf, sizeof(buf), fname, 1);
		fd = open(buf, O_WRONLY | O_CREAT, 0666);
	}
	int queueLen() { return 0; }
#endif
};	

template <class T>
void SDCardBufferedLogThread(void *p) {
	((SDCardBufferedLog<T> *)p)->thread(); 
}

class ChangeTimer { 
	public:
	float lastVal = 0;
	uint64_t lastChangeMillis;
	ChangeTimer() : lastChangeMillis(0) {}
	float unchanged(float v) { 
		if (v == lastVal) { 
			return (millis() - lastChangeMillis) / 1000.0;
		} else { 
			lastVal = v;
			lastChangeMillis = millis();
			return 0.0;
		}
	}
};

// JDisplay jd;
// JDisplayItem<float> f1(&jd,10,10,"LABEL1:", "%+02.2f");
// JDisplayItem<int> f2(&jd,10,20,"LABEL2:", "%02d");

// jd.begin();
// f2 = 0;
// f1 = 1.1;

#ifndef UBUNTU
#include <Adafruit_GFX.h>               // Core graphics library
#include <Adafruit_ST7735.h>            // Hardware-specific library

#define TFT_CS 16
#define TFT_RST 9  
#define TFT_DC 17
#define TFT_SCLK 5   
#define TFT_MOSI 23  
#define ST7735
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#endif

void JDisplayUpdateThread(void *);
class JDisplayItemBase;
class JDisplay {
	std::vector<JDisplayItemBase *> items;
	Semaphore changeSem;
public:
	int textSize, xoffset, yoffset;
	bool autoupdate;
	JDisplay(int tsize = 1, int x = 0, int y = 0, bool au = false) : textSize(tsize), xoffset(x), yoffset(y), autoupdate(au) {}
	static const struct Colors { 
		int lf, lb, vf, vb; 
	} defaultColors; 
#ifndef UBUNTU 
	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
	void begin() { 
		{
			ScopedMutex lock(mutexSPI);
			pinMode(27,OUTPUT); 		//Backlight:27  TODO:JIM 27 appears to be an external pin 
			digitalWrite(27,HIGH);		//New version added to backlight control
			tft.initR(INITR_18GREENTAB);                             // 1.44 v2.1
			tft.fillScreen(ST7735_BLACK);                            // CLEAR
			tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);           // GREEN
			tft.setRotation(1);
			tft.setTextSize(textSize); 
		}
		forceUpdate();
		xTaskCreate(JDisplayUpdateThread, "JDisplayUpdateThread", 8192, this, tskIDLE_PRIORITY, NULL);
	}
	void clear() {
		ScopedMutex lock(mutexSPI);
		tft.fillScreen(ST7735_BLACK);                            // CLEAR
	}
	void printAt(int x, int y, const char *s, int fg, int bg, int size) { 
		ScopedMutex lock(mutexSPI);
		x = x + xoffset;
		y = y + yoffset;
		tft.setTextColor(fg, bg); 
		tft.setCursor(x, y);
		tft.setTextSize(size); 
		tft.print(s);
	}
#else
	static const int xsize = 40, ysize = 20;
	char lines[ysize][xsize];
	void begin() {
		for(int y = 0; y < ysize; y++)  {
			for(int x = 0; x < xsize; x++) { 
				lines[y][x] = ' ';
			}
			lines[y][xsize - 1] = '\0';
		}
	}
	void clear() {}
	static bool displayToConsole;
	void printAt(int x, int y, const char *f, int, int, int) {
		if (displayToConsole) {
			char *line = lines[y/10];
			int flen = strlen(f);
			for (int n = 0; n < flen; n++) { 
				line[x/6 + n] = f[n];
			}
			::printf("%s\n", line);
		}
	}
#endif
	void markChange() { changeSem.give(); } 
	void waitChange(int tmo = portMAX_DELAY) { changeSem.take(tmo); }

	void addItem(JDisplayItemBase *i) { 
		items.push_back(i);
	}
	inline void forceUpdate();	
	inline bool update(bool update, bool onlyOne);	
};


#ifndef UBUNTU
const JDisplay::Colors JDisplay::defaultColors = { ST7735_GREEN, ST7735_BLACK, ST7735_WHITE, ST7735_BLACK };
#else
const JDisplay::Colors JDisplay::defaultColors = { 0, 0, 0, 0 };
bool JDisplay::displayToConsole = false;
static void JDisplayToConsole(bool b) { JDisplay::displayToConsole = b; } 
#endif


class JDisplayItemBase {
	int x, y, updates;
	int lb, lf, vb, vf;
	JDisplay *d;
	String val, lastVal;
	bool labelInverse = false, valueInverse = false;
	bool first = true;
	int labelFontSize = 0;
public:
	bool changed = false;
	int labelSpace = 0;
	std::string label;
	JDisplay::Colors color;
	JDisplayItemBase(JDisplay *d, int x, int y, const char *label, JDisplay::Colors colors, int labelFontSize) {
		this->labelFontSize = labelFontSize;
		this->x = x;
		this->y = y;
		this->label = label;
		this->d = d;
		this->color = colors;
		updates = 0;
		if (d != NULL)
			d->addItem(this);
	}
	void setInverse(bool li, bool vi) {
		if (li != labelInverse || vi != valueInverse) {
			labelInverse = li;
			valueInverse = vi;
			changed = true;
			if (d != NULL) 
				d->markChange();
		}
		//update(changed);
	}
	bool update(bool force) {
		if (d == NULL)
			return false;
		if (first || ++updates % 1000 == 1) 
			force = true;
		if (changed) { 
			force = true;
			changed = false; 
		}
		first = false;
#ifdef UBUNTU
		labelFontSize = d->textSize = 1;
#endif
		if (force)
			d->printAt(x * d->textSize, y * d->textSize, label.c_str(), labelInverse ? color.lb : color.lf , labelInverse ? color.lf : color.lb, 
			labelFontSize != 0 ? labelFontSize : d->textSize);
		if (force || val != lastVal) {
			int xpixel = labelSpace + x * d->textSize + 6 * strlen(label.c_str()) * (labelFontSize != 0 ? labelFontSize : d->textSize);
			d->printAt(xpixel, y * d->textSize, val.c_str(), valueInverse ? color.vb : color.vf, valueInverse ? color.vf : color.vb, d->textSize);
			lastVal = val;
			return true;
		}
		return false;
	}
	void setValueString(const String &s) {
		val = s;
		if (lastVal != val && d != NULL) {
			d->markChange();
			if (d->autoupdate)
				update(true);
		}
	}
};

inline bool JDisplay::update(bool force, bool onlyOne) { 
	for (std::vector<JDisplayItemBase *>::iterator it = items.begin(); it != items.end(); it++) { 
		if ((*it)->update(force)) {
			if (onlyOne) 
				return true;
		}
	}
	return false;
}

inline void JDisplay::forceUpdate() { 
	update(true, false);
}

template<class T>
class JDisplayItem : public JDisplayItemBase { 
	const char *fmt;
public:
	T value;
	JDisplayItem(JDisplay *d, int x, int y, const char *label, const char *format, int labelFontSize = 0,  JDisplay::Colors colors = JDisplay::defaultColors) :
		JDisplayItemBase(d, x, y, label, colors, labelFontSize), fmt(format) {}
	JDisplayItem<T>& operator =(const T&v) { setValue(v); return *this; }

	std::function<String(const T&)> toString = [this](const T& v)->String { 
		char buf[64];
		sprintf(buf, this->fmt, v);
		return String(buf);
	};		
	void setValue(const T&v) { 
		value = v;
		setValueString(toString(v));
	}
};


class JDisplayEditableItem;

class JDisplayEditor {
	std::vector<JDisplayEditableItem *> items;
	bool editing;
	int selectedItem;
public:
	RotaryEncoder re;
	JDisplayEditor(int p1, int p2) : re(p1, p2) {}
	void add(JDisplayEditableItem *i) { 
		items.push_back(i);
	}
	void begin() {
		re.limMin = 0;
		re.limMax = items.size() - 1;
		editing = false;
		re.wrap = false;
		re.value = 0;
		selectedItem = 0;
		//re.begin( [this]()->void{ this->re.ISR(); });
	}
	inline void negateSelectedValue(); 
	inline void update(); 
	inline void buttonPress(bool longpress);			
		
};

class JDisplayEditableItem { 
protected:
	JDisplayItem<float> *di; 
public:
	bool recentChange = false;
	bool changed() { 
		if (recentChange) {
			recentChange = false;
			return true;
		}
		return false;
	}
	float value, newValue, increment;
	float min, max;
	enum { UNSELECTED, SELECTED, EDITING } state;
	JDisplayEditableItem(JDisplayItem<float> *i, float inc, float mi = -100000, float ma = +10000) : 
		di(i), increment(inc), min(mi), max(ma) {}
	void update() {
		if (di == NULL)
			return;
		if (state == EDITING) {
			di->setValue(newValue);
			di->setInverse(false, true);
		} else { 
			di->setValue(value);
			di->setInverse(state == SELECTED, false);
		}
		//di->update(false);
	};
	void setValue(float v) { 
		value = v;
		if (di != NULL) 
			di->setValue(v);
		update();
	}
};

inline void JDisplayEditor::negateSelectedValue() { 
	if (!editing) { 
		items[selectedItem]->value *= -1.0;
	} else {
		items[selectedItem]->newValue *= -1.0;
	}
	items[selectedItem]->update();
}

inline void JDisplayEditor::update() { 
	if (!editing) { 
		if (selectedItem != re.value) { 
			selectedItem = re.value;
			for(int n = 0; n < items.size(); n++) {
				items[n]->state = (n == selectedItem) ? JDisplayEditableItem::SELECTED : 
					JDisplayEditableItem::UNSELECTED;
				items[n]->update();
			}
		}
	} else { 
		items[selectedItem]->newValue = items[selectedItem]->value + re.value * items[selectedItem]->increment;
		items[selectedItem]->update();
	}
	//for(int n = 0; n < items.size(); n++) 
	//	items[n]->update();
}


			
inline void JDisplayEditor::buttonPress(bool longpress) { 
	if (!editing) { 
		editing = true;
		selectedItem = re.value;
		JDisplayEditableItem *it = items[selectedItem];
		it->state = JDisplayEditableItem::EDITING;
		it->newValue = it->value;
		it->update();
		re.limMin = (it->min - it->value) / it->increment;
		re.limMax = (it->max - it->value) / it->increment;
		re.wrap = false;
		re.value = 0;
	} else { 
		editing = false;
		if (longpress == false) { 
			if (items[selectedItem]->newValue != items[selectedItem]->value) 
				items[selectedItem]->recentChange = true;
			items[selectedItem]->value = items[selectedItem]->newValue;
		}
		items[selectedItem]->state = JDisplayEditableItem::SELECTED;
		re.limMin = 0;
		re.limMax = items.size() - 1;
		re.wrap = false;
		re.value = selectedItem;
	}
	items[selectedItem]->update();
}


void JDisplayUpdateThread(void *p) { 
	JDisplay *jd = (JDisplay *)p;
	jd->forceUpdate();
#ifndef UBUNTU	
	esp_task_wdt_delete(NULL);	
	while(true) {
		jd->waitChange(10); 
		esp_task_wdt_reset();
		while(logFileBusySPI == false) {
			esp_task_wdt_reset();
			if (jd->update(false, true) == false || logFileBusySPI)
				break;
		}
		delayMicroseconds(10);
	}
#endif
}



template <class T>  
class CircularBoundedQueue { 
	Semaphore empty, full;
	int size, head, tail;
	T *array;
public:
	CircularBoundedQueue(int s) : size(s), empty(s, s), full(s, 0) {
		array = new T[size];
		head = tail = 0;
	}
	T *peekHead(int tmo) {
		if (!empty.take(tmo)) 
			return false;

		T *rval = &array[head];
		head = (head + 1) % size;
		return rval;
	}
	void postHead() {
		full.give();
	}
	T *peekTail(int tmo) {
		if (!full.take(tmo)) 
			return false;

		T *rval = &array[tail];
		tail = (tail + 1) % size;
		return rval;
	}
	void freeTail() {
		empty.give();
	}
};
#endif //#ifdef ESP32

// From data format described in web search "SL30 Installation Manual PDF" 
class SL30 {
public:
        std::string twoenc(unsigned char x) {
                char r[3];
                r[0] = (((x & 0xf0) >> 4) + 0x30);
                r[1] = (x & 0xf) + 0x30;
                r[2] = 0;
                return std::string(r);
        }
        int chksum(const std::string& r) {
                int sum = 0;
                const char* s = r.c_str();
                while (*s)
                        sum += *s++;
                return sum & 0xff;
        }
        void open() {
        }
        void pmrrv(const std::string& r) {
                std::string s = std::string("$PMRRV") + r + twoenc(chksum(r)) + "\r\n";
#ifdef ESP32
                Serial2.write(s.c_str());
#endif
				//Serial.printf("G5: %s", s.c_str());
                //Serial.write(s.c_str());
        }
        void setCDI(double hd, double vd) {
                int flags = 0b11111010;
                hd *= 127 / 3;
                vd *= 127 / 3;
                pmrrv(std::string("21") + twoenc(hd) + twoenc(vd) + twoenc(flags));
        }
};


class PinPulse { 
public:
	int pin;
	uint64_t toggleTime = 0;
	PinPulse(int p, int initval = 0) : pin(p) { pinMode(p, OUTPUT); digitalWrite(p, initval); } 
	void  pulse(int v, int ms) { 
		toggleTime = ms > 0 ? millis() + ms: 0;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, v);
	}
	void run() { 
		if (toggleTime > 0 && millis() >= toggleTime) {
			toggleTime = 0;
			pinMode(pin, OUTPUT);
			digitalWrite(pin, !digitalRead(pin));
		}
	}
};


std::string nmeaChecksum(const std::string &s) { 
	char check = 0;
	for (const char &c : s)  
		check ^= c;
	char buf[8];
	snprintf(buf, sizeof(buf), "*%02X\n", (int)check);
	return std::string("$") + s + std::string(buf);
		
}



//
// DS18 one-wire temperature sensor library

#ifndef UBUNTU
#include <OneWireNg.h>
#endif 

/* DS therms commands */
#define CMD_CONVERT_T           0x44
#define CMD_COPY_SCRATCHPAD     0x48
#define CMD_WRITE_SCRATCHPAD    0x4E
#define CMD_RECALL_EEPROM       0xB8
#define CMD_READ_POW_SUPPLY     0xB4
#define CMD_READ_SCRATCHPAD     0xBE

/* supported DS therms families */
#define DS18S20     0x10
#define DS1822      0x22
#define DS18B20     0x28
#define DS1825      0x3B
#define DS28EA00    0x42

inline void swapEndian(void *p1, void *p2, int s) { 
	for(int n = 0; n < s; n++) 
		((char *)p1)[n] = ((char *)p2)[s - n - 1];
}

struct DsTempData { 
	uint64_t id;
	uint64_t time;
	float degC;
};

class OneWireNg;	
inline std::vector<DsTempData> readTemps(OneWireNg *ow) { 

	std::vector<DsTempData> rval;
#ifndef UBUNTU
    OneWireNg::Id id;
    OneWireNg::ErrorCode ec;
    ow->searchReset();

    do
    {
        ec = ow->search(id);
        if (!(ec == OneWireNg::EC_MORE || ec == OneWireNg::EC_DONE))
            break;

        /* start temperature conversion */
        ow->addressSingle(id);
        ow->writeByte(CMD_CONVERT_T);

        delay(750);

        uint8_t touchScrpd[] = {
            CMD_READ_SCRATCHPAD,
            /* the read scratchpad will be placed here (9 bytes) */
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
        };

        ow->addressSingle(id);
        ow->touchBytes(touchScrpd, sizeof(touchScrpd));
        uint8_t *scrpd = &touchScrpd[1];  /* scratchpad data */

        if (OneWireNg::crc8(scrpd, 8) != scrpd[8]) {
            //Serial.println("  Invalid CRC!");
            continue;
        }

        long temp = ((long)(int8_t)scrpd[1] << 8) | scrpd[0];

        if (id[0] != DS18S20) {
            unsigned res = (scrpd[4] >> 5) & 3;
            temp = (temp >> (3-res)) << (3-res);  /* zeroed undefined bits */
            temp = (temp*1000)/16;
        } else
        if (scrpd[7]) {
            temp = 1000*(temp >> 1) - 250;
            temp += 1000*(scrpd[7] - scrpd[6]) / scrpd[7];
        } else {
            /* shall never happen */
            temp = (temp*1000)/2;
			continue;
        }
		
		uint64_t lid;
		swapEndian(&lid, &id, sizeof(lid));
		float ftemp = temp / 1000.0;
		DsTempData dsData;
		dsData.degC = temp / 1000.0;
		dsData.id = lid;
		dsData.time = millis();
		
		rval.push_back(dsData);
    } while (ec == OneWireNg::EC_MORE);
#endif
    return rval;
}



#ifdef ESP32
const char* host = "esp32";
const char* ssid = "xxx";
const char* password = "xxxx";

WebServer server(80);

/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";



class JimWiFi { 
	EggTimer report = EggTimer(1000);
	bool firstRun = true, firstConnect = true;
	std::function<void(void)> connectFunc = NULL;
	bool updateInProgress = false;
public:
	WiFiUDP udp;
	WiFiMulti wifi;
	void onConnect(std::function<void(void)> oc) { 
		connectFunc = oc;
	}
	void run() { 
#ifndef UBUNTU
		if (firstRun) {
			wifi.addAP("ChloeNet", "niftyprairie7");
			//wifi.addAP("TUK-FIRE", "FD priv n3t 20 q4");
			firstRun = false;
		}
		wifi.run();
		if (WiFi.status() == WL_CONNECTED) { 
			if (firstConnect ==  true) { 
				firstConnect = false;
			  server.on("/", HTTP_GET, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/html", loginIndex);
			  });
			  server.on("/serverIndex", HTTP_GET, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/html", serverIndex);
			  });
			  /*handling uploading firmware file */
			  server.on("/update", HTTP_POST, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
				ESP.restart();
			  }, []() {
				HTTPUpload& upload = server.upload();
				esp_task_wdt_reset();

				if (upload.status == UPLOAD_FILE_START) {
				  Serial.printf("Update: %s\n", upload.filename.c_str());
				  if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
				  }
				} else if (upload.status == UPLOAD_FILE_WRITE) {
				  Serial.printf("Update: write %d bytes\n", upload.currentSize);
				  /* flashing firmware to ESP*/
				  if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
				  }
				} else if (upload.status == UPLOAD_FILE_END) {
				  if (Update.end(true)) { //true to set the size to the current progress
					Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
				  } else {
					Update.printError(Serial);
				  }
				}
			  });
			  server.begin();


				ArduinoOTA.onStart([]() {
				String type;
				if (ArduinoOTA.getCommand() == U_FLASH) {
				  type = "sketch";
				} else { // U_FS
				  type = "filesystem";
				}

				// NOTE: if updating FS this would be the place to unmount FS using FS.end()
				Serial.println("Start updating " + type);
				});
				ArduinoOTA.onEnd([]() {
				Serial.println("\nEnd");
				});
				ArduinoOTA.onProgress([&](unsigned int progress, unsigned int total) {
				Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
				esp_task_wdt_reset();
				updateInProgress = true;
				});
				ArduinoOTA.onError([&](ota_error_t error) {
				Serial.printf("Error[%u]: ", error);
				if (error == OTA_AUTH_ERROR) {
				  Serial.println("Auth Failed");
				} else if (error == OTA_BEGIN_ERROR) {
				  Serial.println("Begin Failed");
				} else if (error == OTA_CONNECT_ERROR) {
				  Serial.println("Connect Failed");
				} else if (error == OTA_RECEIVE_ERROR) {
				  Serial.println("Receive Failed");
				} else if (error == OTA_END_ERROR) {
				  Serial.println("End Failed");
				}
				updateInProgress = false;
				});
 
				
				ArduinoOTA.begin();

				udp.begin(9999);
				if (connectFunc != NULL) { 
					connectFunc();
				}
			}
			
			do {
				ArduinoOTA.handle();
			} while(updateInProgress == true); 
			server.handleClient();
			
			if (report.tick()) { 
				udp.beginPacket("255.255.255.255", 9000);
				char b[128];
				snprintf(b, sizeof(b), "%d %s    " __BASE_FILE__ "   " __DATE__ "   " __TIME__ "   0x%08x\n", 
					(int)(millis() / 1000), WiFi.localIP().toString().c_str(), /*(int)ESP.getEfuseMac()*/0);
				udp.write((const uint8_t *)b, strlen(b));
				udp.endPacket();
			}
			
			
		}
#endif
	}
	bool connected() { return WiFi.status() == WL_CONNECTED;  }  

};

class ShortBootDebugMode {
	SPIFFSVariable<int> shortBootCount = SPIFFSVariable<int>("/shortBootCount", 1);
	bool initialized = false;
	bool cleared = false;
	int sbCount; 
  public:
	void begin() { 
		SPIFFS.begin();
		shortBootCount = shortBootCount + 1;
		sbCount = shortBootCount;
		
		Serial.printf("Short boot count %d\n", sbCount);
		initialized = true;
	}
	int check() {
		if (initialized == false) { 
			begin();
		}
		if (millis() > 5000 && cleared == false) { 
			cleared = true;
			shortBootCount = 0 ;
		}
		return sbCount;
	} 
};
#endif


template<class T>
class ExtrapolationTable {
        bool between(T a, T b, T c) { 
                return (c >= a && c < b) || (c <= a && c > b);
        }
public:
        struct Pair {
             T a,b;        
        } *table;
        ExtrapolationTable(struct Pair *t) : table(t) {}
        T extrapolate(T in, bool reverse = false) {
                for(int index = 1; table[index].a != -1 || table[index].b != -1; index++) {     
                        if (!reverse && between(table[index - 1].a, table[index].a, in))  
                                return table[index - 1].b + (in - table[index - 1].a) 
                                        * (table[index].b - table[index - 1].b) 
                                        / (table[index].a - table[index - 1].a);
                        if (reverse && between(table[index - 1].b, table[index].b, in)) 
                                return table[index - 1].a + (in - table[index - 1].b) 
                                        * (table[index].a - table[index - 1].a) 
                                        / (table[index].b - table[index - 1].b);                
          
                }
                return -1;
        }
        T operator () (T in) { return extrapolate(in); }
};



#endif //#ifndef INC_JIMLIB_H
