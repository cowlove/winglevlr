
class EggTimer {
	uint64_t last;
	int interval; 
public:
	EggTimer(int ms) : interval(ms), last(0) { reset(); }
	bool tick() { 
		uint64_t now = millis();
		if (now - last > interval) { 
			last = now;
			return true;
		} 
		return false;
	}
	void reset() { 
		last = millis();
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
public:
	int duration;
	int count;
	DigitalDebounce(int ms = 50) : timer(ms), recentlyPressed(false),count(0) {}
	bool checkOneshot(bool button) {
		bool rval = false; 
		if (button == true) {
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
	boolean wrap;
	unsigned long lastChange;
	
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

	void IRAM_ATTR ISR() {	
		portENTER_CRITICAL_ISR(&(this->mux));
		check();
		portEXIT_CRITICAL_ISR(&(this->mux));
	}
	RotaryEncoder(int p1, int p3, int debounce = 0) : a(debounce), b(debounce), pin1(p1), pin3(p3){
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
	void begin(void (*ISR_callback)(void)) {
		attachInterrupt(digitalPinToInterrupt(pin1), ISR_callback, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pin3), ISR_callback, CHANGE);
	}
	void check() {
		int buta = !digitalRead(pin1);
		int butb = !digitalRead(pin3);
		int delta = 0;
		//Serial.printf("%d %d\n", buta, butb);
		if (a.checkOneshot(buta) && !butb) 
			delta = -1;
		if (b.checkOneshot(butb) && !buta)
			delta = +1;
		
		if (delta != 0) {
			unsigned long now = millis();
			if(lastChange > 0 && now - lastChange < 30)
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
	int wasDuration;
	int count, wasCount;
	int lastDuration;
	int events;
	bool countedLongPress;

	LongShortFilter(int longMs, int resetTimeMs) : longPressMs(longMs), resetMs(resetTimeMs) {
		events = lastDuration = count = wasCount = lastEndTime = 0;
		countedLongPress = false;
	}
	
	bool check(int last) { 
		unsigned long now = millis();
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
	bool run() { 
		filter.check(button.duration());
	}
	bool newEventR() { run(); return newEvent(); }
	int count() { return filter.wasCount; } 
	int wasLong() { return filter.wasLong; } 
};


template <class T, int SIZE>
class RollingAverage {
	T values[SIZE];
	T sum = 0;
	int count = 0;
	int index = 0;
public:
	RollingAverage() {}
	void add(const T &v) { 
		if (count == SIZE) 
			sum -= values[index];
		else 
			count++;
		values[index++] = v;
		sum += v;
		if (index >= SIZE)
			index = 0;
	}
	T average() { 
		return count > 0 ? sum/count : 0;
	}
	T min() { 
		T m = values[0];
		for (int n = 0; n < count; n++)
			if (values[n] < m) m = values[n];
		return m;
	}
	T max() { 
		T m = values[0];
		for (int n = 0; n < count; n++)
			if (values[n] > m) m = values[n];
		return m;
	}
};


void open_TTGOTS_SD() { 
	for (int retries = 0; retries < 2; retries++) { 	
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

template <class T>
void SDCardBufferedLogThread(void *p);

template <class T>
class SDCardBufferedLog {
	QueueHandle_t queue;
	int timo;
	int flushInterval;
	bool exitNow = false;
	char *filename;

	void exit() { 
		exitNow = true;
		T v;
		add(&v);
		while(exitNow)
			delay(1);
	}
	boolean textMode;
public:
	String currentFile;
	
	SDCardBufferedLog(char *fname, int len, int timeout, int flushInt, boolean textMode = true) : 
		filename(fname), 
		flushInterval(flushInt), 
		timo((timeout+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS) {
		queue = xQueueCreate(len, sizeof(T));
		this->textMode = textMode;
		open_TTGOTS_SD();
		setFilename();
		xTaskCreate(
                    SDCardBufferedLogThread<T>,       /* Function that implements the task. */
                    "NAME",          /* Text name for the task. */
                    8192,      /* Stack size in words, not bytes. */
                    ( void * ) this,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */
	}
	~SDCardBufferedLog() { 
		 this->exit();
		 SD.end();
	}
	void add(T *v) {
		xQueueSend(queue, v, timo);
	}
	void add(T *v, int t) {
		xQueueSend(queue, v, (t+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS);
	}
	void setFilename() { 
		int fileVer = 1;
		char fname[1024];
		msdFile f;
		if (strchr(filename, '%')) { // if filename contains '%', scan for existing files  
			for(fileVer = 1; fileVer < 999; fileVer++) {
				sprintf(fname, filename, fileVer);
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

		SD.remove((char *)currentFile.c_str());
		msdFile f = SD.open((char *)currentFile.c_str(), (F_READ | F_WRITE | F_CREAT));
		
		//Serial.printf("Opened %s\n", fname);
		while(!exitNow) { // TODO: doesn't flush queue before exiting 	
			T v;
			if (xQueueReceive(queue, &v, timo) == pdTRUE) {
				if (!exitNow) {
					uint64_t now = millis();
					if (false && f) {
						if (textMode)  
							f.println(v.toString());
						else 
							f.write((uint8_t *)&v, sizeof(v));
					}
					lastWrite = now;
					//Serial.printf("WROTE\n");
				}
			}
			uint64_t now = millis();
			if (now - lastFlush >= flushInterval) {
				if (f) 
					f.flush();
				lastFlush = now;
			}
			delay(1);
		}
		if (f) 
			f.close();
		exitNow = false;
		vTaskDelete(NULL);
	}
	void flush() { 
		while(uxQueueMessagesWaiting(queue) > 0) 
			sleep(1);
	}
};	

template <class T>
void SDCardBufferedLogThread(void *p) {
	((SDCardBufferedLog<T> *)p)->thread(); 
}







// JDisplay jd;
// JDisplayItem<float> f1(&jd,10,10,"LABEL1:", "%+02.2f");
// JDisplayItem<int> f2(&jd,10,20,"LABEL2:", "%02d");

// jd.begin();
// f2 = 0;
// f1 = 1.1;

#include <Adafruit_GFX.h>               // Core graphics library
#include <Adafruit_ST7735.h>            // Hardware-specific library


#define TFT_CS 16
#define TFT_RST 9  
#define TFT_DC 17
#define TFT_SCLK 5   
#define TFT_MOSI 23  
#define ST7735
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

class JDisplayItemBase;
class JDisplay { 
	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
	std::vector<JDisplayItemBase *> items;
public:
	static const struct Colors { 
		int lf, lb, vf, vb; 
	} defaultColors; 
	void begin() { 
		//pinMode(27,OUTPUT); 		//Backlight:27  TODO:JIM 27 appears to be an external pin 
		//digitalWrite(27,HIGH);	//New version added to backlight control
		tft.initR(INITR_18GREENTAB);                             // 1.44 v2.1
		tft.fillScreen(ST7735_BLACK);                            // CLEAR
		tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);           // GREEN
		tft.setRotation(1);                                      // 
		forceUpdate();
	}
	void clear() { 
		tft.fillScreen(ST7735_BLACK);                            // CLEAR
	}
	void printAt(int x, int y, const char *s, int fg, int bg) { 
		tft.setTextColor(fg, bg); 
		tft.setCursor(x, y);
		tft.print(s);
	}
	void addItem(JDisplayItemBase *i) { 
		items.push_back(i);
	}
	inline void forceUpdate();
	
};

const JDisplay::Colors JDisplay::defaultColors = { ST7735_WHITE, ST7735_BLACK, ST7735_YELLOW, ST7735_BLACK };


class JDisplayItemBase {
	int x, y, updates;
	int lb, lf, vb, vf;
	JDisplay *d;
	String val, lastVal;
	bool labelInverse = false, valueInverse = false;
	bool first = true;
public:
	const char *label, *fmt;
	JDisplay::Colors color;
	JDisplayItemBase(JDisplay *d, int x, int y, const char *label, const char *fmt, JDisplay::Colors colors) {
		this->x = x;
		this->y = y;
		this->label = label;
		this->fmt = fmt;
		this->d = d;
		this->color = colors;
		updates = 0;
		if (d != NULL)
			d->addItem(this);
	}
	void setInverse(bool li, bool vi) {
		bool changed = (li != labelInverse || vi != valueInverse);
		labelInverse = li;
		valueInverse = vi;
		update(changed);
	}
	void update(bool force) {
		if (d == NULL)
			return;
		if (first || ++updates % 1000 == 1) 
			force = true;
		first = false;
		if (force)
			d->printAt(x, y, label, labelInverse ? color.lb : color.lf , labelInverse ? color.lf : color.lb);
		if (force || val != lastVal) { 
			d->printAt(x + 6 * strlen(label), y, val.c_str(), valueInverse ? color.vb : color.vf, valueInverse ? color.vf : color.vb);
			lastVal = val;
		}
	}
	template<class T>
	void setValue(const T&v) { 
		char buf[64];
		sprintf(buf, fmt, v);
		val = String(buf);
		update(false);
	}
};

inline void JDisplay::forceUpdate() { 
	for (std::vector<JDisplayItemBase *>::iterator it = items.begin(); it != items.end(); it++) { 
		(*it)->update(true);
	}
}


template<class T>
class JDisplayItem : public JDisplayItemBase { 
public:
	JDisplayItem(JDisplay *d, int x, int y, const char *label, const char *fmt, JDisplay::Colors colors = JDisplay::defaultColors) :
		JDisplayItemBase(d, x, y, label, fmt, colors) {}
	JDisplayItem<T>& operator =(const T&v) { setValue(v); }
};


