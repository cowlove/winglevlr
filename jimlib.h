
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

class ScopedMutex {
	Mutex &mutex; 
  public:
	ScopedMutex(Mutex &m) : mutex(m) { mutex.lock(); } 
	~ScopedMutex() { mutex.unlock(); } 
};


class Semaphore { 
	SemaphoreHandle_t xSem;
 public:
	Semaphore() { 
		xSem = xSemaphoreCreateCounting(1,1);
	}
	void take() { xSemaphoreTake(xSem, portMAX_DELAY); } 
	void give() { xSemaphoreGive(xSem); }
};


class EggTimer {
	uint64_t last;
	int interval; 
public:
	EggTimer(int ms) : interval(ms), last(0) { reset(); }
	bool tick() { 
		uint64_t now = micros();
		if (now - last >= interval * 1000) { 
			last += interval * 1000;
			// reset last to now if more than one full interval behind 
			if (now - last >= interval * 1000) 
				last = now + interval;
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
	void run() { 
		filter.check(button.duration());
	}
	bool newEventR() { run(); return newEvent(); }
	int count() { return filter.wasCount; } 
	int wasLong() { return filter.wasLong; } 
};

#ifdef ESP32
// mutex to serialize SD card and TFT writes. 
Mutex mutex;

void open_TTGOTS_SD() { 
	for (int retries = 0; retries < 2; retries++) { 	
		ScopedMutex lock(mutex);
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

template <class T>
class SDCardBufferedLog {
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
	
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true) : 
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
		 vQueueDelete(queue);
		 printSD();
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
		char fname[20];
		msdFile f;
		if (strchr(filename, '%')) { // if filename contains '%', scan for existing files  
			for(fileVer = 1; fileVer < 999; fileVer++) {
				snprintf(fname, sizeof(fname), filename, fileVer);
				ScopedMutex lock(mutex);
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
		{
			ScopedMutex lock(mutex);
			SD.remove((char *)currentFile.c_str());
			f = SD.open((char *)currentFile.c_str(), (F_READ | F_WRITE | F_CREAT));
		}
		//Serial.printf("Opened %s\n", fname);
		while(!exitNow) { // TODO: doesn't flush queue before exiting 	
			T v;
			if (xQueueReceive(queue, &v, timo) == pdTRUE) {
				if (!exitNow) {
					uint64_t now = millis();
					if (f) {
						ScopedMutex lock(mutex);
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
				if (f) {
					ScopedMutex lock(mutex);
					f.flush();
				}
				lastFlush = now;
			}
		}
		if (f) {
			ScopedMutex lock(mutex);
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
	void add(T *v, int t) { 
		int s = write(fd, (void *)v, sizeof(T));
		printf("%s LOG\n", v->toString().c_str()); 
	}
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true) {
		currentFile = fname;
		char buf[64];
		snprintf(buf, sizeof(buf), fname, 1);
		fd = open(buf, O_WRONLY | O_CREAT, 0666);
	}
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






class JDisplayItemBase;
class JDisplay {
	std::vector<JDisplayItemBase *> items;
	Semaphore changeSem;
public:
	static const struct Colors { 
		int lf, lb, vf, vb; 
	} defaultColors; 
#ifndef UBUNTU 
	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
	void begin() { 
		{
			ScopedMutex lock(mutex);
			//pinMode(27,OUTPUT); 		//Backlight:27  TODO:JIM 27 appears to be an external pin 
			//digitalWrite(27,HIGH);	//New version added to backlight control
			tft.initR(INITR_18GREENTAB);                             // 1.44 v2.1
			tft.fillScreen(ST7735_BLACK);                            // CLEAR
			tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);           // GREEN
			tft.setRotation(1);                                      // 
		}
		forceUpdate();
	}
	void clear() {
		ScopedMutex lock(mutex);
		tft.fillScreen(ST7735_BLACK);                            // CLEAR
	}
	void printAt(int x, int y, const char *s, int fg, int bg) { 
		ScopedMutex lock(mutex);
		tft.setTextColor(fg, bg); 
		tft.setCursor(x, y);
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
	void printAt(int x, int y, const char *f, int, int) { 
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
	void waitChange() { changeSem.take(); }

	void addItem(JDisplayItemBase *i) { 
		items.push_back(i);
	}
	inline void forceUpdate();	
	inline void update(bool update);	
};


#ifndef UBUNTU
const JDisplay::Colors JDisplay::defaultColors = { ST7735_WHITE, ST7735_BLACK, ST7735_YELLOW, ST7735_BLACK };
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
public:
	bool changed = false;
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
		if (li != labelInverse || vi != valueInverse) {
			labelInverse = li;
			valueInverse = vi;
			changed = true;
			d->markChange();
		}
		//update(changed);
	}
	void update(bool force) {
		if (d == NULL)
			return;
		if (first || ++updates % 1000 == 1) 
			force = true;
		if (changed) { 
			force = true;
			changed = false; 
		}
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
		if (lastVal != val) 
			d->markChange();
	}
};

inline void JDisplay::update(bool force) { 
	for (std::vector<JDisplayItemBase *>::iterator it = items.begin(); it != items.end(); it++) { 
		(*it)->update(force);
	}
}

inline void JDisplay::forceUpdate() { 
	update(true);
}

template<class T>
class JDisplayItem : public JDisplayItemBase { 
public:
	JDisplayItem(JDisplay *d, int x, int y, const char *label, const char *fmt, JDisplay::Colors colors = JDisplay::defaultColors) :
		JDisplayItemBase(d, x, y, label, fmt, colors) {}
	JDisplayItem<T>& operator =(const T&v) { setValue(v); return *this; }
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
	JDisplayItemBase *di; 
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
	JDisplayEditableItem(JDisplayItemBase *i, float inc, float mi = -100000, float ma = +10000) : 
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

#endif
