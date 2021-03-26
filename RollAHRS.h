#include <cmath>
#include <math.h>
#include "RollingLeastSquares.h"

using namespace std;


#define USE_ACCEL

struct AhrsInputA { 
	float sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y;
	float ax, ay, az;  
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float q3, palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f %f %f %f" /* 1 - 10 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f" /* 11 - 20 */
			"%.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f",  /* 21 - 27 */ 
		sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, q3, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5TimeStamp);
		return String(buf);	
	 }
	 AhrsInputA fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &selTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt, &g5TimeStamp);
		return *this;
	}
};

struct AhrsInputB { 
	float sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt; 
	float ax, ay, az;  
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() const { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f " /* 1 - 10 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f " /* 11 - 20 */
			"%.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f",  /* 21 - 27 */ 
		sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5TimeStamp);
		return String(buf);	
	 }
	 AhrsInputB fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &selTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt, &g5TimeStamp);
		return *this;
	}
	AhrsInputB &operator =(const AhrsInputA &a) { 
		sec = a.sec;
		selTrack = a.selTrack;
		gpsTrackGDL90 = a.gpsTrackGDL90;
		gpsTrackVTG = a.gpsTrackVTG;
		gpsTrackRMC = a.gpsTrackRMC;
		alt = a.alt;
		ax = a.ax; ay = a.ay; az = a.az;
		gx = a.gx; gy = a.gy; gz = a.gz;
		mx = a.mx; my = a.my; mz = a.mz;
		palt = a.palt; gspeed = a.gspeed; g5Pitch = a.g5Pitch; g5Roll = a.g5Roll; g5Hdg = a.g5Hdg; g5Ias = a.g5Ias; g5Tas = a.g5Tas;
		g5Palt = a.g5Palt; g5TimeStamp = a.g5TimeStamp;
		return *this;
	}
		
};

typedef AhrsInputB AhrsInput; 


class Windup360 {
public:
	float value = 0;
	bool first = true;
	operator float () { 
		return value;
	}
	Windup360 &operator =(float f) {
		if (first) {
			value = f;
			first = false;
		}
		float hd = f - value;
		if (abs(hd) > 100000) {
			value = f;
		} else {
			while (hd < -180) hd += 360;
			while (hd > +180) hd -= 360;
			value += hd;
		}
		return *this;
	}
	float angle() { 
		float a = value;
		if (abs(a) < 10000) { 
			while(a <= 0) a += 360;
			while(a > 360) a -= 360;
		}
		return a;
	}
};

float constrain360(float a) { 
	if (abs(a) < 10000) { 
		while(a <= 0) a += 360;
		while(a > 360) a -= 360;
	}
	return a;
}

float angularDiff(float d) { 
	if (abs(d) > 100000) 
		return d;
	while(d < -180) d += 360;
	while(d > 180) d -= 360;
	return d;
}

float angularClosest(float a, float b) {
	return b + angularDiff(a - b);
}

inline static float windup360(float now, float prev) { 
	float hd = now - prev;
	if (hd > 1000000 || hd < -1000000) 
		return now;
	while (hd < -180) hd += 360;
	while (hd >= +180) hd -= 360;
	return prev + hd;
}
	
	
class MultiCompFilter { 
	bool first = true;
	float defaultCr = 0.03;
	float age = 5.0;
public:
	float value, prevMainValue, bestCr, bestAux, bestAuxPri, expires, priority;
	void reset() { first = true; } 
	float calculate(float now, float v) {
		if (first || abs(now - expires) > 20) {
			prevMainValue = value = v;
			bestAuxPri = -1;
			first = false;
			expires = now + age;
		}
		if(now > expires || bestAuxPri >= priority) { 
			if (bestAuxPri == -1) {
				bestAux = v;
				bestAuxPri = 0;			
				bestCr = defaultCr;
			}
			expires = now + age;
			bestAux = angularClosest(bestAux, value);
			priority = bestAuxPri;
			value = (1 - bestCr) * (value) + (bestCr * bestAux);
		} 
		value += angularDiff(v - prevMainValue);	
		bestAuxPri = -1;
		prevMainValue = v;
		return value;	  
	}
	void addAux(float v, int pri, float cr) {
		if (pri >= bestAuxPri) {
			bestAux = v;
			bestAuxPri = pri;
			bestCr = cr;
		}
	}
};


class RollAHRS {
public:
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
	MultiCompFilter mComp;
	
	float magOffX = 27.5;
	float magOffY = 24.4;
	float magOffZ = -30;

	
	float magScaleX = 1.0;
	float magScaleY = 1.0;
	float magScaleZ = 1.0;
	
	float gyrOffX = -0.3; 
	float gyrOffY = +0.3; 
	float gyrOffZ = +0.3;
		  
	float accOffX = +0,
		  accOffY = -0,
		  accOffZ = -0;

	float compRatio1 = 0.00072;  // roll comp filter ratio 
	float driftCorrCoeff1 = 2.80; // how fast to add in drift correction
	float hdgCompRatio = .00013;  // composite filter ratio for hdg 
	float magDipConstant = 2.464; // unexplained correction factor for bank angle in dip calcs
	float magBankTrimCr = 0.00005;
	float magBankTrimMaxBankErr = 12;
	
	RollAHRS() { 
		gyrYOffsetFit.add(gyrOffY);
		gyrZOffsetFit.add(gyrOffZ);
		gyrXOffsetFit.add(gyrOffX);
	}
		
	int zeroSampleCount = 0; 
	float g5LastTimeStamp = 0; 
	struct { 
		TwoStageRollingAverage<float,20,20> ax,ay,az,gx,gy,gz;
	} zeroAverages;

	TwoStageRollingAverage<float,20,150>
		gyrZOffsetFit,
		gyrXOffsetFit,
		gyrYOffsetFit;

	float magBank, magBankTrim = 0.0;
	float gpsBankAngle, magBankAngle, dipBankAngle, dipBankAngle2, magHdg, rawMagHdg, /*bankCorrection,*/ bankAngle;
	float gyroTurnBank, pG;
	float pitchComp = 0, pitchRaw = 0, pitchDrift = 0, pitchCompDriftCorrected = 0;
	float magStability = -1;
	float hdg;
	bool hdgInitialized = false;
	
	typedef TwoStageRollingLeastSquares<float> TwoStageRLS;
	RollingLeastSquares // all at about 200 HZ */
		gyroDriftFit = RollingLeastSquares(300), // 10HZ 
		magHdgFit = RollingLeastSquares(50); // 10Hz
				
	RollingAverage<float,200> magStabFit;
	RollingAverage<float,50> avgRoll;
	RollingAverage<float,20> avgMagHdg;
	RollingAverage<float,50> avgGZ, avgGX;
	RollingAverage<float,200> gyroZeroCount;
	
	TwoStageRLS 
		magZFit = TwoStageRLS(20, 20),
		magXFit = TwoStageRLS(20, 20),
		magYFit = TwoStageRLS(20, 20),
		magHdgAvg = TwoStageRLS(20,20);
		
	float fakeTimeMs = 0; // period for fake timestamps, 0 to use real time 
	int count = 0;
	AhrsInput prev;
	Windup360 magHdg360;
	float compR = 0, compYH =0, rollG = 0;
	float gyroDrift = 0;
	float gpsPitch = 0, accelPitch = 0;
	float lastGz, magCorr = 0;

	bool valid() { 
		return prev.selTrack != -1;
	}
	
	void zeroSensors() { 
		gyrOffX = zeroAverages.gx.average(); 
		gyrOffY = zeroAverages.gy.average();
		gyrOffZ = zeroAverages.gz.average();
		accOffX = zeroAverages.ax.average();
		accOffY = zeroAverages.ay.average();
		accOffZ = zeroAverages.az.average() + 1.0;
		Serial.printf("ZERO SENSORS gyro %f %f %f accel %f %f %f\n", gyrOffX, gyrOffY, gyrOffZ, accOffX, accOffY, accOffZ); 
	}
	
	float add(const AhrsInput &i) {
		AhrsInput l(i);
		float dt = 0;
		if (fakeTimeMs > 0)
			l.sec = (count * fakeTimeMs) / 1000.0;	
		if (count > 0 ) 
			dt = min((float).1, l.sec - prev.sec);
		
		bool tick10HZ = (floor(l.sec / .1) != floor(prev.sec / .1));

		zeroAverages.ax.add(l.ax);
		zeroAverages.ay.add(l.ay);
		zeroAverages.az.add(l.az);
		zeroAverages.gx.add(l.gx);
		zeroAverages.gy.add(l.gy);
		zeroAverages.gz.add(l.gz);
		
		l.mx = (l.mx - magOffX) / magScaleX;
		l.my = (l.my - magOffY) / magScaleY;
		l.mz = (l.mz - magOffZ) / magScaleZ;
	
#ifdef USE_ACCEL
		l.ax -= accOffX;
		l.ay -= accOffY;
		l.az -= accOffZ;
#endif
	
		if (gyrZOffsetFit.full()) { 
			l.gx -= gyrXOffsetFit.average();
			l.gy -= gyrYOffsetFit.average();
			l.gz -= gyrZOffsetFit.average();
		} else { 
			l.gx -= gyrOffX;
			l.gz -= gyrOffZ;
			l.gy -= gyrOffY;
		}
		avgGZ.add(l.gz);
		avgGX.add(l.gx);
		
		magHdg = atan2(l.my, l.mx) * 180 / M_PI;

		//magMagnitudeFit.add(l.sec, magTotalMagnitude);
		magZFit.add(l.sec, l.mz);
		magXFit.add(l.sec, l.mx);
		magYFit.add(l.sec, l.my);

		if (magZFit.full()) {
			magStabFit.add(abs(magZFit.slope()) + abs(magXFit.slope()) + abs(magYFit.slope()));
			if (magStabFit.full()) {
				magStability = min(15.0, (double)magStabFit.average()); 
				const float stabThreshold = 0.0;
				if (magStability < stabThreshold) { 
					//gyrZOffsetFit.add(l.sec, magZFit.stage1.averageY(), max(stabThreshold/2, (stabThreshold/2 - magStability)*100));
					gyrZOffsetFit.add(zeroAverages.gz.average());
					gyrXOffsetFit.add(zeroAverages.gx.average());
					gyrYOffsetFit.add(zeroAverages.gy.average());
					zeroSampleCount++;
				}
			}
		}
		if (tick10HZ) { 
			gyroZeroCount.add(zeroSampleCount);
			zeroSampleCount = 0;
		}
					
		// prevent discontinuities in hdg, just keep wrapping it around 360,720,1080,...
		if (count > 0) { 	
			l.selTrack = windup360(l.selTrack, prev.selTrack);
		}

		//magHdgRawFit.add(l.sec, rawMagHdg);
		//gpsHdgFit.add(l.sec, l.gpsTrack);
		
		if (count % 3217 == 0) { 
			//gpsHdgFit.rebaseX();
			magHdgFit.rebaseX();
			gyroDriftFit.rebaseX();
		}

		float tas = 100; //l.g5Ias; // true airspeed in knots		

		//const float driftCorrCoeff1 = pow(1 - compRatio1, 200) * 9;
		
		//printf("DEBUG %f\n", driftCorrCoeff1);
		float rollRad = avgRoll.average() * M_PI / 180;
		float zgyrBankAngle = atan((cos(rollRad) * avgGZ.average() + (sin(abs(rollRad)) * avgGX.average())) * tas / 1091) * 180/M_PI;
		bankAngle = (isnan(zgyrBankAngle) ? 0 : zgyrBankAngle);
		bankAngle *= 1.00;
		//bankAngle = max(-30.0, min(30.0, (double)bankAngle));
		bankAngle = max(avgRoll.average() - 15.0, min(avgRoll.average() + 15.0, (double)bankAngle));
		//bankAngle =0;
	
		compR = (compR + l.gy * 1.00 /*gyroGain*/ * dt) * (1-compRatio1) + (bankAngle * compRatio1);
		rollG  += l.gy * dt;
		
		if (tick10HZ) { 
			gyroDriftFit.add(l.sec, compR - rollG);
			gyroDrift = gyroDriftFit.slope();
		}
		
		compYH = compR + gyroDrift * driftCorrCoeff1;
		//if (abs(bankAngle) < 30) {			
		//	gyroDrift += (bankAngle - compYH) * 0.0001;
		//}
			
		avgRoll.add(compYH);

		// TODO replace this kinda-help heuristic until we get proper dip correction 
		//magHdg += -cos(magHdg / 180 * M_PI) * sin(avgRoll.average() / 180 * M_PI) * 150;



		// attempt magnetic dip bank error correction 
		float ra = magDipConstant * avgRoll.average() / 180 * M_PI;   
		float z = sin(67.0*M_PI/180) * cos(ra); 
		float y = sin(magHdg*M_PI/180);
		float y1 = y * cos(ra) - z * sin(ra);
		magHdg =  atan2(y1, cos(magHdg*M_PI/180)) * 180 / M_PI;
		magHdg = angularClosest(magHdg, avgMagHdg.average());
		avgMagHdg.add(magHdg);
		magHdg360 = avgMagHdg.average();

		if (hdgInitialized == false && avgMagHdg.full()) { 
			hdgInitialized = true;
			hdg = magHdg360;
		}
		hdg =  (hdg - (cos(rollRad) * l.gz + sin(abs(rollRad)) * l.gx) * dt) * (1 - hdgCompRatio) + magHdg360 * hdgCompRatio;		

		
		float cHdg = mComp.calculate(l.sec, hdg);
		if (tick10HZ) {
			magHdgFit.add(l.sec, cHdg);
		}
		magHdg = constrain360(cHdg);		
		
		if (magHdgFit.full()) { 
			compYH -= magBankTrim;
			magBank = -atan(magHdgFit.slope() * tas / 1091) * 180/M_PI;
			if (abs(compYH - magBank) < magBankTrimMaxBankErr) { 
				magBankTrim += (compYH - magBank) * magBankTrimCr;
				magBankTrim = max(min((double)magBankTrim, 4.0), -4.0);
			}
		}
		
		count++;
		prev = l;
		
		compYH;
				
		return compYH;
	}	
	
	float getGyroQuality() {
		return gyroZeroCount.average();
	}
	void reset() {
		mComp.reset();
		pitchRaw = pitchComp = 0;
		gyroDriftFit.reset();
		magHdgFit.reset();
		magStabFit.reset();
		avgRoll.reset();
		avgMagHdg.reset();
		avgGZ.reset();
		avgGX.reset();
		gyroZeroCount.reset();
		magZFit.reset();
		magXFit.reset();
		magYFit.reset();
		magHdgAvg.reset();
	}
};

struct LogItem0 {
	AhrsInput ai;
	String toString() { return ai.toString(); } 
	LogItem0 fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

struct LogItemA {
	short pwmOutput, servoTrim;
	float pidP, pidI, pidD;
	float gainP, gainI, gainD, finalGain;
	AhrsInput ai;
	String toString() { return ai.toString(); } 
	LogItemA fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

struct LogItemB {
	short pwmOutput, flags;
	float pidP, pidI, pidD;
	float gainP, gainI, gainD, finalGain;
	float desRoll, pitchCmd;
	AhrsInput ai;
	String toString() { return ai.toString(); } 
	LogItemB fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

struct LogItemC {
	short pwmOutput, flags;  // 30 31
	float pidP, pidI, pidD;  // 32  
	float gainP, gainI, gainD, finalGain; // 35 
	float desRoll, pitchCmd, roll; // 39 
	AhrsInputA ai;
	String toString() {
		char buf[200];
		snprintf(buf, sizeof(buf), " %d %d %f %f", (int)pwmOutput, (int)flags,
			desRoll, roll);
		return ai.toString() +  String(buf);
	} 
	LogItemC fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

struct LogItemD {
	short pwmOutput, flags;  
	float desRoll, roll; 
	AhrsInputB ai;
	String toString() const {
		char buf[200];
		snprintf(buf, sizeof(buf), " %d %d %f %f", (int)pwmOutput, (int)flags,
			desRoll, roll);
		return ai.toString() +  String(buf);
	} 
	LogItemD fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
	LogItemD &operator =(const LogItemC &c) {
		ai = c.ai;
		pwmOutput = c.pwmOutput;
		flags = c.flags;
		desRoll = c.desRoll;
		roll = c.roll;
		return *this;
	}
};


struct LogItemE {
	short pwmOutput, flags;  
	float desRoll, roll, magHdg, bankAngle, magBank;
	AhrsInputB ai;
	String toString() const {
		char buf[200];
		snprintf(buf, sizeof(buf), " %d %d %f %f %f %f %f", (int)pwmOutput, (int)flags,
			desRoll, roll, magHdg,  bankAngle, magBank);
		return ai.toString() +  String(buf);
	} 
	LogItemE fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
	LogItemE &operator =(const LogItemD &c) {
		ai = c.ai;
		pwmOutput = c.pwmOutput;
		flags = c.flags;
		desRoll = c.desRoll;
		roll = c.roll;
		magHdg = -1000;
		bankAngle = -1000;
		magBank = -1000;
		return *this;
	}
};

typedef LogItemE LogItem;	


#ifdef UBUNTU
void ESP32sim_convertLogOldToNew(ifstream &i, ofstream &o) {
	LogItemD l; 
	while (i.read((char *)&l, sizeof(l))) {
		LogItemE l2;
		bzero(&l2, sizeof(l2));
		l2 = l;
		o.write((char *)&l2, sizeof(l2));
	}
}
#endif

