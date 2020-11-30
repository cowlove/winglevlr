#include <cmath>
#include "RunningLeastSquares.h"
#include <math.h>

using namespace std;


#define USE_ACCEL

struct AhrsInputA { 
	float sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y;
	float ax, ay, az;  
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float q3, palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f %f %f %f" /* 1 - 10 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f" /* 11 - 20 */
			"%.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f",  /* 21 - 27 */ 
		sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, q3, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5TimeStamp);
		return String(buf);	
	 }
	 AhrsInputA fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &gpsTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt, &g5TimeStamp);
		return *this;
	}
};


struct AhrsInputB { 
	float sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt; 
	float ax, ay, az;  
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f " /* 1 - 10 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f " /* 11 - 20 */
			"%.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f",  /* 21 - 27 */ 
		sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5TimeStamp);
		return String(buf);	
	 }
	 AhrsInputB fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &gpsTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt, &g5TimeStamp);
		return *this;
	}
	AhrsInputB &operator =(const AhrsInputA &a) { 
		sec = a.sec;
		gpsTrack = a.gpsTrack;
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
	
class RollAHRS {
public:
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
/*	float magOffX = (-52.0 + 50) / 2,  // + is to the rear  
		  magOffY = (-0.0 + 106) / 2, //  + is left
		  magOffZ = (-82 + 32) / 2; // + is up
*/

/*
	float magOffX = 15.4,//(-30.0 + 10) / 2,  // + is to the rear  
		  magOffY = 8.6,//(33 + 72) / 2 - 3, //  + is left
		  magOffZ = -30;//(-82 + 32) / 2; // + is up
*/

	float magOffX = 17.9;
	float magOffY = 08.3;
	float magOffZ = -30;
	

	float magScaleX = (10.0 - (-30.0)) / 100.0;
	float magScaleY = (72.0 - 33.0) / 100.0;
	float magScaleZ = 1.0;
	
	
//ERO SENSORS gyro 0.858590 0.834096 1.463080 accel 0.171631 -0.085765 -0.037540
	
	float gyrOffX = -0.87, 
		  gyrOffY = -1.0, 
		  gyrOffZ = +1.6;
		  
	float accOffX = +0,
		  accOffY = -0,
		  accOffZ = -0;
		  
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


	float gpsBankAngle, magBankAngle, dipBankAngle, dipBankAngle2, magHdg, rawMagHdg, /*bankCorrection,*/ bankAngle;
	float gyroTurnBank, pG;
	float pitchComp = 0, pitchRaw = 0, pitchDrift = 0, pitchCompDriftCorrected = 0;
	float magStability = -1;
	float hdg;
	bool hdgInitialized = false;
	
	typedef TwoStageRunningLeastSquares<float> TwoStageRLS;
	RunningLeastSquares // all at about 200 HZ */
		//accelPitchFit = RunningLeastSquares(100), 
		//altFit = RunningLeastSquares(200), // 10Hz
		//gpsHdgFit = RunningLeastSquares(200),  // TODO run GPS histories at lower rate 
		//magHdgRawFit = RunningLeastSquares(50),
		//magMagnitudeFit = RunningLeastSquares(300),  
		//magXyAngFit = RunningLeastSquares(10), 
		//magZAngFit = RunningLeastSquares(10),
		//gyZFit = RunningLeastSquares(100),
		//gyXFit = RunningLeastSquares(100),
		//gyYFit = RunningLeastSquares(100),
		//dipBankFit = RunningLeastSquares(100),
		//gyroTurnBankFit = RunningLeastSquares(100),
		//pitchDriftFit = RunningLeastSquares(300),  // 10HZ
		gyroDriftFit = RunningLeastSquares(300), // 10HZ 
		magHdgFit = RunningLeastSquares(50);
		
		
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


	float compRatio1 = 0.00111;
	float driftCorrCoeff1 = 3.4;
	float hdgCompRatio = .00013;  // composite filter ratio for hdg 

	bool valid() { 
		return prev.gpsTrack != -1;
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

		//zeroAverages.ax.add(l.ax);
		//zeroAverages.ay.add(l.ay);
		//zeroAverages.az.add(l.az);
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
			l.gpsTrack = windup360(l.gpsTrack, prev.gpsTrack);
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


		
		float magHdg2 = magHdg;
		if (0) { 
			// recalculate magHdg w/ bank correction 
			// scoring the ra coefficnet with quartile-quartile metric: ./loglook.sh 112 -q3 -range '[50:100]' -stats 21
			// off=36, 1.0=29 2.0=23, 4.0=19, 5.0=18, 6.0=22
			float ra = 1.0 * avgRoll.average() / 180 * M_PI;   
			float my1 = l.my * cos(ra) + l.mz * sin(ra);		
			magHdg2 = atan2(my1, l.mx) * 180 / M_PI;
		}
		if (1) { 
			float ra = 2.8 * avgRoll.average() / 180 * M_PI;   
			float z = sin(67.0*M_PI/180) * cos(ra); 
			float y = sin(magHdg*M_PI/180);
			float y1 = y * cos(ra) - z * sin(ra);
			magHdg =  atan2(y1, cos(magHdg*M_PI/180)) * 180 / M_PI;
		}
		magHdg = angularClosest(magHdg, avgMagHdg.average());
		avgMagHdg.add(magHdg);
		magHdg360 = avgMagHdg.average();

		if (hdgInitialized == false && avgMagHdg.full()) { 
			hdgInitialized = true;
			hdg = magHdg360;
		}

		hdg =  (hdg - (cos(rollRad) * l.gz + sin(abs(rollRad)) * l.gx) * dt) * (1 - hdgCompRatio) + magHdg360 * hdgCompRatio;
		
		magHdg = constrain360(hdg);
		
		if (1) { 
			static int skipped = 0;
			if (skipped > 3 || !magHdgFit.full() || abs(magHdgFit.predict(l.sec) - magHdg) < 5.0) {
				magHdgFit.add(l.sec, magHdg360);
				magHdgAvg.add(l.sec, magHdg360);
				skipped = 0;
			} else {
				skipped++;
			}
			//magHdg = constrain360(magHdgAvg.average());
		}
		
		
		
		float a = atan2(l.ax * M_PI/180, l.az * M_PI/180) * 180 / M_PI;
		avgXzAngle.add(a);
		avgXzAngle2.add(a);
		xzAngle = avgXzAngle.average()- avgXzAngle2.average();;
		
		count++;
		prev = l;
		return compYH;
	}
	RollingAverage<float,100> avgXzAngle;
	RollingAverage<float,800> avgXzAngle2;
	float xzAngle;
	
	float getGyroQuality() {
		return gyroZeroCount.average();
	}
	void reset() {
		pitchRaw = pitchComp = 0;
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
	String toString() {
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


typedef LogItemD LogItem;	


#ifdef UBUNTU
void ESP32sim_convertLogCtoD(ifstream &i, ofstream &o) {
	LogItemC l; 
	while (i.read((char *)&l, sizeof(l))) {
		LogItemD l2;
		l2 = l;
		o.write((char *)&l2, sizeof(l2));
	}
}
#endif

