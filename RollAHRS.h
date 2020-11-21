#include <cmath>
#include "RunningLeastSquares.h"
#include <math.h>

using namespace std;


#define USE_ACCEL

struct AhrsInput { 
	float sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y;
	float ax, ay, az;  // accel updates removed from imuRead() to speed up loop, drops about 1.2ms 
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float q3, palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f %.3f " /* 1 - 10 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f " /* 11 - 20 */
			"%.3f %.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f",  /* 21 - 27 */ 
		sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, q3, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5TimeStamp);
		return String(buf);	
	 }
	 AhrsInput fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &gpsTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &p, &r, &y, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &q3, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt, &g5TimeStamp);
		return *this;
	}
		 
};


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
	if(d < -180) d += 360;
	if(d > 180) d -= 360;
	return d;
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
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
/*	float magOffX = (-52.0 + 50) / 2,  // + is to the rear  
		  magOffY = (-0.0 + 106) / 2, //  + is left
		  magOffZ = (-82 + 32) / 2; // + is up
*/

	float magOffX = 15,//(-30.0 + 10) / 2,  // + is to the rear  
		  magOffY = 8,//(33 + 72) / 2 - 3, //  + is left
		  magOffZ = (-82 + 32) / 2; // + is up


	float magScaleX = (10.0 - (-30.0)) / 100.0;
	float magScaleY = (72.0 - 33.0) / 100.0;
	
	
//ERO SENSORS gyro 0.858590 0.834096 1.463080 accel 0.171631 -0.085765 -0.037540
	
	float gyrOffX = -0.87, 
		  gyrOffY = -0.99, 
		  gyrOffZ = +0.95;
		  
	float accOffX = +0.171,
		  accOffY = -0.856,
		  accOffZ = -0.037;
		  
public:
	RollAHRS() { 
		gyrZOffsetFit.add(gyrOffZ);
		gyrXOffsetFit.add(gyrOffX);
	}
		
	float g5LastTimeStamp = 0; 
	struct { 
		TwoStageRollingAverage<float,20,20> ax,ay,az,gx,gy,gz;
	} zeroAverages;

	TwoStageRollingAverage<float,20,150>
		gyrZOffsetFit,
		gyrXOffsetFit;


	float gpsBankAngle, magBankAngle, dipBankAngle, dipBankAngle2, magHdg, rawMagHdg, /*bankCorrection,*/ bankAngle;
	float gyroTurnBank, pG;
	float pitchComp = 0, pitchRaw = 0, pitchDrift = 0, pitchCompDriftCorrected = 0;
	float magStability = -1;

	typedef TwoStageRunningLeastSquares<float> TwoStageRLS;
	RunningLeastSquares // all at about 200 HZ */
		//accelPitchFit = RunningLeastSquares(100), 
		//altFit = RunningLeastSquares(200), // 10Hz
		//gpsHdgFit = RunningLeastSquares(200),  // TODO run GPS histories at lower rate 
		magHdgFit = RunningLeastSquares(50), 
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
		gyroDriftFit = RunningLeastSquares(300); // 10HZ 
		
	RollingAverage<float,200> magStabFit;
	RollingAverage<float,50> avgRoll;
	RollingAverage<float,50> avgGZ;
	
	TwoStageRLS 
		magZFit = TwoStageRLS(20, 60),
		magXFit = TwoStageRLS(20, 60),
		magYFit = TwoStageRLS(20, 60),
		magHdgAvg = TwoStageRLS(20,20);
		
	float fakeTimeMs = 0; // period for fake timestamps, 0 to use real time 
	int count = 0;
	AhrsInput prev;
	Windup360 magHdg360;
	float compR = 0, compYH =0, rollG = 0;
	float gyroDrift = 0;
	float gpsPitch = 0, accelPitch = 0;
	float lastGz;
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
		l.mz = -l.mz + magOffZ;
	
#ifdef USE_ACCEL
		l.ax -= accOffX;
		l.ay -= accOffY;
		l.az -= accOffZ;
#endif
	
		if (gyrZOffsetFit.full()) { 
			l.gx -= gyrXOffsetFit.average();
			l.gy -= gyrOffY;
			l.gz -= gyrZOffsetFit.average();
		} else { 
			l.gx -= gyrOffX;
			l.gz -= gyrOffZ;
			l.gy -= gyrOffY;
		}
		avgGZ.add(l.gz);
		
		magHdg = atan(l.my/l.mx) * 180 / M_PI;
		if (l.mx < 0) 
			magHdg += 180;

#if 0
		// calculate bank from magnetic dip effect
		float magTotalMagnitude = sqrt(l.mx*l.mx + l.my*l.my + l.mz*l.mz);
		float yzMagnitude = sqrt(l.my*l.my + l.mz*l.mz);
		float xyMagnitude = sqrt(l.mx*l.mx + l.my*l.my);
		float localDip = 62; //deg
		float levelBankZComponent = sin(localDip * M_PI/180) * magTotalMagnitude;  // constant Z component at all level headings, due to dip
		levelBankZComponent = min(levelBankZComponent, yzMagnitude);
		float levelYZAngle = asin(levelBankZComponent / yzMagnitude) * 180 / M_PI;
		float actualYZAngle = asin(l.mz / yzMagnitude) * 180 / M_PI;
		dipBankAngle = actualYZAngle - levelYZAngle;
		dipBankAngle2 = actualYZAngle - (180 - levelYZAngle);

		magXyAngFit.add(l.sec, atan(l.my/l.mx));
		magZAngFit.add(l.sec, atan(xyMagnitude/l.mz));
#endif

		//magMagnitudeFit.add(l.sec, magTotalMagnitude);
		magZFit.add(l.sec, l.mz);
		magXFit.add(l.sec, l.mx);
		magYFit.add(l.sec, l.my);

		if (magZFit.full()) {
			magStabFit.add(abs(magZFit.slope()) + abs(magXFit.slope()) + abs(magYFit.slope()));
			if (magStabFit.full()) {
				magStability = min(5.0, (double)magStabFit.average()); 
				const float stabThreshold = .2;
				if (magStability < stabThreshold) { 
					//gyrZOffsetFit.add(l.sec, magZFit.stage1.averageY(), max(stabThreshold/2, (stabThreshold/2 - magStability)*100));
					gyrZOffsetFit.add(zeroAverages.gz.average());
					gyrXOffsetFit.add(zeroAverages.gx.average());
				}
			}
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

		float tas = 100; // true airspeed in knots		

		const float compRatio1 = 0.0012	;
		const float driftCorrCoeff1 = pow(1 - compRatio1, 200) * 7;
		
		float zgyrBankAngle = atan(avgGZ.average() * tas / 1091) * 180/M_PI;
		bankAngle = (isnan(zgyrBankAngle) ? 0 : zgyrBankAngle);
		bankAngle *= 1.00;
		//bankAngle = max(-30.0, min(30.0, (double)bankAngle));
		//bankAngle = max(avgRoll.average() - 20.0, min(avgRoll.average() + 20.0, (double)bankAngle));
		//bankAngle =0;
	
		compR = (compR + l.gy * 1.00 /*gyroGain*/ * dt) * (1-compRatio1) + (bankAngle * compRatio1);
		rollG  += l.gy * dt;
		
		/*if (tick10HZ) { 
			gyroDriftFit.add(l.sec, compR - rollG);
			gyroDrift = gyroDriftFit.slope();
		*/
		compYH = compR + gyroDrift * driftCorrCoeff1;
		if (abs(bankAngle) < 4) {			
			gyroDrift += (bankAngle - compYH) * 0.00001;
		}
			
		avgRoll.add(compYH);

		// TODO replace this kinda-help heuristic until we get proper dip correction 
		magHdg += -cos(magHdg / 180 * M_PI) * sin(avgRoll.average() / 180 * M_PI) * 150;

		if (magHdg < 0) magHdg += 360;		
		magHdg360 = magHdg;
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
		
		count++;
		prev = l;
		return compYH;
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
	AhrsInput ai;
	String toString() {
		char buf[200];
		snprintf(buf, sizeof(buf), " %d %d %f %f %f %f %f %f %f %f %f %f", (int)pwmOutput, (int)flags, pidP, pidI, pidD, gainP, gainI, gainD, finalGain,
			desRoll, pitchCmd, roll);
		return ai.toString() +  String(buf);
	} 
	LogItemC fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

typedef LogItemC LogItem;
