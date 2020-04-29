#include <cmath>
#include "RunningLeastSquares.h"
#include <math.h>

using namespace std;

struct AhrsInput { 
	float sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, palt, gspeed;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.1f", 
		sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, palt, gspeed);
		return String(buf);	
	 }
	 AhrsInput fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &gpsTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &p, &r, &y, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &q1, &q2, &q3, &palt, &gspeed);
		return *this;
	}
		 
};

inline static float windup360(float now, float prev) { 
	float hd = now - prev;
	if (hd > 1000000 || hd < -1000000) 
		return now;
	while (hd < -180) hd += 360;
	while (hd > +180) hd -= 360;
	return prev + hd;
}
	
class RollAHRS {
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
	float magOffX = (-19.0 + 80) / 2,  // + is to the rear  
		  magOffY = (-5.0 + 93) / 2, //  + is left
		  magOffZ = (-80 + 20) / 2; // + is up
		  
		  
//ZERO SENSORS gyro 1.385096 1.073140 0.889719 accel 0.109684 0.022846 -0.059723
	
	float gyrOffX = +1.36, 
		  gyrOffY = +1.07, 
		  gyrOffZ = -0.89;
		  
	float accOffX = +0.109,
		  accOffY = +0.022,
		  accOffZ = -0.060;
		  
public:
	struct { 
		TwoStageRollingAverage<float,40,40> ax,ay,az,gx,gy,gz;
	} zeroAverages;



	float gpsBankAngle, magBankAngle, dipBankAngle, dipBankAngle2, magHdg, rawMagHdg, bankCorrection, bankAngle;
	float gyroTurnBank, pG;
	float pitchComp = 0, pitchRaw = 0, pitchDrift = 0, pitchCompDriftCorrected = 0;
	RunningLeastSquares // all at about 200 HZ */
		bankFit = RunningLeastSquares(100), 
		accelPitchFit = RunningLeastSquares(400), 
		altFit = RunningLeastSquares(200), // 10Hz
		gpsHdgFit = RunningLeastSquares(500),  // TODO run GPS histories at lower rate 
		magHdgFit = RunningLeastSquares(50), 
		magHdgRawFit = RunningLeastSquares(50), 
		dipBankFit = RunningLeastSquares(100),
		gyroTurnBankFit = RunningLeastSquares(1000),
		gyroDriftFit = RunningLeastSquares(300), // 10HZ 
		pitchDriftFit = RunningLeastSquares(300);  // 10HZ
		
	float fakeTimeMs = 0; // period for fake timestamps, 0 to use real time 
	int count = 0;
	AhrsInput prev;
	float lastMagHdg = 0;
	float compR = 0, compYH =0, rollG = 0;
	float gyroDrift = 0;
	float gpsPitch = 0, accelPitch = 0;
		
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
			dt = l.sec - prev.sec;
		
		bool tick10HZ = (floor(l.sec / .1) != floor(prev.sec / .1));

		zeroAverages.ax.add(l.ax);
		zeroAverages.ay.add(l.ay);
		zeroAverages.az.add(l.az);
		zeroAverages.gx.add(l.gx);
		zeroAverages.gy.add(l.gy);
		zeroAverages.gz.add(l.gz);
		
		l.mx = -l.mx + magOffX;
		l.my = -l.my + magOffY;
		l.mz = -l.mz + magOffZ;
		
		l.ax -= accOffX;
		l.ay -= accOffY;
		l.az -= accOffZ;
		
		l.gx -= gyrOffX;
		l.gy -= gyrOffY;
		l.gz -= gyrOffZ;
		
		magHdg = atan(l.my/l.mx) * 180 / M_PI;
		if (l.mx < 0) 
			magHdg += 180;

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
		
		/*
		float dipBank = (asin(levelBankZComponent / yzMagnitude) - asin(l.mz / yzMagnitude)) * 180 / M_PI;
		dipBankAngle2 = (M_PI - asin(levelBankZComponent / yzMagnitude) - asin(l.mz / yzMagnitude)) * 180 / M_PI;
		if (!isnan(dipBank))
			dipBankFit.add(l.sec, dipBank);
		*/
		// correct magHdg for dip error 
		
		float y1 = sqrt(yzMagnitude*yzMagnitude - levelBankZComponent*levelBankZComponent);
		float y2 = sqrt(yzMagnitude*yzMagnitude - l.mz * l.mz);
		y1 = min(y1, xyMagnitude);
		y2 = min(y2, xyMagnitude);
		if (y2 > xyMagnitude) y2 = xyMagnitude;
		bankCorrection = (acos(y1/xyMagnitude) - acos(y2/xyMagnitude)) * 180 / M_PI;
		//printf("actualZ %f levelZ %f y1:%f y2:%f yzM:%f xyM:%f mz:%f lzBC:%f\n", actualZAngle, levelZAngle, y1, y2, yzMagnitude, xyMagnitude, l.mz, levelBankZComponent);
		if (isnan(bankCorrection)) {
			bankCorrection = 0;
		}
		
		rawMagHdg = fit360(magHdg);
		magHdg += bankCorrection;
		magHdg = fit360(magHdg);
	
		// prevent discontinuities in hdg, just keep wrapping it around 360,720,1080,...
		if (count > 0) { 	
			l.gpsTrack = windup360(l.gpsTrack, prev.gpsTrack);
			magHdg = windup360(magHdg, lastMagHdg);
			lastMagHdg = magHdg;
		}

		magHdgRawFit.add(l.sec, rawMagHdg);
		magHdgFit.add(l.sec, magHdg);
		gpsHdgFit.add(l.sec, l.gpsTrack);
		
		if (count % 3217 == 0) { 
			gpsHdgFit.rebaseX();
			magHdgFit.rebaseX();
		}

		float tas = 90 * .51444; // true airspeed in m/sec.  Units in bank angle may be wrong, why need 140Kts?  
		
		gpsBankAngle = -atan((2*M_PI*tas)/(9.81*360 / gpsHdgFit.slope()))*180/M_PI;
		magBankAngle = -atan((2*M_PI*tas)/(9.81*360 / magHdgFit.slope()))*180/M_PI;
		//dipBankAngle = dipBank; //dipBankFit.averageY();
		
		bankAngle = (isnan(gpsBankAngle) ? 0 : (1.0 * gpsBankAngle)) +
						  (isnan(magBankAngle) ? 0 : (0.0 * magBankAngle)) + 
						  (isnan(dipBankAngle) ? 0 : (0.0 * dipBankAngle));
		bankFit.add(l.sec, bankAngle);
		float compRatio1 = .0006; // will depend on sample rate, this works for 50Hz 
		compR = (compR + l.gy * 1.00 /*gyroGain*/ * dt) * (1-compRatio1) + (bankAngle * compRatio1);
		rollG  += l.gy * dt;
		if (tick10HZ) { 
			gyroDriftFit.add(l.sec, compR - rollG);
			gyroDrift = gyroDriftFit.slope();
		}
		compYH = compR + gyroDrift * 8.8;
		
	
		if (tick10HZ) { 
			altFit.add(l.sec, l.alt);
		}
		//accelPitch = atan(-l.ax / sqrt(l.ay * l.ay + l.az * l.az)) * 180 / M_PI;
		//float ap = atan(-l.ay / sqrt(l.ax * l.ax + l.az * l.az)) * 180 / M_PI;
		float ap = atan(l.ay / l.az) * 180 / M_PI;
		accelPitchFit.add(l.sec, ap);
		accelPitch = accelPitchFit.predict(l.sec);
		
		gpsPitch = asin((altFit.slope() / .5144) / 90) * 180  / M_PI; 
		if (isnan(gpsPitch)) gpsPitch = 0;
		gpsPitch = min(5.0, max(-5.0, (double)gpsPitch));
	
		float gyroTurnBank = atan(l.gx/l.gz) * 180 / M_PI;
		float gtmag = sqrt(l.gx * l.gx + l.gz * l.gz);
		//pG = sin(abs(compYH) * M_PI / 180) * gtmag;
		pG = l.gx - sin(abs(compYH) * M_PI / 180) * gtmag;
		
		const float compRatio2 = 0.0015	;
		const float driftCorrCoeff = pow(1 - compRatio2, 200) * 7;
		pitchRaw += pG * dt;
		//pitchComp = (pitchComp + pG * dt) * (1-compRatio2) + (accelPitch  * compRatio2/2) + (gpsPitch  * compRatio2/2);
		pitchComp = (pitchComp + pG * dt) * (1-compRatio2) + (accelPitch  * compRatio2) ;
		
		if (tick10HZ) { 
			pitchDriftFit.add(l.sec, pitchComp - pitchRaw);
			pitchDrift = pitchDriftFit.slope();
		}
		pitchCompDriftCorrected = pitchComp;
		//pitchCompDriftCorrected += pitchDrift * driftCorrCoeff;
		
		count++;
		prev = l;
		return compYH;
	}
	
	void reset() {
		pitchRaw = pitchComp = 0;
		pitchDriftFit.reset();
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
	short pwmOutput, flags;  // 23 24
	float pidP, pidI, pidD;  // 25 26 27 
	float gainP, gainI, gainD, finalGain; // 28 29 30 31 
	float desRoll, pitchCmd, roll; // 32 33 34
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
