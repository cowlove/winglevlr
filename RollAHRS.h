#include <cmath>
#include <math.h>
#include "RollingLeastSquares.h"

using namespace std;

#ifndef AHRS_RATE
#define AHRS_RATE 50
#endif
#define AHRS_RATE_SCALE(x) ((x) * AHRS_RATE / 200.0)
#define AHRS_RATE_INV_SCALE(x) ((x) * 200.0 / AHRS_RATE)
#define AHRS_RATE_CR_SCALE(x) (1 - pow(1 - x, 200.0 / AHRS_RATE))

#define USE_ACCEL

struct AhrsInputA { 
	float sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, p, r, y;
	float ax, ay, az;  
	float gx, gy, gz, mx, my, mz, dtk, g5Track;
	float q3, palt, gspeed, g5Pitch = 0, g5Roll = 0, g5Hdg = 0, g5Ias = 0, g5Tas = 0, g5Palt = 0, g5TimeStamp = 0;
	String toString() { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %f %f %f %f %f %f %f %f %f" /* 1 - 10 */
			"%f %f %f %f %f %f %f %f %f %f %f" /* 11 - 20 */
			"%f %f %f %f %f %f %f %f %f",  /* 21 - 27 */ 
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




struct AhrsInputC { 
	float sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt;  // 1-6 
	float ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track; // 1-17
	float palt, gspeed, g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5Slip; // 19-26
	float ubloxHdg, ubloxHdgAcc, ubloxAlt, ubloxGroundSpeed; // 27-30
	double lat, lon; // 31,32
	String toString() const { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %f %f %f %f %f %f %f " /* 1 - 8 */
			"%f %f %f %f %f %f %f %f %f %f " /* 9-18 */
			"%f %f %f %f %f %f %f %f %f %f "  /* 21 - 28 */ 
			"%f %f %lf %lf", 						// 29-32
		sec, selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, ax, ay, az, gx, gy, gz, mx, my, mz, dtk, g5Track, palt, gspeed, 
		g5Pitch, g5Roll, g5Hdg, g5Ias, g5Tas, g5Palt, g5Slip, 
		ubloxHdg, ubloxHdgAcc, ubloxAlt, ubloxGroundSpeed, lat, lon);
		return String(buf);	
	 }
	 AhrsInputC fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %lf %lf", 
		&sec, &selTrack, &gpsTrackGDL90, &gpsTrackVTG, &gpsTrackRMC, &alt, &ax, &ay, &az, &gx, &gy, 
		&gz, &mx, &my, &mz, &dtk, &g5Track, &palt, &gspeed, &g5Pitch, &g5Roll, &g5Hdg, &g5Ias, &g5Tas, &g5Palt,
		 &ubloxHdg, &ubloxHdgAcc, &ubloxAlt, &ubloxGroundSpeed, &lat, &lon);
		return *this;
	}
	AhrsInputC &operator =(const AhrsInputB &a) { 
		sec = a.sec;
		selTrack = a.selTrack;
		gpsTrackGDL90 = a.gpsTrackGDL90;
		gpsTrackVTG = a.gpsTrackVTG;
		gpsTrackRMC = a.gpsTrackRMC;
		ax = a.ax; ay = a.ay; az = a.az;
		gx = a.gx; gy = a.gy; gz = a.gz;
		mx = a.mx; my = a.my; mz = a.mz;
		palt = a.palt; gspeed = a.gspeed; g5Pitch = a.g5Pitch; g5Roll = a.g5Roll; 
		g5Hdg = a.g5Hdg; g5Ias = a.g5Ias; g5Tas = a.g5Tas; g5Palt = a.g5Palt; 
		lat  = lon =	ubloxHdg = ubloxHdgAcc = ubloxAlt = ubloxGroundSpeed = 0;
		return *this;
	}
		
};



typedef AhrsInputC AhrsInput; 



struct AuxMpuData {
	float ax, ay, az, gx, gy, gz, mx, my, mz; 
	String toString() { 
		return String(strfmt("MPU %f %f %f %f %f %f %f %f %f", ax, ay, az, gx, gy, gz, mx, my, mz).c_str());
	}
	bool fromString(const char *s) { 
		return sscanf(s, "MPU %f %f %f %f %f %f %f %f %f", &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz) == 9;
	}
};// auxMPU;

inline float constrain360(float a) { 
	if (abs(a) < 10000) { 
		while(a <= 0) a += 360;
		while(a > 360) a -= 360;
	}
	return a;
}

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
	float defaultCr = AHRS_RATE_CR_SCALE(0.001);
	float age = 5.0;
public:
	float value, prevMainValue, bestCr, bestAux, bestAuxPri, expires, priority;
	void reset() { first = true; } 
	float calculate(float now, float v) {
		if (first || abs(now - expires) > 20) {
			prevMainValue = value = v;
			bestAuxPri = priority = 0;
			first = false;
			expires = now + age;
		}
		// 
		if(now > expires || bestAuxPri >= priority) { 
			if (bestAuxPri == 0) {
				bestAux = v;
				bestCr = defaultCr;
			}
			expires = now + age;
			bestAux = angularClosest(bestAux, value);
			priority = bestAuxPri;
			value = (1 - bestCr) * (value) + (bestCr * bestAux);
		} 
		value += angularDiff(v - prevMainValue);	
		bestAuxPri = 0;
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
	bool rotate180 = false;
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
	MultiCompFilter mComp;
	
	float magOffX = 27;
	float magOffY = 24;
	float magOffZ = 5;

	
	float magScaleX = 1.0;
	float magScaleY = 1.0;
	float magScaleZ = 1.0;

//ZERO SENSORS gyro 1.567909 0.214304 -0.009128 accel 0.006692 0.003586 2.031624
//ZERO SENSORS gyro 0.476272 1.088531 -0.026562 accel -0.024028 -0.031612 2.004098
float gyrOffX=+1.357, gyrOffY=-0.028, gyrOffZ=-1.816, accOffX=-0.037, accOffY=-0.072, accOffZ=+0.013;

	//float gyroOffX = +0.476, gyrOffY = +1.080, gyrOffZ = -0.026;
	//float accOffX = -0.024, accOffY = -0.031, accOffZ = +0.005;


	float compRatio1 = AHRS_RATE_CR_SCALE(0.00072);  // roll comp filter ratio 
	float driftCorrCoeff1 = 2.80; // how fast to add in drift correction
	float hdgCompRatio = AHRS_RATE_CR_SCALE(.00025);  // composite filter ratio for hdg 
	float magDipConstant = 2.11; // unexplained correction factor for bank angle in dip calcs
	float magBankTrimCr = AHRS_RATE_INV_SCALE(0.00005);
	float magBankTrimMaxBankErr = 12;
	float bankAngleScale = 1.10;
	float debugVar = 0.0;
	float gXdecelCorrelation = 1.00;
	float compRatioPitch = .0202;//AHRS_RATE_CR_SCALE(0.012);
	float pitchRaw =0;
	
	RollAHRS() { 
		//gyrYOffsetFit.add(gyrOffY);
		//gyrZOffsetFit.add(gyrOffZ);
		//gyrXOffsetFit.add(gyrOffX);
	}
		
	int zeroSampleCount = 0; 
	float g5LastTimeStamp = 0; 
	struct { 
		TwoStageRollingAverage<float,20,(int)AHRS_RATE_SCALE(20)> ax,ay,az,gx,gy,gz;
	} zeroAverages;

	TwoStageRollingAverage<float,20,(int)AHRS_RATE_SCALE(150)>
		gyrZOffsetFit,
		gyrXOffsetFit,
		gyrYOffsetFit;

	float magBank, magBankTrim = 0.0;
	float gpsBankAngle, magBankAngle, dipBankAngle, dipBankAngle2, magHdg, rawMagHdg, /*bankCorrection,*/ bankAngle;
	float gyroTurnBank, pG;
	float pitch = 0;
	float magStability = -1;
	float hdg;
	bool hdgInitialized = false;

	typedef TwoStageRollingLeastSquares<float> TwoStageRLS;
	RollingLeastSquares 
		gyroDriftFit = RollingLeastSquares(300), // 10HZ 
		magHdgFit = RollingLeastSquares(50), // 10Hz
		speedFit = RollingLeastSquares(25); // 50Hz

	RollingAverage<float, (int)AHRS_RATE_SCALE(200)> magStabFit;
	RollingAverage<float, (int)AHRS_RATE_SCALE(50)> avgRoll;
	RollingAverage<float, (int)AHRS_RATE_SCALE(100)> avgPitch;
	RollingAverage<float, (int)AHRS_RATE_SCALE(20)> avgMagHdg;
	RollingAverage<float, (int)AHRS_RATE_SCALE(200)> avgGZ, avgGX, avgAX, avgAZ, avgAY;
	RollingAverage<float, 200> gyroZeroCount;
	
	TwoStageRLS 
		magZFit = TwoStageRLS(20, (int)AHRS_RATE_SCALE(20)),
		magXFit = TwoStageRLS(20, (int)AHRS_RATE_SCALE(20)),
		magYFit = TwoStageRLS(20, (int)AHRS_RATE_SCALE(20));
		
	float fakeTimeMs = 0; // period for fake timestamps, 0 to use real time 
	int count = 0;
	AhrsInput prev;
	float compR = 0, compYH =0, rollG = 0;
	float gyroDrift = 0;
	float gpsPitch = 0, accelPitch = 0;
	float lastGz, magCorr = 0;
	float speedDelta;
	bool valid() { 
		return prev.selTrack != -1;
	}
	
	std::string zeroSensors() { 
		gyrOffX = zeroAverages.gx.average(); 
		gyrOffY = zeroAverages.gy.average();
		gyrOffZ = zeroAverages.gz.average();
		accOffX = zeroAverages.ax.average();
		accOffY = zeroAverages.ay.average();
		accOffZ = zeroAverages.az.average() - 1.0;
		return strfmt("float gyroOffX=%+05.3f, gyrOffY=%+05.3f, gyrOffZ=%+05.3f, accOffX=%+05.3f, accOffY=%+05.3f, accOffZ=%+05.3f;\n", 
			gyrOffX, gyrOffY, gyrOffZ, accOffX, accOffY, accOffZ); 
	}
	
	float add(const AhrsInput &i_NODONTUSE) {
		AhrsInput l(i_NODONTUSE);
		float dt = 0;
		if (fakeTimeMs > 0)
			l.sec = (count * fakeTimeMs) / 1000.0;	
		if (count > 0 ) 
			dt = min((float).1, l.sec - prev.sec);
		
		bool tick10HZ = (floor(l.sec / .1) != floor(prev.sec / .1));
		bool tick50HZ = (floor(l.sec / .02) != floor(prev.sec / .02));

		zeroAverages.ax.add(l.ax); // FULL RATE
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

		if (rotate180) { 
			l.ax *= -1;
			l.ay *= -1;
			l.mx *= -1;
			l.my *= -1;
			l.gx *= -1;
			l.gy *= -1;
		}

		// gyroOffsets are-post rotation.  All other constant offsets are pre-rotation
		if (gyrZOffsetFit.full()) { 
			l.gx -= gyrXOffsetFit.average();
			l.gy -= gyrYOffsetFit.average();
			l.gz -= gyrZOffsetFit.average();
		} else { 
			l.gx -= gyrOffX;
			l.gz -= gyrOffZ;
			l.gy -= gyrOffY;
		}

		avgGX.add(l.gx); // FULL RATE
		avgGZ.add(l.gz);
		avgAX.add(l.ax);
		avgAY.add(l.ay);
		avgAZ.add(l.az);
		
		//magMagnitudeFit.add(l.sec, magTotalMagnitude);
		magZFit.add(l.sec, l.mz); // FULL RATE 
		magXFit.add(l.sec, l.mx);
		magYFit.add(l.sec, l.my);

		if (magZFit.full()) {
			magStabFit.add(abs(magZFit.slope()) + abs(magXFit.slope()) + abs(magYFit.slope()));
			if (magStabFit.full()) {
				magStability = min(15.0, (double)magStabFit.average()); 
				const float stabThreshold = 0.0;
				if (magStability < stabThreshold) { 
					//gyrZOffsetFit.add(l.sec, magZFit.stage1.averageY(), max(stabThreshold/2, (stabThreshold/2 - magStability)*100));
					gyrZOffsetFit.add(zeroAverages.gz.average()); // FULL RATE
					gyrXOffsetFit.add(zeroAverages.gx.average());
					gyrYOffsetFit.add(zeroAverages.gy.average());
					zeroSampleCount++;
				}
			}
		}
		if (tick50HZ) { 
			gyroZeroCount.add(zeroSampleCount);
			speedFit.add(l.sec, l.g5Ias);
			zeroSampleCount = 0;
		}
					
		// prevent discontinuities in hdg, just keep wrapping it around 360,720,1080,...
		if (count > 0) { 	
			l.selTrack = windup360(l.selTrack, prev.selTrack);
		}

		if (count % 3217 == 0) { 
			//gpsHdgFit.rebaseX();
			magHdgFit.rebaseX();
			gyroDriftFit.rebaseX();
		}

		float tas = 100; //l.g5Ias; // true airspeed in knots		

		//const float driftCorrCoeff1 = pow(1 - compRatio1, 200) * 9;
		
		//printf("DEBUG %f\n", driftCorrCoeff1);
		float rollRad = avgRoll.average() * M_PI / 180;
		/// TODO - need to consider complete yaw rate corrected for bankAngle, not only avgGZ. 
		float zgyrBankAngle = atan((cos(rollRad) * avgGZ.average() + (sin(abs(rollRad)) * avgGX.average())) * tas / 1091) * 180/M_PI;
		bankAngle = (isnan(zgyrBankAngle) ? 0 : zgyrBankAngle);
		bankAngle *= bankAngleScale;
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
		
		speedDelta = speedFit.slope() * MPS_PER_KNOT;
		accelRoll = RAD2DEG(atan2(avgAX.average(), avgAZ.average()));
		rollRad = DEG2RAD(avgRoll.average() + accelRoll);
		float pitchRad = DEG2RAD(avgPitch.average());
		accelPitch = -RAD2DEG(atan2(cos(rollRad) * avgAY.average() - sin(rollRad) * avgAX.average(), avgAZ.average()));
		//accelPitch = 0;
		//accelRoll = 0;
		
		double decelAng = RAD2DEG(atan2(speedDelta, 9.8)) * gXdecelCorrelation;
		decelAng = min(8.0, max(-8.0, decelAng));
		accelPitch -= decelAng; 
		
		float pG = cos(rollRad) * l.gx - sin(rollRad) * l.gz;//  - speedDelta * gXdecelCorrelation;
		pitchRaw = (pitchRaw + pG * 1.00 /*gyroGain*/ * dt) * (1-compRatioPitch) + (accelPitch * compRatioPitch);

		pitch = pitchRaw - /*HACK*/debugVar * (sin(abs(2.2 * rollRad)) * pitchRaw);
	 	pitch = isnan(pitch) ? 0 : pitch;
	 	pitch = isinf(pitch) ? 0 : pitch;

		avgPitch.add(pitch);
		pitch = avgPitch.average();

	
		float rmx = cos(pitchRad) * l.mx - sin(pitchRad) * l.mz;
		float rmz = cos(pitchRad) * l.mz + sin(pitchRad) * l.mx;
		float rmy = cos(rollRad) * l.my + sin(rollRad) * rmz;
		      rmz = cos(rollRad) * rmz - sin(rollRad) * l.my;
		//rmx = l.mx;
		//rmy = l.my;
		float h = RAD2DEG(atan2(rmy, rmx));
		
		
		h = angularClosest(h, avgMagHdg.average());
		avgMagHdg.add(h);

		// startup initialization of "hdg" filter result 
		if (hdgInitialized == false && avgMagHdg.full()) { 
			hdgInitialized = true;
			hdg = avgMagHdg.average();
		}

		float rgz = l.gz * cos(pitchRad) + l.gy * sin(pitchRad);
		hdg =  (hdg - (cos(rollRad) * rgz - sin(abs(rollRad)) * l.gx) * dt) * (1 - hdgCompRatio) + avgMagHdg.average() * hdgCompRatio;		
		magHdg = constrain360(mComp.calculate(l.sec, hdg));		
		//magHdg = constrain360(hdg);
		//magHdg = constrain360(magHdg);

		count++;
		prev = l;

		return compYH;
	}	

	float getGyroQuality() {
		return gyroZeroCount.average();
	}

	float accelRoll;
	float dummy[10];
	void reset() {
		mComp.reset();
		gyroDriftFit.reset();
		magHdgFit.reset();
		magStabFit.reset();
		avgRoll.reset();
		avgMagHdg.reset();
		avgGZ.reset();
		avgGX.reset();
		avgAZ.reset();
		avgAX.reset();
		gyroZeroCount.reset();
		magZFit.reset();
		magXFit.reset();
		magYFit.reset();
	}
};

struct LogItemA {
	short pwmOutputRoll, pwmOutputPitch, flags;  
	float desRoll, roll, magHdg, bankAngle, magBank, pitch, spare1, spare2;
	AhrsInputB ai;
	String toString() const {
		char buf[200];
		snprintf(buf, sizeof(buf), " %d %d %d %f %f %f %f %f %f %f %f", (int)pwmOutputRoll, (int)pwmOutputPitch, (int)flags,
			desRoll, roll, magHdg,  bankAngle, magBank, pitch, spare1, spare2);
		return ai.toString() +  String(buf);
	} 
	LogItemA fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
};

struct LogItemB {
	short pwmOutput0, pwmOutput1, flags;  
	float desRoll, roll, magHdg, bankAngle, magBank, pitch, desAlt, desPitch;
	struct AuxMpuData auxMpu;
	AhrsInputB ai;
	String toString() const {
		char buf[200];
		const AuxMpuData &a = auxMpu;
		snprintf(buf, sizeof(buf), " %d %d %d %f %f %f %f %f %f %f %f "
		" %f %f %f %f %f %f %f %f %f ",
		(int)pwmOutput0, (int)pwmOutput1, (int)flags, desRoll, roll, magHdg,  bankAngle, magBank, pitch, desAlt, desPitch,
		 a.ax, a.ay, a.az, a.gx, a.gy, a.gz, a.mx, a.my, a.mz);
		return ai.toString() + String(buf);
	} 
	LogItemB fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
	LogItemB &operator =(const LogItemA &c) {
		ai = c.ai;
		pwmOutput0 = c.pwmOutputRoll;
		flags = c.flags;
		desRoll = c.desRoll;
		roll = c.roll;
		magHdg = c.magHdg;
		bankAngle = c.bankAngle;
		magBank = c.magBank;
		pwmOutput1 = pitch = desAlt = desPitch  = -1000;
		return *this;
	}
};

struct LogItemC {
	short pwmOutput0, pwmOutput1, flags;  
	float desRoll, roll, magHdg, bankAngle, xte, pitch, desAlt, desPitch;
	//struct AuxMpuData auxMpu;
	AhrsInputC ai;
	String toString() const {
		char buf[200];
		//const AuxMpuData &a = auxMpu;
		snprintf(buf, sizeof(buf), " %d %d %d %f %f %f %f %f %f %f "
		//" %f %f %f %f %f %f %f %f %f "
		,(int)pwmOutput0, (int)pwmOutput1, (int)flags, desRoll, roll, magHdg, xte, pitch, desAlt, desPitch
		// ,a.ax, a.ay, a.az, a.gx, a.gy, a.gz, a.mx, a.my, a.mz
		);
		return ai.toString() + String(buf);
	} 
	LogItemC fromString(const char *s) { 
		ai.fromString(s);
		return *this;
	}
	LogItemC &operator =(const LogItemB &c) {
		ai = c.ai;
		pwmOutput0 = c.pwmOutput0;
		flags = c.flags;
		desRoll = c.desRoll;
		roll = c.roll;
		magHdg = c.magHdg;
		bankAngle = c.bankAngle;
		//magBank = c.magBank;
		pwmOutput1 = pitch = desAlt = desPitch  = -1000;
		return *this;
	}
};




// idea for compact logItem
template <class T, int V> 
class ScaledStorage {
	public:
		T v;
		ScaledStorage() {}
		ScaledStorage &operator =(double x) { v = x * V; return *this; }
		operator double() const { return v * 1.0 / V; }
		//operator float() { return v * 1.0 / V; }
};


/* Invesitgate changes in new packed structure with: 
  make winglevlr_ubuntu  && 
  	./winglevlr_ubuntu --logConvert logs.OLD/AHRSD015.DAT logs/AHRSD015.DAT && 
	./winglevlr_ubuntu --replay logs/AHRSD015.DAT --log + | grep LOG > logs/015-N.plog && 
	./loglook.sh XX -file logs/015-O.plog -roll -pitch -maghdg -file logs/015-N.plog -roll -pitch -maghdg

	Packed log structure 60% size of full logItem  
*/

struct AhrsPackedStructure { 
	typedef ScaledStorage<int16_t, 90> Heading;
	typedef ScaledStorage<int16_t, 100> MagResult; 
	typedef ScaledStorage<int32_t, 10000000> LatLon;
	typedef ScaledStorage<uint32_t, 10000> Time;
	typedef ScaledStorage<int32_t, 50000> GyroResult;
	typedef ScaledStorage<int16_t, 500> AccResult;
	typedef ScaledStorage<int16_t, 100> Altitude;
	typedef ScaledStorage<int16_t, 100> Knots;
	typedef ScaledStorage<int16_t, 100> SmallAngle; 
	typedef ScaledStorage<int16_t, 100> SmallDistance; 
};

struct AhrsInputPacked : public AhrsPackedStructure { 
	Time sec;
	Heading selTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, ubloxHdg, ubloxHdgAcc, dtk, g5Track, g5Hdg;
	LatLon lat, lon;
	AccResult ax, ay, az;
	GyroResult gx, gy, gz;
	MagResult mx, my, mz;
	Altitude alt, palt, g5Palt, ubloxAlt; 
	Knots gspeed, g5Ias, g5Tas, ubloxGroundSpeed;
	SmallAngle g5Pitch, g5Roll, g5Slip;
	//int32_t spare1, spare2; 
	void pack(const AhrsInputC &a) { 
		AhrsInputPacked &b = *this;
		b.sec = a.sec;
		b.selTrack = a.selTrack; b.gpsTrackGDL90 = a.gpsTrackGDL90; b.gpsTrackVTG = a.gpsTrackVTG;
		b.gpsTrackRMC = a.gpsTrackRMC; b.dtk = a.dtk; b.g5Track = a.g5Track; b.g5Hdg = a.g5Hdg;
		  
		b.ax = a.ax; b.ay = a.ay; b.az = a.az;
		b.gx = a.gx; b.gy = a.gy; b.gz = a.gz;
		b.mx = a.mx; b.my = a.my; b.mz = a.mz;
		b.alt = a.alt; b.palt = a.palt; b.g5Palt = a.g5Palt; 
		b.gspeed = a.gspeed; b.g5Ias = a.g5Ias; b.g5Tas = a.g5Tas;
		b.g5Pitch = a.g5Pitch; b.g5Roll = a.g5Roll; b.g5Slip = a.g5Slip; 
		b.lat = a.lat; b.lon = a.lon; b.ubloxHdg = a.ubloxHdg; b.ubloxHdgAcc = a.ubloxHdgAcc;
		b.ubloxAlt = a.ubloxAlt; b.ubloxGroundSpeed = a.ubloxGroundSpeed;
		//spare1 = spare2 = 0;
	}
	//void unpack(AhrsInputC &b) { 
	//	AhrsInputPacked &a = *this;
	//}
	AhrsInputPacked &operator =(const AhrsInputC &a) { 
		pack(a);
		return *this;
	}
	String toString() const { 
		static char buf[512];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.1f %.1f %.1f %.3f %.3f " /* 1 - 8 */
			"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f " /* 9-18 */
			"%.3f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.3f %.3f "  /* 21 - 29 */ 
			"%.1f %.1f %+12.8lf %+12.8lf", 						// 29-32
		(double)sec, (double)selTrack, (double)gpsTrackGDL90, (double) gpsTrackVTG, (double) gpsTrackRMC,
		(double) alt, (double)ax,(double) ay,(double) az, (double)gx,(double) gy, (double)gz,(double) mx, 
		(double)my, (double)mz, (double)dtk, (double)g5Track, (double)palt, (double)gspeed, 
		(double)g5Pitch, (double)g5Roll, (double)g5Hdg, (double)g5Ias, (double)g5Tas, (double)g5Palt, (double)g5Slip,
		(double)ubloxHdg, (double)ubloxHdgAcc, (double)ubloxAlt, (double)ubloxGroundSpeed, (double)lat, (double)lon);
		return String(buf);	
	}
};

struct LogItemPacked : public AhrsPackedStructure { 
	uint16_t pwmOutput0, pwmOutput1, flags;
	SmallAngle desRoll, roll, bankAngle, pitch, desPitch;
	Altitude desAlt; 
	Heading magHdg;
	SmallDistance xte;
	AhrsInputPacked ai;
	LogItemPacked &operator=(const LogItemC &l) { 
		ai = l.ai;
		pwmOutput0 = l.pwmOutput0;
		pwmOutput1 = l.pwmOutput1;
		flags = l.flags;
		//desRoll, roll, bankAngle, pitch, desPitch;
		desRoll = l.desRoll; roll = l.roll; bankAngle = l.bankAngle; pitch = l.pitch; desPitch = l.desPitch;
		desAlt = l.desAlt; magHdg = l.magHdg; xte = l.xte; 
		return *this;
	}

	String toString() const { 
		char buf[200];
		//const AuxMpuData &a = auxMpu;
		snprintf(buf, sizeof(buf), " %d %d %d %f %f %f %f %f %f %f "
		//" %f %f %f %f %f %f %f %f %f "
		,(int)pwmOutput0, (int)pwmOutput1, (int)flags, (double)desRoll, (double)roll, (double)magHdg, 
		(double)xte, (double)pitch, (double)desAlt, (double)desPitch
		// ,a.ax, a.ay, a.az, a.gx, a.gy, a.gz, a.mx, a.my, a.mz
		);
		return ai.toString() + String(buf);
		
	} 
};

//typedef LogItemPacked LogItem;	
typedef LogItemC LogItem;	

	
inline void testPack() { 
	AhrsInputPacked p, q;
	AhrsInputC a;
	p.pack(a);
	//p.unpack(a);
	q = p;
}

#ifdef UBUNTU
void Csim_convertLogOldToNew(ifstream &i, ofstream &o) {
	LogItemC l; 
	while (i.read((char *)&l, sizeof(l))) {
		LogItemPacked l2;
		bzero(&l2, sizeof(l2));
		l2 = l;
		o.write((char *)&l2, sizeof(l2));
	}
}
#endif
