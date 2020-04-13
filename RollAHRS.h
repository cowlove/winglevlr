#include <cmath>
#include "RunningLeastSquares.h"
#include <math.h>

using namespace std;

struct LogItem { 
	float sec, hdg, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, q4, gspeed;
	String toString() { 
		static char buf[2048];
		snprintf(buf, sizeof(buf), "%f %.1f %.1f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.1f", 
		sec, hdg, alt, p, r, y, ax, ay, az, gx, gy, gz, mx, my, mz, q1, q2, q3, q4, gspeed);
		return String(buf);	
	 }
	 LogItem fromString(const char *s) { 
		sscanf(s, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
		&sec, &hdg, &alt, &p, &r, &y, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &q1, &q2, &q3, &q4, &gspeed);
		return *this;
	}
		 
};

class RollAHRS {
	float fit360(float h) { 
		while(h <= 0) h += 360;
		while(h > 360) h -= 360;
		return h;
	}
	float magOffX = (-19.0 + 80) / 2,  // + is to the rear  
		  magOffY = (-5.0 + 93) / 2, //  + is left
		  magOffZ = (-80 + 20) / 2; // + is up 
public:
	float gpsBankAngle, magBankAngle, dipBankAngle, magHdg, rawMagHdg, bankCorrection, bankAngle;

	RunningLeastSquares 
		bankFit = RunningLeastSquares(100), 
		gpsHdgFit = RunningLeastSquares(100), 
		magHdgFit = RunningLeastSquares(300), 
		dipBankFit = RunningLeastSquares(100);
		
	float fakeTimeMs = 0; // period for fake timestamps, 0 to use real time 
	int count = 0;
	LogItem prev;
	float lastMagHdg = 0;
	float compYH =0;
	
	float add(LogItem &l) { 
		float dt = 0;
		if (fakeTimeMs > 0)
			l.sec = (count * fakeTimeMs) / 1000.0;	
		if (count > 0 ) 
			dt = l.sec - prev.sec;
		l.mx = -l.mx + magOffX;
		l.my = -l.my + magOffY;
		l.mz = -l.mz + magOffZ;
		magHdg = atan(l.my/l.mx) * 180 / M_PI;
		if (l.mx < 0) 
			magHdg += 180;

		// calculate bank from magnetic dip effect
		float magTotalMagnitude = sqrt(l.mx*l.mx + l.my*l.my + l.mz*l.mz);
		float yzMagnitude = sqrt(l.my*l.my + l.mz*l.mz);
		float xyMagnitude = sqrt(l.mx*l.mx + l.my*l.my);
		float localDip = 62; //deg
		float levelBankZComponent = sin(localDip * M_PI/180) * magTotalMagnitude;  // constant Z component at all level headings, due to dip
		float dipBank = (asin(levelBankZComponent / yzMagnitude) - asin(l.mz / yzMagnitude)) * 180 / M_PI;
		if (!isnan(dipBank))
			dipBankFit.add(l.sec, dipBank);

		// correct magHdg for dip error 
		float y1 = sqrt(yzMagnitude*yzMagnitude - levelBankZComponent*levelBankZComponent);
		float y2 = sqrt(yzMagnitude*yzMagnitude - l.mz * l.mz);
		bankCorrection = (acos(y1/xyMagnitude) - acos(y2/xyMagnitude)) * 180 / M_PI;
		if (isnan(bankCorrection)) bankCorrection = 0;
		
		rawMagHdg = fit360(magHdg);
		magHdg += bankCorrection;
		magHdg = fit360(magHdg);
	
		// prevent discontinuities in hdg, just keep wrapping it around 360,720,1080,...
		if (count > 0) { 			
			float hd = l.hdg - prev.hdg;
			while (hd < -180) hd += 360;
			while (hd > +180) hd -= 360;
			l.hdg = prev.hdg + hd;
			
			hd = magHdg - lastMagHdg;
			while (hd < -180) hd += 360;
			while (hd > +180) hd -= 360;
			float mh = magHdg;
			magHdg = lastMagHdg + hd;
			lastMagHdg = mh;
		}		

		magHdgFit.add(l.sec, magHdg);
		gpsHdgFit.add(l.sec, l.hdg);
		
		if (count % 90000000 == 0) { 
			gpsHdgFit.rebaseX();
			magHdgFit.rebaseX();
		}

		float tas = 120 * .51444; // true airspeed in m/sec.  Units in bank angle may be wrong, why need 140Kts?  
		
		gpsBankAngle = -atan((2*M_PI*tas)/(9.81*360 / gpsHdgFit.slope()))*180/M_PI;
		magBankAngle = -atan((2*M_PI*tas)/(9.81*360 / magHdgFit.slope()))*180/M_PI;
		dipBankAngle = dipBankFit.averageY();
		
		bankAngle = (isnan(gpsBankAngle) ? 0 : (1.0 * gpsBankAngle)) +
						  (isnan(magBankAngle) ? 0 : (0.0 * magBankAngle)) + 
						  (isnan(dipBankAngle) ? 0 : (0.0 * dipBankAngle));
		bankFit.add(l.sec, bankAngle);
		float compRatio = .002; // will depend on sample rate, this works for 50Hz 
		compYH = (compYH + l.gy * 1.45 /*gyroGain*/ * dt) * (1-compRatio) + (bankAngle * compRatio);
				
		count++;
		prev = l;
		return compYH;
	}
};

