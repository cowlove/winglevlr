#ifdef UBUNTU
#include <iostream>
#include <fstream>

#define String std::string
#include "RollAHRS.h"
#include "PidControl.h"

#ifndef LOGITEM_VERSION
#define LOGITEM_VERSION LogItemB
#endif


int main(int argc, char **argv) {
	ifstream i(argv[1], ios_base::in | ios::binary);
	RollAHRS ahrs;
	PidControl pidR(30) /*200Hz*/, pidP(40); /*20HZ*/
	pidR.setGains(7.52, 0.05, 0.11);
	pidR.finalGain = 16.8;
	pidR.maxerr.i = 20;

	pidP.setGains(0.5, 0.005, .7);
	pidP.finalGain = 1.0;
	pidP.maxerr.i = .5;
	
	float pCmd = 0, pulse = 0;
	
	LOGITEM_VERSION l, prev;
	while(i.read((char *)&l, sizeof(l))) { 
		//l.ai.gpsTrack = l.ai.gpsTrackGDL90;
			
		float roll = ahrs.add(l.ai);
		float cmd = pidR.add(roll, roll, l.ai.sec);
		int servo = min(2500.0,(max(500.0, 1500.0 + cmd)));
		pulse = nan("invalid");
		if (floor(l.ai.sec / .05) != floor(prev.ai.sec / .05)) { 
			pCmd = pidP.add(ahrs.pitchCompDriftCorrected, ahrs.pitchCompDriftCorrected, l.ai.sec);
			if (floor(l.ai.sec / 3) != floor(prev.ai.sec / 3)) {
				if (abs(pCmd) > 0.5) { 
					pulse = min(120.0, 50 + (abs(pCmd) - 0.5) * 70 / 1.5);
					pulse *= pCmd / abs(pCmd);
				}
			}
		}
		 
		cout << l.toString() <<" "<<
/*36*/	ahrs.compYH <<" "<< servo <<" "<< ahrs.pitchCompDriftCorrected <<" "<< ahrs.gpsPitch  <<" "<< pCmd  << " " << pulse  / 100 <<" "<< 
/*42*/  ahrs.pitchDrift <<" "<< ahrs.accelPitch <<" "<< ahrs.gyroTurnBank <<" "<< ahrs.pG <<" "<<
    endl;
	    
	    
		 prev = l;
	}
}
#endif
