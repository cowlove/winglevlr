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
	PidControl pid(30);
	pid.setGains(7.52, 0.05, 0.11);
	pid.finalGain = 16.8;
	pid.maxerr.i = 20;
	
	LOGITEM_VERSION l;
	while(i.read((char *)&l, sizeof(l))) { 
		//l.ai.gpsTrack = l.ai.gpsTrackGDL90;
			
		float roll = ahrs.add(l.ai);
		float cmd = pid.add(roll, roll, l.ai.sec);
	
		int servo = min(2500.0,(max(500.0, 1500.0 + cmd)));
		
		cout << ahrs.count << " " << l.ai.sec << " " << l.ai.gpsTrackGDL90 << " " << l.ai.gpsTrackVTG << " " << l.ai.gpsTrack <<" "
/*6*/	<< ahrs.rollG << " " << (ahrs.compYH) << " "	<< ahrs.bankAngle  << " " << l.roll << " " << l.ai.gspeed  << " " 
/*11*/	 << l.ai.mx << " " << l.ai.my << " " << ahrs.magHdgRawFit.averageY()  << " " << ahrs.magHdgFit.averageY()	 << " " << ahrs.bankCorrection << " "
/*16*/	 << ahrs.gpsBankAngle << " " << ahrs.magBankAngle << " " << ahrs.dipBankAngle << " " << ahrs.bankFit.averageY() << " " 
/*20*/	 << (l.pwmOutput*1500/4915) <<" "<< servo <<" "<< pid.err.d<<" " << ahrs.dipBankAngle2
/*23*/	 << l.finalGain <<" "<< cmd << " " << ahrs.gyroDrift << " " 
		 <<  endl;
	}
}
#endif
