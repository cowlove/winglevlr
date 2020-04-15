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
	pid.setGains(7.52, 0, .105);
	
	LOGITEM_VERSION l;
	while(i.read((char *)&l, sizeof(l))) { 
		float roll = ahrs.add(l.ai);
		float cmd = pid.add(roll, roll, l.ai.sec);
	
		int servo = min(2500.0,(max(500.0, 1500.0 + cmd)));
		
		cout << ahrs.count << " " << l.ai.sec << " " << 0 << " " << l.ai.hdg << " " << 0 << " " << 0  << " " << (ahrs.compYH) << " " 
		 << ahrs.bankAngle /*8*/ << " " << 0 << " " << l.ai.gspeed  << " " 
/*11*/	 << l.ai.mx << " " << l.ai.my << " " << ahrs.magHdgRawFit.averageY()  << " " << ahrs.magHdgFit.averageY()	 << " " << ahrs.bankCorrection << " "
/*16*/	 << ahrs.gpsBankAngle << " " << ahrs.magBankAngle << " " << ahrs.dipBankAngle << " " << ahrs.bankFit.averageY() << " " 
/*20*/	 << servo <<" "<< pid.err.p <<" "<< pid.err.d<<" "
#ifdef LOGITEM_VERSION_C
/*23*/	 << l.finalGain <<" "<< cmd
#endif
		 <<  endl;
	}
}
