#ifdef UBUNTU
#include <iostream>
#include <fstream>
#include <cmath>


inline uint64_t millis() { return 0; }
#define TWO_PI (M_PI*2)
#define sq(x) (x*x)
#define radians(x) (x*180/M_PI)
#define degrees(x) (x*M_PI/180)
typedef char byte;




#include "TinyGPS++.h"
#include "TinyGPS++.cpp"


using namespace std;

static TinyGPSPlus gps;
static TinyGPSCustom desiredHeading(gps, "GPRMB", 11);
static TinyGPSCustom vtgCourse(gps, "GPVTG", 1);

int main(int argc, char **argv) {
	ifstream i(argv[1], ios_base::in | ios::binary);
	char l[1];
	while(i.read((char *)&l, sizeof(l))) { 
		gps.encode(l[0]);
		if (vtgCourse.isUpdated()) { 
			float vtg = 0.01 * gps.parseDecimal(vtgCourse.value());

			printf("%f %f %f\n", vtg, (float)gps.course.deg(), (float)gps.speed.knots());
		}
	}
}
#endif
