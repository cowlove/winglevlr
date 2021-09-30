#include <cstdint>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
#include <functional>
#include <sstream>
#include <ios>
#include <iostream>

namespace WaypointNav {

    struct LatLon {
        LatLon(double la, double lo) : lat(la), lon(lo) {}
        LatLon() {} 
        double lat;
        double lon; 
        LatLon toRadians() { 
            return LatLon(lat * M_PI/180, lon * M_PI/180); 
        } 
        LatLon toDegrees() { 
            return LatLon(lat * 180/M_PI, lon *180/M_PI); 
        } 
    };

    double bearing(LatLon a, LatLon b) { // returns relative brg in degrees
        a = a.toRadians();
        b = b.toRadians();
        double dL = b.lon - a.lon;
        double x = cos(b.lat) * sin(dL);
        double y = cos(a.lat) * sin(b.lat) - sin(a.lat) * 	cos(b.lat)* cos(dL);	
        double brg = atan2(x, y) * 180/M_PI;
        if (brg < 0) brg += 360;
        return brg;
    }
        
    double distance(LatLon p1, LatLon p2) { // returns distance in meters 
        p1 = p1.toRadians();
        p2 = p2.toRadians();

        const double R = 6371e3; // metres
        double dlat = p2.lat - p1.lat;
        double dlon = p2.lon - p1.lon;
        
        double a = sin(dlat/2) * sin(dlat/2) +
            cos(p1.lat) * cos(p2.lat) *
            sin(dlon/2) * sin(dlon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        return c * R;
    }

    double crossTrackErr(LatLon start, LatLon end, LatLon pos) { 
        double a = bearing(start, end);
        double b = bearing(start, pos);
        return sin(DEG2RAD(a - b)) * distance(start, pos);
    }


    LatLon locationBearingDistance(LatLon p, double brng, double d) { 
        p = p.toRadians();
        brng *= M_PI/180;

        LatLon p2;
        const double R = 6371e3; // metres
        p2.lat = asin(sin(p.lat) * cos(d/R) + cos(p.lat) * sin(d/R) * cos(brng));
        p2.lon = p.lon + 
                    atan2(sin(brng) * sin(d/R) * cos(p.lat),
                        cos(d/R) - sin(p.lat) * sin(p2.lat));
        return p2.toDegrees();
    }

    double glideslope(float distance, float alt1, float alt2) { 
            return atan2(alt1 - alt2, distance) * 180/M_PI; 
    }

    class IlsSimulator {
    public:
        const LatLon tdzLocation; // lat/long of touchdown point 
        const float tdze;  // TDZE elevation of runway 
        const float faCrs; // final approach course
        const float gs; // glideslope in degrees
        float currentAlt;
        LatLon currentLoc;
        IlsSimulator(LatLon l, float elev, float crs, float g) : tdzLocation(l), tdze(elev), faCrs(crs), gs(g) {}
        void setCurrentLocation(LatLon now, double alt) { 
            currentLoc = now;
            currentAlt = alt;
        }
        double glideSlopeError() {
            return glideslope(distance(currentLoc, tdzLocation), currentAlt, tdze) - gs;
        }
        double courseErr() { 
            return bearing(currentLoc, tdzLocation) - faCrs;
        }
        double cdiPercent() { 
            return max(-1.0, min(1.0, courseErr() / 2.5));
        }
        double gsPercent() { 
            return max(-1.0, min(1.0, glideSlopeError() / 0.7));
        }
        std::string toString() { 
            return strfmt("ILS at %.6lf,%.6lf tdze %.0f, fac %.1f magnetic", 
            tdzLocation.lat, tdzLocation.lon, tdze, faCrs);
        }
    } *ils = NULL;

    struct  Approach{
        const char *name;
        double lat, lon, fac, tdze, gs;
    };

    Approach approaches[] = {
        {"KBFI 14R", 47.53789999126084, -122.30912681366567, 135.0, 18, 3.00}, 
        {"KBFI 32L", 47.52145193515430, -122.29521847198963, 315.0, 18, 3.00}, 
        {"S43 15",   47.90660383843286, -122.10299154204903, 150.0, 22, 4.00},
        {"S43 33",   47.90250761678649, -122.10136258707182, 328.0, 22, 4.00},
        {"2S1 17",   47.46106431485166, -122.47652028295599, 170.0, 300, 4.00}, 
        {"2S1 35",   47.45619317486062, -122.47745151953475, 350.0, 300, 4.00}, 
        {"WN14 02",  47.46276277164776, -122.56987914788057, 040.0, 300, 4.50},	
    };
        
        
    float angularDiff(float a, float b) { 
        float d = a - b;
        if (d > +180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }

    inline float constrain360(float a) { 
        if (abs(a) < 10000) { 
            while(a <= 0) a += 360;
            while(a > 360) a -= 360;
        }
        return a;
    }

    inline float magToTrue(float ang) { 
        return constrain360(ang + 15.5);		;
    }

    inline float trueToMag(float ang) { 
        return constrain360(ang - 15.5);		;
    }

    struct LatLonAlt { 
        LatLon loc;
        float alt; // meters 
        bool valid;
        LatLonAlt() : valid(false) {}
        LatLonAlt(LatLon l, float a) : loc(l), alt(a), valid(true) {}
        LatLonAlt(double lat, double lon, float a) : loc(lat, lon), alt(a), valid(true) {};
    };

    Approach *findBestApproach(LatLon p) { 
        Approach *best = NULL;
        for(int n = 0; n < sizeof(approaches)/sizeof(approaches[0]); n++) {
            Approach &a = approaches[n];
            LatLon tdz = LatLon(a.lat, a.lon); 
            float brg = bearing(p, LatLon(a.lat, a.lon));
            float distTravelled = distance(p, tdz);
            if (abs(angularDiff(brg, a.fac)) >= 60 || distTravelled > 20000)
                continue;
            if (best == NULL || 
                distance(p, tdz) < distance(p, LatLon(best->lat, best->lon)))
                best = &approaches[n]; 
        }
        if (best != NULL) { 
            printf("Chose '%s', bearing %.1f, distTravelled %.1f\n", best->name, 
                bearing(p, LatLon(best->lat, best->lon)), distance(p, LatLon(best->lat, best->lon)));
        } else { 
            printf("No best approach?\n");
        }
        return best;
    }



    using namespace std;
    class WaypointTracker {
    public: 
        LatLonAlt curPos, prevPos;
        int wayPointCount = 0;
        LatLonAlt startWaypoint, activeWaypoint, nextWaypoint;
        bool waypointPassed;
        float speed; // knots 
        float vvel;  // fpm
        float steerHdg, commandTrack, commandAlt;
        float lastHd, lastVd;
        float corrH = 0, corrV = 0;
        float hWiggle = 0, vWiggle = 0; // add simulated hdg/alt variability
        float xte = 0;

        void setCDI(float hd, float vd, float decisionHeight) {
            float gain = min(1.0, curPos.alt / 200.0);
            corrH = +0.2 * gain * ((abs(hd) < 2.0) ? hd + (hd - lastHd) * 400 : 0);
            if (abs(vd) < 2.0) 
                corrV += (gain * -0.01 * (vd + (vd - lastVd) * 30));
            lastHd = hd;
            lastVd = vd;
        }

        float nextTurnLead = 0;
        void run(float sec) { 
            float distToWaypoint = 0;
            if (!curPos.valid)
                return;
            if (activeWaypoint.valid && !waypointPassed) {
                steerHdg = bearing(curPos.loc, activeWaypoint.loc) + hWiggle;
                distToWaypoint = abs(distance(curPos.loc, activeWaypoint.loc));
                if (distToWaypoint < nextTurnLead) { 
                    waypointPassed = true;
                }
                float legDist = abs(distance(activeWaypoint.loc, startWaypoint.loc));
                // TODO - implement gradual alt transitions 
                //commandAlt = (activeWaypoint.alt - startWaypoint.alt) * min((float)1.0, max((float)0.0,(1 - distToWaypoint / legDist))) + startWaypoint.alt;
                commandAlt = activeWaypoint.alt;
            } else { 
                //commandAlt = curPos.alt;                
            }
            float distTravelled = speed * .51444 * sec;
            
            //commandAlt += corrV;//distToWaypoint / 1000;
            //steerHdg += corrH;//distToWaypoint / 1000;
            vvel = (commandAlt - curPos.alt) / sec * 196.85; // m/s to fpm 
            float brg = onSteer(steerHdg);
            if (distToWaypoint > 250) { 
                commandTrack = brg;
            }
            xte = crossTrackErr(startWaypoint.loc, activeWaypoint.loc, curPos.loc);
            // under 1000m and heading away from waypoint?  probably passed it 
            if (prevPos.valid && distToWaypoint < 1000  
                && abs(angularDiff(bearing(prevPos.loc, curPos.loc), bearing(curPos.loc, activeWaypoint.loc))) >= 90 ) {
                waypointPassed = true;
            }
            prevPos = curPos;
        }	

        void sim(float sec) { 
            float distTravelled = speed * .51444 * sec;
            LatLon newPos = locationBearingDistance(curPos.loc, commandTrack, distTravelled);
            curPos.loc = newPos;
        }

        std::function<float(float)> onSteer = [](float steer){ return steer; };

        // Set *next* waypoint. 
        void setWaypoint(const LatLonAlt &p) {
            if (activeWaypoint.valid) {
                startWaypoint = activeWaypoint;
            } else { 
                startWaypoint = curPos;
            }
            //startWaypoint.alt = curPos.alt;
            activeWaypoint = nextWaypoint;
            nextWaypoint = p;
            waypointPassed = false;
            wayPointCount++;
            if (wayPointCount == 1 && curPos.valid == false) { // first waypoint, initial position
                curPos = activeWaypoint;
                waypointPassed = true;
            } 
            if (activeWaypoint.valid == false) {
                waypointPassed = true;
            }

            // TODO: implement flyby waypoints 
            if (0 && startWaypoint.valid && activeWaypoint.valid && nextWaypoint.valid) { 
                float turnRad = speed * speed / 11.26 / tan(DEG2RAD(12/*bank angle*/)) / FEET_PER_METER;
                nextTurnLead = turnRad * tan(DEG2RAD(bearing(startWaypoint.loc, activeWaypoint.loc)  
                    - bearing(activeWaypoint.loc, nextWaypoint.loc)) * 2);
                nextTurnLead = min((float)2000.0, nextTurnLead);
            } else { 
                nextTurnLead = 100;
            }

            corrV = corrH = 0;
        }
    };

    class WaypointSequencer { 
        istream &in;
    public:
        std::map<std::string,float> inputs; 
        WaypointTracker wptTracker;
        float endAlt = -1;
        int autoPilotOn = 0, repeat = 0, decisionHeight = -1;
        WaypointSequencer(std::istream &i) : in(i) {}
        void run(float timestep) { 
            if (wptTracker.activeWaypoint.valid == false || wptTracker.waypointPassed)  
                readNextWaypoint(timestep);
            wptTracker.run(timestep);
    #ifdef UBUNTU
            if (wptTracker.curPos.valid && wptTracker.curPos.alt < endAlt) 
                ESP32sim_exit();
    #endif
        }
        float waitTime = 0;
        void readNextWaypoint(float timestep) { 
            double lat, lon, alt, track;
            using namespace std;
            std::string s("empty");
            if (waitTime > 0 && (waitTime -= timestep) > 0) 
                return;

            wptTracker.activeWaypoint.valid = false;
            while(wptTracker.activeWaypoint.valid == false) {
                if (in.eof() || !std::getline(in, s)) {
                    if (repeat) {
                        in.clear();
                        in.seekg(0, std::ios_base::beg);
                        continue;
                    } else {
                        break;
                    }
                }      
                cout << "READ LINE: " << s << endl;
                char buf[128];
                float f; 
                if (s.find("#") != std::string::npos)
                    continue;
                sscanf(s.c_str(), "SPEED %f", &wptTracker.speed);
                sscanf(s.c_str(), "ENDALT %f", &endAlt);
                sscanf(s.c_str(), "AP %d", &autoPilotOn);
                sscanf(s.c_str(), "HDG %f", &wptTracker.steerHdg);
                sscanf(s.c_str(), "WAIT %f", &waitTime);
                
                if (sscanf(s.c_str(), "INPUT.%s %f", buf, &f) == 2) { inputs[buf] = f; }
                sscanf(s.c_str(), "REPEAT %d", &repeat);
                if (sscanf(s.c_str(), "%lf, %lf %lf %lf", &lat, &lon, &alt, &track) == 4) {  
                    wptTracker.setWaypoint(LatLonAlt(lat, lon, alt / FEET_PER_METER));
                    wptTracker.steerHdg = track;
                } else if (sscanf(s.c_str(), "%lf, %lf %lf", &lat, &lon, &alt) == 3) 
                    wptTracker.setWaypoint(LatLonAlt(lat, lon, alt / FEET_PER_METER));
                else if (sscanf(s.c_str(), "%lf, %lf", &lat, &lon) == 2) 
                    wptTracker.setWaypoint(LatLonAlt(lat, lon, wptTracker.nextWaypoint.alt));
            }	
        }
    };
}

class WaypointsSequencerFile : public WaypointNav::WaypointSequencer {
	public:
	std::ifstream is;
	WaypointsSequencerFile(const char *fname) : is(fname, std::ios_base::in), WaypointSequencer(is) {};
};

class WaypointsSequencerString : public WaypointNav::WaypointSequencer {
	public:
	std::string s;
	std::istringstream is;
	WaypointsSequencerString(std::string &i) : s(i), is(s), WaypointSequencer(is) {};
};


