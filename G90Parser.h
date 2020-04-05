#include <stdio.h>
#include <stdint.h>

class GDL90Parser {
	bool lock() { return true; } 
	void unlock() {};
	
	unsigned char buf[1024];
	int index = -1;
	bool esc = 0;
	unsigned long bs3(unsigned char* b) {
		return ((unsigned long)b[0] << 16) | ((unsigned long)b[1] << 8) | b[2];
	}
public:
	int msgCount = 0, errCount = 0;
	struct State {
		double lat, lon, track;
		int palt, alt, hvel, vvel, timestamp;
		bool updated, valid;
	} state, lastState;

	GDL90Parser() : index(-1), esc(0) {
		//hMutex = CreateMutex(NULL, FALSE, NULL);
		state.valid = false;
		lastState.valid = false;
		crcInit();
	}
	~GDL90Parser() {
		//CloseHandle(hMutex);
	}
	State getState() {
		State rval;
		if (lock()) {
			rval = state;
			state.updated = false;
			unlock();
		}
		else rval.valid = false;
		return rval;
	}
	bool checkForValid() {
		// TODO use these ad-hoc sanity checks until CRC works 
		if (state.lon < -123 || state.lon > 120) return false;
		if (state.lat < 45 || state.lat > 49) return false;
		if (state.hvel < 0 || state.hvel > 140) return false;
		if (state.alt < -500 || state.alt > 19000) return false;
		unsigned int crc = crcCompute(buf, index - 2);
		return true;
	}
	unsigned char Crc16Table[256];

	void crcInit(void) { 
		unsigned int i, bitctr, crc;     
		for (i = 0; i < 256; i++) { 
			crc = (i << 8);         
			for (bitctr = 0; bitctr < 8; bitctr++) { 
				crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
			}         
			Crc16Table[i] = crc;
		} 
	}
	unsigned int crcCompute(
		unsigned char *block,       
		unsigned long int length) {
			unsigned long int i;
			unsigned short crc = 0;
			for (i = 0; i < length; i++)     {
				crc = Crc16Table[crc >> 8] ^ (crc << 8) ^ block[i];
			}     
		return crc; 
	}

	void setValid() {
		if (checkForValid() == false) {
			state.valid = false;
			errCount++;
		} else {
			state.valid = true;
			msgCount++;
		}
	}
	void add(char b) { // handle one character in GDL90 stream
		if (b == 0x7e) { // got a flag byte
			if (index >= 0) { // end of packet flag, packet complete
				if (buf[0] == 0) { // MSG00 heartbeat
					state.timestamp = (((unsigned long)buf[4]) << 8) + ((unsigned long)(buf[3]));
				}
				if (buf[0] == 10) { // MSG10 ownship position report packet
					if (lock()) {
						state.lat = ((long)bs3(buf + 5)) * 180.0 / 0x800000;
						state.lon = ((long)bs3(buf + 8)) * 180.0 / 0x800000;
						if (state.lat > 180) state.lat -= 360;
						if (state.lon > 180) state.lon -= 360;
						state.palt = (((unsigned long)buf[11]) << 4) | (buf[12] >> 4);
						state.hvel = (((unsigned long)buf[14]) << 4) | (buf[15] >> 4);
						state.vvel = ((((unsigned long)buf[15]) & 0xf) << 8) | buf[16];
						state.track = buf[17] * 360.0 / 256;
						state.updated = true;
						setValid();
						unlock();
						//printf("MSG10: %.4f %.4f %.1f %d\n", state.lat, state.lon, state.track, state.hvel);
					}
				}
				if (buf[0] == 11) { // MSG11 geometric altitude packet
					if (lock()) { 
						state.alt = ((int16_t)(((buf[1]) << 8) | buf[2])) * 5.0 / 3.3208 + 20; // 20m geoid height
						setValid();
						unlock();
						printf("MSG11: %d\n", state.alt);
					}
				}
				if (index > 0) // finished packet, wait for next one 
					index = -1;
				return;
			}
			else { // start of packet flag
				index = 0;
			}
		}
		else {
			if (b == 0x7d) { // escape character
				esc = 1;
				return;
			}
			else if (esc == 1) { // following an escape character
				b ^= 0x20;
				esc = 0;
			}
			if (index >= 0 && index < sizeof(buf))
				buf[index++] = b; // store valid byte
		}

	};
};

