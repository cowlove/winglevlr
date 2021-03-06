#include <stdio.h>
#include <stdint.h>
#include <strings.h>

class GDL90Parser {
	bool lock() { return true; } 
	void unlock() {};
	
	unsigned char buf[256];
	int index = 0;
	bool esc = 0;
	unsigned long bs3(const unsigned char* b) {
		return ((unsigned long)b[0] << 16) | ((unsigned long)b[1] << 8) | b[2];
	}
	static void unBS3(unsigned char *b, long x) { 
		b[2] = x & 0xff;
		b[1] = (x >> 8) & 0xff;
		b[0] = (x >> 16) & 0xff;
	}
public:
	int msgCount = 0, errCount = 0;
	struct State {
		double lat, lon;
		float track;
		int palt; // (palt * 25) - 1000
		int alt;  // meters above geoid 
		int vvel; // units	 of 64 fpm, fpm = 64 * hvel
		int hvel; // knots
		int timestamp;
		bool updated, valid;
		int misc;
	} state, lastState;

	GDL90Parser() : index(0), esc(0) {
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
		rval.valid = false;
		if (lock()) {
			rval = state;
			state.updated = false;
			unlock();
		} 
		return rval;
	}
	bool checkForValid() {
		if (index < 2) 
			return false; 
		unsigned int crc = crcCompute(buf, index - 2);
		unsigned int found_crc =(((uint16_t)(buf[index - 1]))<<8) | buf[index-2];
		if (found_crc != crc) 
			return false;
		
		// TODO HACK use these ad-hoc sanit	y checks until CRC works 
		//if (state.lon == 0 || state.lat == 0) return false;
		//if (state.palt > 20000 / 25) return false; 
		//if (state.lon < -130 || state.lon > -120) return false;
		//if (state.lat < 45 || state.lat > 49) return false;
		//if (state.hvel < 0 || state.hvel > 140) return false;
		//if (state.alt < -500 || state.alt > 15000) return false;
		return true;
	}

	static uint16_t Crc16Table[256];
	static void crcInit(void) { 
		uint16_t bitctr, crc, i;     
		for (i = 0; i < 256; i++) { 
			crc = (i << 8);         
			for (bitctr = 0; bitctr < 8; bitctr++) { 
				crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
			}         
			Crc16Table[i] = crc;
		} 
	}
	
	static uint16_t crcCompute(
		unsigned char *block,       
		int length) {
		uint16_t crc = 0;
		for (int i = 0; i < length; i++)     {
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
	static int packStuff(unsigned char *out, int buflen, unsigned char *in, int l) { 
		int outl = 0;
		out[outl++] = 0x7e;
		for(int n = 0; n < l && outl << buflen; n++) { 
			if (in[n] == 0x7e || in[n] == 0x7d) {
				out[outl++] = 0x7d;
				out[outl++] = in[n] ^ 0x20;
			} else 
				out[outl++] = in[n];
		}
		out[outl++] = 0x7e;
		return outl;
	}
	static int packMsg10(unsigned char *out, int buflen, State s) {
		unsigned char b[30];
		bzero(b, sizeof(b));
		b[0] = 10;
		if (s.lat < 0) s.lat += 360;
		if (s.lon < 0) s.lon += 360;
		unBS3(b + 5, s.lat * 0x800000 / 180.0);
		unBS3(b + 8, s.lon * 0x800000 / 180.0);
		
		b[11] = s.palt >> 4;
		b[12] = ((s.palt & 0xf) << 4) | (s.misc & 0xf) ;
		b[14] = s.hvel >> 4;
		b[15] = (s.hvel & 0xf) << 4;
		b[17] = s.track * 256 / 360.0;

		//unsigned int_crc =(((uint16_t)(buf[index - 1]))<<8) | buf[index-2];
		unsigned int crc = crcCompute(b, 28);
		b[28] = crc & 0xff;
		b[29] = (crc >> 8) & 0xff;
		return packStuff(out, buflen, b, sizeof(b));
	}
	static int packMsg11(unsigned char *out, int buflen, State s) {
		unsigned char b[5];
		bzero(b, sizeof(b));
		b[0] = 11;
		int a = (s.alt - 20) * 3.3208 / 5.0;
		b[1] = a >> 8;
		b[2] = a & 0xff;
		//state.alt = ((int16_t)(((buf[1]) << 8) | buf[2])) * 5.0 / 3.3208 + 20; // 20m geoid height
		unsigned int crc = crcCompute(b, 3);
		b[3] = crc & 0xff;
		b[4] = (crc >> 8) & 0xff;
		return packStuff(out, buflen, b, sizeof(b));
	}
	void add(char b) { // handle one character in GDL90 stream
		if (b == 0x7e) { // got a flag byte
			if (index > 1) { // end of packet flag, packet complete
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
						state.misc = buf[12] & 0xf;
						state.updated = true;
						unlock();
						//printf("MSG10: %.4f %.4f %.1f %d\n", state.lat, state.lon, state.track, state.hvel	);
					}
				}
				if (buf[0] == 11) { // MSG11 geometric altitude packet
					if (lock()) { 
						state.alt = ((int16_t)(((buf[1]) << 8) | buf[2])) * 5.0 / 3.3208 + 20; // 20m geoid height
						unlock();
						//printf("MSG11: %d\n", state.alt);
					}
				}
				setValid();
				index = 0;
				return;
			} 
		}
		else { // everything other than a flag byte 0x7e 
			if (b == 0x7d) { // escape character
				esc = 1;
				return;
			} else if (esc == 1) { // following an escape character
				b ^= 0x20;
				esc = 0;
			}
			if (index >= 0 && (unsigned int)index < sizeof(buf))
				buf[index++] = b; // store valid byte
		}

	};
};
float protect[10]; 
uint16_t GDL90Parser::Crc16Table[256];

