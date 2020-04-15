#include <iostream>
#include <fstream>

#include "G90Parser.h"


GDL90Parser gdl90;

using namespace std;

int main(int argc, char **argv) {
	ifstream i(argv[1], ios_base::in | ios::binary);
	char l[1];
	
	unsigned char example[] = {0x7E ,0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02, 0xB3, 0x8B, 0x7E};
	
	for (int n = 0; n < sizeof(example); n++) { 
		gdl90.add(example[n]);
	}
	while(i.read((char *)&l, sizeof(l))) { 
		gdl90.add(l[0]);
	}
}
