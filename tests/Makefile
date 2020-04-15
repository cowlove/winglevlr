


plog:	plog.cpp
	g++ -g plog.cpp -I.. -o plog


%.plog: %.dat plog
	./plog $< > $@

%.ATT:  %.bin
	mavlogdump.py --types ATT --format csv --csv_sep ' '  $< > $@

%.RCOU:  %.bin
	mavlogdump.py --types RCOU --format csv --csv_sep ' '  $< > $@

%.PIDR:  %.bin
	mavlogdump.py --types PIDR --format csv --csv_sep ' '  $< > $@
	
%-roll.plot:  %.po %-esp.plog %-mav.ATT
	echo HI %
	

