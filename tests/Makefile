all:	test_plog test_tinygps parsed_vtg.txt

clean:
	rm test_plog test_tinygps parsed_vtg.txt
	
parsed_vtg.txt:	nmea.txt test_tinygps 
	./test_tinygps $< > $@
	
%:	%.cpp
	g++ -DUBUNTU -I.. -I/home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src $< -o $@ -g

test_plog_0: test_plog.cpp ../RollAHRS.h
	g++ -DUBUNTU -I.. -DLOGITEM_VERSION=LogItem0 -I/home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src $< -o $@ -g

test_plog_B:	test_plog.cpp  ../RollAHRS.h
	g++ -DUBUNTU -I.. -DLOGITEM_VERSION=LogItemB -I/home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src $< -o $@ -g

test_plog_C:	test_plog.cpp  ../RollAHRS.h
	g++ -DUBUNTU -I.. -DLOGITEM_VERSION=LogItemC -I/home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src $< -o $@ -g

%.plog:	 %.DAT test_plog_C
	./test_plog_C $< > $@
	


%.ATT:  %.bin
	mavlogdump.py --types ATT --format csv --csv_sep ' '  $< > $@

%.RCOU:  %.bin
	mavlogdump.py --types RCOU --format csv --csv_sep ' '  $< > $@

%.PIDR:  %.bin
	mavlogdump.py --types PIDR --format csv --csv_sep ' '  $< > $@
	
%-roll.plot:  %.po %-esp.plog %-mav.ATT
	echo HI %
	

