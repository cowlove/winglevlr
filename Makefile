#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s
VERBOSE=1

GIT_VERSION := "$(shell git describe --abbrev=4 --dirty --always --tags)"
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"

plot:	winglevlr_ubuntu
	./winglevlr_ubuntu --jdisplay --serial --seconds 700 | grep DTK | tr ':' ' ' > out.dat \
		&& echo "f='./out.dat'; p f u 2 w l ti 'DTK', f u 4 w l ti 'TRK'; pause 100"| gnuplot

test:
	./winglevlr_ubuntu --jdisplay --serial --seconds 700  | uniq | tee test.out
	(cd tests && ./regress.sh 052 -html && ./regress.sh 068 -html)
	google-chrome logs/regression/*.html

test.out:	winglevlr_ubuntu
	./winglevlr_ubuntu  --serial --seconds 1000  | uniq | tee $@

backtrace:
	tr ' ' '\n' | /home/jim/.arduino15/packages/esp32/tools/xtensa-esp32-elf-gcc/*/bin/xtensa-esp32-elf-addr2line -f -i -e /tmp/mkESP/winglevlr_esp32/*.elf
	

	
simplot:	test.out
	cat test.out | grep -a " R " > /tmp/simplot.txt && gnuplot -e 'f= "/tmp/simplot.txt"; set y2tic; set ytic nomirror; p [*:*][-15:15] f u 1:5 w l t "Pitch", f u 1:3 w l t "Roll", f u 1:9 w l t "Hdg" ax x1y2; pause 111'

winglevlr_ubuntu:	winglevlr.ino ESP32sim_ubuntu.h jimlib.h WaypointNav.h RollAHRS.h PidControl.h RollingLeastSquares.h GDL90Parser.h
	g++  -x c++ -g $< -o $@ -DESP32 -DUBUNTU -I./ -I${HOME}/Arduino/libraries/TinyGPSPlus-1.0.2/src/ -I${HOME}/Arduino/libraries/TinyGPSPlus/src/
# add -pg to profile 
CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1

include ${HOME}/Arduino/libraries/makeEspArduino/makeEspArduino.mk

print-%  : ; @echo $* = $($*)

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 921600 

cat:	fixtty
	cat ${UPLOAD_PORT}

setap1:
	wget 'http://192.168.4.1/wifisave?s=ChloeNet&p=niftyprairie7'

%-A50.plog: %.DAT winglevlr_ubuntu
	./winglevlr_ubuntu --replay $< | avg.pl 50 > $@


%.plog:  %.DAT winglevlr_ubuntu
	./winglevlr_ubuntu --replay $< --log - > $@
        
%R.plog:  %.DAT winglevlr_ubuntu
	./winglevlr_ubuntu --replay $< --log + > $@


clean_tmp:
	rm -rf /tmp/loglook.sh/

joystick:	joystick.c	
	gcc $< -lm -o $@

jsrun: 	joystick fixtty	
	stdbuf -o0 ./joystick | tee ${UPLOAD_PORT}
