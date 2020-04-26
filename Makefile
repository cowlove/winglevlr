#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s
#VERBOSE=1

plot:	winglevlr_ubuntu
	./winglevlr_ubuntu --jdisplay --serial --seconds 700 | grep DTK | tr ':' ' ' > out.dat \
		&& echo "f='./out.dat'; p f u 2 w l ti 'DTK', f u 4 w l ti 'TRK'; pause 100"| gnuplot

test:
	./winglevlr_ubuntu --jdisplay --serial --seconds 700  | tee test.out
	(cd tests && ./regress.sh 052 -html && ./regress.sh 068 -html)
	google-chrome logs/regression/*.html

test.out:	winglevlr_ubuntu
	./winglevlr_ubuntu --jdisplay --serial --seconds 700  | tee $@
	
winglevlr_ubuntu:	winglevlr.ino ESP32sim_ubuntu.h jimlib.h RollAHRS.h PidControl.h
	g++ -x c++ -g $< -o $@ -DESP32 -DUBUNTU -I/home/jim/Arduino/libraries/mavlink/common -I /home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src/

CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1

include ${HOME}/Arduino/makeEspArduino/makeEspArduino.mk

print-%  : ; @echo $* = $($*)

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 115200 

cat:	fixtty
	cat ${UPLOAD_PORT}

setap1:
	wget 'http://192.168.4.1/wifisave?s=ChloeNet&p=niftyprairie7'

