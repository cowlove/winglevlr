

#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s

test.out:	winglevlr_ubuntu
	./winglevlr_ubuntu --jdisplay --serial --seconds 10  > $@
	


winglevlr_ubuntu:	winglevlr.ino ESP32sim_ubuntu.h jimlib.h RollAHRS.h PidControl.h
	g++ -x c++ -g $< -o $@ -DESP32 -DUBUNTU -I/home/jim/Arduino/libraries/mavlink/common -I /home/jim/Arduino/libraries/TinyGPSPlus-1.0.2/src/

CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1

include ${HOME}/Arduino/makeEspArduino/makeEspArduino.mk

print-%  : ; @echo $* = $($*)

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 57600 

cat:	fixtty
	cat ${UPLOAD_PORT}

setap1:
	wget 'http://192.168.4.1/wifisave?s=ChloeNet&p=niftyprairie7'

