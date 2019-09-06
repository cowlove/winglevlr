

#BOARD=esp32doit-devkit-v1
BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s


CHIP=esp32
OTA_ADDR=192.168.0.101
IGNORE_STATE=1

include ${HOME}/src/makeEspArduino/makeEspArduino.mk

print-%  : ; @echo $* = $($*)

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 19200

setap1:
	wget 'http://192.168.4.1/wifisave?s=ChloeNet&p=niftyprairie7'

