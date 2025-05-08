BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
CHIP ?= esp32
VERBOSE=1
EXCLUDE_DIRS=/home/jim/Arduino/libraries/LovyanGFX
PART_FILE=${ESP_ROOT}/tools/partitions/min_spiffs.csv
GIT_VERSION := "$(shell git describe --abbrev=6 --dirty --always)"

#CDC_ON_BOOT=1
EXCLUDE_DIRS=${HOME}/Arduino/libraries/lvgl|${HOME}/Arduino/libraries/LovyanGFX
IGNORE_STATE=1
BUILD_MEMORY_TYPE = qio_qspi

#BUILD_EXTRA_FLAGS += -DARDUINO_PARTITION_huge_app 
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\" 
        
#sed  's|^\(.*/srmodels.bin\)|#\1|g' -i ~/.arduino15/packages/esp32/hardware/esp32/3.2.0/boards.txt  

MAKEFLAGS=-j4  

fixtty:
	stty -F ${PORT} -hupcl -crtscts -echo raw 115200
cat:    fixtty
	cat ${PORT} | tee ./cat.out
socat:  
	socat udp-recvfrom:9000,fork - 
mocat:
	mosquitto_sub -h rp1.local -t "${MAIN_NAME}/#" -F "%I %t %p"   
uc:
	${MAKE} upload && ${MAKE} cat

backtrace:
	tr ' ' '\n' | addr2line -f -i -e ./build/${BOARD}/*.elf

clean-all:
	${MAKE} csim-clean
	rm -rf ./build
	rm -f ./${BOARD}.mk
	
ifeq ($(BOARD),csim)
SKETCH_NAME=$(shell basename `pwd`)

##############################################
# CSIM rules 
.DEFAULT_GOAL=csim
ARDUINO_LIBS_DIR=${HOME}/Arduino/libraries
CSIM_BUILD_DIR=./build/csim
CSIM_LIBS=Arduino_CRC32 ArduinoJson TinyGPSPlus esp32jimlib Adafruit_HX711
CSIM_LIBS+=esp32csim
CSIM_SRC_DIRS+=$(foreach L,$(CSIM_LIBS),${ARDUINO_LIBS_DIR}/${L}/src ${ARDUINO_LIBS_DIR}/${L})
CSIM_SRCS=$(foreach DIR,$(CSIM_SRC_DIRS),$(wildcard $(DIR)/*.cpp)) 
CSIM_SRC_WITHOUT_PATH = $(notdir $(CSIM_SRCS))
CSIM_OBJS=$(CSIM_SRC_WITHOUT_PATH:%.cpp=${CSIM_BUILD_DIR}/%.o)
CSIM_INC=$(foreach DIR,$(CSIM_SRC_DIRS),-I${DIR}) \
         -I${ARDUINO_LIBS_DIR}/esp32csim/src/csim_include/
CSIM_INC+=-I../lvglConfigPanel/

CSIM_CFLAGS+=-g -MMD -fpermissive -DGIT_VERSION=\"${GIT_VERSION}\" -DESP32 -DCSIM -DUBUNTU 
#CSIM_CFLAGS+=-DGPROF=1 -pg
#CSIM_CFLAGS+=-O2

LVLINUX=${HOME}/src/lv_port_linux
CSIM_INC+=-I${LVLINUX}/lvgl
CSIM_INC+=-I${LVLINUX}/src/lib/
CSIM_LDLIBS += ${LVLINUX}/build/lvgl/lib/liblvgl_demos.a
CSIM_LDLIBS += ${LVLINUX}/build/liblvgl_linux.a
CSIM_LDLIBS += ${LVLINUX}/build/lvgl/lib/liblvgl.a
CSIM_LDLIBS += ${LVLINUX}/build/lvgl/lib/liblvgl_examples.a
CSIM_LDLIBS += ${LVLINUX}/build/lvgl/lib/liblvgl_thorvg.a
CSIM_LDLIBS += -lX11 -lGL -lglut 

CSIM_CFLAGS+=-g -MMD -fpermissive -DGIT_VERSION=\"${GIT_VERSION}\" -DESP32 -DCSIM -DUBUNTU 
#CSIM_CFLAGS+=-DGPROF=1 -pg
#CSIM_CFLAGS+=-O2

csim: ${SKETCH_NAME}_csim
	cp $< $@

${CSIM_BUILD_DIR}/%.o: %.cpp 
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${CSIM_BUILD_DIR}/%.o: %.ino
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${SKETCH_NAME}_csim: ${CSIM_BUILD_DIR} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o 
	echo $@
	g++ -g ${CSIM_CFLAGS} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o ${CSIM_LDLIBS} -o $@         

${CSIM_BUILD_DIR}:
	mkdir -p ${CSIM_BUILD_DIR}

VPATH = $(sort $(dir $(CSIM_SRCS)))

.PHONY: csim-clean
csim-clean:
	rm -f ${CSIM_BUILD_DIR}/*.[od] ${SKETCH_NAME}_csim csim

-include ${CSIM_BUILD_DIR}/*.d
else
        include ~/Arduino/libraries/makeEspArduino/makeEspArduino.mk
endif

