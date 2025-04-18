BOARD ?= esp32
PORT ?= /dev/ttyUSB0

GIT_VERSION := "$(shell git describe --abbrev=6 --dirty --always)"
EXTRA_CFLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
SKETCH_NAME=$(shell basename `pwd`)
BOARD_OPTIONS = PartitionScheme=min_spiffs,FlashFreq=80,FlashMode=dio

CCACHE=ccache
MAKEFLAGS=-j4  

usage:
	@echo make \{elf,bin,upload,cat,uc\(upload then cat\),csim,clean,csim-clean\}

include ${BOARD}.mk

${BOARD}.mk:
	@echo Running arduino-cli compile --clean, this could take a while.  Upload failure is OK.
	arduino-cli -v compile --clean --build-path ./build/${BOARD}/ \
		-b esp32:esp32:${BOARD} --board-options ${BOARD_OPTIONS} \
		-u -p ${PORT} | tee cli.out | bin/cli-parser.py > ${BOARD}.mk

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
	

##############################################
# CSIM rules 

CSIM_BUILD_DIR=./build/csim
CSIM_LIBS=esp32jimlib Arduino_CRC32 ArduinoJson TinyGPSPlus
CSIM_SRC_DIRS=$(foreach L,$(CSIM_LIBS),${HOME}/Arduino/libraries/${L}/src)
CSIM_SRCS=$(foreach DIR,$(CSIM_SRC_DIRS),$(wildcard $(DIR)/*.cpp)) 
CSIM_SRC_WITHOUT_PATH = $(notdir $(CSIM_SRCS))
CSIM_OBJS=$(CSIM_SRC_WITHOUT_PATH:%.cpp=${CSIM_BUILD_DIR}/%.o)
CSIM_INC=$(foreach DIR,$(CSIM_SRC_DIRS),-I${DIR})
CSIM_INC+=-I./csim_includes/
CSIM_CFLAGS+=-g -MMD -fpermissive -DGIT_VERSION=\"${GIT_VERSION}\" -DESP32 -DCSIM -DUBUNTU 
#CSIM_CFLAGS+=-DGPROF=1 -pg
#CSIM_CFLAGS+=-O2

${CSIM_BUILD_DIR}/%.o: %.cpp 
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${CSIM_BUILD_DIR}/%.o: %.ino
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${SKETCH_NAME}_csim: ${CSIM_BUILD_DIR} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o
	echo $@
	g++ -g ${CSIM_CFLAGS} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o -lGL -lglut -o $@         

csim: ${SKETCH_NAME}_csim 
	cp $< $@

${CSIM_BUILD_DIR}:
	mkdir -p ${CSIM_BUILD_DIR}

VPATH = $(sort $(dir $(CSIM_SRCS)))

.PHONY: csim-clean
csim-clean:
	rm -f ${CSIM_BUILD_DIR}/*.[od] ${SKETCH_NAME}_csim csim

-include ${CSIM_BUILD_DIR}/*.d

