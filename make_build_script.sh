#!/bin/bash
BOARD=$1
TMP=/tmp/arduino/$$.txt
PORT=/dev/ttyUSB0
BOARD_OPTS=PartitionScheme=min_spiffs

if [ ! $BOARD == "esp32" ]; then
    BOARD_OPTS="$BOARD_OPTS",CDCOnBoot=cdc
    PORT=/dev/ttyACM0
fi

cd `dirname $0`
arduino-cli compile  -v \
    -b esp32:esp32:${BOARD} --board-options ${BOARD_OPTS} -u --port ${PORT} --build-cache-path core.a \
    | tee $TMP

SKETCH=`basename \`pwd\``
SKETCHDIR=/tmp/arduino/sketches/`rematch '/tmp/arduino/sketches/([A-Z0-9]+)/' /tmp/arduino/1359891.txt | head -1`
OUT=./build-${BOARD}.sh

echo "#!/bin/bash" > $OUT
echo "echo '#include <Arduino.h>' > ${SKETCHDIR}/sketch/${SKETCH}.ino.cpp" >> $OUT
echo "echo '#line 1 \"`pwd`/${SKETCH}.ino\"' > ${SKETCHDIR}/sketch/${SKETCH}.ino.cpp" >> $OUT
echo "cat ${SKETCH}.ino >> ${SKETCHDIR}/sketch/${SKETCH}.ino.cpp" >> $OUT

egrep "${SKETCH}.ino.cpp.o" $TMP >> $OUT
egrep '^python3' $TMP >> $OUT
grep esptool_py $TMP | grep -v ${PORT} >> $OUT
echo "echo Waiting for ${PORT}...; while [ ! -e ${PORT} ]; do sleep 1; done" >> $OUT
grep esptool_py $TMP | grep ${PORT} >> $OUT
echo "echo Waiting for ${PORT}...; while [ ! -e ${PORT} ]; do sleep 1; done" >> $OUT
echo "stty -F ${PORT} raw -echo && cat ${PORT}" >> $OUT

#if [ ! -d build ]; then mkdir build; fi
#cp -a "${SKETCHDIR}/${SKETCH}"* build/
chmod 755 $OUT
#rm -f $TMP
