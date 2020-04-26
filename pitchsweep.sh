#!/bin/bash -x

PN=0
PU=$(( $PN + 5 ))
PD=$(( $PN - 5 ))
SLEEP=10
for f in {1..3}; do echo dtrk=-1 > /dev/ttyUSB0; done
for f in {1..3}; do echo roll=0 > /dev/ttyUSB0; done
for f in {1..3}; do echo maxb=10 > /dev/ttyUSB0; done

for BANK in 0 -10 10; do
	for f in {1..3}; do echo roll=$BANK > /dev/ttyUSB0; done
	sleep $SLEEP
	sleep $SLEEP
	sleep $SLEEP

	for ITERATION in {1..10}; do
		echo $ITERATION
		for f in {1..3}; do echo pitch=$PU > /dev/ttyUSB0; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > /dev/ttyUSB0; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PD > /dev/ttyUSB0; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > /dev/ttyUSB0; done
		sleep $SLEEP
	done
done
for f in {1..3}; do echo roll=0 > /dev/ttyUSB0; done
