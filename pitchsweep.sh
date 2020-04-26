#!/bin/bash -x

PN=-5
PU=$(( $PN + 2 ))
PD=$(( $PN - 2 ))
SLEEP=30
for f in {1..3}; do echo dtrk=-1 > /dev/ttyUSB1; done
for f in {1..3}; do echo roll=0 > /dev/ttyUSB1; done
for f in {1..3}; do echo maxb=10 > /dev/ttyUSB1; done

for BANK in 0 -10 10; do
	for f in {1..3}; do echo roll=$BANK > /dev/ttyUSB1; done
	
	for ITERATION in {1..10}; do
		echo $ITERATION
		for f in {1..3}; do echo pitch=$PU > /dev/ttyUSB1; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > /dev/ttyUSB1; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PD > /dev/ttyUSB1; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > /dev/ttyUSB1; done
		sleep $SLEEP
	done
done
for f in {1..3}; do echo roll=0 > /dev/ttyUSB1; done
