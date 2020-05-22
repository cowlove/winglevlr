#!/bin/bash -x

TTY=/dev/ttyUSB0

PN=7
PU=$(( $PN + 10 ))
PD=$(( $PN - 10))
SLEEP=30
for f in {1..3}; do echo dtrk=-1 > ${TTY}; done
for f in {1..3}; do echo roll=0 > ${TTY}; done
for f in {1..3}; do echo maxb=10 > ${TTY}; done

for BANK in 0 -5 5 ; do
	for f in {1..3}; do echo roll=$BANK > ${TTY}; done
	
	for ITERATION in {1..5}; do
		echo $ITERATION
		for f in {1..3}; do echo pitch=$PU > ${TTY}; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > ${TTY}; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PD > ${TTY}; done
		sleep $SLEEP
		for f in {1..3}; do echo pitch=$PN > ${TTY}; done
		sleep $SLEEP
	done
done
for f in {1..3}; do echo roll=0 > ${TTY}; done
