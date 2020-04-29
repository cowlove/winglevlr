#!/bin/bash -x

TTY=/dev/ttyUSB0

PN=0
PU=$(( $PN + 2 ))
PD=$(( $PN - 2 ))
SLEEP=30
for f in {1..3}; do echo dtrk=-1 > ${TTY}; done
for f in {1..3}; do echo roll=0 > ${TTY}; done
for f in {1..3}; do echo maxb=10 > ${TTY}; done

for BANK in 0 -10 10; do
	for f in {1..3}; do echo roll=$BANK > ${TTY}; done
	
	for ITERATION in {1..10}; do
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
