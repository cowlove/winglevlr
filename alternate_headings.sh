#!/bin/bash 

make fixtty

# alternate_headings.sh <hdg1> <hdg2> <seconds>
SER=/dev/ttyUSB0
for n  in {1..5}; do
	echo "ITERATION $n"
	echo "dtrk=$1" > $SER
	echo "dtrk=$1" > $SER
	sleep $3
	echo "dtrk=$2" > $SER
	echo "dtrk=$2" > $SER
	sleep $3;  
done
