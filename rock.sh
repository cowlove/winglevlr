#!/bin/bash 

make fixtty

# rock.sh <bank> <seconds>
SER=/dev/ttyUSB0
for n  in {1..5}; do
	echo "ITERATION $n"
	echo "dtrk=-1" > $SER
	echo "roll=$1" > $SER
	sleep $2
	echo "roll=-$1" > $SER
	sleep $2;  
done

echo "roll=0" > $SER
echo "roll=0" > $SER
