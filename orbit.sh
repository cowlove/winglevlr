#!/bin/bash

for h in {11..0}; do 
	HDG=$(($h*30))
	echo $HDG
	for f in {1..3}; do echo dtrk=$HDG > /dev/ttyUSB0; done
	sleep $1
done

for h in {0..11}; do 
	HDG=$(($h*30+15))
	echo $HDG
	for f in {1..3}; do echo dtrk=$HDG > /dev/ttyUSB0; done
	sleep $1
done
