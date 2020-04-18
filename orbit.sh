#!/bin/bash

for h in {35..0}; do 
	HDG=$(($h*10))
	echo $HDG
	for f in {1..3}; do echo dtrk=$HDG > /dev/ttyUSB0; done
	sleep $1
done

for h in {0..35}; do 
	HDG=$(($h*10))
	echo $HDG
	for f in {1..3}; do echo dtrk=$HDG > /dev/ttyUSB0; done
	sleep $1
done
