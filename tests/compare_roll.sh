#!/bin/bash

F2=../logs/${1}-esp.plog

make $F2
#XRANGE='[100:400]' #'[100:*]'
#XRANGE='[100:1000]'
XRANGE='[*:*]'
cat << EOF | gnuplot

f2="$F2"
stats f2 u 2:20 name "F2"
set grid
set y2tic
set ytic nomirror
p ${XRANGE} \
	f2 u (\$2-F2_min_x):7 w l tit "Roll Comp Filter" \
	, f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Delta Bank" \
	

pause 1000
EOF
exit
	# Too junky to even use 
	, f2 u (\$2-F2_min_x):(\$17) w l tit "Mag Delta Bank" \
	, f2 u (\$2-F2_min_x):(\$18) w l tit "Dip Bank" \



	f2 u (\$2-F2_min_x):(\$20) w l ax x1y2  tit "AHRS Servo Output" \

	f4 u (\$1-F4_min_x):(\$6) w l ax x1y2 tit "ArduPilot PID P" \
	,f4 u (\$1-F4_min_x):(\$8) w l ax x1y2 tit "ArduPilot PID D" \
	,f2 u (\$2-F2_min_x):(\$21) w l ax x1y2 tit "AHRS PID P" \
	,f2 u (\$2-F2_min_x):(\$22) w l ax x1y2 tit "AHRS PID D" \


	f3 u (\$1-F4_min_x):(\$6) w l ax x1y2 tit "ArduPilot PID P" \
	,f3 u (\$1-F4_min_x):(\$8) w l ax x1y2 tit "ArduPilot PID D" \
	,f2 u (\$2-F2_min_x):(\$20) w l ax x1y2 tit "AHRS Servo Output" \
	,f2 u (\$2-F2_min_x):(\$21) w l ax x1y2 tit "AHRS PID P" \
	,f2 u (\$2-F2_min_x):(\$22) w l ax x1y2 tit "AHRS PID D" \

	
	, f2 u (\$2-F2_min_x):7 w l tit "Roll Comp Filter" \
	, f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Bank" \
	, f2 u (\$2-F2_min_x):(\$18) w l tit "Dip Bank" \
	, f2 u (\$2-F2_min_x):(\$19) w l tit "Comb Bank" \

	, f2 u (\$2-F2_min_x):(\$20) w l tit "Servo Output" \


	, f3 u (\$1-F3_min_x):(\$1) w l tit "ArduPilot Servo Output" \
\
	# Mag heading dip correction is still super junky.  
	# dip-deduced bank angle is pretty good 
	, f2 u (\$2-F2_min_x):(\$4) w l tit "GPS Hdg" \
	, f2 u (\$2-F2_min_x):(\$13) w l tit "Raw Mag Hdg" \
	, f2 u (\$2-F2_min_x):(\$14) w l tit "Mag Hdg" \
	, f2 u (\$2-F2_min_x):(\$15) w l tit "Dip Bank Correction" \

	, f2 u (\$2-F2_min_x):7 w l tit "Roll Comp Filter" \
	, f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Bank" \
	, f2 u (\$2-F2_min_x):(\$17) w l tit "Mag Bank" \
	, f2 u (\$2-F2_min_x):(\$18) w l tit "Dip Bank" \
	, f2 u (\$2-F2_min_x):(\$19) w l tit "Comb Bank" \


	, f2 u (\$2-F2_min_x):7 w l tit "Roll Comp Filter" \
	, f2 u (\$2-F2_min_x):10 w l tit "Ground Speed" ax x1y2  \
	, f2 u (\$2-F2_min_x):10 w l tit "GPS Hdg Bank Angle"   \
	, f2 u (\$2-F2_min_x):9 w l tit "Drift Correction" ax x1y2  \
	, f2 u (\$2-F2_min_x):6 w l tit "Drift Compensated" \
	, f2 u (\$2-F2_min_x):(\$14) w l tit "MagHdg" \
	, f2 u (\$2-F2_min_x):(\$15) w l tit "Magnetic Bank" \
	, f2 u (\$2-F2_min_x):(\$3) w l tit "GPS Hdg" \
	, f2 u (\$2-F2_min_x):(\$14) w l tit "Mag Hdg" \
	, f2 u (\$2-F2_min_x):(\$17) w l tit "GPS Bank" \
	, f2 u (\$2-F2_min_x):(\$18) w l tit "Mag Bank" \
	, f2 u (\$2-F2_min_x):(\$19) w l tit "Dip Bank" \
	
