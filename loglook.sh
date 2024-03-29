#!/bin/bash 



# Plot and compare a prev git version w/ current
################################################
# -args  : additional args for winglevlr_ubuntu
# -debug : --debug X=1,Y=2 style args for winglevlr_ubuntu
# -git   : git version to checkout
# -replay: regenerate an output file
# -roll  : things to plot from that file
# -maghdg: things to plot from that file 
#
# Then use more -git,-args,-debug,-replay to plot from different versions or arguments
# Note -roll,-maghdg etc come AFTER the -git,-args,-replay sequence
#################################################3  

# Simple use: plot comparing original roll data from test #020 with simulated replay 
#####################################
# ./loglook.sh 020 -roll -replay -roll

# Simple use: plot comparing simulated replay of roll data from test #020 with simulated replay using an older git version  
#####################################
# ./loglook.sh 020 -replay -roll -git adc07fev -replay -roll 

# Plot comparing different actual recorded data from test #020 with replays using two different debug values
#####################################
# ./loglook.sh 020 -roll -replay "cr1=0.00111,cr2=3.2" -roll -replay "cr1=0.00222,cr2=4.2" -roll


make winglevlr_ubuntu

NUM=$1
shift
TERMSPEC="set term qt size 2048,1024"
WAIT="pause 1000"
XRANGE='[]'

make logs/AHRSD$NUM.plog
#          1      2         3             4            5           6   7   8   9   10  11  12  13  14  15  16  17     18    19  
# 	float sec, gpsTrack, gpsTrackGDL90, gpsTrackVTG, gpsTrackRMC, alt, ax, ay, az, gx, gy, gz, mx, my, mz, dtk,g5trk, palt, gspeed;
#         20       21      22     23     24     25      26
#   float g5Pitch, g5Roll, g5Hdg, g5ias, g5tas, g5palt, g5Timestamp 
#	short pwmOutput, flags;  // 27,28
#	float desRoll, roll; // 29,30

F2=./logs/AHRSD$NUM.plog
PROG=./winglevlr_ubuntu
#make $F2


while (( $# > 0 )); do 
	if [ "$1" == "-hdgGPS" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($2) w l ax x1y2 tit "GPS Selected Heading", '; fi
	if [ "$1" == "-hdgGDL90" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($3) w l ax x1y2 tit "GPS GDL90 Heading", '; fi
	if [ "$1" == "-hdgVTG" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($4) w l ax x1y2 tit "GPS VTG Heading", '; fi
	if [ "$1" == "-hdgRMC" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($5) w l ax x1y2 tit "GPS RMC Heading", '; fi
	if [ "$1" == "-galt" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($6 > 0 ? $6 : 1/0) w l ax x1y2 tit "GPS Alt", '; fi
	if [ "$1" == "-ax" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($7) w l  tit "ax", '; fi
	if [ "$1" == "-ay" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($8) w l  tit "ay", '; fi
	if [ "$1" == "-az" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($9) w l  tit "az", '; fi
	if [ "$1" == "-gx" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($10) w l  tit "gx", '; fi
	if [ "$1" == "-gy" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($11) w l  tit "gy", '; fi
	if [ "$1" == "-gz" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($12) w l  tit "gz", '; fi
	if [ "$1" == "-mx" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($13) w l  tit "mx", '; fi
	if [ "$1" == "-my" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($14)  w l  tit "my", '; fi
	if [ "$1" == "-mz" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($15) w l  tit "mz", '; fi
	if [ "$1" == "-dtk" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($16)  w l ax x1y2 tit "DTK", '; fi
	if [ "$1" == "-g5track" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($17) w l  tit "G5 Track", '; fi
	if [ "$1" == "-palt" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($18 > 0 ? $18 : 1/0) w l ax x1y2 tit "Press Alt", '; fi
	if [ "$1" == "-gspeed" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($19 > 0 ? $19 : 1/0) w l ax x1y2 tit "Ground Speed", '; fi
	if [ "$1" == "-g5pitch" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($20) w l  tit "G5 Pitch", '; fi
	if [ "$1" == "-g5roll" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):(-$21) w l   tit "G5 Roll", '; fi
	if [ "$1" == "-g5hdg" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($22) ax x1y2 w l  tit "G5 Heading", '; fi
	if [ "$1" == "-g5ias" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($23) ax x1y2 w l  tit "G5 IAS", '; fi
	if [ "$1" == "-g5tas" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($24) ax x1y2 w l  tit "G5 TAS", '; fi
	if [ "$1" == "-g5palt" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($25) ax x1y2 w l  tit "G5 PALT", '; fi
	if [ "$1" == "-g5slip" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($26) ax x1y2 w l  tit "G5 Slip", '; fi
	if [ "$1" == "-ubloxhdg" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($27) ax x1y2 w l  tit "UBLOX hdg", '; fi
	if [ "$1" == "-ubloxha" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($28) ax x1y2 w l  tit "UBLOX hdg acc", '; fi
	if [ "$1" == "-ubloxalt" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($29) ax x1y1 w l  tit "UBLOX Altitude", '; fi
	if [ "$1" == "-ubloxgs" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($30) ax x1y2 w l  tit "UBLOX groundspeed", '; fi
	if [ "$1" == "-lat" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($31) ax x1y2 w l  tit "UBLOX lat", '; fi
	if [ "$1" == "-lon" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($32) ax x1y2 w l  tit "UBLOX lon", '; fi
	
	
	
	if [ "$1" == "-servo0" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($33) ax x1y2 w l tit "Servo 0", '; fi
	if [ "$1" == "-servo1" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($34) ax x1y2 w l tit "Servo 1", '; fi
	if [ "$1" == "-flag" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($35) w l ax x1y2 tit "flags", '; fi	
	if [ "$1" == "-desroll" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($36) w l tit "Desired Roll", ' ; fi
	if [ "$1" == "-roll" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($37) w l  tit "Logged Roll", '; fi
	if [ "$1" == "-maghdg" ]; then PS=$PS\ \"$F2\"' u  ($1-F2_min_x):($38) w l ax x1y2 tit "magHdg", '; fi
	if [ "$1" == "-xte" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($39) w l ax x1y1 tit "XTE", '; fi
	if [ "$1" == "-pitch" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($40) w l ax x1y1 tit "logged pitch", '; fi
	if [ "$1" == "-dalt" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($41) w l ax x1y1 tit "des alt", '; fi
	if [ "$1" == "-dpitch" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($42) w l ax x1y1 tit "des pitch", '; fi
	#if [ "$1" == "-aax" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($41) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-aay" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($42) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-aaz" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($43) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-agx" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($44) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-agy" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($45) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-agz" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($47) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-amx" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($48) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-amy" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($49) w l ax x1y1 tit "'$1'", '; fi
	#if [ "$1" == "-amz" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($50) w l ax x1y1 tit "'$1'", '; fi
	if [ "$1" == "-stickX" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($43) w l ax x1y2 tit "'$1'", '; fi
	if [ "$1" == "-stickY" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($44) w l ax x1y2 tit "'$1'", '; fi
	if [ "$1" == "-map" ]; then PREPLOT="set size ratio -1;"; 	PS=$PS\ \"$F2\"' u 32:31  w l tit "Ground Track", '; fi
	if [ "$1" == "-magplot" ]; then PS=$PS\ \"$F2\"' u 13:14 w p tit "Mag Plot", '; fi
	if [ "$1" == "-pitcherr" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($40-$20) w l ax x1y1 tit "Pitch Error", '; fi
	if [ "$1" == "-rollerr" ]; then PS=$PS\ \"$F2\"' u ($1-F2_min_x):($37-$21) w l ax x1y1 tit "Roll Error", '; fi
	if [ "$1" == "-stats" ]; then 
		STATS='stats '"$XRANGE"' "'$F2'" u ($1-F2_min_x):($'$2')'; 
		shift
	fi
	if [ "$1" == "-file" ]; then 
		F2="$2";
		#make $F2 
		shift; 
	fi  
	if [ "$1" == "-git" ]; then
		GIT="$2"
        GITDIR=/tmp/loglook.sh/$GIT
        mkdir -p $GITDIR
        git archive $GIT | tar -x -C $GITDIR
        echo Building in $GITDIR ...
        (cd $GITDIR && sed -i 's/include /#/' Makefile && make winglevlr_ubuntu)
        PROG=$GITDIR/winglevlr_ubuntu
        echo done 
 		shift
	fi
	if [ "$1" == "-debug" ]; then
		DARGS="$2"
		shift
	fi
	if [ "$1" == "-args" ]; then
		MARGS="$2"
		shift
	fi
	if [ "$1" == "-replay" ]; then
		F2="./logs/AHRSD$NUM-R${GIT}${DARGS}${MARGS}.plog";
		IN="./logs/AHRSD$NUM.DAT"
		echo "Replay $F2"
		if [ "$PROG" -nt "$F2" ] || [ ! -f "$F2" ]; then
			echo $PROG $MARGS --debug \"$DARGS\" --replay ./logs/AHRSD$NUM.DAT --log + \> \""$F2"\" 
			$PROG $MARGS --debug "$DARGS" --replay ./logs/AHRSD$NUM.DAT --log + | grep LOG > "$F2"
		fi
	fi  

	if [ "$1" == "-html" ]; then 
		TERMSPEC="set term canvas size 1400,600;set output '${F2}.gnuplot.html'"
		WAIT=""
	fi
	if [ "$1" == "-range" ]; then 
		XRANGE="$2"
		shift
	fi
	shift;
done
echo $XRANGE $PS
echo "$STATS"

cat << EOF | gnuplot
$TERMSPEC
set title "${F2}"

f2="$F2"
f1="$F1"
stats f2 u 1:1 name "F2"
set grid
set y2tic
set x2tic
set ytic nomirror
set xtic nomirror
${PREPLOT}
print "**********************************************"


${STATS}
p ${XRANGE} ${PS}
	
$WAIT

EOF
exit
	f2 u (\$2-F2_min_x):(\$28) w l ax x1y2 tit "GPS Alt", \
	f2 u (\$2-F2_min_x):(\$33) tit "Pulse", \
	f2 u (\$2-F2_min_x):(((int(\$23) & 3) == 0) ? 1/0 : (int(\$23) & 3))  tit "FLAGS", \
	f2 u (\$2-F2_min_x):(\$31) w l tit "GPS Pitch", \
	f2 u (\$2-F2_min_x):(\$29) w l ax x1y2  tit "P Alt", \

	f2 u (\$2-F2_min_x):(\$27) w l  tit "Gyro X", \
	f2 u (\$2-F2_min_x):7 w l tit "Calc Roll",\
	f2 u (\$2-F2_min_x):(\$25) w l ax x1y2 tit "Gyro Drift", \

	f2 u (\$2-F2_min_x):(\$25) w l ax x1y2 tit "Gyro Drift", \
	f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Bank", \
	f2 u (\$2-F2_min_x):10 w l tit "Ground Speed" ax x1y2, \
	f2 u (\$2-F2_min_x):(\$21) w l ax x1y2 tit "Sim Servo Output", \
	f1 u (\$1-F1_min_x):(-\$4) w l tit "ArduPilot Roll", \
	f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Bank", \
	f2 u (\$2-F2_min_x):(\$4) ax x1y2 w l tit "GPS Hdg", \
	f2 u (\$2-F2_min_x):6 w l tit "Roll Raw Gyro",\
	f2 u (\$2-F2_min_x):7 w l tit "Roll Comp Filter",\

	f1 u (\$1-F1_min_x-134):(-\$4) w l tit "ArduPilot Roll", \
	f2 u (\$2-F2_min_x):(\$16) w l tit "GPS Delta Bank",\

exit
	# Too junky to even use 
	, f2 u (\$2-F2_min_x):(\$17) w l tit "Mag Delta Bank" \
	, f2 u (\$2-F2_min_x):(\$18) w l tit "Dip Bank" \
	f2 u (\$2-F2_min_x):6 w l tit "Roll Raw Gyro",\



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
	
