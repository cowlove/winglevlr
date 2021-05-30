#!/bin/bash -x
T=2


#./joystick --testThrow .05 --testTime $T --testNow
./joystick --testThrow .1 --testTime $T --testNow
./joystick --testThrow .2 --testTime $T --testNow
./joystick --testThrow .5 --testTime $T --testNow
./joystick --testThrow 1 --testTime $T --testNow
