#!/bin/bash -x

./joystick --testThrow .05 --testTime 1 --testNow
./joystick --testThrow .1 --testTime 1 --testNow
./joystick --testThrow .2 --testTime 1 --testNow
./joystick --testThrow .5 --testTime 1 --testNow
./joystick --testThrow 1 --testTime 1 --testNow
