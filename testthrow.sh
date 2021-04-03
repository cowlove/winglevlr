#!/bin/bash -x

./joystick --testThrow .05 --testTime 1 --testNow
./joystick --testThrow .1 --testTime 1 --testNow
./joystick --testThrow .2 --testTime 1 --testNow
./joystick --testThrow .5 --testTime 1 --testNow
./joystick --testThrow 1 --testTime 1 --testNow
./joystick --testThrow .05 --testTime .5 --testNow
./joystick --testThrow .1 --testTime .5 --testNow
./joystick --testThrow .2 --testTime .5 --testNow
./joystick --testThrow .5 --testTime .5 --testNow
./joystick --testThrow 1 --testTime .5 --testNow
./joystick --testThrow .5 --testTime .2 --testNow
./joystick --testThrow .1 --testTime .2 --testNow
./joystick --testThrow .2 --testTime .2 --testNow
./joystick --testThrow .5 --testTime .2 --testNow
./joystick --testThrow 1 --testTime .2 --testNow
