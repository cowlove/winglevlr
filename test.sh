#!/bin/bash 
touch testexpected.out
PROG=./winglevlr_ubuntu
OUT=testresult.out
EXPECTED=testexpected.out

rm $PROG

make $PROG && $PROG --replay ./logs/AHRSD006.DAT --debug ",zeros.mx=23.0000000,zeros.my=30.0000000,dipconstant=4.8000000,"  --log - | tail -1 > $OUT &&\
tail -1 $OUT | diff - $EXPECTED


