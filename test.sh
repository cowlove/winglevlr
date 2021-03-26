#!/bin/bash 
touch testexpected.out
PROG=./winglevlr_ubuntu
OUT=testresult.out
EXPECTED=testexpected.out

rm $PROG

make $PROG && $PROG --replay ./logs/AHRSD006.DAT  | tail -1 > $OUT &&\
tail -1 $OUT | diff - $EXPECTED


