#!/bin/bash

for a in {0..10}; 
    do for b in {0..10}; do 
        ARG1=`echo "scale=10;-12+$a/2"|bc`
        ARG2=`echo "scale=10; 1.2+$b/10"|bc`
        PROG="./winglevlr_ubuntu --replay ./logs/AHRSD018.DAT --debug ahrs.pitchoffset=$ARG1,ahrs.debug=$ARG2"; 
        RES=`eval $PROG` 
        echo $RES $PROG; 
    done
done