#!/bin/bash

for a in {0..10}; 
    do for b in {0..10}; do 
        ARG1=`echo "scale=10;.0005+$a*.0001"|bc`
        ARG2=`echo "scale=10; -.4+$b*.2"|bc`
        PROG="./winglevlr_ubuntu --replay ./logs/AHRSD018.DAT --debug ahrs.crpitch=$ARG1,ahrs.gxdecel=$ARG2"; 
        RES=`eval $PROG` 
        echo $ARG1 $ARG2 $RES $PROG; 
    done
    echo
done