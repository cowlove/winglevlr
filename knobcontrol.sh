#!/bin/bash
#ADDR=192.168.4.102:7895
ADDR=192.168.5.126:7895
#echo smode 0 | socat - udp-sendto:$ADDR

SENDCMD="socat - udp-sendto:$ADDR"
SENDCMD="tee -a /dev/ttyUSB0"

while true; do
    read -rsn1 input
    #printf "%x\n" \'$input 
    if [ "$input" = $'\x41' ]; then
        #echo "up"
        echo knobturn +1 | $SENDCMD
    fi
    if [ "$input" = $'\x42' ]; then
        echo knobturn -1 | $SENDCMD
    fi
    if [ "$input" = $'\x43' ]; then
        echo knobturn +10 | $SENDCMD
    fi
    if [ "$input" = $'\x44' ]; then
        echo knobturn -10 | $SENDCMD
    fi
    if [ "$input" = $'\x0' ]; then
        echo knobpress 0 | $SENDCMD
    fi
done
