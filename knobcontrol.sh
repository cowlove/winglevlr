#!/bin/bash

while true; do
    read -rsn1 input
    #printf "%x\n" \'$input 
    if [ "$input" = $'\x41' ]; then
        #echo "up"
        echo knobturn 1 | socat - udp-sendto:255.255.255.255:7895,broadcast
    fi
    if [ "$input" = $'\x42' ]; then
        echo knobturn -1 | socat - udp-sendto:255.255.255.255:7895,broadcast
    fi
    if [ "$input" = $'\x43' ]; then
        echo knobturn 10 | socat - udp-sendto:255.255.255.255:7895,broadcast
    fi
    if [ "$input" = $'\x44' ]; then
        echo knobturn -10 | socat - udp-sendto:255.255.255.255:7895,broadcast
    fi
    if [ "$input" = $'\x0' ]; then
        echo knobpress 0 | socat - udp-sendto:255.255.255.255:7895,broadcast
    fi
done