#!/bin/bash
make winglevlr_ubuntu && ./winglevlr_ubuntu --wind 180@40 --startpos 47.5,-122.45,1000,180,88 --serialInput 1 dtrk=220 --button 180,39,3,0 --log + --seconds 3600 | grep "LOG" > out.plog && ./plot_on_map.py ./out.plog  && ./ll.py ./out.plog map


