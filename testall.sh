#!/bin/bash 

BLACK=$(tput setaf 0)
RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
YELLOW=$(tput setaf 3)
LIME_YELLOW=$(tput setaf 190)
POWDER_BLUE=$(tput setaf 153)
BLUE=$(tput setaf 4)
MAGENTA=$(tput setaf 5)
CYAN=$(tput setaf 6)
WHITE=$(tput setaf 7)
BRIGHT=$(tput bold)
NORMAL=$(tput sgr0)
BLINK=$(tput blink)
REVERSE=$(tput smso)
UNDERLINE=$(tput smul)

for f in $@ */test.sh; do 
    D=`dirname $f`
    SK=`basename $D`.ino
    UB=`basename $D`_ubuntu
    if ( [ -f $D/Makefile ] && [ -f $D/$SK ] ); then 
        echo -n Clean build $D/$SK ...
        if (cd $D && make clean && make ) > /dev/null 2>&1 ; then
            echo -e "\r${GREEN}BUILT$NORMAL: $BLUE $SK $NORMAL                                "
        else  
            echo -e "\r${RED}BROKE$NORMAL: $BLUE $SK $NORMAL                                "
        fi
    
        echo -n ${BLUE}Testing $D/$SK...$NORMAL                                  
        if (cd $D && ./test.sh > /dev/null 2>&1 ); then
            echo -e "\r${GREEN} PASS$NORMAL: $BLUE $SK $NORMAL                                "
        else  
            echo -e "\r${RED} FAIL$NORMAL: $BLUE $SK $NORMAL                                "
        fi
    fi
done

