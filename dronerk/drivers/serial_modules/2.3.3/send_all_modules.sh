#!/bin/sh


HOST=192.168.1.$1

FILE0='rt3572sta.ko'
FILE1='usbserial.ko'
FILE2='ftdi_sio.ko'


ftp -n $HOST <<END_SCRIPT
quote USER 'asd'
binary

put $FILE0
put $FILE1
put $FILE2

quit
END_SCRIPT


exit 0

