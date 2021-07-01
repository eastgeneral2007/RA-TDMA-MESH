#!/bin/sh

# please make sure you have:
# 1) wdev.conf with the name of the wireless device to be used.
# 2) chan.conf with the number of the channel to use in swarm network.

sudo rfkill unblock wifi
#sudo service network-manager stop

# read wireless device you wanna use
#WDEV=$(cat wdev.conf)
WDEV="$(iw dev | grep Interface | grep wlx | awk 'NR==1{ print $2;}')"


RED='\033[0;31m'
BLUE='\033[;34m'
NC='\033[0m' # No Color


clear


if [ "$#" -ne 1 ]; then
  echo "You must provide a subadress"
  exit
fi


sudo ifconfig ${WDEV} up
CHANNEL="$( sudo iw ${WDEV} scan | grep ardrone2_$1 -A 2 | awk 'NR==3{print $5}'  )"
# print the 3rd record number , 5th col

if [ -z "$CHANNEL" ]
then
	echo "AP ardrone2_"$1 "Not found.. finishing."
	exit
fi
echo "AP on channel is ["${BLUE}$CHANNEL${NC}"]"
sudo ifconfig ${WDEV} down

echo "ok"

sudo iwconfig ${WDEV} mode managed essid ardrone2_$1
sudo ifconfig ${WDEV} 192.168.1.22 up
echo ${BLUE}iwconfig${NC}
iwconfig ${WDEV} | grep "Access Point" -B 1
echo ${BLUE}ifconfig${NC}
ifconfig ${WDEV} | grep "inet"

RET="$(iwconfig ${WDEV} | grep 'Access Point' | awk '{print $4}')"


#sudo route add default gw 192.168.1.1 wlan4 
while [ "$RET" = "Not-Associated" ]
do
	RET="$(iwconfig ${WDEV} | grep 'Access Point' | awk '{print $4}')"
	echo "Not ready yet"
	sleep 1
done


printf "Connected to ${RED}ardrone2_$1${NC} (RET=${RET})\n"
iwconfig ${WDEV}


HOST='192.168.1.1'

sleep 2
nowe=$(date +"%Y.%m.%d-%T" -u)
printf "Setting cur date: ${BLUE}$nowe${NC}\n"

CHN=$(cat chan.conf)

telnet $HOST <<END_SCRIPT
echo $nowe
date $nowe

ifconfig ath0 down
iwconfig ath0 mode ad-hoc; 
iwconfig ath0 channel ${CHN} ;
iwconfig ath0 retry 2 ;
iwconfig ath0 essid swarm ;  
ifconfig ath0 192.168.2.$1 netmask 255.255.255.0 up; 
iwconfig ath0 rate 24M ;
/data/video/arptable.sh
killall -9 udhcpd

quit
END_SCRIPT
printf "${BLUE}Done${NC}\n"

exit 0

