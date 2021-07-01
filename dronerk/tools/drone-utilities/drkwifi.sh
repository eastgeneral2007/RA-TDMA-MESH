#!/bin/sh
# 8/18/2011 - Nat Storer
# Put this script in the drone's /bin directory, then call it as
# the last line of /etc/init.d/rcS by adding:
# sh /bin/drkwifi.sh [number]
# Where number is the last byte of the IP address
# ex: sh /bin/drkwifi.sh 42 will set your IP to 192.168.1.42

if [ "$#" -ne 1 ]; then
  echo "You must provide a subadress"
  exit
fi

echo $1 | grep "^[0-9]*$"
if [ "$?" -ne 0 ]; then
  echo "Address must be a postive integer"
  exit
elif [ "$1" -ge 256 ]; then
  echo "Address cannot be greater than 255"
  exit
elif [ "$1" -lt  0 ]; then
  echo "Address cannot be less than zero"
  exit
fi

echo Drone-RK WIFI configuration
echo channel 8, ad-hoc, essid 'Drone-RK Swarm', 192.168.1.$1
ifconfig ath0 down
iwconfig ath0 mode ad-hoc
iwconfig ath0 channel 8
iwconfig ath0 essid 'Drone-RK Swarm'
ifconfig ath0 192.168.1.$1 up
echo Done setting up WIFI
