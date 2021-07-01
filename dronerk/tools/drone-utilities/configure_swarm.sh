#!/bin/sh

if [ "$#" -ne 1 ]; then
  echo "Provide a subaddress"
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

count=`wc -l /etc/init.d/rcS | awk '{print($1)}'`
echo "Line count is $count"

if [ "$count" -ge 63 ]; then
	echo "Wifi has already been set up."
	exit
fi

chmod +x /data/video/drkwifi.sh
cp /data/video/drkwifi.sh /bin/drkwifi.sh
echo "sh /bin/drkwifi.sh $1" >> /etc/init.d/rcS
echo "New address: 192.168.1.$1"
sync
