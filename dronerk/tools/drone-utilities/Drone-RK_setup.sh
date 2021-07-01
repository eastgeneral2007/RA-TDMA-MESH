#!/bin/sh
echo "### Drone-RK Setup ###"
echo "Setting up ad-hoc wireless network"
echo "Enter your desired IP Address. Invalid number will skip wifi setup."
echo -n "192.168.1."
read ip
echo "Your IP is 192.168.1.$ip"

echo $ip | grep "^[0-9]*$"
if [[ "$?" -ne 0 ]] || [[ "$ip" -ge 255 ]] || [[ "$ip" -le  0 ]]; then
	echo "Skipping Network Setup"
else
	echo "Got a good subadress"
	count=`wc -l /etc/init.d/rcS | awk '{print($1)}'`
	echo "Line count is $count"
	if [ "$count" -ge 63 ]; then
		sed '$d' < /etc/init.d/rcS > /etc/init.d/rcS2 ; mv /etc/init.d/rcS2 /etc/init.d/rcS	
	fi
	
	echo "sh /bin/drkwifi.sh $ip" >> /etc/init.d/rcS
	echo '#!/bin/sh' > /bin/drkwifi.sh
	
	echo 'ifconfig ath0 down' >> /bin/drkwifi.sh
	echo 'iwconfig ath0 mode ad-hoc' >> /bin/drkwifi.sh
	echo 'iwconfig ath0 channel 8' >> /bin/drkwifi.sh
	echo "iwconfig ath0 essid 'Drone-RK Swarm'" >> /bin/drkwifi.sh
	echo 'ifconfig ath0 192.168.1.$1 up' >> /bin/drkwifi.sh
	
	echo 'Done with WIFI setup'
	sync
fi

if [[ -e "/data/video/Drone-RK_tool" ]]; then
	echo "Drone-RK_tool found, setting up links"
	cp /data/video/Drone-RK_tool /bin/Drone-RK_tool
	chmod +x /bin/Drone-RK_tool
	echo -e "#!/bin/sh\nDrone-RK_tool -l" > /bin/land
	echo -e "#!/bin/sh\nDrone-RK_tool -e" > /bin/emergency
	echo -e "#!/bin/sh\nDrone-RK_tool -t" > /bin/takeoff
	echo -e "#!/bin/sh\nDrone-RK_tool -b" > /bin/battery
	echo -e "#!/bin/sh\nDrone-RK_tool -k" > /bin/keyboard
	chmod +x /bin/battery /bin/emergency /bin/takeoff /bin/land /bin/keyboard
else
	echo "Drone-RK_tool was not found, links will not be set up."
fi

echo "Creating a symlink to /data/video for the lazy (/d)"
ln -s /data/video /d

echo "Done with Drone-RK setup!"
