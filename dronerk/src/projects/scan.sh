WDEV=$(cat wdev.conf)
sudo ifconfig $WDEV up
sudo iw $WDEV scan | grep swarm -B 8
echo ""
sudo iw $WDEV scan | grep -e  'SSID: ard' -e 'SSID: swarm' -B 6 | grep -e freq -e SSID -e '--'
