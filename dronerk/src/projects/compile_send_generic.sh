# This script is able to compile a binary of the library for the drones, 
# then returns to $HERE folder and compiles&links the app binary
# finally sends it to the Drone, via ftp

#!/bin/sh
#Define some colorcodes
RED='\033[0;31m'
BLUE='\033[;34m'
NC='\033[0m' # No Color


FILE0='libdrk_ARM.so' 
FILE1='libsbp_ARM.so' 
#(This should be set beforehand, whoever calls this scripts) 
#FILE2= "that's the name of the binary"
#HERE = "where to return after compiling libraries"
#HOST = "ip of the destination"

# compile drk lib
godrk
make VERBOSE=y LOGDATA=y 
#echo " --- " ${FILE0} " done --- "

# compile sbp lib
gosbp
make VERBOSE=y LOGDATA=y
#echo " --- " ${FILE1} " done --- "

#compile app + link all
cd ${HERE}
make VERBOSE=y LOGDATA=y
#echo " --- " ${FILE2} " app done --- "

#copy .so files to local dir
cp ${DRK_ROOT}/lib/*ARM.so* .
#echo id $1 > myid.txt
#printf "${BLUE}${FILE0} ${FILE1} copied to cur folder${NC}\n"
printf "${BLUE}Sending all via FTP...${NC}"
#out.log > /dev/null
ftp -n $HOST <<END_SCRIPT1
quote USER 'asd'
binary
put ${FILE0}
put ${FILE1}
put ${FILE2}
quit
END_SCRIPT1


#failed=$(grep "No route to host" out.log)

if [ "" != "$failed" ]; then
	echo "[[" $(cat out.log) "]]"
	echo "Terminating"
	return
fi
printf "${BLUE}Sent!${NC}\n"
rm *ARM.so*
#printf "${BLUE}${FILE0} ${FILE1} removed from cur folder${NC}\n"


printf "${BLUE}Setting executable permissions to both${NC}\n"

telnet $HOST <<END_SCRIPT2
chmod +x /data/video/${FILE2}
chmod +x /data/video/*ARM.so*
mv /data/video/*ARM.so* /usr/lib/
quit
END_SCRIPT2

printf "${BLUE}Installed into the drone${NC}\n"



