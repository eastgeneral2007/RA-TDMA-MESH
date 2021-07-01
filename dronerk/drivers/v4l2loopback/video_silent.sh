kk
mv /dev/video1 /dev/videoC
insmod v4l2loopback.ko
sleep 2
./fakevideo /dev/video1 2>/dev/null 1>/dev/null &
/bin/program.elf 2>/dev/null 1>/dev/null &
