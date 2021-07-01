gcc \
-g \
-O0 \
-D VERBOSE -D LOGFILE -D WORLDSIM -D OUTSIDEDRK \
*.c \
-o uavworld_simulator \
-pthread -lm -lrt \
-Wall -Wextra -Wformat -pedantic -std=gnu99

mkdir -p usb
