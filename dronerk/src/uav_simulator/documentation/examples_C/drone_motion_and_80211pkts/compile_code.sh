#compile main and link it to the libworld.so :
gcc -D SIMEXAMPLE -D VERBOSE \
main.c \
generic.c \
utils.c \
tx_*.c rx_*.c  \
-Iinclude \
-lsimclock -L../libclock/ -pthread -lrt -lm -o uav_sim \
-Wl,-rpath -Wl,../libclock/

