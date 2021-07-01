gcc *.c \
-o read_clock \
-I../libclock \
-lsimclock -L../libclock -pthread -lrt -lm -o read_sim \
-Wl,-rpath -Wl,../libclock

