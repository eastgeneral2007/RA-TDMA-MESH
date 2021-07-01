 # create the libworld.so :
gcc *.c \
-D VERBOSE -D OUTSIDEDRK \
-fpic -Wall -Wextra -pedantic -std=gnu11 -shared \
-I../include \
-lrt -lm -pthread \
-o libsimclock.so
