CC=gcc
CFLAGS=-I.

%.o: %.c 
	$(CC) -c -o $@ $< $(CFLAGS)

all: control_node slip-server

control_node: main.o slipstream.o
	gcc -o control_node main.o slipstream.o -I.
slip-server: slip-server.o
	gcc -o slip-server slip-server.o -I.
    
clean: 
	rm -f *.o *~ core control_node slip-server

