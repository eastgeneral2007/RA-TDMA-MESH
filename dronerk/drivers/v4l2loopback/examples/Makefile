TARGETS=test yuv4mpeg_to_v4l2 ondemandcam

.PHONY: all clean

all: $(TARGETS)

ondemandcam: ondemandcam.c
	${DRK_ROOT}/toolchain/bin/arm-cortex_a8-linux-gnueabi-gcc -o ondemandcam ondemandcam.c -lrt -lpthread

test: test.c
	${DRK_ROOT}/toolchain/bin/arm-cortex_a8-linux-gnueabi-gcc -o test test.c -lrt -lpthread
	
clean:
	-rm $(TARGETS)
