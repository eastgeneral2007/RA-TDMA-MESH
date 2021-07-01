include toolchain.mk

all: slip_server

libdrk.so: lib/libdrk.so
lib/libdrk.so: $(DRK_ROOT)/src/libs/drk_api/*.c
	make -C $(DRK_ROOT)/src/libs/drk_api -j $(CORES)

all_sensor_data: bin/all_sensor_data
bin/all_sensor_data: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/all_sensor_data/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/all_sensor_data -j $(CORES)

api_test: bin/api_test
bin/api_test: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/api_test/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/api_test -j $(CORES)

busy_loop: bin/busy_loop
bin/busy_loop: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/busy_loop/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/busy_loop -j $(CORES)

busy_time: bin/busy_time
bin/busy_time: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/busy_time/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/busy_time -j $(CORES)

clocktest: bin/clocktest
bin/clocktest: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/clocktest/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/clocktest -j $(CORES)

communication: bin/communication
bin/communication: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/communication/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/communication -j $(CORES)

fly_to_diff: bin/fly_to_diff
bin/fly_to_diff: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/fly_to_diff/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/fly_to_diff -j $(CORES)

gps: bin/gps
bin/gps: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/gps/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/gps -j $(CORES)

hello_world: lib/libdrk.so bin/hello_world
bin/hello_world: $(DRK_ROOT)/src/projects/examples/hello_world/*.c
	make -C $(DRK_ROOT)/src/projects/examples/hello_world -j $(CORES)

hover_tracking: bin/hover_tracking
bin/hover_tracking: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/hover_tracking/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/hover_tracking -j $(CORES)

infrared: bin/infrared
bin/infrared: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/infrared/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/infrared -j $(CORES)

land: bin/land
bin/land: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/land/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/land -j $(CORES)

net_test: bin/net_test
bin/net_test: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/net_test/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/net_test -j $(CORES)

PSMA_BS: bin/PSMA_BS
bin/PSMA_BS: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/PSMA_BS/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/PSMA_BS -j $(CORES)

PSMA_drone: bin/PSMA_drone
bin/PSMA_drone: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/PSMA_drone/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/PSMA_drone -j $(CORES)

radio: bin/radio
bin/radio: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/radio/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/radio -j $(CORES)

scatter_gather: bin/scatter_gather
bin/scatter_gather: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/scatter_gather/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/scatter_gather -j $(CORES)

simple_flight: bin/simple_flight
bin/simple_flight: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/simple_flight/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/simple_flight -j $(CORES)

slip_client: bin/slip_client
bin/slip_client: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/slip_client/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/slip_client -j $(CORES)

slip_server: bin/slip_server
bin/slip_server: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/slip_server/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/slip_server -j $(CORES)

spin_test: bin/spin_test
bin/spin_test: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/spin_test/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/spin_test -j $(CORES)

stream_data: bin/stream_data
bin/stream_data: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/stream_data/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/stream_data -j $(CORES)

RSSI_localization: bin/RSSI_localization
bin/RSSI_localization: lib/libdrk.so $(DRK_ROOT)/src/projects/experimental/RSSI_localization/*.c
	make -C $(DRK_ROOT)/src/projects/experimental/RSSI_localization -j $(CORES)

clean:
	make -C $(DRK_ROOT)/src/libs/drk_api clean
	make -C $(DRK_ROOT)/src/projects/experimental/simple_flight clean
	make -C $(DRK_ROOT)/src/projects/examples/hello_world clean
	make -C $(DRK_ROOT)/src/projects/experimental/RSSI_localization clean
	make -C $(DRK_ROOT)/src/projects/experimental/all_sensor_data clean
	make -C $(DRK_ROOT)/src/projects/experimental/slip_server clean

realclean: clean
	make -C $(DRK_ROOT)/src/libs/drk_api realclean
	make -C $(DRK_ROOT)/src/projects/experimental/simple_flight realclean
	make -C $(DRK_ROOT)/src/projects/examples/hello_world realclean
	make -C $(DRK_ROOT)/src/projects/experimental/RSSI_localization realclean
	make -C $(DRK_ROOT)/src/projects/experimental/all_sensor_data realclean
	make -C $(DRK_ROOT)/src/projects/experimental/slip_server realclean
	rm -rf bin
