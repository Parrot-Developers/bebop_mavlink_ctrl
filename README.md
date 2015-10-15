# bebop_mavlink_ctrl
With the executable test_gcs it is possible to control the bebop using xbox controller.
Mainly we send mavlink MANUAL_CONTROL commands.

test_gcs:
	Simple mavlink UDP ground control station to control Parrot Bebop Drone
	It manages xbox controller commands and send them to a remote drone
	usage: ./test_gcs "drone_ip_address" "/dev/input/jspath"

test_drone:
	Simple drone mavlink client UDP
	It uses libmavlink.so to create a simple mavlink client
	It sends heartbeats, system status and receive/print mavlink messages

libmavlink.so:
	common lib for test_drone and test_gcs

build:
	git submodule init && git submodule update
	cd mavlink/ && python -m pymavlink.tools.mavgen --lang C -o ../out/ message_definitions/v1.0/common.xml ; cd ../
	make all tests
