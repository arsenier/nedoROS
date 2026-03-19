upload:
	arduino-cli compile --fqbn arduino:avr:uno firmware/firmware.ino && \
	arduino-cli upload -p /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_12350481010616918465-if00 --fqbn arduino:avr:uno firmware/firmware.ino
