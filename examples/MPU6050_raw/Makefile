# BOARD_TAG = arduino:mbed_rp2040:pico
# BOARD_TAG = rp2040:rp2040:rpipico
BOARD_TAG = esp32:esp32:XIAO_ESP32C3
MONITOR_PORT = /dev/cu.usbmodem132401
ARDUINO_CLI_PATH := arduino-cli

compile:
	$(ARDUINO_CLI_PATH) compile --fqbn $(BOARD_TAG) --export-binaries

upload:
	@$(ARDUINO_CLI_PATH) upload -p $(MONITOR_PORT) --fqbn $(BOARD_TAG) --verbose

monitor:
	@$(ARDUINO_CLI_PATH) monitor -p $(MONITOR_PORT) --config baudrate=9600
	# screen $(MONITOR_PORT) 9600

clean:
	@$(ARDUINO_CLI_PATH) cache clean

all: compile upload monitor
