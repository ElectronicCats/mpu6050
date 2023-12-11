# BOARD_TAG = arduino:mbed_rp2040:pico
BOARD_TAG = rp2040:rp2040:rpipico
MONITOR_PORT = /dev/cu.usbmodem11401
ARDUINO_CLI_PATH := arduino-cli

help:
	@echo "Usage: make [target] [BOARD=rp2040|esp32|esp32s3]"
	@echo "Targets:"
	@echo "  compile: compile the firmware"
	@echo "  upload: upload the firmware to the board"
	@echo "  monitor: monitor the serial port"
	@echo "  clean: clean the cache"
	@echo "  all: compile, upload, monitor"
	@echo "Examples:"
	@echo "  make compile BOARD=rp2040"
	@echo "  make upload BOARD=rp2040"
	@echo "  make all BOARD=esp32s3"

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
