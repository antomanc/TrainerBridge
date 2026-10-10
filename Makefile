FQBN ?= esp32:esp32:esp32
PORT ?= /dev/ttyUSB0
ESP32_CORE := 3.3.12
NIMBLE := 2.5.1
ESP32_INDEX := https://espressif.github.io/arduino-esp32/package_esp32_index.json

.PHONY: check test sanitize deps compile upload monitor

check: sanitize compile ## Host tests with sanitizers, then the real firmware build

test: ## Host regression tests (C++17 compiler only)
	bash tests/run.sh

sanitize: ## Host tests with AddressSanitizer + UBSan
	SANITIZE=1 bash tests/run.sh

deps: ## Install the pinned ESP32 core and NimBLE-Arduino
	arduino-cli core update-index --additional-urls $(ESP32_INDEX)
	arduino-cli core install esp32:esp32@$(ESP32_CORE) --additional-urls $(ESP32_INDEX)
	arduino-cli lib install NimBLE-Arduino@$(NIMBLE)

compile: ## Build the firmware with all warnings
	arduino-cli compile --warnings all --fqbn $(FQBN) TrainerBridge

upload: compile ## Build and flash (PORT=/dev/ttyUSB0)
	arduino-cli upload --port $(PORT) --fqbn $(FQBN) TrainerBridge

monitor: ## Serial console at 115200 baud
	arduino-cli monitor --port $(PORT) --config baudrate=115200
