FIRMWARE_DIR := body-firmware

# --- Build ---

.PHONY: build-firmware

build-firmware:
	cd $(FIRMWARE_DIR) && pio run -e wemos_d1_uno32

# --- Test ---

.PHONY: test-firmware test-firmware-head

test-firmware:
	cd $(FIRMWARE_DIR) && pio test -e native

test-firmware-head:
	cd $(FIRMWARE_DIR) && pio test -e wemos_d1_uno32

# --- Flash & monitor ---

.PHONY: flash monitor

flash:
	cd $(FIRMWARE_DIR) && pio run -t upload

monitor:
	cd $(FIRMWARE_DIR) && pio device monitor -b 115200
