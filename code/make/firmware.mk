BODY_FW_DIR := body-firmware
HEAD_FW_DIR := head-firmware

# --- Body firmware ---

.PHONY: build-firmware-body test-firmware-body test-firmware-body-hw flash-body monitor-body

build-firmware-body:
	cd $(BODY_FW_DIR) && pio run -e wemos_d1_uno32

test-firmware-body:
	cd $(BODY_FW_DIR) && pio test -e native

test-firmware-body-hw:
	cd $(BODY_FW_DIR) && pio test -e wemos_d1_uno32

flash-body:
	cd $(BODY_FW_DIR) && pio run -e wemos_d1_uno32 -t upload

monitor-body:
	cd $(BODY_FW_DIR) && pio device monitor -b 115200

# --- Head firmware ---

.PHONY: build-firmware-head test-firmware-head flash-head monitor-head

build-firmware-head:
	cd $(HEAD_FW_DIR) && pio run -e seeed_xiao_esp32s3

test-firmware-head:
	cd $(HEAD_FW_DIR) && pio test -e native

flash-head:
	cd $(HEAD_FW_DIR) && pio run -e seeed_xiao_esp32s3 -t upload

monitor-head:
	cd $(HEAD_FW_DIR) && pio device monitor -b 115200

# --- Composites (backward-compat aliases) ---

.PHONY: build-firmware test-firmware flash monitor

build-firmware: build-firmware-body build-firmware-head

test-firmware: test-firmware-body test-firmware-head

flash: flash-body

monitor: monitor-body
