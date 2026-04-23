FIRMWARE_DIR := body-firmware

# --- Prereqs ---

.PHONY: setup-prereqs

define check_cmd
	@if ! command -v $(1) >/dev/null 2>&1; then \
		MISSING="$$MISSING $(1)"; \
	fi
endef

setup-prereqs:
	@MISSING=""; \
	command -v node    >/dev/null 2>&1 || MISSING="$$MISSING node"; \
	command -v python3 >/dev/null 2>&1 || MISSING="$$MISSING python3"; \
	command -v pnpm    >/dev/null 2>&1 || MISSING="$$MISSING pnpm"; \
	command -v pip3    >/dev/null 2>&1 || MISSING="$$MISSING pip3"; \
	if [ -z "$$MISSING" ]; then \
		echo "All prerequisites installed."; \
	elif [ "$$(uname)" = "Darwin" ] && command -v brew >/dev/null 2>&1; then \
		echo "Missing:$$MISSING"; \
		printf "Install via Homebrew? [y/N] "; \
		read -r ans; \
		if [ "$$ans" = "y" ] || [ "$$ans" = "Y" ]; then \
			brew install $$MISSING; \
		else \
			echo "Skipped. Install manually:$$MISSING"; \
		fi; \
	else \
		echo "Missing:$$MISSING"; \
		echo "Please install them manually:"; \
		echo "  node    — https://nodejs.org"; \
		echo "  python3 — https://python.org"; \
		echo "  pnpm    — npm install -g pnpm"; \
		echo "  pip3    — included with python3"; \
	fi

# --- Web ---

.PHONY: setup-web

setup-web:
	pnpm install

# --- Firmware ---

.PHONY: setup-firmware

setup-firmware:
	@if ! command -v pio >/dev/null 2>&1; then \
		echo "Installing PlatformIO..."; \
		pip3 install platformio; \
	else \
		echo "PlatformIO already installed."; \
	fi
	cd $(FIRMWARE_DIR) && pio pkg install
