include make/web.mk
include make/firmware.mk
include make/setup.mk

.DEFAULT_GOAL := help

# --- Composite targets ---

.PHONY: test build setup clean help

test: test-firmware test-webapp test-stream

build: build-webapp build-stream build-firmware

setup: setup-prereqs setup-web setup-firmware

clean:
	rm -rf body-firmware/.pio head-firmware/.pio
	rm -rf webapp/.next webapp/node_modules/.cache
	rm -rf stream-client/.next stream-client/node_modules/.cache

# --- Help ---

help:
	@echo ""
	@echo "  Development"
	@echo "    make dev              Start all services (webapp + stream + relay)"
	@echo "    make dev-webapp       Webapp only (port 3003)"
	@echo "    make dev-stream       Stream client only (port 3004)"
	@echo "    make dev-relay        Relay server only (port 3005)"
	@echo "    make dev-stop         Stop all dev servers (ports 3003-3005)"
	@echo ""
	@echo "  Testing"
	@echo "    make test               Run all tests"
	@echo "    make test-firmware      All firmware native tests (body + head)"
	@echo "    make test-firmware-body Body native tests"
	@echo "    make test-firmware-head Head native tests"
	@echo "    make test-firmware-body-hw  Body tests on hardware"
	@echo "    make test-webapp        Webapp vitest"
	@echo "    make test-stream        Stream client vitest"
	@echo ""
	@echo "  Building"
	@echo "    make build              Build everything"
	@echo "    make build-webapp       Build webapp"
	@echo "    make build-stream       Build stream client"
	@echo "    make build-firmware     Compile all firmware (no upload)"
	@echo "    make build-firmware-body  Compile body firmware"
	@echo "    make build-firmware-head  Compile head firmware"
	@echo ""
	@echo "  Code quality"
	@echo "    make lint               ESLint both web projects"
	@echo "    make typecheck          TypeScript type checking"
	@echo ""
	@echo "  Firmware"
	@echo "    make flash              Upload body firmware"
	@echo "    make flash-head         Upload head firmware"
	@echo "    make monitor            Body serial monitor (115200)"
	@echo "    make monitor-head       Head serial monitor (115200)"
	@echo ""
	@echo "  Setup"
	@echo "    make setup            Full setup (prereqs + web + firmware)"
	@echo "    make setup-prereqs    Check/install system dependencies"
	@echo "    make setup-web        Install pnpm dependencies"
	@echo "    make setup-firmware   Install PlatformIO + firmware libs"
	@echo ""
	@echo "  Other"
	@echo "    make clean            Remove build artifacts"
	@echo ""
