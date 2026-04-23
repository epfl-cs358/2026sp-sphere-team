PNPM := pnpm

# --- Dev ---

DEV_PORTS := 3003 3004 3005

.PHONY: dev dev-webapp dev-stream dev-relay dev-stop

dev:
	$(PNPM) dev:webapp & $(PNPM) dev:stream & $(PNPM) dev:relay & wait

dev-webapp:
	$(PNPM) --filter webapp dev

dev-stream:
	$(PNPM) --filter stream-client dev

dev-relay:
	$(PNPM) --filter stream-client relay

dev-stop:
	@for port in $(DEV_PORTS); do \
		pid=$$(lsof -ti :$$port 2>/dev/null); \
		if [ -n "$$pid" ]; then \
			kill $$pid 2>/dev/null && echo "Stopped process on port $$port (pid $$pid)"; \
		fi; \
	done
	@echo "All dev servers stopped."

# --- Build ---

.PHONY: build-webapp build-stream

build-webapp:
	$(PNPM) --filter webapp build

build-stream:
	$(PNPM) --filter stream-client build

# --- Test ---

.PHONY: test-webapp test-stream

test-webapp:
	$(PNPM) --filter webapp test

test-stream:
	$(PNPM) --filter stream-client test

# --- Code quality ---

.PHONY: lint typecheck

lint:
	$(PNPM) --filter webapp lint
	$(PNPM) --filter stream-client lint

typecheck:
	$(PNPM) --filter webapp exec tsc --noEmit
	$(PNPM) --filter stream-client exec tsc --noEmit
