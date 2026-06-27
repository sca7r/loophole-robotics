# Loophole Arm — developer Makefile.
# Real-hardware operations use `lerobot` CLI directly; this Makefile covers
# sim, tooling, and local builds.

.PHONY: help install install-hw dev fetch-assets test test-cov lint format \
        typecheck pre-commit clean docker-build docker-run \
        sim-scene sim-optimize sim-render

PYTHON ?= python3
SIM := $(PYTHON) -m loophole_arm

help:  ## Show this help
	@awk 'BEGIN{FS=":.*##";printf "Loophole Arm — targets\n\n"} /^[a-zA-Z_-]+:.*?##/ {printf "  %-18s %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# ── Install ─────────────────────────────────────────────────────────────
install:  ## Install package (sim only)
	$(PYTHON) -m pip install -e .

install-hw:  ## Install with real-hardware extras (LeRobot + Feetech)
	$(PYTHON) -m pip install -e ".[hardware]"

dev:  ## Install dev tooling + hooks
	$(PYTHON) -m pip install -e ".[dev]"
	pre-commit install

fetch-assets:  ## Vendor UR5e + Robotiq from MuJoCo Menagerie
	bash scripts/fetch_menagerie.sh

# ── Quality ─────────────────────────────────────────────────────────────
test:  ## Run sim tests
	pytest -v

test-cov:  ## Run tests with coverage report
	pytest --cov=loophole_arm --cov-report=term-missing --cov-report=html
	@echo "open htmlcov/index.html"

lint:  ## Lint with ruff
	ruff check .

format:  ## Auto-format with ruff
	ruff format .
	ruff check --fix .

typecheck:  ## Strict type-check with mypy
	mypy src

pre-commit:  ## Run all pre-commit hooks
	pre-commit run --all-files

# ── Sim (the loophole-arm part of the project) ─────────────────────────
sim-scene:  ## Print the composed scene info
	$(SIM) scene

sim-optimize:  ## Run a quick optimization with the shaped reward
	$(SIM) optimize --reward shaped_lift --generations 30

sim-render:  ## Render the most recent best_params.npy to MP4
	$(SIM) render --params $$(ls -td runs/*/ | head -1)best_params.npy --out runs/latest.mp4

# ── Docker ──────────────────────────────────────────────────────────────
docker-build:  ## Build the production image
	docker build -t loophole-arm:dev .

docker-run:  ## Run the sim CLI inside the image
	docker run --rm -it loophole-arm:dev loophole-arm-sim --help

# ── Real hardware (delegates to lerobot CLI) ────────────────────────────
hw-calibrate:  ## Calibrate the connected arm (interactive)
	@command -v lerobot-calibrate >/dev/null 2>&1 || { echo "Install hardware extras: make install-hw"; exit 1; }
	lerobot-calibrate --robot.type=loophole_arm --robot.port=$${PORT:-/dev/ttyUSB0}

hw-teleop:  ## Teleoperate the arm (requires leader arm)
	@command -v lerobot-teleoperate >/dev/null 2>&1 || { echo "Install hardware extras: make install-hw"; exit 1; }
	lerobot-teleoperate \
	  --robot.type=loophole_arm --robot.port=$${PORT:-/dev/ttyUSB0} \
	  --teleop.type=$${TELEOP:-so100_leader} --teleop.port=$${LEADER_PORT:-/dev/ttyUSB1}

# ── Cleanup ─────────────────────────────────────────────────────────────
clean:  ## Remove caches and build artifacts
	rm -rf build dist *.egg-info .pytest_cache .mypy_cache .ruff_cache htmlcov .coverage coverage.xml
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
