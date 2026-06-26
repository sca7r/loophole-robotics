.PHONY: install dev fetch-assets test lint typecheck format scene train-naive train-shaped render-naive render-shaped clean

PYTHON ?= python3
RUN := $(PYTHON) -m loophole_arm

# ── Setup ─────────────────────────────────────────────────────────────
install:
	$(PYTHON) -m pip install -e .

dev:
	$(PYTHON) -m pip install -e ".[dev]"
	pre-commit install

fetch-assets:
	@scripts/fetch_menagerie.sh

# ── Quality ───────────────────────────────────────────────────────────
test:
	pytest -v

lint:
	ruff check .

typecheck:
	mypy src

format:
	ruff format .
	ruff check --fix .

# ── Run ───────────────────────────────────────────────────────────────
scene:
	$(RUN) scene --inspect

train-naive:
	$(RUN) train --config configs/naive.yaml

train-shaped:
	$(RUN) train --config configs/shaped.yaml

render-naive:
	$(RUN) render --config configs/naive.yaml --out runs/naive.mp4

render-shaped:
	$(RUN) render --config configs/shaped.yaml --out runs/shaped.mp4

# ── Cleanup ───────────────────────────────────────────────────────────
clean:
	rm -rf build dist *.egg-info .pytest_cache .mypy_cache .ruff_cache
	find . -type d -name __pycache__ -exec rm -rf {} +
