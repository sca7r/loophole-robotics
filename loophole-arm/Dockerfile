# syntax=docker/dockerfile:1.7
# ──────────────────────────────────────────────────────────────────────────
# Loophole Arm production image
#
# Multi-stage build:
#   1. ``builder``  — installs the wheel + hardware extras into a venv.
#   2. ``runtime``  — copies the venv onto a slim base; non-root user; no
#                     build tools in the final layer.
#
# Targets two scenarios:
#   • Sim-only inference / training (CPU, no USB).
#   • Real-hardware deployment on an industrial PC / Jetson, with /dev/ttyUSB*
#     and optional cameras mapped in at runtime.
#
# Build:        docker build -t loophole-arm:dev .
# Sim run:      docker run --rm loophole-arm:dev loophole-arm-sim scene
# Hardware run: docker run --rm --device=/dev/ttyUSB0 loophole-arm:dev \
#                          lerobot-record --robot.type=loophole_arm \
#                          --robot.port=/dev/ttyUSB0 ...
# ──────────────────────────────────────────────────────────────────────────

ARG PYTHON_VERSION=3.12

FROM python:${PYTHON_VERSION}-slim AS builder

ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        libffi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY pyproject.toml README.md LICENSE ./
COPY src/ src/
COPY assets/ assets/

# Build the wheel & install into a self-contained venv that we'll copy.
RUN python -m venv /opt/venv \
 && /opt/venv/bin/pip install --upgrade pip build \
 && /opt/venv/bin/python -m build --wheel --outdir /tmp/dist \
 && /opt/venv/bin/pip install /tmp/dist/*.whl

# ──────────────────────────────────────────────────────────────────────────
FROM python:${PYTHON_VERSION}-slim AS runtime

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PATH="/opt/venv/bin:$PATH" \
    MUJOCO_GL=egl

# Minimal runtime deps:
#   • libgl1 / libglib2.0-0 — MuJoCo rendering
#   • libusb-1.0-0          — Feetech serial bus
#   • ffmpeg                — LeRobotDataset video encoding
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libusb-1.0-0 \
        ffmpeg \
        tini \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with dialout group for /dev/ttyUSB* access.
RUN groupadd -g 1000 robot \
 && useradd -u 1000 -g robot -G dialout -m -s /bin/bash robot

COPY --from=builder /opt/venv /opt/venv

# Vendored assets — needed for the sim layer at runtime.
COPY --from=builder /build/assets /opt/loophole-arm/assets

USER robot
WORKDIR /home/robot

# Healthcheck: the sim CLI is the lightest end-to-end smoke test.
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD loophole-arm-sim --version || exit 1

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["loophole-arm-sim", "--help"]
