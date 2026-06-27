# System requirements

Python and pip handle the Python dependencies. The following system packages
must be installed separately before `pip install -r requirements.txt`.

---

## Linux (Ubuntu 22.04 / 24.04 — recommended)

```bash
sudo apt-get update && sudo apt-get install -y \
    # Headless MuJoCo rendering (no display needed)
    libosmesa6 \
    # Video encoding for render output
    ffmpeg \
    # Feetech serial bus (real hardware only)
    # (usually already present)
    libusb-1.0-0 \
    # Git (for fetching Menagerie assets)
    git
```

Add your user to the `dialout` group for `/dev/ttyUSB*` access (real arm only):

```bash
sudo usermod -aG dialout $USER
# Log out and back in, or run:
newgrp dialout
```

---

## macOS (Apple Silicon or Intel)

```bash
brew install ffmpeg
```

MuJoCo on macOS uses the Metal backend — `libosmesa6` is not needed.
Set the render backend:

```bash
export MUJOCO_GL=glfw   # or: export MUJOCO_GL=egl
```

---

## Windows (WSL2 recommended)

Run Ubuntu 22.04 under WSL2 and follow the Linux instructions above.
Native Windows support for MuJoCo rendering requires additional setup
(see [MuJoCo docs](https://mujoco.readthedocs.io/)).

---

## Docker (production / CI)

The `Dockerfile` handles all of the above. Build and run:

```bash
make docker-build
make docker-run
```

---

## Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `MUJOCO_GL` | `osmesa` | Render backend. `osmesa` = headless, `glfw` = window, `egl` = GPU headless |
| `HF_TOKEN` | — | Hugging Face token (only needed if pushing private datasets) |
