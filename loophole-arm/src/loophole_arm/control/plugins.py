"""Plugin Manager: the backend registry.

Per the PRD: "Separate hardware from software." A backend plugin is any class
implementing :class:`RobotInterface`. The registry maps a short name to that
class so configuration files and CLIs can say ``backend: sim`` or
``backend: mock`` without importing implementation modules.

Deliberately small: a dict plus two functions. If a third party ever ships a
backend as a separate pip package, this can grow entry-point discovery; until
then a dict is the whole feature (simplicity before complexity).

Built-in plugins:

============  ==========================================  =================
name          class                                       notes
============  ==========================================  =================
sim           loophole_arm.control.sim_backend.SimBackend needs MuJoCo model
hardware      ...hardware_backend.HardwareBackend         needs LeRobot deps
mock          ...mock_backend.MockBackend                 no deps, instant
remote        loophole_arm.server.remote_backend.
              RemoteBackend                               talks to loophole-armd
============  ==========================================  =================
"""
from __future__ import annotations

import logging
from collections.abc import Callable

from loophole_arm.control.interface import RobotInterface

logger = logging.getLogger("loophole_arm.plugins")

# name -> zero-argument-importable factory returning the backend CLASS.
# Lazy imports keep optional dependencies (LeRobot, MuJoCo viewer) out of
# processes that never use them.
_REGISTRY: dict[str, Callable[[], type[RobotInterface]]] = {}


class UnknownBackendError(KeyError):
    """Raised when no backend is registered under the requested name."""


def register_backend(name: str, loader: Callable[[], type[RobotInterface]]) -> None:
    """Register a backend class under ``name``.

    ``loader`` is a zero-arg callable returning the class, so registration
    itself imports nothing heavy.
    """
    if not name:
        raise ValueError("backend name cannot be empty")
    _REGISTRY[name] = loader
    logger.debug("registered backend plugin %r", name)


def available_backends() -> list[str]:
    return sorted(_REGISTRY)


def get_backend_class(name: str) -> type[RobotInterface]:
    """Resolve a backend name to its class (importing lazily)."""
    if name not in _REGISTRY:
        raise UnknownBackendError(
            f"no backend named {name!r}; available: {available_backends()}"
        )
    return _REGISTRY[name]()


def create_backend(name: str, **kwargs) -> RobotInterface:
    """Instantiate a backend by name with its constructor kwargs."""
    cls = get_backend_class(name)
    return cls(**kwargs)


# ── Built-ins ───────────────────────────────────────────────────────────
def _load_sim() -> type[RobotInterface]:
    from loophole_arm.control.sim_backend import SimBackend
    return SimBackend


def _load_hardware() -> type[RobotInterface]:
    from loophole_arm.control.hardware_backend import HardwareBackend
    return HardwareBackend


def _load_mock() -> type[RobotInterface]:
    from loophole_arm.control.mock_backend import MockBackend
    return MockBackend


def _load_remote() -> type[RobotInterface]:
    from loophole_arm.server.remote_backend import RemoteBackend
    return RemoteBackend


register_backend("sim", _load_sim)
register_backend("hardware", _load_hardware)
register_backend("mock", _load_mock)
register_backend("remote", _load_remote)
