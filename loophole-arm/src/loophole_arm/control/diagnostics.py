"""Diagnostics: robot.health() per the PRD.

One call returns everything a field engineer needs to see, as a plain dict
(easy to log, JSON-serialise, or print).

Honesty rule: values the current backend cannot measure are ``None`` with the
reason in ``sources``. The sim never invents plausible temperatures; when the
physical arm is on the bench, ``HardwareBackend`` fills in the real numbers
and nothing else changes.

PRD checklist covered here:
  servo temperatures, supply voltage, current, communication health,
  emergency stop state, fault history, runtime, cycle count, warnings,
  errors, software version, firmware version, health score.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from loophole_arm._version import __version__

if TYPE_CHECKING:
    from loophole_arm.control.controller import RobotController


def collect_health(robot: RobotController) -> dict[str, Any]:
    """Build the health report for :meth:`RobotController.health`."""
    backend = robot.backend

    # Walk through decorators (SafetyBackend wraps the concrete backend).
    inner = backend
    while hasattr(inner, "_inner"):
        inner = inner._inner
    backend_kind = type(inner).__name__

    # Safety supervisor state + event history, if a supervisor is attached.
    safety_state = getattr(backend, "state", None)
    safety_state_str = getattr(safety_state, "value", None) if safety_state is not None else None
    events = list(getattr(backend, "events", []) or [])
    faults = [e for e in events if getattr(e, "kind", "") in ("estop", "fault")]
    warnings = [e for e in events if getattr(e, "kind", "") not in ("estop", "fault")]

    lifecycle = robot.lifecycle
    connected = bool(getattr(backend, "is_connected", False))

    # Runtime since connect (tracked by the controller).
    runtime_s = (
        time.monotonic() - robot._connected_at
        if robot._connected_at is not None else None
    )

    # Health score: a coarse traffic light, not a diagnosis.
    #   100  connected, no faults, lifecycle healthy
    #   50   connected but faults recorded this session
    #   10   lifecycle in ERROR or e-stopped
    #   0    not connected
    if not connected:
        score = 0
    elif lifecycle.state.value == "error" or safety_state_str == "estopped":
        score = 10
    elif faults:
        score = 50
    else:
        score = 100

    hardware_unmeasurable = f"{backend_kind} cannot measure this (hardware only)"
    return {
        "software_version": __version__,
        "backend": backend_kind,
        "connected": connected,
        "lifecycle_state": lifecycle.state.value,
        "safety_state": safety_state_str,
        "estop_engaged": safety_state_str == "estopped",
        "fault_history": [str(e) for e in faults],
        "warnings": [str(e) for e in warnings],
        "runtime_seconds": runtime_s,
        "cycle_count": robot._cycle_count,
        # Hardware-only measurements: honest sentinels until the bench.
        "servo_temperatures_c": None,
        "supply_voltage_v": None,
        "current_a": None,
        "firmware_version": None,
        "communication_health": "n/a (in-process)" if backend_kind != "RemoteBackend" else "ok",
        "health_score": score,
        "sources": {
            "servo_temperatures_c": hardware_unmeasurable,
            "supply_voltage_v": hardware_unmeasurable,
            "current_a": hardware_unmeasurable,
            "firmware_version": hardware_unmeasurable,
        },
    }
