"""Wire protocol for the sim server.

Line-delimited JSON over TCP. Each line is one :class:`Request` from a client
or one :class:`Response` from the server. The protocol is deliberately small —
it is exactly the read/write surface of :class:`RobotInterface`, plus a
``hello`` handshake. No framing tricks, no length prefixes; one JSON object per
line so the protocol is human-readable in ``netcat``.

Versioning: every message includes the protocol version. A server rejects a
client whose major version does not match. Bumping the major version is a
breaking change; minor bumps are additive.

This module is dependency-free (no MuJoCo, no LeRobot) so it can be imported by
the thinnest client without dragging the simulator in.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Literal

PROTOCOL_VERSION = "1.0"


# ── Request types ───────────────────────────────────────────────────────
# A single request always carries an ``op`` field selecting the method, plus
# whatever payload that method needs. Keep this list aligned with
# :class:`RobotInterface` — that is the whole point.
RequestOp = Literal[
    "hello",                  # handshake — version negotiation, robot name
    "list_robots",            # introspection — what robots are available?
    "joint_positions",        # read
    "joint_velocities",       # read
    "end_effector_pose",      # read
    "send_joint_targets",     # write
    "set_gripper",            # write
    "enable",                 # safety
    "estop",                  # safety
    "reset_safety",           # safety
    "state",                  # safety supervisor state (idle/operational/...)
    "n_arm_joints",           # introspection
]


@dataclass
class Request:
    """A client → server message."""

    op: RequestOp
    robot: str = ""                       # which robot endpoint (empty for hello/list)
    args: dict[str, Any] = field(default_factory=dict)
    version: str = PROTOCOL_VERSION


@dataclass
class Response:
    """A server → client reply.

    On success, ``ok=True`` and ``value`` carries the result (if any).
    On failure, ``ok=False`` and ``error`` carries a human-readable reason.
    """

    ok: bool
    value: Any = None
    error: str = ""
    version: str = PROTOCOL_VERSION


# ── Serialisation helpers (line-delimited JSON) ─────────────────────────
def serialise_request(req: Request) -> bytes:
    """Encode a request to a single newline-terminated JSON line."""
    return (json.dumps(asdict(req)) + "\n").encode("utf-8")


def serialise_response(resp: Response) -> bytes:
    return (json.dumps(asdict(resp)) + "\n").encode("utf-8")


def parse_request(line: bytes | str) -> Request:
    """Parse one line into a :class:`Request`. Raises on malformed input."""
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    data = json.loads(line)
    if "op" not in data:
        raise ValueError("request missing 'op' field")
    return Request(
        op=data["op"],
        robot=data.get("robot", ""),
        args=data.get("args", {}),
        version=data.get("version", PROTOCOL_VERSION),
    )


def parse_response(line: bytes | str) -> Response:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    data = json.loads(line)
    return Response(
        ok=bool(data.get("ok", False)),
        value=data.get("value"),
        error=data.get("error", ""),
        version=data.get("version", PROTOCOL_VERSION),
    )


# ── Version compatibility ───────────────────────────────────────────────
def versions_compatible(client: str, server: str = PROTOCOL_VERSION) -> bool:
    """True if client and server share a major version."""
    return client.split(".")[0] == server.split(".")[0]
