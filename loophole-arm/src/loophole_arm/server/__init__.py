"""Server/client architecture for multi-terminal robot control.

  Terminal 1:  loophole-armd  (the sim server, holds the MuJoCo window)
  Terminal N:  any client     (teach, teleop, replay, etc.)

The server hosts the simulation and exposes each robot in the scene as a
named endpoint over a local TCP socket. Clients connect by name and drive
the robot through a remote :class:`RobotInterface` — identical to how they
would drive a local :class:`SimBackend` or :class:`HardwareBackend`.

The wire protocol is intentionally small: it is exactly the read/write
surface of :class:`RobotInterface`, serialised as line-delimited JSON.
"""
from loophole_arm.server.protocol import (
    PROTOCOL_VERSION,
    Request,
    Response,
    parse_request,
    parse_response,
    serialise_request,
    serialise_response,
)
from loophole_arm.server.remote_backend import RemoteBackend

__all__ = [
    "PROTOCOL_VERSION",
    "RemoteBackend",
    "Request",
    "Response",
    "parse_request",
    "parse_response",
    "serialise_request",
    "serialise_response",
]
