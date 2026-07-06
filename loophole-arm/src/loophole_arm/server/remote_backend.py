"""RemoteBackend — the client-side :class:`RobotInterface` implementation.

A client speaks to the sim server over TCP using the line-delimited JSON
protocol. By implementing :class:`RobotInterface` it slots in wherever
:class:`SimBackend` or :class:`HardwareBackend` would go — the controller, IK
solver, safety layer, and teach product all work against it unchanged.

This is the whole architectural point: distributing across processes does not
require changing any control code, because the seam is the interface, not a
specific transport.

Reading the kinematic model: the server sends the *path* to the compiled
model on first connect, and the client loads it locally for FK. (We do not
stream meshes over the wire.) That means the client machine must have access
to the same arm assets — which is true today since clients run on the same
machine as the server, and stays true when we add a hardware host.
"""
from __future__ import annotations

import socket
import threading
import time
from collections.abc import Sequence
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.control.interface import RobotInterface
from loophole_arm.server.protocol import (
    PROTOCOL_VERSION,
    Request,
    Response,
    parse_response,
    serialise_request,
    versions_compatible,
)


class RemoteConnectionError(RuntimeError):
    """Raised when the connection to the sim server fails."""


class RemoteBackend(RobotInterface):
    """Drives a named robot on a remote sim server.

    Parameters
    ----------
    robot_name:
        The endpoint name registered with the server (e.g. ``"arm_a"``).
    host, port:
        Where the server listens. Localhost is the common case.
    timeout:
        Per-request socket timeout in seconds.
    """

    def __init__(
        self,
        robot_name: str,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout: float = 5.0,
    ) -> None:
        self.robot_name = robot_name
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._reader: Any = None       # makefile('rb') — buffered line reader
        self._lock = threading.Lock()  # serialise request/response on one socket
        self._connected = False
        self._n_arm_joints = 0
        self._kinematic_model: mujoco.MjModel | None = None
        self._model_path = ""
        # Prefixed names the server reports on hello — clients use these to wire
        # a local IK solver against the same names the server is driving.
        self.arm_joint_names: list[str] = []
        self.gripper_actuator: str = ""
        self.tcp_site: str = ""
        self.home_pose: tuple[float, ...] = ()

    # ── Lifecycle ───────────────────────────────────────────────────────
    def connect(self) -> None:
        """Open the socket and complete the hello handshake."""
        try:
            self._sock = socket.create_connection((self.host, self.port), self.timeout)
        except OSError as e:
            raise RemoteConnectionError(
                f"could not reach sim server at {self.host}:{self.port} — "
                f"is loophole-armd running?  ({e})"
            ) from e
        self._sock.settimeout(self.timeout)
        self._reader = self._sock.makefile("rb")

        hello = self._call(Request(op="hello", robot=self.robot_name))
        self._n_arm_joints = int(hello["n_arm_joints"])
        self._model_path = str(hello.get("model_path", ""))
        self.arm_joint_names = list(hello.get("arm_joint_names", []))
        self.gripper_actuator = str(hello.get("gripper_actuator", ""))
        self.tcp_site = str(hello.get("tcp_site", ""))
        self.home_pose = tuple(float(v) for v in hello.get("home", []))
        if not versions_compatible(hello.get("version", "0.0")):
            raise RemoteConnectionError(
                f"server protocol {hello.get('version')} incompatible with client {PROTOCOL_VERSION}"
            )
        # Load the kinematic model locally for FK if a path was provided.
        # If loading fails (e.g. mesh paths can't be resolved on this machine),
        # we degrade gracefully — the client just won't expose FK. Most clients
        # (teach, teleop) only need joint state and command rpc, not FK.
        if self._model_path:
            try:
                self._kinematic_model = mujoco.MjModel.from_xml_path(self._model_path)
            except Exception as e:
                # Not fatal — the client can still send commands and read state.
                # FK calls will raise a clear error if attempted.
                import logging
                logging.getLogger(__name__).warning(
                    "could not load remote kinematic model (%s); FK disabled on this client", e,
                )
        self._connected = True

    def disconnect(self) -> None:
        import contextlib
        if self._reader is not None:
            with contextlib.suppress(Exception):
                self._reader.close()
        if self._sock is not None:
            with contextlib.suppress(Exception):
                self._sock.close()
        self._sock = None
        self._reader = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Introspection ───────────────────────────────────────────────────
    @property
    def n_arm_joints(self) -> int:
        return self._n_arm_joints

    def kinematic_model(self) -> mujoco.MjModel:
        if self._kinematic_model is None:
            raise RuntimeError(
                "remote backend has no local kinematic model — "
                "did the server send a model_path in the hello reply?"
            )
        return self._kinematic_model

    # ── State (read) ────────────────────────────────────────────────────
    @property
    def joint_positions(self) -> NDArray[np.float64]:
        return np.asarray(self._call(Request(op="joint_positions", robot=self.robot_name)), dtype=float)

    @property
    def joint_velocities(self) -> NDArray[np.float64]:
        return np.asarray(self._call(Request(op="joint_velocities", robot=self.robot_name)), dtype=float)

    def end_effector_pose(self) -> NDArray[np.float64]:
        return np.asarray(self._call(Request(op="end_effector_pose", robot=self.robot_name)), dtype=float)

    # ── Commands (write) ────────────────────────────────────────────────
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        self._call(Request(
            op="send_joint_targets",
            robot=self.robot_name,
            args={"targets": [float(t) for t in targets]},
        ))

    def set_gripper(self, closed_fraction: float) -> None:
        self._call(Request(
            op="set_gripper",
            robot=self.robot_name,
            args={"closed_fraction": float(closed_fraction)},
        ))

    # ── Perception ──────────────────────────────────────────────────────
    def object_positions(self) -> dict[str, list[float]]:
        """Live object poses, served by the simulation over the wire."""
        value = self._call(Request(op="object_positions", robot=self.robot_name))
        return {str(k): [float(x) for x in v] for k, v in (value or {}).items()}

    # ── Timing ──────────────────────────────────────────────────────────
    def step(self, dt: float) -> None:
        """Block until ``dt`` has elapsed.

        Physics on the server runs continuously, so the client just paces its
        control-loop rate. We do *not* tick the server here — the server owns
        time. This matches what real hardware does (you block until the next
        servo control tick), so the same controller code drives sim and real.
        """
        time.sleep(dt)

    # ── Optional safety control pass-throughs ───────────────────────────
    def enable(self) -> None:
        self._call(Request(op="enable", robot=self.robot_name))

    def estop(self) -> None:
        self._call(Request(op="estop", robot=self.robot_name))

    def reset(self) -> None:
        self._call(Request(op="reset_safety", robot=self.robot_name))

    @property
    def state(self) -> str:
        return str(self._call(Request(op="state", robot=self.robot_name)))

    # ── Wire I/O ────────────────────────────────────────────────────────
    def _call(self, req: Request) -> Any:
        """Send one request, await one response. Thread-safe per backend."""
        if self._sock is None or self._reader is None:
            raise RemoteConnectionError("not connected; call connect() first")
        with self._lock:
            try:
                self._sock.sendall(serialise_request(req))
                line = self._reader.readline()
            except OSError as e:
                self._connected = False
                raise RemoteConnectionError(f"connection to server lost: {e}") from e
        if not line:
            self._connected = False
            raise RemoteConnectionError("server closed the connection")
        resp: Response = parse_response(line)
        if not resp.ok:
            raise RuntimeError(f"server error on {req.op}: {resp.error}")
        return resp.value
