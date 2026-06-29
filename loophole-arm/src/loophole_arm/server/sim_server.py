"""The simulation server — one process holds the MuJoCo window; many clients connect.

Architecture:

    main thread:                physics loop @ ~500 Hz + viewer sync
    accept thread:              listens for new client connections
    one thread per client:      reads requests, dispatches, writes responses

Concurrency model: the physics loop runs on the main thread. All client-driven
mutations (``send_joint_targets``, ``set_gripper``, ``enable``/``estop``) go
through a single lock around the shared MuJoCo ``data`` so the physics step and
client writes never interleave. Reads (``joint_positions`` etc.) take the same
lock briefly to snapshot a consistent state.

Each robot in the scene gets a name (e.g. ``"arm_a"``) and is exposed as a
separate endpoint. Multiple clients can connect to different robots
simultaneously; the server routes each request to the right per-robot backend.
"""
from __future__ import annotations

import logging
import socketserver
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import mujoco

from loophole_arm.control.interface import RobotInterface
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.server.protocol import (
    PROTOCOL_VERSION,
    Request,
    Response,
    parse_request,
    serialise_response,
)

logger = logging.getLogger("loophole_arm.server")


@dataclass
class RobotEndpoint:
    """One named robot that clients can connect to.

    The ``backend`` here is the *server-side* RobotInterface — typically a
    :class:`SimBackend` wrapped in :class:`SafetyBackend`, but it could also be
    a :class:`HardwareBackend` (then the server fronts a real arm and clients
    don't know the difference).
    """
    name: str
    backend: RobotInterface
    solver: TCPSolver | None = None


@dataclass
class SimServer:
    """The shared-simulation host.

    Parameters
    ----------
    model:
        The compiled MuJoCo model (already includes all robots + scene).
    data:
        The MuJoCo data state (mutated by the physics loop and by clients).
    endpoints:
        Mapping ``robot_name -> RobotEndpoint``. Built once at startup.
    host, port:
        Where the TCP server listens.
    model_path:
        Filesystem path to the compiled XML, sent to clients on hello so they
        can load the same kinematic model locally for FK.
    physics_hz:
        Target rate for the physics loop. Default 500 Hz — well above the
        control-tick rate so client commands resolve smoothly.
    """
    model: mujoco.MjModel
    data: mujoco.MjData
    endpoints: dict[str, RobotEndpoint]
    host: str = "127.0.0.1"
    port: int = 8765
    model_path: str = ""
    physics_hz: float = 500.0
    _stop: threading.Event = field(default_factory=threading.Event, repr=False)
    _state_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _tcp_server: socketserver.ThreadingTCPServer | None = field(default=None, repr=False)
    _viewer: Any = field(default=None, repr=False)

    # ── Lifecycle ───────────────────────────────────────────────────────
    def run(self) -> None:
        """Start TCP listener + physics loop; open the viewer if a display exists."""
        for ep in self.endpoints.values():
            if not ep.backend.is_connected:
                ep.backend.connect()
        self._start_tcp_listener()
        self._open_viewer_if_possible()
        try:
            self._physics_loop()
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._stop.set()
        if self._tcp_server is not None:
            self._tcp_server.shutdown()
            self._tcp_server.server_close()
            self._tcp_server = None
        if self._viewer is not None:
            import contextlib
            with contextlib.suppress(Exception):
                self._viewer.close()
            self._viewer = None
        for ep in self.endpoints.values():
            if ep.backend.is_connected:
                ep.backend.disconnect()

    # ── Networking ──────────────────────────────────────────────────────
    def _start_tcp_listener(self) -> None:
        server = self
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        socketserver.ThreadingTCPServer.daemon_threads = True

        class Handler(socketserver.StreamRequestHandler):
            def handle(self_h) -> None:  # noqa: N805
                addr = self_h.client_address
                logger.info("client connected: %s", addr)
                try:
                    while not server._stop.is_set():
                        line = self_h.rfile.readline()
                        if not line:
                            break
                        try:
                            req = parse_request(line)
                            resp = server._dispatch(req)
                        except Exception as e:
                            resp = Response(ok=False, error=f"{type(e).__name__}: {e}")
                        self_h.wfile.write(serialise_response(resp))
                        self_h.wfile.flush()
                finally:
                    logger.info("client disconnected: %s", addr)

        self._tcp_server = socketserver.ThreadingTCPServer((self.host, self.port), Handler)
        t = threading.Thread(target=self._tcp_server.serve_forever, daemon=True)
        t.start()
        logger.info("sim server listening on %s:%d", self.host, self.port)

    # ── Request dispatch ────────────────────────────────────────────────
    def _dispatch(self, req: Request) -> Response:
        # ── Server-level ops (no specific robot) ────────────────────────
        if req.op == "hello":
            return self._on_hello(req)
        if req.op == "list_robots":
            return Response(ok=True, value=sorted(self.endpoints.keys()))

        # ── Robot-scoped ops ────────────────────────────────────────────
        ep = self.endpoints.get(req.robot)
        if ep is None:
            return Response(ok=False, error=f"unknown robot {req.robot!r}")
        backend = ep.backend

        try:
            with self._state_lock:
                if req.op == "joint_positions":
                    return Response(ok=True, value=backend.joint_positions.tolist())
                if req.op == "joint_velocities":
                    return Response(ok=True, value=backend.joint_velocities.tolist())
                if req.op == "end_effector_pose":
                    return Response(ok=True, value=backend.end_effector_pose().tolist())
                if req.op == "n_arm_joints":
                    return Response(ok=True, value=backend.n_arm_joints)
                if req.op == "send_joint_targets":
                    backend.send_joint_targets(req.args["targets"])
                    return Response(ok=True)
                if req.op == "set_gripper":
                    backend.set_gripper(req.args["closed_fraction"])
                    return Response(ok=True)
                if req.op == "enable":
                    fn = getattr(backend, "enable", None)
                    if fn is not None:
                        fn()
                    return Response(ok=True)
                if req.op == "estop":
                    fn = getattr(backend, "estop", None)
                    if fn is not None:
                        fn()
                    return Response(ok=True)
                if req.op == "reset_safety":
                    fn = getattr(backend, "reset", None)
                    if fn is not None:
                        fn()
                    return Response(ok=True)
                if req.op == "state":
                    s = getattr(backend, "state", None)
                    return Response(ok=True, value=str(s.value) if s is not None else "operational")
        except Exception as e:
            return Response(ok=False, error=f"{type(e).__name__}: {e}")

        return Response(ok=False, error=f"unknown op {req.op!r}")

    def _on_hello(self, req: Request) -> Response:
        if not req.robot:
            return Response(ok=True, value={
                "version": PROTOCOL_VERSION,
                "robots": sorted(self.endpoints.keys()),
            })
        ep = self.endpoints.get(req.robot)
        if ep is None:
            return Response(ok=False, error=f"unknown robot {req.robot!r}; "
                                            f"available: {sorted(self.endpoints.keys())}")
        # Walk through any decorators (e.g. SafetyBackend) to the concrete backend,
        # which carries the prefixed joint/site/actuator names the client needs to
        # build a local IK solver. The decorators don't add or change these names.
        inner = ep.backend
        while hasattr(inner, "_inner"):
            inner = inner._inner
        return Response(ok=True, value={
            "version": PROTOCOL_VERSION,
            "n_arm_joints": ep.backend.n_arm_joints,
            "model_path": self.model_path,
            "arm_joint_names": list(getattr(inner, "arm_joint_names", [])),
            "gripper_actuator": getattr(inner, "gripper_actuator", ""),
            "tcp_site": getattr(inner, "tcp_site", ""),
        })

    # ── Physics loop ────────────────────────────────────────────────────
    def _physics_loop(self) -> None:
        dt = 1.0 / self.physics_hz
        next_tick = time.perf_counter()
        sync_every = max(1, int(self.physics_hz / 30))  # viewer @ ~30 Hz
        i = 0
        try:
            while not self._stop.is_set():
                with self._state_lock:
                    n = max(1, int(dt / self.model.opt.timestep))
                    for _ in range(n):
                        mujoco.mj_step(self.model, self.data)
                if self._viewer is not None and i % sync_every == 0:
                    try:
                        if not self._viewer.is_running():
                            logger.info("viewer closed; shutting down server")
                            break
                        self._viewer.sync()
                    except Exception:
                        pass
                i += 1
                next_tick += dt
                slack = next_tick - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                else:
                    next_tick = time.perf_counter()
        except KeyboardInterrupt:
            logger.info("interrupted; shutting down")

    # ── Viewer (optional) ───────────────────────────────────────────────
    def _open_viewer_if_possible(self) -> None:
        import os
        import platform
        if platform.system() == "Linux" and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            logger.info("no display detected — running headless")
            return
        try:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.cam.distance = 1.0
            self._viewer.cam.azimuth = 135
            self._viewer.cam.elevation = -22
            self._viewer.cam.lookat[:] = [0.20, 0.0, 0.15]
        except Exception as e:
            logger.info("could not open viewer (%s); running headless", e)
