"""Offscreen MuJoCo rendering — turns a trajectory into a video file."""
from __future__ import annotations

import logging
import os
from pathlib import Path

import imageio
import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.env import CupLiftEnv

logger = logging.getLogger(__name__)


def render_trajectory(
    env: CupLiftEnv,
    params: NDArray[np.float64],
    output_path: Path,
    *,
    resolution: tuple[int, int] = (640, 480),
    fps: int = 30,
    camera: str | None = None,
    gl_backend: str | None = "osmesa",
) -> Path:
    """Replay ``params`` through ``env`` and save an MP4 to ``output_path``.

    Parameters
    ----------
    env:
        A :class:`CupLiftEnv`. Mutated in place during the rollout.
    params:
        Trajectory genome.
    output_path:
        Destination MP4 path. Parent directories are created if missing.
    resolution:
        ``(width, height)`` in pixels.
    fps:
        Frames per second in the output video.
    camera:
        Named camera in the model. ``None`` uses a free camera positioned for
        a typical UR5e tabletop shot.
    gl_backend:
        Value to set for ``MUJOCO_GL``. ``"osmesa"`` works headlessly on Linux.
    """
    if gl_backend is not None:
        os.environ["MUJOCO_GL"] = gl_backend

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    m, d = env.model, env.data
    width, height = resolution

    renderer = mujoco.Renderer(m, height=height, width=width)
    cam = mujoco.MjvCamera()
    if camera is not None:
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = m.camera(camera).id
    else:
        cam.distance = 1.6
        cam.azimuth = 135.0
        cam.elevation = -22.0
        cam.lookat[:] = [0.4, 0.0, 0.35]

    mujoco.mj_resetData(m, d)
    d.qpos[:6] = env.scene.home_qpos
    mujoco.mj_forward(m, d)

    setpoints = env.decode(params)
    dt = m.opt.timestep
    steps_per_wp = max(1, int((env.sim_seconds / env.n_waypoints) / dt))
    frame_every = max(1, int((1.0 / fps) / dt))

    frames = []
    step_idx = 0
    for w in range(env.n_waypoints):
        d.ctrl[:] = setpoints[w]
        for _ in range(steps_per_wp):
            mujoco.mj_step(m, d)
            if step_idx % frame_every == 0:
                renderer.update_scene(d, cam)
                frames.append(renderer.render())
            step_idx += 1

    imageio.mimsave(
        str(output_path),
        frames,
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=1,
    )
    logger.info("wrote %s (%d frames)", output_path, len(frames))
    return output_path
