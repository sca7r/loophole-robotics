"""Loophole Robotics — optimize once with the naive reward, then render the best trajectory to MP4."""
import os
os.environ["MUJOCO_GL"] = "osmesa"
import numpy as np, mujoco, imageio
import optimize as O   # reuse model, rollout, ES

def render(params, path, seconds=O.SIM_SECONDS, fps=30):
    m, d = O.model, O.data
    mujoco.mj_resetData(m, d)
    sp = O.unpack(params)
    steps_per_wp = int(O.HOLD_PER_WP / m.opt.timestep)
    frames, every = [], int((1/fps)/m.opt.timestep)
    r = mujoco.Renderer(m, height=360, width=480)
    cam = mujoco.MjvCamera(); cam.distance = 1.4; cam.azimuth = 135; cam.elevation = -18
    cam.lookat[:] = [0.15, 0, 0.2]
    k = 0
    for w in range(O.N_WAYPOINTS):
        for _ in range(steps_per_wp):
            d.ctrl[:] = sp[w]; mujoco.mj_step(m, d)
            if k % every == 0:
                r.update_scene(d, cam); frames.append(r.render())
            k += 1
    imageio.mimsave(path, frames, fps=fps, codec="libx264", quality=8)
    print("wrote", path, len(frames), "frames")

if __name__ == "__main__":
    print("optimizing naive (hackable) reward for the clip...")
    params, _ = O.evolution_strategy(O.reward_naive, generations=50, pop=32,
                                     elite=8, sigma=0.6, seed=1)
    render(params, "/home/claude/reward_hack_naive.mp4")
