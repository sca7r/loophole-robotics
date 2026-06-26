"""
Loophole Robotics
-----------------
Optimize an open-loop arm trajectory to "lift the cup".

The optimizer is a plain (mu, lambda) evolution strategy in NumPy.
It maximizes EXACTLY the reward you write -- including any loopholes.
Try the two reward functions below and watch the behavior change.
"""
import numpy as np
import mujoco

MODEL = "cup_lift.xml"
N_WAYPOINTS = 4          # trajectory = N_WAYPOINTS setpoints, interpolated over time
SIM_SECONDS = 2.5
HOLD_PER_WP = SIM_SECONDS / N_WAYPOINTS

model = mujoco.MjModel.from_xml_path(MODEL)
data = mujoco.MjData(model)

NU = model.nu                     # number of actuators (5)
CTRL_LO = model.actuator_ctrlrange[:, 0]
CTRL_HI = model.actuator_ctrlrange[:, 1]
cup_qpos_adr = int(model.joint("cupfree").qposadr[0])  # cup free-joint x address

PARAM_DIM = N_WAYPOINTS * NU      # genome length


def unpack(params):
    """Map a flat genome to per-waypoint actuator setpoints, clipped to limits."""
    wp = params.reshape(N_WAYPOINTS, NU)
    return CTRL_LO + (CTRL_HI - CTRL_LO) * (0.5 * (np.tanh(wp) + 1.0))


def rollout(params, reward_fn):
    mujoco.mj_resetData(model, data)
    setpoints = unpack(params)
    steps_per_wp = int(HOLD_PER_WP / model.opt.timestep)
    max_cup_z = -1.0
    for w in range(N_WAYPOINTS):
        for _ in range(steps_per_wp):
            data.ctrl[:] = setpoints[w]
            mujoco.mj_step(model, data)
            max_cup_z = max(max_cup_z, data.qpos[cup_qpos_adr + 2])
    return reward_fn(data, max_cup_z)


# ---- Reward A: the naive, hackable one -------------------------------------
# "Just make the cup end up high."  Nothing charges for wild motion, throwing,
# or for the gripper actually holding the cup. The optimizer will often learn
# to whack/fling the cup upward instead of grasping it.
def reward_naive(data, max_cup_z):
    return max_cup_z


# ---- Reward B: a saner version ---------------------------------------------
# Reward final (settled) height, require the cup to be near the gripper, and
# lightly penalize how far the arm joints travel. Harder to game.
def reward_shaped(data, max_cup_z):
    cup = data.qpos[cup_qpos_adr: cup_qpos_adr + 3]
    tip = 0.5 * (data.body("lfing").xpos + data.body("rfing").xpos)
    final_z = cup[2]
    near = np.exp(-10.0 * np.linalg.norm(cup - tip))   # 1 if cup at gripper, ->0 far
    return final_z * near


def evolution_strategy(reward_fn, generations=50, pop=32, elite=8, sigma=0.6, seed=0):
    rng = np.random.default_rng(seed)
    mean = rng.normal(0, 0.3, size=PARAM_DIM)
    best, best_r = mean.copy(), -1e9
    for g in range(generations):
        pop_params = mean + sigma * rng.normal(size=(pop, PARAM_DIM))
        rewards = np.array([rollout(p, reward_fn) for p in pop_params])
        idx = np.argsort(rewards)[::-1][:elite]
        mean = pop_params[idx].mean(axis=0)
        sigma *= 0.97
        if rewards[idx[0]] > best_r:
            best_r, best = rewards[idx[0]], pop_params[idx[0]].copy()
        if g % 10 == 0 or g == generations - 1:
            print(f"  gen {g:3d}  best={best_r:+.4f}  sigma={sigma:.3f}")
    return best, best_r


if __name__ == "__main__":
    for name, fn in [("NAIVE (hackable)", reward_naive),
                     ("SHAPED", reward_shaped)]:
        print(f"\n=== optimizing reward: {name} ===")
        params, r = evolution_strategy(fn, seed=1)
        # report what actually happened to the cup
        final = rollout(params, lambda d, z: (d.qpos[cup_qpos_adr + 2], z))
        print(f"  result: final_cup_z={final[0]:.3f}  peak_cup_z={final[1]:.3f}")
