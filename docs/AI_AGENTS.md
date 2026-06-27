# AI agents — the "teach and learn" roadmap

> *"Also see if we can use AI agents where we teach them and then they do it
> or something like that."*

This is **imitation learning** territory. The current Loophole Arm uses
*evolution strategies* — random trial-and-error against a reward. Imitation
learning is fundamentally different: you show the robot what to do, and it
learns to copy you.

Both have a place in a startup. ES is great for "you wrote the reward, the
optimizer finds the loophole." Imitation learning is great for "the task is
hard to score but easy to demonstrate" — which is most real-world tasks.

This doc lays out the path from where we are today to a learned policy.

---

## Where the current architecture helps

We already have:

- ✅ A working sim (MuJoCo) that runs at >100× real-time
- ✅ A unified env API (`CupLiftEnv`) that doesn't care if the controller is
  a trained policy or a hand-written one
- ✅ A ROS 2 bridge — so demos can be collected from either teleop or scripted motion
- ✅ Structured logging of joint states and trajectories

That's the foundation. The four stages below build on it.

---

## Stage 1 — Behavior cloning (BC)

**Idea:** Collect demonstrations. Train a neural network to map *(joint
states, cup position) → next action*. At deploy time, run the network.

**What you need to add:**

1. A demonstration recorder (ROS 2 node subscribing to `/joint_states` and
   `/joint_trajectory_controller/joint_trajectory`, writing to disk).
2. A teleop input — for sim this is keyboard or 3D mouse; for real this is a
   leader arm or VR controller.
3. A small policy network (start with a 3-layer MLP, ~100k params).
4. A training script using PyTorch (~50 lines).

**Pros:** Simplest deep-learning approach. Trains in minutes. Works
surprisingly well when demos are clean.

**Cons:** Compounding errors — the policy hits states it never saw in
training and gets confused. Falls over on long-horizon tasks.

**When to use it:** Single-shot tasks like cup grasping with a fixed cup
position. Good first deep-learning win.

```
loophole-arm-bc/
├── record_demos.py        # records (obs, action) trajectories
├── train_bc.py            # MLP, ~50 lines PyTorch
├── policy.py              # wraps the trained net for the ROS 2 bridge
└── demos/                 # ~50-200 recorded trajectories
```

---

## Stage 2 — DAgger (Dataset Aggregation)

**Idea:** Fix BC's compounding-error problem. Let the policy run, but when it
makes a mistake, ask the expert (you) what they would have done. Add those
corrections to the dataset. Retrain. Repeat.

**What you add on top of BC:**

1. An *intervention* mode in the recorder — operator can take over mid-trajectory
2. A loop: train → run → record interventions → retrain
3. A merge step in the dataset

**Pros:** Much more robust than BC. Recovers from mistakes.

**Cons:** Requires the human to be available during training. Not zero-shot.

---

## Stage 3 — Diffusion policies (current SOTA for manipulation)

**Idea:** Instead of a feedforward network outputting one action, train a
*diffusion model* that samples whole trajectory chunks. Empirically the best
single-task manipulation approach as of 2024-2025.

**What you add:**

1. A larger model — typically a 1D U-Net or transformer (~10-50M params)
2. A small amount of diffusion training infra
3. More demos — typically 100-500 episodes

**Pros:** Robust to multimodal demonstrations ("there are several good ways
to pick up the cup"). Smooth trajectories. State-of-the-art results.

**Cons:** Inference is slower (denoising loop). Needs a GPU.

**References:**
- Original paper: Chi et al., "Diffusion Policy" (2023)
- Code: github.com/real-stanford/diffusion_policy

---

## Stage 4 — Vision-Language-Action (VLA) foundation models

**Idea:** Don't train from scratch. Start with a pretrained model that's been
trained on millions of robot demos (OpenVLA, π0, RT-2). Fine-tune on your
arm with as few as 20-100 demos.

**What you add:**

1. A camera in the sim and (eventually) on the real arm
2. Language annotations on demos ("pick up the red cup", "place on the shelf")
3. A fine-tuning pipeline (often LoRA — much cheaper than full fine-tuning)
4. A GPU. A real one.

**Pros:** Generalizes across tasks. Language-conditioned ("pick up the *blue*
one"). Best chance of a single policy doing many things.

**Cons:** Big model, big compute. Most teams need an H100 or rented A100s.

**Open-source options to start from:**
- **OpenVLA** (7B params, Apache 2.0) — github.com/openvla/openvla
- **π0** (just open-sourced by Physical Intelligence) — github.com/Physical-Intelligence/openpi
- **LeRobot** (Hugging Face, has BC + diffusion + VLA in one repo) — github.com/huggingface/lerobot

---

## My recommendation — order of operations

1. **Now:** Keep using the evolution strategy. It's working. Use it to
   generate "expert" demonstrations programmatically (run the optimizer with
   `shaped_lift`, save 100 trajectories with slight variations).

2. **Next month:** Add the demo recorder (Stage 1 infrastructure). Then
   either:
   - **Path A:** Train a BC policy on optimizer-generated demos. Verify it
     replays correctly.
   - **Path B:** Record teleop demos via a leader arm. More work, but
     produces a real "teach and learn" loop.

3. **Quarter from now:** Move to a diffusion policy on whichever data source
   worked. This is roughly when you'd publish a first demo video.

4. **Later:** Fine-tune a VLA. By this point you've got real hardware data,
   so this is where it pays off.

**Don't skip stages.** Every team that jumps to VLAs without first having a
working BC pipeline regrets it — the data collection infrastructure is the
hard part, not the model.

---

## A note on the Loophole brand

The "reward hacking" concept fits really well alongside imitation learning,
because **imitation learning has its own loopholes**:

- A BC policy will happily reproduce a demonstrator's mistakes.
- A diffusion policy averages multiple demos — sometimes badly.
- A VLA finetuned on biased data inherits the bias.

You could publish a series called "Loophole Lessons" where each video shows
a new failure mode in a different learning approach. That's a real point of
view that nobody is making concretely on the open-source side.
