# Hardware costs — honest numbers

> *"i want to implement in industry and not for educational/research so if
> the tech need expensive hardware then let me know"*

This document tells you exactly what each capability tier costs. Prices are
USD, sampled mid-2026. They will drift; treat ranges, not exact figures.

---

## TL;DR

| Tier | Capability | Total cost per cell | Verdict for your startup |
|---|---|---|---|
| **1** | **Deploy a pre-trained policy** on the Feetech arm | **$1.5 k – $3 k** | ✅ Right starting point |
| **2** | **Train policies locally** (ACT, Diffusion) from demos | $4 k – $7 k | ✅ Right second step |
| **3** | **Fine-tune small VLAs** (SmolVLA 450M full, 3B-class with LoRA+4bit) | $5 k – $10 k | ⚠ Only if Tier 2 isn't enough |
| **4** | **Fine-tune GR00T-class foundation models** (3B+ full fine-tune) | $30 k owned · ~$50–500/run cloud | ❌ Cloud rent only |
| **5** | **NVIDIA Cosmos / large-scale synthetic data** | $100k+ infra | ❌ Not needed at this stage |

The honest bottom line: **Tier 1 + 2 covers 95 % of industrial use cases for
a 6-DOF Feetech arm.** Tier 3 and above are research/SOTA chasing — fine if
you can rent compute, but not required for shipping.

---

## Tier 1 — Pure deployment ($1.5 k–$3 k)

**What you can do:** Run inference on policies someone else trained (you, a
contractor, or a pre-trained checkpoint from HuggingFace). Teleoperate the
arm. Record demonstrations for later training. Run the MuJoCo sim.

**You DON'T need a GPU.** All of the above runs on CPU.

### Bill of materials

| Item | Spec | Cost |
|---|---|---|
| **Feetech 6-DOF arm + 1-DOF gripper** | 7× STS3215 servos, ~3 N·m, ~35 cm reach, <1 kg payload | $500 – $1 000 |
| **Industrial PC** (or NUC) | x86-64, 16 GB RAM, no GPU | $400 – $900 |
| **USB-to-serial adapter** | FTDI or similar, 1 Mbit/s | $20 – $40 |
| **USB camera** *(optional but recommended)* | 1080p, 30 fps | $30 – $200 |
| **Power supply** | 12 V / 5 A regulated | $30 – $50 |
| **Cabling, mounts, e-stop button** | various | $100 – $300 |
| **Software** | All open-source (LeRobot, MuJoCo, Loophole Arm) | $0 |
| | | **$1.5 k – $3 k total** |

### What policies can run on CPU at Tier 1

| Policy | Inference latency on CPU | Training? |
|---|---|---|
| Hand-coded scripts / classic motion planning | <1 ms | n/a |
| **ACT** (~80 M params, the LeRobot default) | ~10–30 ms on i7 | ❌ needs GPU |
| **Diffusion Policy** (small, ~10 M params) | ~50–100 ms | ❌ needs GPU |
| **SmolVLA** (450 M params, int8 quantized) | 200–400 ms | ❌ needs GPU |

ACT inference at 30 ms on CPU is fast enough for a 10 Hz control loop with
trajectory-chunk smoothing — production-viable for non-time-critical tasks.

---

## Tier 2 — Local policy training ($4 k–$7 k)

**What you add:** A workstation with a mid-range consumer GPU. You can now
train ACT and Diffusion Policy on demonstrations you collect yourself.

### Add to Tier 1

| Item | Spec | Cost |
|---|---|---|
| **GPU workstation** | RTX 4070 Ti SUPER (16 GB VRAM) or RTX 5070 Ti | $2 500 – $3 500 |
| | *Alternative: used RTX 3090 (24 GB)* | $700 – $900 |

### What you can train at Tier 2

| Task | Time on RTX 4070 Ti |
|---|---|
| Train ACT from 50–200 demos | 1–4 hours |
| Train Diffusion Policy from 100–500 demos | 4–12 hours |
| Train SmolVLA from scratch | ❌ too slow; LoRA fine-tune possible |

This is the sweet spot. **Most industrial pilots ship at this tier.**

---

## Tier 3 — Small VLA fine-tuning ($5 k–$10 k)

**What you add:** A higher-end consumer GPU. With recent advances in LoRA +
quantisation, the VRAM bar is lower than it used to be.

### Add to Tier 2

| Item | Spec | Cost |
|---|---|---|
| **Consumer GPU with ≥12 GB VRAM** | RTX 3080 Ti (12 GB used, ~$500) | $500 – $900 |
| **OR higher-VRAM card** for full fine-tunes | RTX 4090 (24 GB) | $1 800 – $2 200 |

### What you can do — verified

- **Full fine-tune SmolVLA-450M:** ~11.5 GB VRAM at batch size 44 on a
  3080 Ti. ([HF blog](https://huggingface.co/blog/smolvla))
- **LoRA fine-tune larger VLAs (3 B params):** ~8 GB VRAM with LoRA +
  4-bit quantisation ([arXiv 2512.11921](https://arxiv.org/html/2512.11921v1)).
- **Inference latency:** ~100–300 ms per action chunk on consumer GPUs.
- **Time to fine-tune:** ~4 hours on a single A100 for 20 k steps
  ([phospho docs](https://docs.phospho.ai/learn/train-smolvla)).

**Diminishing returns warning:** Going from Tier 2 to Tier 3 buys you
generalisation (VLAs handle novel objects/instructions), but for repetitive
industrial tasks the Tier 2 ACT/Diffusion policy is usually more reliable
and easier to debug.

---

## Tier 4 — Foundation model fine-tuning

This is where costs get serious. **Do not buy this hardware for a single
robot deployment** — rent it instead.

### Owned hardware

| Item | Spec | Cost |
|---|---|---|
| NVIDIA A6000 Ada (48 GB) | Workstation card, fits GR00T fine-tune | ~$5 000 |
| NVIDIA L40 (48 GB) | Datacentre card, faster than A6000 | ~$7 500 |
| NVIDIA H100 80 GB SXM | Reference fine-tune target | $25 000 – $35 000 |
| 8× H100 server (DGX or similar) | Full pretraining capability | $250 000+ |

### Rent instead

| Provider | Card | Cost/hour | A typical fine-tune run |
|---|---|---|---|
| RunPod / Lambda / Vast.ai | RTX 4090 | $0.30 – $0.60 | $5–30 |
| Same | A100 40 GB | $1 – $2 | $30–200 |
| Same | H100 80 GB | $2 – $4 | $50–500 |
| AWS / GCP / Azure | H100 | $4 – $10 | $200–2 000 |

**Practical rule:** rent until you're doing >50 fine-tune runs/month — that's
when an owned A6000 starts to pay for itself.

---

## What we deliberately don't use

| Tech | Why we skip it |
|---|---|
| **NVIDIA Isaac Lab** | Needs CUDA + RTX 3090 minimum. Powerful but overkill for a 6-DOF arm at our scale. MuJoCo + MJX gives ~80 % of the value on CPU. Revisit if/when you need GPU-parallel RL. |
| **NVIDIA Isaac Sim** | High-fidelity rendering, but ~30 GB install and an RTX card to run. We don't need photorealism for joint-space training. |
| **NVIDIA Cosmos world models** | Datacentre-scale synthetic data generation. Genuinely incredible, completely unnecessary at our scale. |
| **GR00T full fine-tuning** | 3 B-parameter model; needs H100 cluster for serious work. Useful as a future endpoint, not as a starting point. |

---

## Recommended startup deployment

For a real industrial customer pilot:

1. **One Tier 1 cell per deployment site.** This is what you sell/lease to
   the customer. Sub-$3 k BOM. Runs on the factory floor.
2. **One Tier 2 workstation centrally.** This is where you collect demos
   from all customer sites (via the LeRobotDataset format on Hugging Face
   Hub) and re-train policies as data grows.
3. **No Tier 3/4 spend until you have a clear training-cost ceiling.** Once
   your monthly fine-tune compute exceeds ~$300 of rented cloud, evaluate
   buying a Tier 3 GPU.

That structure scales linearly with customers without burning capital on
research-grade hardware.

---

## A note on the Feetech arm itself

Be realistic about what a sub-$1 k arm can do:

| ✅ Suitable for | ❌ Not suitable for |
|---|---|
| Light pick-and-place (<500 g objects) | Heavy lifting (>1 kg) |
| Inspection with a camera | Welding, machining, fast assembly |
| Educational / cobotic demos | Anything safety-critical with humans nearby |
| Indoor, controlled lighting | Outdoor, wet, or high-vibration |
| Prototype/pilot deployments | High-throughput production lines |

For production lines that need >1 kg payload or >0.5 mm repeatability,
you'd graduate to a UR5e ($30 k), a Franka FR3 ($25 k), or an xArm 6
($10 k–$15 k). Loophole Arm's architecture supports any of these — drop in
a different URDF in `assets/`, change the `LoopholeArmConfig` motor layout,
done.
