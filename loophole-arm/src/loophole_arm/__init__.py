"""Loophole Arm — reward-hacking demonstration on a UR5e + Robotiq 2F-85."""
from loophole_arm._version import __version__
from loophole_arm.env import CupLiftEnv, RolloutResult
from loophole_arm.optimizer import EvolutionStrategy, OptimizerResult
from loophole_arm.rewards import REGISTRY as REWARDS
from loophole_arm.rewards import RewardFn

__all__ = [
    "__version__",
    "CupLiftEnv",
    "RolloutResult",
    "EvolutionStrategy",
    "OptimizerResult",
    "REWARDS",
    "RewardFn",
]
