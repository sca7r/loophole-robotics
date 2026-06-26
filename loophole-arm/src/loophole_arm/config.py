"""Typed configuration with YAML loading.

Keeps experiment definitions versionable and human-readable. Configs override
defaults additively; missing fields fall back to the dataclass defaults.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml


@dataclass
class EnvConfig:
    n_waypoints: int = 6
    sim_seconds: float = 3.0


@dataclass
class OptimizerConfig:
    population: int = 32
    elite: int = 8
    sigma: float = 0.6
    sigma_decay: float = 0.97
    init_scale: float = 0.3
    generations: int = 50
    seed: int = 0


@dataclass
class RunConfig:
    reward: str = "naive_peak_height"
    env: EnvConfig = field(default_factory=EnvConfig)
    optimizer: OptimizerConfig = field(default_factory=OptimizerConfig)
    output_dir: str = "runs"

    @classmethod
    def from_yaml(cls, path: Path) -> RunConfig:
        """Load a config from YAML, falling back to defaults for missing keys."""
        raw: dict[str, Any] = yaml.safe_load(Path(path).read_text()) or {}
        return cls._from_dict(raw)

    @classmethod
    def _from_dict(cls, raw: dict[str, Any]) -> RunConfig:
        env = EnvConfig(**raw.get("env", {}))
        opt = OptimizerConfig(**raw.get("optimizer", {}))
        kwargs = {
            f.name: raw[f.name]
            for f in fields(cls)
            if f.name not in {"env", "optimizer"} and f.name in raw
        }
        return cls(env=env, optimizer=opt, **kwargs)
