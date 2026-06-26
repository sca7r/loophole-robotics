"""Shared pytest fixtures."""
from __future__ import annotations

import pytest

from loophole_arm.env import CupLiftEnv


@pytest.fixture(scope="session")
def env() -> CupLiftEnv:
    """Small env reused across tests for speed."""
    return CupLiftEnv(n_waypoints=3, sim_seconds=1.0)
