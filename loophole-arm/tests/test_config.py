"""Tests for the YAML scene loader."""
from __future__ import annotations

import textwrap
from pathlib import Path

import numpy as np
import pytest

from loophole_arm.control.limits import SafetyLimits
from loophole_arm.server.config import SceneConfigError, load_scene_config


def _write(tmp_path: Path, text: str) -> Path:
    p = tmp_path / "scene.yaml"
    p.write_text(textwrap.dedent(text))
    return p


def test_minimal_config(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - name: arm
            mount_pos: [0.0, 0.0, 0.10]
    """)
    scene, arms, limits = load_scene_config(p)
    assert [a.name for a in arms] == ["arm"]
    assert arms[0].kind == "feetech"      # default
    assert arms[0].mount_pos == (0.0, 0.0, 0.10)
    assert scene.tables == [] and scene.objects == []
    # Default limits when no `safety:` block.
    assert np.allclose(limits["arm"].max_joint_step, SafetyLimits.feetech_default().max_joint_step)


def test_dual_arm_with_per_arm_limits(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - name: arm_a
            mount_pos: [0.0, 0.0, 0.10]
          - name: arm_b
            mount_pos: [0.55, 0.0, 0.10]
            safety:
              workspace_min: [0.40, -0.20, 0.10]
              workspace_max: [0.70, 0.20, 0.40]
              max_joint_step: 0.10
    """)
    _, arms, limits = load_scene_config(p)
    assert len(arms) == 2
    # arm_a falls back to default.
    assert limits["arm_a"].max_joint_step[0] == pytest.approx(0.15)
    # arm_b gets its overrides.
    assert limits["arm_b"].max_joint_step[0] == pytest.approx(0.10)
    assert limits["arm_b"].workspace_min.tolist() == [0.40, -0.20, 0.10]
    assert limits["arm_b"].workspace_max.tolist() == [0.70, 0.20, 0.40]
    # Unspecified fields (joint_lower etc.) still come from the default.
    assert np.allclose(limits["arm_b"].joint_lower, SafetyLimits.feetech_default().joint_lower)


def test_scene_with_tables_and_objects(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - name: arm
        scene:
          reference_axes: true
          table_grid: true
          tables:
            - size: [0.35, 0.45]
              height: 0.10
              pos: [0.0, 0.0]
              name: workbench
          objects:
            - kind: cube
              size: 0.03
              pos: [0.2, 0.0, 0.13]
              color: red
            - kind: sphere
              size: 0.02
              pos: [0.25, 0.0, 0.13]
              color: [0.1, 0.5, 0.9]
    """)
    scene, _, _ = load_scene_config(p)
    assert scene.reference_axes is True
    assert scene.table_grid is True
    assert len(scene.tables) == 1 and scene.tables[0].name == "workbench"
    assert len(scene.objects) == 2
    assert scene.objects[0].color == "red"
    assert scene.objects[1].color == [0.1, 0.5, 0.9]


def test_max_joint_step_as_list(tmp_path: Path) -> None:
    """Per-joint velocity caps via a 6-element list, not a scalar."""
    p = _write(tmp_path, """
        arms:
          - name: arm
            safety:
              max_joint_step: [0.10, 0.10, 0.10, 0.05, 0.05, 0.05]
    """)
    _, _, limits = load_scene_config(p)
    assert limits["arm"].max_joint_step.tolist() == [0.10, 0.10, 0.10, 0.05, 0.05, 0.05]


def test_missing_arms_section_rejected(tmp_path: Path) -> None:
    p = _write(tmp_path, "scene: {}")
    with pytest.raises(SceneConfigError, match="at least one entry under 'arms'"):
        load_scene_config(p)


def test_arm_without_name_rejected(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - mount_pos: [0.0, 0.0, 0.10]
    """)
    with pytest.raises(SceneConfigError, match="'name' is required"):
        load_scene_config(p)


def test_bad_workspace_dimension_rejected(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - name: arm
            safety:
              workspace_min: [0.0, 0.0]   # only 2 values; needs 3
    """)
    with pytest.raises(SceneConfigError, match="expected 3 values"):
        load_scene_config(p)


def test_table_without_size_rejected(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        arms:
          - name: arm
        scene:
          tables:
            - height: 0.10
    """)
    with pytest.raises(SceneConfigError, match="'size' and 'height' are required"):
        load_scene_config(p)


def test_nonexistent_file_rejected(tmp_path: Path) -> None:
    with pytest.raises(SceneConfigError, match="not found"):
        load_scene_config(tmp_path / "does_not_exist.yaml")


def test_example_yamls_load() -> None:
    """The shipped example YAMLs all parse + build correctly."""
    from loophole_arm.control.workcell import build_multi_arm_model

    examples = Path(__file__).resolve().parent.parent / "examples" / "scenes"
    yamls = sorted(examples.glob("*.yaml"))
    assert len(yamls) >= 3, f"expected >=3 example scenes, found {len(yamls)}"
    for y in yamls:
        scene, arms, limits = load_scene_config(y)
        # Every arm has a limits entry.
        assert set(limits) == {a.name for a in arms}
        # The model compiles.
        model, _, _ = build_multi_arm_model(scene, arms)
        assert model.nu == 7 * len(arms)


def test_per_arm_limits_reach_safety_backend(tmp_path: Path) -> None:
    """The per-arm limits dict actually wires into SafetyBackend, not just sits in a dict."""
    import mujoco

    from loophole_arm.control.workcell import build_multi_arm_model
    from loophole_arm.server.cli import _build_endpoints

    p = _write(tmp_path, """
        arms:
          - name: arm_tight
            mount_pos: [0.0, 0.0, 0.10]
            safety:
              max_joint_step: 0.05
              workspace_max: [0.20, 0.20, 0.30]
        scene:
          tables:
            - size: [0.35, 0.45]
              height: 0.10
              pos: [0.0, 0.0]
    """)
    scene, arms, per_arm_limits = load_scene_config(p)
    model, _, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    endpoints = _build_endpoints(model, data, handles, per_arm_limits)

    # The SafetyBackend inside the endpoint holds OUR limits, not the default.
    inner_limits = endpoints["arm_tight"].backend._limits
    assert inner_limits.max_joint_step[0] == pytest.approx(0.05)
    assert inner_limits.workspace_max.tolist() == [0.20, 0.20, 0.30]
