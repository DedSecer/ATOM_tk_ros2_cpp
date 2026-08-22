from pathlib import Path

import numpy as np
import pytest
import yaml

from tienkung_motion_source.motion_lib_adapter import MotionLibAdapter, load_default_mimic_observation


def test_default_mimic_comes_from_robot_config(tmp_path: Path) -> None:
    path = tmp_path / "robot.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "motor_num": 2,
                "default_dof_pos": [0.25, -0.5],
                "observation": {"n_mimic_obs": 8},
            }
        ),
        encoding="utf-8",
    )
    mimic = load_default_mimic_observation(path)
    assert mimic.tolist() == [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25, -0.5]


def test_motion_adapter_rejects_wrong_dof_count(tmp_path: Path) -> None:
    path = tmp_path / "motion.pkl"
    import pickle

    with path.open("wb") as file:
        pickle.dump(
            {
                "fps": 50.0,
                "root_pos": np.zeros((2, 3), dtype=np.float32),
                "root_rot": np.tile(np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float32), (2, 1)),
                "dof_pos": np.zeros((2, 29), dtype=np.float32),
            },
            file,
        )

    with pytest.raises(ValueError, match="expected 30"):
        MotionLibAdapter(path, expected_dof=30)