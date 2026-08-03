from pathlib import Path

import yaml

from tienkung_motion_source.motion_lib_adapter import load_default_mimic_observation


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