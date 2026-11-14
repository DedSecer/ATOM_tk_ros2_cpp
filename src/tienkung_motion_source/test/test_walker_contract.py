"""rm_future contract checks without ROS or hardware."""

import importlib.util
import pickle
from pathlib import Path

import numpy as np
import pytest
import yaml

from tienkung_motion_source.motion_lib_adapter import (
    MotionLibAdapter,
    build_mimic_obs,
    load_default_mimic_observation,
    load_motion_reference_steps,
)


BRINGUP = Path(__file__).resolve().parents[2] / "tienkung_bringup"


def test_walker_config_matches_manifest_and_current_reference():
    config_path = BRINGUP / "config/walker_config.yaml"
    config = yaml.safe_load(config_path.read_text())
    manifest = yaml.safe_load((BRINGUP / "config/walker_policy_manifest.example.yaml").read_text())
    obs = config["observation"]
    assert obs["append_mimic"] is False
    assert obs["total_obs_size"] == obs["n_obs_single"] * (1 + obs["history_len"]) == 1441
    assert config["actions_size"] == manifest["num_actions"] == 30
    for key in ("n_mimic_obs", "n_proprio", "n_obs_single", "history_len", "total_obs_size"):
        assert obs[key] == manifest[key]
    for key in ("joint_order_policy", "default_dof_pos", "policy_frequency_hz"):
        assert config[key] == manifest[key]
    assert manifest["action_scale"] == [config["action_scale"]] * 30
    assert config["blocked_action_indices"] == [13, 14, 15]
    assert load_default_mimic_observation(config_path).shape == (36,)
    assert load_motion_reference_steps(config_path) == [0]
    assert load_motion_reference_steps(BRINGUP / "config/tg22_config.yaml") == [1]
    parameters = yaml.safe_load((BRINGUP / "config/motion_source.yaml").read_text())
    assert "future_steps" not in parameters["tienkung_motion_source"]["ros__parameters"]


def test_walker_samples_current_frame_not_next_frame(tmp_path):
    path = tmp_path / "motion.pkl"
    positions = np.arange(120, dtype=np.float32).reshape(4, 30)
    with path.open("wb") as file:
        pickle.dump({
            "fps": 50.0,
            "root_pos": np.array([[0.02 * i, 0.04 * i, 1.0] for i in range(4)]),
            "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (4, 1)),
            "dof_pos": positions,
        }, file)
    adapter = MotionLibAdapter(path, expected_dof=30)
    steps = load_motion_reference_steps(BRINGUP / "config/walker_config.yaml")
    current = build_mimic_obs(adapter, 1, 0.02, steps)
    np.testing.assert_allclose(current[:6], [1.0, 2.0, 1.0, 0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(current[6:], positions[1])
    future = build_mimic_obs(adapter, 1, 0.02, [1])
    np.testing.assert_allclose(future[6:], positions[2])
    assert not np.array_equal(current, future)
    for step in (3, 4, 100):
        final = build_mimic_obs(adapter, step, 0.02, steps, loop=False)
        np.testing.assert_allclose(final[6:], positions[-1])
    # Looping still wraps to the first frame at exactly the clip length.
    np.testing.assert_allclose(build_mimic_obs(adapter, 3, 0.02, steps)[6:], positions[0])


@pytest.mark.parametrize("reference_step", [0, 1])
def test_non_looping_node_reaches_and_holds_last_frame(monkeypatch, reference_step):
    # Exercise the actual callback with only ROS transport stubbed out.
    import sys
    from types import SimpleNamespace
    from unittest.mock import Mock

    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)

    mode = SimpleNamespace(STOP=0, POLICY=2)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=object))
    monkeypatch.setitem(sys.modules, "tienkung_interfaces", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "tienkung_interfaces.msg", SimpleNamespace(
        ControlMode=mode, MotionReference=Message))
    node_path = BRINGUP.parent / "tienkung_motion_source/tienkung_motion_source/motion_source_node.py"
    spec = importlib.util.spec_from_file_location("tienkung_motion_source._node_test", node_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    adapter = MotionLibAdapter.__new__(MotionLibAdapter)
    adapter.length_sec = 0.06
    adapter.num_frames = 4
    adapter.root_pos = np.tile([0.0, 0.0, 1.0], (4, 1))
    adapter.root_rot = np.tile([0.0, 0.0, 0.0, 1.0], (4, 1))
    adapter.dof_pos = np.repeat(np.arange(4)[:, None], 30, axis=1)
    adapter.root_vel = np.zeros((4, 3))
    adapter.root_ang_vel = np.zeros((4, 3))
    publisher = Mock()
    node = SimpleNamespace(
        current_mode=mode.POLICY, motion_lib=adapter, t_step=0, control_dt=0.02,
        future_steps=[reference_step], loop=False, _startup_interpolation_start_sec=None,
        _clip_ended_logged=False, motion_reference_pub=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        get_logger=lambda: Mock(),
    )
    frames = []
    for _ in range(7):
        module.MotionSourceNode._publish_reference(node)
        frames.append(publisher.publish.call_args.args[0].body_mimic[6])
    assert frames == ([0, 1, 2, 3, 3, 3, 3] if reference_step == 0 else [1, 2, 3, 3, 3, 3, 3])


@pytest.mark.parametrize("steps", [[], [0, 1], [0.5], [True], "0"])
def test_rejects_invalid_reference_steps(tmp_path, steps):
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"motion_reference_steps": steps}))
    with pytest.raises(ValueError, match="exactly one integer"):
        load_motion_reference_steps(path)


@pytest.mark.parametrize("input_size,output_size,valid", [
    (1441, 30, True), (1477, 30, False), (1759, 30, False), (1441, 27, False),
])
def test_onnx_preflight_contract(tmp_path, input_size, output_size, valid):
    # Optional dependencies: these tests run with real ONNX Runtime when installed.
    onnx = pytest.importorskip("onnx")
    ort = pytest.importorskip("onnxruntime")
    spec = importlib.util.spec_from_file_location("check_walker_policy", BRINGUP / "check_walker_policy.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    tensor = onnx.TensorProto.FLOAT
    graph = onnx.helper.make_graph(
        [onnx.helper.make_node("MatMul", ["input", "weights"], ["output"])],
        "walker_contract_fixture",
        [onnx.helper.make_tensor_value_info("input", tensor, ["batch", input_size])],
        [onnx.helper.make_tensor_value_info("output", tensor, ["batch", output_size])],
        [onnx.numpy_helper.from_array(np.zeros((input_size, output_size), dtype=np.float32), "weights")],
    )
    model = onnx.helper.make_model(graph, opset_imports=[onnx.helper.make_opsetid("", 13)])
    model.ir_version = 8
    path = tmp_path / "fixture.onnx"
    onnx.save(model, path)
    if valid:
        checker.validate_policy(path)
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        result = session.run(None, {"input": np.zeros((1, input_size), dtype=np.float32)})[0]
        assert result.shape == (1, 30)
        assert np.isfinite(result).all()
    else:
        with pytest.raises(ValueError, match="Expected .* shape"):
            checker.validate_policy(path)
