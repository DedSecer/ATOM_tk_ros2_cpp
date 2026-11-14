"""Strict JSON/schema verification for the standalone C++ telemetry fixture."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def reject_constant(value):
    raise AssertionError(f"Non-JSON numeric token: {value}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        assert key not in result, f"Duplicate JSON key: {key}"
        result[key] = value
    return result


def main():
    executable = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="telemetry_json_") as directory:
        subprocess.run([str(executable), directory], check=True)
        records = {}
        for path in Path(directory).glob("*.jsonl"):
            records[path.name] = [
                json.loads(line, parse_constant=reject_constant, object_pairs_hook=unique_object)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
        metadata, sample, unpublished = records["telemetry_schema_fixture.jsonl"]
        assert metadata["type"] == "metadata" and metadata["schema_version"] == 2
        assert metadata["joint_order_policy"] == ["leg", "arm", "unseen_head"]
        assert metadata["can_id_by_index"] == [51, 11, 1]
        assert metadata["policy_path"] == '/tmp/a"b\\c\n\t.onnx'
        assert metadata["manifest_path"] == ""
        assert metadata["robot_config_path"] == "/tmp/机器人.yaml"
        assert metadata["config"]["ankle_transmission"]["kind"] == "identity"
        assert metadata["config"]["motor_dir"] == [1, -1, 1]
        assert sample["type"] == "sample" and sample["schema_version"] == 2
        assert {
            "timestamp", "mode", "obs", "target", "dof_pos", "dof_vel", "motion_ref",
            "raw_action", "effective_motion_ref", "rpy", "angular_velocity",
            "dof_torque_estimated", "inference_executed", "inference_latency_ms",
            "command_published", "state_ready", "motion_ready", "motion_reference_valid",
            "input_age_sec", "timing", "motor_feedback", "command", "dropped_records",
        } <= sample.keys()
        assert sample["obs"] == [1, None, None]
        assert sample["motion_ref"] == [4, 5, 6]
        assert sample["effective_motion_ref"] == [1, 2, 3]
        assert sample["command_published_mask"] == [True, True, False]
        assert sample["input_age_sec"]["head"] is None
        assert sample["timing"]["tick_dt_sec"] is None
        assert sample["timing"]["processing_ms"] is None
        feedback = sample["motor_feedback"]
        assert feedback["filtered"]["valid"] == [True, True, False]
        assert feedback["filtered"]["current"] == [1.5, 2, 0]
        assert feedback["raw_current"] == [10, 2, 0]
        assert feedback["raw_valid"] == [True, True, False]
        assert feedback["head"]["samples"][0]["position"] is None
        assert feedback["head"]["samples"][0]["finite"] is False
        assert sample["command"]["leg"] == [
            {"name": 51, "kp": 20, "kd": 2, "pos": 0.25, "spd": 0.5, "tor": 0}
        ]
        assert sample["command"]["waist"] == [
            {"name": 31, "pos": 0.75, "spd": 0.2, "cur": 0.3}
        ]
        assert unpublished["command"] is None and not unpublished["command_published"]
        assert unpublished["command_published_mask"] == [False, False, False]
        for name, log in records.items():
            if name.startswith("telemetry_test_"):
                assert log[0]["type"] == "metadata"
                assert log[-1]["type"] == "logger_stats"
                stats = log[-1]
                assert stats["written_records"] == len(log) - 1
                assert stats["dropped_records"] + len(log) - 2 == 3000
                assert stats["write_errors"] == 0
                stamps = [row["timestamp"] for row in log[1:-1]]
                assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)
        print("Strict JSON parsing, schema, masks, command fields, and writer statistics passed.")


if __name__ == "__main__":
    main()
