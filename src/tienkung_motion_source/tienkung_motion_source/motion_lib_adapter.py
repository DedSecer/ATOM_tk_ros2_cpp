from __future__ import annotations

import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml


MIMIC_PREFIX = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)


def load_default_mimic_observation(robot_config_path: str | Path) -> np.ndarray:
    path = Path(robot_config_path)
    if not path.is_file():
        raise ValueError(f"robot_config_path does not exist: {path}")
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    motor_num = int(config["motor_num"])
    default_dof_pos = np.asarray(config["default_dof_pos"], dtype=np.float32)
    if default_dof_pos.shape != (motor_num,):
        raise ValueError(
            f"default_dof_pos shape {default_dof_pos.shape} does not match motor_num={motor_num}"
        )
    mimic = np.concatenate([MIMIC_PREFIX, default_dof_pos]).astype(np.float32)
    expected = int(config["observation"]["n_mimic_obs"])
    if mimic.shape != (expected,):
        raise ValueError(f"Default mimic shape {mimic.shape} does not match n_mimic_obs={expected}")
    return mimic


def quat_to_euler_xyzw(quat: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = quat
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.sign(sinp) * (np.pi / 2.0) if abs(sinp) >= 1.0 else np.arcsin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float32)


def quat_conjugate_xyzw(quat: np.ndarray) -> np.ndarray:
    result = np.array(quat, dtype=np.float32, copy=True)
    result[..., :3] *= -1.0
    return result


def quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = np.moveaxis(q1, -1, 0)
    x2, y2, z2, w2 = np.moveaxis(q2, -1, 0)
    return np.stack([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], axis=-1).astype(np.float32)


def quat_rotate_inverse_xyzw(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    q = quat / np.linalg.norm(quat)
    vq = np.array([vec[0], vec[1], vec[2], 0.0], dtype=np.float32)
    return quat_multiply_xyzw(quat_multiply_xyzw(quat_conjugate_xyzw(q), vq), q)[:3]


def quat_to_exp_map_xyzw(quat: np.ndarray) -> np.ndarray:
    q = quat / np.linalg.norm(quat, axis=-1, keepdims=True)
    q = np.where(q[..., 3:4] < 0.0, -q, q)
    xyz = q[..., :3]
    xyz_norm = np.linalg.norm(xyz, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(xyz_norm, np.clip(q[..., 3:4], -1.0, 1.0))
    axis = np.divide(xyz, xyz_norm, out=np.zeros_like(xyz), where=xyz_norm > 1e-8)
    return (angle * axis).astype(np.float32)


@dataclass
class MotionFrame:
    root_pos: np.ndarray
    root_rot: np.ndarray
    root_vel: np.ndarray
    root_ang_vel: np.ndarray
    dof_pos: np.ndarray


class MotionLibAdapter:
    def __init__(self, motion_file: str | Path, expected_dof: int | None = None) -> None:
        self.path = Path(motion_file)
        with self.path.open("rb") as file:
            motion_data = pickle.load(file)
        self.fps = float(motion_data["fps"])
        self.root_pos = np.asarray(motion_data["root_pos"], dtype=np.float32)
        self.root_rot = np.asarray(motion_data["root_rot"], dtype=np.float32)
        self.dof_pos = np.asarray(motion_data["dof_pos"], dtype=np.float32)
        if self.root_pos.ndim != 2 or self.root_pos.shape[1] != 3:
            raise ValueError(f"root_pos must have shape [frames, 3], got {self.root_pos.shape}")
        if self.root_rot.ndim != 2 or self.root_rot.shape[1] != 4:
            raise ValueError(f"root_rot must have shape [frames, 4], got {self.root_rot.shape}")
        if self.dof_pos.ndim != 2:
            raise ValueError(f"dof_pos must have shape [frames, dofs], got {self.dof_pos.shape}")
        if not (self.root_pos.shape[0] == self.root_rot.shape[0] == self.dof_pos.shape[0]):
            raise ValueError("Motion arrays must have the same frame count")
        if not np.isfinite(self.root_pos).all() or not np.isfinite(self.root_rot).all() or not np.isfinite(self.dof_pos).all():
            raise ValueError("Motion arrays must contain only finite values")
        if expected_dof is not None and self.dof_pos.shape[1] != expected_dof:
            raise ValueError(
                f"dof_pos has {self.dof_pos.shape[1]} joints, expected {expected_dof}"
            )
        self.num_frames = self.root_pos.shape[0]
        self.dt = 1.0 / self.fps
        self.length_sec = self.dt * max(0, self.num_frames - 1)
        self.root_vel = np.gradient(self.root_pos, self.dt, axis=0).astype(np.float32)
        self.root_ang_vel = self._compute_so3_angular_velocity()

    def _compute_so3_angular_velocity(self) -> np.ndarray:
        if self.num_frames < 2:
            return np.zeros((self.num_frames, 3), dtype=np.float32)
        if self.num_frames < 3:
            q_rel = quat_multiply_xyzw(self.root_rot[1:], quat_conjugate_xyzw(self.root_rot[:-1]))
            omega = quat_to_exp_map_xyzw(q_rel) / self.dt
            return np.concatenate([omega, omega[-1:]], axis=0).astype(np.float32)

        def quat_diff_xyzw(q_prev: np.ndarray, q_next: np.ndarray) -> np.ndarray:
            return quat_multiply_xyzw(q_next, quat_conjugate_xyzw(q_prev))

        q_rel_interior = quat_diff_xyzw(self.root_rot[:-2], self.root_rot[2:])
        q_rel_start = quat_diff_xyzw(self.root_rot[:1], self.root_rot[1:2])
        q_rel_end = quat_diff_xyzw(self.root_rot[-2:-1], self.root_rot[-1:])
        omega_interior = quat_to_exp_map_xyzw(q_rel_interior) / (2.0 * self.dt)
        omega_start = quat_to_exp_map_xyzw(q_rel_start) / self.dt
        omega_end = quat_to_exp_map_xyzw(q_rel_end) / self.dt
        return np.concatenate([omega_start, omega_interior, omega_end], axis=0).astype(np.float32)

    def get_motion_length(self) -> float:
        return self.length_sec

    def _blend_indices(self, t_sec: float) -> tuple[int, int, float]:
        if self.length_sec <= 0.0:
            return 0, 0, 0.0
        t = t_sec % self.length_sec
        phase = np.clip(t / self.length_sec, 0.0, 1.0)
        idx0 = int(phase * (self.num_frames - 1))
        idx1 = min(idx0 + 1, self.num_frames - 1)
        blend = phase * (self.num_frames - 1) - idx0
        return idx0, idx1, float(blend)

    def sample_frame(self, t_sec: float) -> MotionFrame:
        idx0, idx1, blend = self._blend_indices(t_sec)
        root_pos = (1.0 - blend) * self.root_pos[idx0] + blend * self.root_pos[idx1]
        root_rot = (1.0 - blend) * self.root_rot[idx0] + blend * self.root_rot[idx1]
        root_rot = root_rot / np.linalg.norm(root_rot)
        dof_pos = (1.0 - blend) * self.dof_pos[idx0] + blend * self.dof_pos[idx1]
        root_vel = self.root_vel[idx0]
        root_ang_vel = self.root_ang_vel[idx0]
        return MotionFrame(root_pos=root_pos, root_rot=root_rot, root_vel=root_vel, root_ang_vel=root_ang_vel, dof_pos=dof_pos)


def build_mimic_obs(adapter: MotionLibAdapter, t_step: int, control_dt: float, future_steps: Sequence[int]) -> np.ndarray:
    future_steps = list(future_steps)
    rows = []
    for step in future_steps:
        t_sec = (t_step + int(step)) * control_dt
        frame = adapter.sample_frame(t_sec)
        euler = quat_to_euler_xyzw(frame.root_rot)
        root_vel_local = quat_rotate_inverse_xyzw(frame.root_rot, frame.root_vel)
        root_ang_vel_local = quat_rotate_inverse_xyzw(frame.root_rot, frame.root_ang_vel)
        row = np.concatenate([
            root_vel_local[:2],
            frame.root_pos[2:3],
            euler[:2],
            root_ang_vel_local[2:3],
            frame.dof_pos,
        ]).astype(np.float32)
        rows.append(row)
    return np.concatenate(rows, axis=0).astype(np.float32)