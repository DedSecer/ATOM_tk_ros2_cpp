# ATOM Tienkung ROS 2 Deployment

Tienkung 强化学习策略的 ROS 2 Humble 部署工作区。策略运行、dry-run 和输入监控使用 C++17；动作片段读取保留 Python，以兼容现有 `.pkl` 文件。

## 包

- `tienkung_interfaces`：`ControlMode`、`MotionReference` 消息。
- `tienkung_policy_runner`：C++ 策略核心、正式节点、dry-run 节点和输入监控节点。
- `tienkung_motion_source`：Python 动作片段采样节点。
- `tienkung_bringup`：统一配置和 launch 文件。

## 统一机器人配置

机器人契约、关节映射、PD、标定和动作缩放集中在：

```text
src/tienkung_bringup/config/tg22_config.yaml
```

配置形式参考 `Deploy_Tienkung/rl_control_new/config/tg22_config.yaml`。关键字段：

```yaml
default_dof_pos: [...20 values...]
action_scale: 0.5
clip_actions: 5.0
joint_kp_p: [...20 values...]
joint_kd_p: [...20 values...]
```

`action_scale` 和 `clip_actions` 与训练配置语义一致，均为全部 20 个关节共用的标量。训练先将目标空间裁剪值换算到原始策略输出空间：

```text
raw_action_limit = clip_actions / action_scale
target = default_dof_pos
       + clip(raw_action, -raw_action_limit, raw_action_limit) * action_scale
```

当前配置得到 `raw_action_limit = 5.0 / 0.5 = 10.0`，因此最终目标相对默认位的最大偏移仍为 `±5.0`。

节点启动时会校验关节向量长度、CAN ID、leg/arm 分组、PD、标定、observation 维度、踝关节索引及 manifest 契约。配置不一致时直接拒绝启动。

## 目标环境

- Ubuntu 22.04 x86-64
- ROS 2 Humble
- 已 source 的 `bodyctrl_msgs`
- Eigen3、yaml-cpp、PyYAML、NumPy
- `joy`（使用带遥控器的 launch 时）
- ONNX Runtime C++ CPU 版本

包内的 `libfuncSPTrans.so` 是 Linux x86-64 二进制，因此 `tienkung_policy_runner` 会拒绝在其他平台构建。

## 构建

先设置 ONNX Runtime 安装前缀；该目录下应包含 `include/onnxruntime_cxx_api.h` 和 `lib/libonnxruntime.so`：

```bash
export ONNXRUNTIME_ROOT=/opt/onnxruntime
source /opt/ros/humble/setup.bash
source /path/to/bodyctrl_msgs/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

也可以通过 CMake 参数传入：

```bash
colcon build --symlink-install \
  --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime
```

## 安全验证

优先使用 dry-run：

```bash
ros2 launch tienkung_bringup dry_run.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml
```

安全语义：

- `STOP`：不发布电机命令。
- `ZERO`：发布真实 PD 归零命令，会驱动机器人。
- `POLICY`：执行 ONNX 推理，但不发布策略电机命令。

带动作源的 dry-run：

```bash
ros2 launch tienkung_bringup dry_run.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml \
  use_motion_source:=true \
  motion_file:=/abs/path/to/motion.pkl
```

## 正式运行

仅启动策略节点：

```bash
ros2 launch tienkung_bringup policy_runner.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml
```

启动遥控器、可选 motion source 和策略节点：

```bash
ros2 launch tienkung_bringup sim2real_minimal.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml
```

启用 motion source：

```bash
ros2 launch tienkung_bringup sim2real_minimal.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml \
  use_motion_source:=true \
  motion_file:=/abs/path/to/motion.pkl
```

启用后，motion reference 超时会阻止进入或保持 `POLICY`。

启用推理运行数据日志（默认关闭，文件写入 `log_dir`，默认为 `/tmp`）：

```bash
ros2 launch tienkung_bringup policy_runner.launch.py \
  policy_path:=/abs/path/to/policy.onnx \
  manifest_path:=/abs/path/to/policy_manifest.yaml \
  enable_runtime_log:=true
```

## 输入监控

```bash
ros2 launch tienkung_bringup input_monitor.launch.py
```

详细显示遥控输入尺寸：

```bash
ros2 launch tienkung_bringup input_monitor.launch.py \
  print_hz:=2.0 verbose:=true
```

监控节点复用正式 RobotIO 的电机标定和踝关节传动，显示的是策略关节空间状态。

## 测试

```bash
colcon test --packages-select tienkung_policy_runner tienkung_motion_source
colcon test-result --verbose
```

C++ 测试覆盖配置、统一动作缩放、observation/history、FSM、标定、线性 transmission 和命令分组。Python 测试覆盖 motion source 从统一配置构造默认 mimic observation。