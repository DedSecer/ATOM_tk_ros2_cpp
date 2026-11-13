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

## Walker 适配（当前状态）

Walker 使用独立配置和启动入口，不修改 Tienkung 的默认配置：

```bash
ros2 launch tienkung_bringup walker_dry_run.launch.py \
  policy_path:=/abs/path/to/walker_policy.onnx \
  manifest_path:=/abs/path/to/walker_policy_manifest.yaml
```

启动前检查 ONNX 输入/输出契约：

```bash
python3 src/tienkung_bringup/check_walker_policy.py \
  /abs/path/to/walker_policy.onnx
```

本仓库 `rm_future` 分支对应训练仓库 `rm_future` 的 **Walker student**，契约为输入 `1441`、输出 `30`、50 Hz。不是 `1759` 维的 privileged teacher；也不能直接加载旧 `1477` 维 ONNX，不能通过裁剪旧模型输入替代重新训练/导出。

| 项目 | 旧部署 main | 本分支 rm_future |
| --- | --- | --- |
| 当前帧 | 36 维 mimic + 95 维 proprio = 131 | 不变 |
| 历史帧 | 10 × 131，最旧到最新 | 不变 |
| 尾部 mimic | 36 | 删除 |
| ONNX 输入 | 1477 | 1441 |
| 动作源采样 | `t + 0.02s` | 当前 `t`，`motion_reference_steps: [0]` |
| ONNX 输出 | 30 维 raw action | 不变 |

输入切片（0-based、左闭右开）：`[0:36]` 当前参考 mimic、`[36:131]` proprio、`[131:1441]` 历史。proprio 依次为 `angular_velocity*0.25`、roll/pitch、`q-default`、`dq*0.05`、前次 **raw action**；保留踝和头部维度。首次推理用当前帧填满历史，之后先读取旧历史再追加当前帧，与训练仓库 Walker sim2sim 的初始化方式一致。不要在部署侧重复应用导出模型已包含的统计归一化。

动作后处理仍为 `default + 0.5 * clip(raw_action, -10, 10)`。头部动作索引 `13,14,15` 在 `walker_config.yaml` 中仍被阻断，固定为默认位且不发布头部命令。这是**部署侧限制**，不是训练模型删除了头部输出。头部反馈仍订阅 `/head/status` 并参与观测；当前 `require_head_status: false`，不作为 readiness 必需项。CAN、PD、标定、FSM 和命令发布行为均未因本次适配更改；Tienkung 保持原有 1027 维布局。

### 模型与动作源迁移

1. 从训练仓库 `rm_future` 的 **student checkpoint** 导出新 ONNX 和其配套 manifest。训练分支切换不会转换已有 ONNX；本次核对时训练仓库已有的 8 个 Walker ONNX 均为 1477 维旧模型。
2. 使用上面的 `check_walker_policy.py` 检查 `1441 → 30`，并在 launch 中传入对应 manifest；示例 manifest 不是已验证模型的替代品。
3. 内置动作源读取 `walker_config.yaml` 的 `motion_reference_steps: [0]`；非循环播放会夹紧并保持真实末帧。外部实时动作源也必须提供当前时刻的 36 维 mimic，不追加尾部字段。旧自定义参数若仍设置 `future_steps: [1]`，需移除或改成 `[0]`，不匹配时节点会拒绝启动。**既有启动限制：** `walker_dry_run.launch.py` 的 `use_motion_source:=true` 会同时要求新鲜参考，而内置动作源只在 POLICY 发布，导致默认暂停状态与 FSM 进入 POLICY 的条件互等。该组合不能作为已验证的启动流程；本次未改变 FSM/readiness 或暂停发布语义，不要通过关闭安全条件绕过，需另行处理后在 ROS 环境验证。
4. 正式 `walker_policy_runner.launch.py` 仍只启动 runner；如使用独立动作源，须传相同的 Walker 配置并对齐双方 topic（正式入口未改，默认 `/tienkung/control_mode` 和 `/tienkung/motion_reference`；Walker dry-run 使用 `/walker/...`）。

**安全与验证边界：** dry-run 不是无硬件副作用：ZERO 仍发送归零命令，当前 STOP 实现也会发送零 Kp 的阻尼命令。先在隔离硬件/仿真环境完成输入和时序检查，再考虑实机；本次未运行任何控制节点。开发机为 macOS，未完成 Linux x86-64 ROS2 构建、核心 C++ 测试或真实新模型的 sim2sim/实机验证。

Walker 配置中的零点、方向、电流比例和踝关节传动必须替换为实机标定值。当前文件中的相关值是占位配置，不能直接用于上电控制。


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

C++ 测试覆盖配置、统一动作缩放、observation/history、FSM、标定、线性 transmission 和命令分组；新增 Walker 1441 维布局、历史时序/reset、旧 manifest 拒绝与 Tienkung 布局不变的回归测试。Python 测试覆盖默认 mimic、Walker 当前参考采样及配置契约。安装 `onnx` 和 `onnxruntime` 后，还会运行合成 ONNX 的 1441/1477/1759 输入及输出维度预检测试（不是实际策略效果测试）。

不依赖 ROS 的 Python 测试（从仓库根目录运行，需要 numpy、PyYAML、pytest）：

```bash
PYTHONPATH=src/tienkung_motion_source python3 -m pytest src/tienkung_motion_source/test -q
```