# tienkung_runtime_monitor

独立、只读的 runtime JSONL 监控包。服务端只有 Python 标准库依赖；浏览器使用本地 JavaScript / WebGL，不依赖 CDN、npm、ROS topic 或机器人控制库。ROS launch 仅启动监控进程，不启动策略、不发布命令。也可直接用 Python 启动。

## 1. 板卡上构建

在已配置好的 Linux ROS 2 Humble 工作区中：

```bash
source /opt/ros/humble/setup.bash
# 按主 README 配置 bodyctrl_msgs 和 ONNX Runtime。
colcon build --symlink-install --packages-select tienkung_runtime_monitor tienkung_policy_runner
source install/setup.bash
```

仅部署监控包可单独构建 `tienkung_runtime_monitor`，不需要 ONNX / native transmission；策略节点仍须使用其正式 Linux 环境构建。

## 2. 开启策略日志

监控依赖新版策略节点输出 schema v2 日志。为每个运行节点使用独立的日志目录，避免自动选择功能在多个写入者间切换。

```bash
mkdir -p /tmp/walker-runtime
ros2 launch tienkung_bringup walker_policy_runner.launch.py \
  policy_path:=/absolute/path/to/policy.onnx \
  manifest_path:=/absolute/path/to/policy_manifest.yaml \
  enable_runtime_log:=true \
  log_dir:=/tmp/walker-runtime
```

**此命令运行真实策略，必须先完成机器人标定和现场安全检查。监控不会让正式运行变成 dry-run。**

日志写入器采用有界异步队列，默认 256 条，节点参数 `runtime_log_queue_capacity` 可通过 ROS 参数文件覆盖。队列竞争或满时丢弃新记录，不等待磁盘；查看 `dropped_records`。生产线程仍需复制采样数据，不能把“异步”理解为零开销。完整 observation 写入磁盘，默认不通过监控 API 传输。

## 3. 传入板卡上的 Walker URDF

把整个 Walker assets 目录复制到板卡，保持 `urdf/meshes/` 相对结构。例如部署到 `/opt/robot_assets/walker`：

```bash
ros2 launch tienkung_runtime_monitor runtime_monitor.launch.py \
  log_dir:=/tmp/walker-runtime \
  urdf_path:=/opt/robot_assets/walker/urdf/walker_tienkung_ei_nofinger.urdf \
  asset_root:=/opt/robot_assets/walker
```

`urdf_path` 是监控 launch 参数，不是策略控制节点参数。它仅用于显示，不影响控制、标定、关节顺序或 transmission。不要传 Mac 本机路径给板卡。

也可以直接启动服务：

```bash
ros2 run tienkung_runtime_monitor runtime_monitor \
  --log-dir /tmp/walker-runtime \
  --urdf /opt/robot_assets/walker/urdf/walker_tienkung_ei_nofinger.urdf \
  --asset-root /opt/robot_assets/walker
```

无需 ROS 的源码启动（工作区根目录）：

```bash
python3 src/tienkung_runtime_monitor/tienkung_runtime_monitor/runtime_monitor.py \
  --log-dir /tmp/walker-runtime \
  --urdf /opt/robot_assets/walker/urdf/walker_tienkung_ei_nofinger.urdf
```

不提供 URDF 时显示关节角度仪表，不虚构机器人形状。支持 URDF 本地 STL visual、box/sphere/cylinder、固定/旋转/移动/mimic 关节；不执行 xacro，不加载 MJCF XML、远程 mesh 或任意文件。相对 mesh 路径相对于 URDF 文件；`asset_root` 默认 URDF 所在目录。`package://` 仅支持显式 asset root 内可解析的同名包布局，不进行 ROS 全局包搜索。

本仓库不复制训练仓库的 URDF/STL。Walker 无手指版的关节名称按 `joint_order_policy` 映射；无有效反馈的部位及其后代显示灰色上下文零位，不视为测量。

## 4. 打开页面

默认监听 `0.0.0.0:8080`（所有 IPv4 网卡），同局域网电脑访问 `http://板卡IP:8080`，例如 `http://192.168.1.100:8080`。板卡本机也可访问 `http://127.0.0.1:8080`。浏览器不要填写 `0.0.0.0`；板卡防火墙需允许可信局域网访问 TCP 8080。

如果希望只允许本机访问，启动时传 `bind:=127.0.0.1` / `--bind 127.0.0.1`，并从操作电脑使用 SSH 转发：

```bash
ssh -N -L 8080:127.0.0.1:8080 robot@BOARD_IP
```

使用 SSH 转发时，在操作电脑浏览器打开 `http://127.0.0.1:8080`。此服务**无认证**，默认局域网监听会暴露模型路径、关节和命令数据，只能放在可信网络或有认证的代理后，不应暴露公网。

### 页面内容

- 三个 URDF 模型：有效 Motion Reference、后处理 target、实际 dof feedback。
- 当前模式、是否已发布命令、推理耗时、周期、readiness、选中关节误差和日志新鲜度。
- 所选关节位置、raw action、速度、力矩估计、电流及机身姿态和时序曲线。
- 全关节表、裁剪/阻断/发布/反馈有效性、实际电机命令、输入年龄、完整配置 metadata。
- 时间窗口、关节切换、视角切换、暂停显示、浅/深主题、辅助线型、键盘/悬浮读数和曲线数据表。
- 三个模型视图各自带棋盘地板，并支持独立调整相机：左键/单指拖动旋转，右键/中键或 Shift+拖动平移，滚轮缩放。双击或 Home 恢复该视图预设；“重置三个视角”统一复位。聚焦模型后可用方向键旋转、Shift+方向键平移、+/- 缩放。实时采样刷新和主题切换不会覆盖手动视角；选择斜视/正视/侧视会重置三个相机。

服务每 100 ms 尾读一次，浏览器请求完成后等待 200 ms 再请求；不是硬实时可视化。默认最多保存/显示 1500 帧（50 Hz 时约 30 秒），还有字节上限。STL 在浏览器加载并渲染，板卡只传输静态资源；实际 Walker 网格较大，三视图 GPU 内存开销约 212 MB，加上 CPU/其他资源，建议用操作电脑浏览器。模型加载失败会明确提示并回退骨架。

### 重要限制

- 模型采用固定根基座，不重建世界位移、根部平移或实际接触；IMU 姿态另有曲线。不是完整世界坐标运动回放。棋盘地板高度固定于 URDF 中性姿态包围盒下界，不随腿部运动升降；仅供空间视觉参考，不是实测地面，也不执行 MuJoCo 物理仿真、碰撞或自动贴地。
- `raw_action` 不是关节角；`target` 是后处理关节目标，不一定发布，也不等于 motor-space 命令。
- 头部阻断、未发布、无效/过期反馈不纳入跟踪误差。部件新鲜度来自收到消息的时间，不是电机采样硬件时钟，也没有新增逐电机超时保护。
- `dof_torque_estimated` 是电流/标定/传动转换后的估计，不是力矩传感器；电流单位及比例须核对驱动器契约。
- 最新文件选择按修改时间，适合单写入者目录；要固定一个运行，用 `log_file:=...` 或 `--log-file ...`。
- dry-run 日志使用 `pattern:=dry_run_log_*.jsonl`。dry-run POLICY 不发布策略动作，但 ZERO 会发真实归零命令。当前 STOP 实现也会发布 kp=0、保留 kd 的电机命令及腰部命令；本次没有改变该语义。
- 数据变旧或连接中断时保留最后帧并明确标记过期，不据此判定硬件安全。磁盘日志不会自动轮转，部署时需监控剩余空间。

## 5. 完整离线回放

回放已停止写入的 JSONL 文件，不需要启动策略节点或连接机器人。`log_file` 指向服务所在机器上的文件；必须显式开启 `replay`，否则仍使用实时尾读模式。

```bash
ros2 launch tienkung_runtime_monitor runtime_monitor.launch.py \
  replay:=true \
  log_file:=/absolute/path/to/walk.jsonl \
  urdf_path:=/opt/robot_assets/walker/urdf/walker_tienkung_ei_nofinger.urdf \
  asset_root:=/opt/robot_assets/walker
```

也可使用不依赖 ROS 的 Python 入口（工作区根目录）：

```bash
python3 src/tienkung_runtime_monitor/tienkung_runtime_monitor/runtime_monitor.py \
  --replay --log-file /absolute/path/to/walk.jsonl \
  --urdf /opt/robot_assets/walker/urdf/walker_tienkung_ei_nofinger.urdf
```

页面会显示索引进度，完成后停在第一条有效采样，不自动播放：

- **播放/暂停**：从当前位置播放到末尾；末尾再次点击播放会回到第一帧。
- **时间轴拖动**：预览并跳转至相应时刻，拖动会暂停播放。
- **按秒跳转**：输入距日志首帧的秒数，例如 `573.74`，点击“跳转”或按 Enter。选择不晚于该时刻的采样，超出末尾则定位最后一帧。
- **倍速**：0.25×、0.5×、1×、2×、4×。页面约每 33 ms 选取一次历史采样，不插值伪造动作；高倍速可能跳过部分显示帧，逐帧检查请用下一项。
- **上一帧/下一帧**：暂停并精确移动一条有效采样，显示当前帧序号（从 1 开始）。
- **曲线与模型同步**：按选中帧显示向前 5/10/30 秒的曲线，保留手动相机视角。切到后台标签页自动暂停。

服务后台逐行扫描全文件，只保留每帧的文件偏移与相对时间索引（约 16 字节/帧，不含固定对象开销），不是把整个 1 GB 日志装入内存。首次定位读取曲线历史及后续帧；连续播放在缓存耗尽前后台预取，只请求缓存末尾之后的新帧，不重复传输历史。预取长度随倍速在 4–12 秒日志时间之间调整，触发时机根据实测请求耗时留出余量。预取成功不暂停或重置播放时钟；只有数据确实耗尽才显示“缓冲后续帧”，冻结在最后有效帧，恢复后继续而不跳过等待期间的数据。

单次响应最多 1500 帧且受字节限制，服务优先保留后续帧；浏览器滚动缓存最多 3000 帧，淘汰旧历史而非整份日志。窗口被截短时有提示。模型采样显示循环约 30 Hz，曲线/关节表/诊断最多 5 Hz，展开的历史数据表最多 1 Hz；暂停或手动定位时立即刷新所有信息。实际帧率仍受 GPU、网络和数据解析耗时影响，不承诺硬实时播放倍速。

时间轴优先使用 `timing.steady_timestamp_sec`，缺失时按可用日志时间回退；时间倒退增量压为 0，并报告时钟回退次数。原始 ROS timestamp 仍显示在帧信息中。无效行、超长行和未完成尾行跳过并计数，逐帧以成功索引的有效采样为单位。

回放期间不要追加、替换或修改日志；检测到文件变化会拒绝继续读取，需要重启服务重新建立索引。历史数据不会显示为“实时过期”，历史 `command_published` 仅表示当时的发布记录，不会重发任何命令。

部署后建议检查：从头播放到末尾、跳转到 `573.74` 秒、连续逐帧前后移动、切换倍速、拖动时间轴取消正在加载的旧请求，以及在实时模式下原有监控是否正常。

## 日志格式（schema v2）

第一行 `type: metadata`，包含 `joint_order_policy`、`can_id_by_index`、解析后的 `config`、`runtime` 和模型/config/manifest 路径。路径不是内容哈希，复现实验时应另行归档原始模型、manifest、配置和代码版本。

每帧 `type: sample` 保留旧字段 `timestamp/mode/obs/target/dof_pos/dof_vel/motion_ref`，并补充：

| 字段 | 含义 |
| --- | --- |
| `raw_action`, `effective_motion_ref` | 本帧原始推理结果、运行时真正使用的 mimic |
| `rpy`, `angular_velocity`, `dof_torque_estimated` | 机身状态、关节力矩估计 |
| `inference_executed`, `inference_latency_ms`, `runtime_executed` | 推理和运行时执行情况 |
| `command_published`, `command_published_mask`, `command` | 发布标记、逐关节掩码、实际赋值到 ROS 消息的命令 |
| `state_ready`, `motion_ready`, `motion_reference_valid`, `motion_reference_state` | readiness 和参考新鲜度；fresh/stale_or_missing 不细分插值器内部阶段 |
| `input_age_sec` | 各分组、IMU、motion、Joy 接收年龄，未收到为 null |
| `timing` | steady 时间、tick 间隔、runtime/命令发布/processing 耗时 |
| `motor_feedback` | 各分组原始反馈、filtered 电机空间数组与有效性、策略索引顺序 raw current |
| `dropped_records` | 日志队列累计丢弃记录数 |

`timing.processing_ms` 不含队列入队、异步序列化/磁盘写入和最后的控制台输出，不是完整实时 deadline 证明。`timestamp` 和输入年龄使用 ROS 时钟，`timing.steady_timestamp_sec`/tick 间隔使用单调时钟。非有限数编码为 JSON null。

正常退出最后写入 `type: logger_stats`，记录 dropped/written/write_errors；突然断电时尾行可能不完整，监控等待完整行而不误解析。`written_records` 表示成功提交给输出流的记录数，不保证断电持久化；磁盘故障时最后的统计本身也可能无法写出。正常退出会等待后台写入完成，卡住的文件系统可能延长退出时间。

## API

- `GET /api/snapshot?after=N`：metadata、增量 frames（附 `monitor_seq`）、generation、最新序号、source、age、错误/跳过计数、URDF 和 logger_stats。启动 after=0 返回有界历史；换文件/截断后 generation 变化。
- `GET /api/health`：轻量服务状态。健康仅代表服务状态，不代表机器人状态。
- 回放模式 `/api/snapshot` 附带 `playback` 索引状态与进度；`GET /api/replay?time=573.74&window_sec=10&ahead_sec=2` 或 `?frame=28683` 按时间/从 0 开始的帧号返回 `selected_frame` 及有界 `frames`。帧含 `replay_frame_index`、`replay_time_sec`；两个定位参数不能同时提供。播放速度和暂停状态在浏览器维护，不操作机器人。
- 只读固定静态页面和 URDF 明确引用的编号 mesh 路由；无电机控制、上传或任意文件下载接口。

## 部署环境验证（未在本机执行 ROS 验证）

```bash
colcon test --packages-select tienkung_policy_runner tienkung_motion_source tienkung_runtime_monitor
colcon test-result --verbose
# 可选：前端纯逻辑测试，需要 Node.js
node --test src/tienkung_runtime_monitor/test/test_robot_view.mjs src/tienkung_runtime_monitor/test/test_replay_buffer.mjs
```

在断开执行器或经现场安全批准的条件下验证对应 dry-run、日志实时增长、参考/target/feedback 关节映射、超时提示、STOP/ZERO/POLICY 命令语义、日志写入开销和长时间磁盘占用。本地网页/纯逻辑检查不能代替 Linux ROS 或实机验证。
