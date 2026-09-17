# SO-101 仿真 Benchmark

基于 MuJoCo 和 Gymnasium 的 SO-ARM101 仿真项目，提供六项桌面操作任务。可以连接真实
SO-101 Leader 控制仿真机械臂、采集 LeRobot 数据集，也可以运行脚本基线和学习策略评测。

本项目包含：

- 六个可编辑的 XML 场景：五色方块入盘、蓝块叠放到红块上。
- Leader → 仿真遥操作，overview、front、wrist 三窗口同时显示。
- 按空格开始采集，成功后保存 episode，并在原窗口中重置。
- 独立 front 相机调参窗口，支持滑条、即时预览和本机参数保存。
- LeRobot v3.0 本地数据集写入，以及 π0、π0.5、SmolVLA 本地/远程推理接口。

![三视角界面示例](docs/three_views.png)

> 图片为界面示例，支架与相机参数以当前 XML 和本机配置为准。

## 目录

- [1. 环境安装](#1-环境安装)
- [2. 快速运行](#2-快速运行)
- [3. 相机调参](#3-相机调参)
- [4. 遥操作与数据采集](#4-遥操作与数据采集)
- [5. 任务与成功判定](#5-任务与成功判定)
- [6. 策略评测与回放](#6-策略评测与回放)
- [7. 配置与 XML 场景](#7-配置与-xml-场景)
- [8. 模型来源与实物对齐](#8-模型来源与实物对齐)
- [9. 开发与验证](#9-开发与验证)
- [10. 常见问题](#10-常见问题)

## 1. 环境安装

### 1.1 运行要求

建议使用 Linux；当前验证平台为 Linux、Python 3.12、MuJoCo 3.3.7。项目声明支持 Python
3.12+，推荐先使用 3.12。机器人网格和 XML 已随仓库提供，无需额外下载模型资产。

| 用途 | 需要的环境 |
| --- | --- |
| 场景预览、相机调参、脚本基线 | 本项目 Python 环境；不需要真实机械臂、相机或模型权重 |
| GUI 窗口 | 可用的桌面显示和 OpenGL；相机调参还需要 Tk |
| 遥操作采集 | 已连接、已校准的 SO-101 Leader，以及带 LeRobot/Feetech 的 Python 环境 |
| 学习策略评测 | LeRobot、对应策略依赖和匹配的检查点；模型通常需要 GPU |

Ubuntu/Debian 可安装以下系统组件：

```bash
sudo apt update
sudo apt install -y git libgl1 libglfw3 libosmesa6 ffmpeg python3-tk
```

`python3-tk` 对应系统 Python；使用 uv 管理的 Python 时，仍需检查该解释器是否带有 Tk。
安装 uv 可参考 [官方安装说明](https://docs.astral.sh/uv/getting-started/installation/)。

### 1.2 首次安装：独立仿真环境

```bash
git clone https://github.com/Fatsharkovo/so101-benchmark.git
cd so101-benchmark

uv python install 3.12
uv venv --python 3.12
uv pip install --python .venv/bin/python --no-sources -e '.[test]'
```

这条安装路径不需要同级 `lerobot/` 仓库，也不安装 PyTorch 或策略模型依赖。
`--no-sources` 忽略开发配置中的 `../lerobot` 来源；`uv pip install` 按项目依赖范围解析，
**不使用 `uv.lock` 锁定全部版本**。MuJoCo 固定为 3.3.7。

后续命令均在 **`so101-benchmark/` 根目录**执行，统一使用 `uv run --no-sync`，避免运行时
重新解析同级 LeRobot 依赖。`env -u PYTHONPATH` 用于避免 ROS 等外部 Python 路径干扰。

```bash
env -u PYTHONPATH uv run --no-sync so101-bench --help
env -u PYTHONPATH uv run --no-sync so101-bench list
env -u PYTHONPATH uv run --no-sync python -c 'import tkinter; print("Tk", tkinter.TkVersion)'
```

已有可用的 `.venv` 时可跳过环境创建，直接运行上述检查。

### 1.3 可选：LeRobot 与模型依赖

**只做遥操作采集时**，仿真环境可保持轻量，Leader 读取和数据写入使用另外一个已有的
LeRobot 环境，具体见[遥操作准备](#41-准备-leader-环境)。

**需要本地模型或远程客户端时**，可在本项目环境中增加依赖：

```bash
uv pip install --python .venv/bin/python --no-sources -e '.[policies,remote,test]'
```

这会安装较大的 PyTorch/LeRobot 依赖。根据实际 GPU 配置 CUDA/PyTorch，并确保客户端、
服务器和检查点使用兼容的 LeRobot 版本。本项目不附带模型权重，也不负责训练模型。

如果要使用同级 LeRobot 源码进行开发，目录应为：

```text
workspace/
├── lerobot/
└── so101-benchmark/
```

在 `so101-benchmark/` 下可使用仓库原有的锁定安装路径：

```bash
uv sync --locked --extra test
# 含策略与远程推理依赖：
# uv sync --locked --extra policies --extra remote --extra test
```

该路径依赖 `../lerobot` 的版本和元数据与锁文件匹配。已知本地 LeRobot checkout 的
元数据差异可能导致 `--locked` 报错，详见 [VALIDATION.md](VALIDATION.md)。仅需独立仿真
时使用 1.2 节即可；不要为修复安装而覆盖已有硬件校准或数据。

## 2. 快速运行

### 查看场景

```bash
# 查看全部任务
env -u PYTHONPATH uv run --no-sync so101-bench list

# 打开仿真预览：同一窗口内显示三个视角，不连接硬件
env -u PYTHONPATH uv run --no-sync so101-bench preview \
  --config configs/fixed.yaml --task stack_blue_on_red --display
```

鼠标左键旋转 overview 视角、右键平移、滚轮缩放，Esc 退出。实体 Leader 遥操作使用
后文的 `tools/teleoperate.py`，它会打开三个独立窗口。

### 运行脚本基线

```bash
# 六项任务各跑一个 episode，不渲染图像、不保存视频
env -u PYTHONPATH uv run --no-sync so101-bench eval --config configs/smoke.yaml
```

脚本基线使用物体真实位置、IK 和物理接触验证任务可完成性，其成绩不是学习策略成绩。
`smoke.yaml` 关闭图像，不能用于视觉模型或直接加 `--display`；需要画面时使用
`configs/fixed.yaml` 或 `configs/benchmark.yaml`。

### 无窗口渲染

默认后端为 `osmesa`，需要 `libOSMesa.so`。有兼容 GPU/驱动时，可在自己的 YAML 中设置 EGL：

```bash
cat > /tmp/so101-preview-egl.yaml <<'YAML'
tasks: [stack_blue_on_red]
sim:
  render_backend: egl
  randomization:
    layout: {enabled: false}
YAML

env -u PYTHONPATH uv run --no-sync so101-bench preview \
  --config /tmp/so101-preview-egl.yaml --output outputs/preview_egl
```

图片输出到 `outputs/preview_egl/preview/`，包含 `front.png`、`wrist.png`、`overview.png`。
`--display` 和 `camera-tune` 使用 GLFW，需要桌面显示。**CLI 会根据 YAML 设置
`MUJOCO_GL`，只在命令前设置该环境变量不能覆盖 CLI 的配置。**

## 3. 相机调参

```bash
env -u PYTHONPATH uv run --no-sync so101-bench camera-tune \
  --config configs/fixed.yaml --task stack_blue_on_red
```

窗口左侧是仿真 front 画面，右侧为滑条和数值输入。无需连接 Leader 或真实相机，
预览夹爪默认闭合，修改参数不会推进物理时间。

| 参数 | 单位与含义 | 场景默认值 |
| --- | --- | --- |
| 前后距离 X | cm，基座前方为 +X | 45 |
| 左右平移 Y | cm，基座左侧为 +Y | 0 |
| 高度 Z | cm，相对基座所在平面 | 35 |
| 下俯角 | 度，向下为正 | 60 |
| 左右转向 | 度，0° 朝向 −X | 0 |
| 画面旋转 | 度，绕镜头光轴旋转 | 0 |
| fovy | 度，垂直视场角；与下俯角不同 | 45 |

滑条和微调按钮步长为 0.1，也可直接输入数值。

- **保存并设为本机默认**：覆盖保存到同一份个人 YAML，下次打开继续编辑。
- **恢复上次保存**：重新读取磁盘文件。
- **恢复场景默认**：恢复任务 XML 的默认值，需再点击保存才会更新本机默认。
- 关闭未保存的窗口时，可以选择保存、放弃或取消；非法输入不能保存。

配置文件为 `$XDG_CONFIG_HOME/so101-benchmark/front_camera.yaml`；未设置 XDG 时为：

```text
~/.config/so101-benchmark/front_camera.yaml
```

个人参数不会改写共享 XML。遥操作启动时自动加载，并固定到当前会话；**保存新参数后，
需要重新启动遥操作才能生效**。任务重置保持相机参数，个人 front 参数不参与相机随机化。
普通 `preview`、`eval` 和历史 `replay` 不自动读取该文件；评测时需在运行 YAML 中显式配置相机。

调参工具目前只调整 front。wrist 的支架和相机参数位于
[`task_scenes/wrist_camera.xml`](task_scenes/wrist_camera.xml)，默认 fovy 为 65°。

## 4. 遥操作与数据采集

### 4.1 准备 Leader 环境

使用已有的 **SO-101 Leader 校准文件**和稳定串口路径。无需实体 Follower，也不读取真实
摄像头；front/wrist 视频来自仿真。程序读取 Leader 关节，动作施加到仿真机械臂。

Leader Python 环境需要 LeRobot、Feetech 驱动和可用的视频编码依赖。已验证的写入环境为
LeRobot 0.6.0，数据集格式为 v3.0。尚无环境时，可先单独创建：

```bash
uv venv --python 3.12 ../so101-leader-env
uv pip install --python ../so101-leader-env/bin/python 'lerobot[feetech]==0.6.0'
```

已有正常工作的 LeRobot/Conda 环境可以直接复用，无需重复安装。安装依赖不等于完成硬件
校准；新 Leader 应先按 [LeRobot](https://huggingface.co/docs/lerobot/so101) 的硬件流程完成校准。
本项目不会自动重做校准，找不到文件或校准不匹配时会报错。

先设置对应解释器，检查软件依赖和串口；以下检查不连接电机：

```bash
export SO101_LEADER_PYTHON="$(pwd)/../so101-leader-env/bin/python"
# 或改为已有环境的绝对路径，例如 /path/to/conda-env/bin/python

env -u PYTHONPATH "$SO101_LEADER_PYTHON" -c \
  'from lerobot.teleoperators.so_leader import SO101Leader; from lerobot.configs import RGBEncoderConfig; from lerobot.datasets.dataset_metadata import CODEBASE_VERSION; print(CODEBASE_VERSION)'
ls -l /dev/serial/by-id/
```

应打印 `v3.0`。默认校准位置为
`~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<leader-id>.json`，
也可能由 LeRobot 的缓存/校准环境变量覆盖。ID 必须与已保存校准一致。

### 4.2 启动三窗口遥操作

将下面两个变量改为自己的 Leader 设备和校准 ID，再运行：

```bash
export SO101_LEADER_PORT="/dev/serial/by-id/usb-YOUR_LEADER_DEVICE"
export SO101_LEADER_ID="YOUR_EXISTING_LEADER_ID"

env -u PYTHONPATH uv run --no-sync python tools/teleoperate.py \
  --config configs/fixed.yaml \
  --port "$SO101_LEADER_PORT" \
  --leader-id "$SO101_LEADER_ID" \
  --leader-python "$SO101_LEADER_PYTHON" \
  --task stack_blue_on_red
```

启动后显示 overview、front、wrist 三个独立窗口。等待数据写入进程就绪、Leader 读数正常后，
主窗口进入 `WAITING`。使用其他任务时修改 `--task`；默认 `[all]` 在遥操作入口选择叠放任务。

| 操作/状态 | 行为 |
| --- | --- |
| `WAITING` | 可试操作，不计回合时间、不采集 |
| **Space** | 恢复完整初始场景，开始当前回合采集 |
| 成功 | 自动保存；原窗口内重置到下一 seed，等待再次按 Space |
| 失败、超时或 **R** | 丢弃未完成回合，恢复当前 seed；等待时 R 也可复位 |
| **Esc** 或关闭任一窗口 | 退出并完成已保存数据的收尾，丢弃未完成回合 |
| `Saved episodes: N` | 本次会话已确认保存的成功回合数，重置不清零 |
| `ERROR` | 停止采集，查看终端错误和会话 `writer.log` |

默认每回合限时 60 秒，等待时间不计入。可用 `--seconds 300` 限制整次会话的墙钟时间；
评测用的 `episodes` 不限制遥操作采集数量。按键应在遥操作窗口获得焦点时使用。

### 4.3 数据保存

每次启动创建新的 `outputs/teleop_<时间戳>/`。可通过 `--output /path/to/new_session`
指定目录，**目标目录必须尚不存在**。

```text
outputs/teleop_<时间戳>/
├── config.json                 # 本次生效的配置，包含本机 front 参数
├── provenance.json             # 软件版本与来源信息
├── summary.json                # 退出时生成的会话统计
├── writer.log                  # 数据写入进程日志
├── dataset/                    # 官方 LeRobot v3.0 数据集
│   ├── meta/
│   ├── data/
│   └── videos/
└── episodes/episode_000000/
    ├── metadata.json           # 该成功回合的配置、任务与初始状态等
    └── scene.xml               # 实际使用的场景与相机参数
```

只保存成功回合。数据包含六维关节状态、实际应用动作、任务说明和 front/wrist 两路 H.264
视频；每帧对应动作执行前的观察，时间戳按控制频率从 0 开始。overview 仅用于查看。
相机位置、朝向和 fovy 记录在配置/场景快照中，不叠加到训练图像。

`recording.repo_id` 默认为 `local/<任务 ID>`，只写本地，不自动上传 Hub。写入器默认使用
`--leader-python`，也可在 YAML 中指定 `recording.writer_python`。该解释器必须支持官方
v3.0 写入接口；写入失败或队列满会停止采集，不会静默丢帧。

**遥操作输出不能直接传给 `so101-bench replay`**：它没有评测回放所需的
`transitions.jsonl`。请用 LeRobot 数据集工具读取 `dataset/`，或直接查看其中的视频。

## 5. 任务与成功判定

| 任务 ID | 目标 |
| --- | --- |
| `place_red_in_plate` | 红块入盘 |
| `place_blue_in_plate` | 蓝块入盘 |
| `place_green_in_plate` | 绿块入盘 |
| `place_yellow_in_plate` | 黄块入盘 |
| `place_orange_in_plate` | 橙块入盘 |
| `stack_blue_on_red` | 将蓝块叠放到红块上 |

五色任务在相同 seed 下使用相同布局，仅目标颜色不同。目标必须有机械臂接触和离桌抬升
记录；入盘要求目标整体在盘沿内、由盘面支撑，其他方块不占盘内区域。允许误放后取出纠正。

叠放要求蓝块由红块支撑、红块留在桌上且两块直立；默认横向偏差小于较小方块边长的 30%，
高度误差小于 4 mm。成功时机械臂须释放所有方块，目标连续稳定 1 秒（线速度低于
0.01 m/s、角速度低于 0.1 rad/s）。跌落和超时判为失败，默认不要求机械臂回到初始姿态。

| 场景参数 | 默认值 |
| --- | --- |
| 方块 | 边长 25 mm，质量 10 g |
| 盘子 | 浅蓝色圆角正方形，外边长 100 mm，圆角半径 12 mm |
| 盘厚度 | 底面/盘沿厚 2 mm，盘沿高出底面 6 mm，总高 8 mm |
| 控制/物理频率 | 30 Hz / 600 Hz |
| 单回合时间 | 60 秒（`smoke.yaml` 为 20 秒） |
| 布局随机化 | 默认位置每轴 ±5 mm，朝向 ±5°；`fixed.yaml` 关闭 |

场景使用明亮天空和无限视觉地面；外围地面不参与碰撞，实际操作区域仍是桌面。
叠放两块位于基座前方约 19 cm，左右各 4.5 cm。

## 6. 策略评测与回放

### 6.1 评测入口

```bash
# 关闭布局扰动，显示脚本执行过程
env -u PYTHONPATH uv run --no-sync so101-bench eval \
  --config configs/fixed.yaml --task stack_blue_on_red --episodes 1 --display

# 多任务评测；无窗口渲染使用 YAML 中指定的后端
env -u PYTHONPATH uv run --no-sync so101-bench eval \
  --config configs/benchmark.yaml \
  --task place_blue_in_plate,stack_blue_on_red --episodes 10 --seed 0 --no-display
```

| 配置 | 用途 |
| --- | --- |
| `configs/fixed.yaml` | 固定布局，适合调试/遥操作 |
| `configs/smoke.yaml` | 无图像、无视频的脚本检查 |
| `configs/benchmark.yaml` | 六任务评测和随机化参数示例 |
| `configs/pi05_local.yaml` | π0.5 本地推理示例 |
| `configs/pi05_remote.yaml` | π0.5 远程推理示例 |
| `configs/pi0_local.yaml`、`configs/smolvla_local.yaml` | 其他本地策略示例 |

**所有学习策略示例中的 `policy.checkpoint` 都需检查并替换。** π0.5 示例包含开发时的
路径，不保证在其他机器存在；本地路径相对运行目录，远程路径由服务器解释。

### 6.2 学习策略

配置 `policy.backend: local` 或 `remote`，策略类型为 `pi0`、`pi05` 或 `smolvla`。
检查点需带有匹配的 processor 和归一化统计。状态/动作顺序为：

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
```

前五维为度，夹爪为 0–100；图像为 RGB uint8，默认 640×480。检查点图像名称不同时可设置：

```yaml
policy:
  rename_map:
    observation.images.front: observation.images.camera1
    observation.images.wrist: observation.images.camera2
```

修改好配置后运行：

```bash
env -u PYTHONPATH uv run --no-sync so101-bench eval --config configs/pi05_local.yaml --mode sync
env -u PYTHONPATH uv run --no-sync so101-bench eval --config configs/pi05_remote.yaml --mode realtime
```

远程服务器在安装了相应 LeRobot/策略依赖的环境中运行：

```bash
python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=8081 --fps=30
```

跨机器部署时将服务器绑定到实际可访问的地址，并修改客户端 `policy.server`。使用匹配的
LeRobot 版本及控制频率，`policy.server_fps` 必须与 `sim.control_hz` 一致。PolicyServer 为
单客户端会话，建议给 benchmark 使用独立实例，每回合初始化会清理该实例的策略状态。

`sync` 暂停物理时间等待推理；`realtime` 按控制周期推进，丢弃过期动作，无新动作时保持
上一目标，默认连续断供 10 秒终止。实时速度低于目标的 90% 时，报告标记
`realtime_timing_valid=false`。模型加载和首段准备时间单独统计。

π0、SmolVLA 提供适配接口，但尚未完成匹配检查点的真实权重验证；具体已验证范围见
[VALIDATION.md](VALIDATION.md)。当前接口支持标准单帧观测配置。

### 6.3 评测结果与回放

```text
outputs/<时间戳>_<模式>_<后端>/
├── config.json, provenance.json
├── results.json, episodes.csv
└── <任务>/seed_000000/
    ├── metadata.json, scene.xml, summary.json
    ├── transitions.jsonl
    └── overview.mp4, front.mp4, wrist.mp4  # 开启 video 时生成
```

报告包含成功率、推理耗时、动作过期/断供和运行速度。每条 transition 保存动作前快照、
请求动作、实际动作、指标及时间，视频一帧对应一条 transition。

```bash
# 替换为实际评测回合目录；显示回放且不重新编码视频
env -u PYTHONPATH uv run --no-sync so101-bench replay \
  --episode-dir 'outputs/YOUR_RUN/stack_blue_on_red/seed_000000' --display --no-video
```

默认开启视频输出，会在回合目录的 `replay/` 中重建三路录像。无窗口回放时同样需要在
`--config` 中指定可用的渲染后端。回放优先使用保存的 `scene.xml` 和物理状态；仍需保留
外部网格及任务代码，不保证任意跨版本兼容。

## 7. 配置与 XML 场景

```text
so101-benchmark/
├── configs/                         # 运行、评测配置
├── task_scenes/                     # 可直接编辑的 MJCF XML
│   ├── common.xml                   # 机械臂、桌面、天空、灯光、front
│   ├── wrist_camera.xml             # 腕部支架、碰撞体、wrist
│   └── <任务 ID>.xml                # 方块、盘子等任务物体
├── src/lerobot_env_so101/
│   ├── assets/so101/                # 机器人网格、上游 XML、来源记录
│   ├── task_configs/                # 任务 ID、指令、类和参数
│   ├── tasks/                      # 成功判定与任务逻辑
│   ├── camera_tuner.py              # 相机调参窗口
│   └── teleop.py                    # 采集状态与数据写入协调
├── tools/teleoperate.py             # Leader 遥操作启动入口
├── tests/
└── outputs/                         # 本地结果，不随 Git 提交
```

运行 YAML 叠加到默认配置，CLI 参数优先于 YAML。相机/场景的生效顺序是 XML 默认值 →
显式任务参数和运行配置 → 启用的随机化；遥操作还会在启动时应用个人 front 参数，并固定该相机。

XML 在内存中展开 include 后加载，不改写源文件。编辑后重新启动环境生效，当前窗口的
R/Space 不会重新读取 XML。可以用 MuJoCo 直接加载六个任务 XML；它们和网格也随 wheel 分发。

随机化分 `layout`、`appearance`、`camera`、`size`、`physics` 五组，默认只开启布局组。
更改范围后应复查可达性。相机可通过 `sim.cameras.<名称>` 指定 `position`/`pos`、`quat`、
`target` 或 `euler`、`fovy`；位置单位为米，Euler 角为弧度，fovy 为度，四元数顺序为 wxyz。
front 为世界坐标，wrist 为所属机械臂部件的局部坐标。使用 `fixed: true` 可排除该相机随机化。

扩展任务时在 `task_paths` 指定的目录中新增任务 YAML，包含唯一 `id`、`class`、
`instruction` 和 `params`；路径相对运行 YAML 解析。可复用现有 `PlaceInPlate`/`StackBlueOnRed` 类，
新增行为则继承 `Task` 并实现 `scene()`、`evaluate(env)`，按需实现 `reset(env)` 和
`make_oracle(env)`。参照 [`task_configs`](src/lerobot_env_so101/task_configs) 和
[`tasks`](src/lerobot_env_so101/tasks)。评分依赖约定的物体/盘子结构，任意新几何不等于自动支持新评分规则。

LeRobot 环境插件名为 `so101_bench`，可从 `lerobot-eval --env.type=so101_bench` 接入；
`SO101SimRobot` 也提供 `connect/get_observation/send_action/reset_episode/disconnect` 接口。

## 8. 模型来源与实物对齐

机械臂主体来自 [MuJoCo Menagerie 的 robotstudio_so101](https://github.com/google-deepmind/mujoco_menagerie/tree/ac6b2b09983786f3036cab1000221017fa2193b4/robotstudio_so101)，
基于 TheRobotStudio SO101 设计，原始资产采用 Apache-2.0。固定版本和 SHA256 见
[`manifest.json`](src/lerobot_env_so101/assets/so101/manifest.json)。

腕部替换为[矽递官方教程](https://wiki.seeedstudio.com/lerobot_so100m_new/#3d-printing-guide)
链接的 [soarm_soft_gripper STEP](https://github.com/xiehuangbao888/soarm_soft_gripper) 中的
`SO-ARM101_CAMERA_MOUNT`，仅提取支架，保留硬夹爪。来源版本、坐标变换和网格校验值见
[`seeed_camera_mount_manifest.json`](src/lerobot_env_so101/assets/so101/seeed_camera_mount_manifest.json)。
该外部仓库未提供明确许可证文件，独立记录来源，不将其标记为 Menagerie 的 Apache-2.0 资产。
转换工具 [`tools/convert_seeed_camera_mount.py`](tools/convert_seeed_camera_mount.py) 可重建 STL，
Gmsh 仅在转换时需要，运行仿真不需要安装。

当前实物对齐设置：

- 关节转换为 `q_sim = radians((角度 - offset) * sign)`；offset 默认
  `[0, 0, 0, 0, 90]`，Leader 的 wrist_roll 零度对应仿真 −90°。
- front 默认前方 45 cm、高 35 cm、下俯 60°、fovy 45°。主窗口的相机位置标记为辅助显示，
  不进入训练图像或参与碰撞；设置 `sim.cameras.front.show_pose: false` 可关闭。
- wrist fovy 65°，光轴沿新支架安装面法线；局部绕 X 轴约 −25°，不是相对桌面的俯角。
  光心按安装面外 1.6 mm PCB 加 5 mm 镜头伸出量估算，支架质量沿用 12 g，均非实测标定。
- 从基座起第二个舵机的外壳/安装座为 `second_servo_housing`、`second_servo_mount`，
  接触参数为 `friction="0 0 0"`、`condim="1"`、`priority="2"`。保留实体碰撞，
  仅消除接触摩擦；关节自身 frictionloss、阻尼和长臂摩擦不变。

不同批次的实物支架、镜头与舵机零位仍需核对。原支架网格保留，以支持已有场景快照回放。

## 9. 开发与验证

```bash
# EGL 平台；没有 EGL 时根据环境改为 osmesa
env -u PYTHONPATH MUJOCO_GL=egl uv run --no-sync pytest tests -q

# 需要桌面的 GUI 检查，只操作仿真
env -u PYTHONPATH MUJOCO_GL=glfw SO101_TEST_GUI=1 \
  uv run --no-sync pytest tests/test_camera_tuner_gui.py tests/test_teleop_viewer.py -q

uv run --no-sync ruff check src tests tools
uv run --no-sync ruff format --check src tests tools
uvx pre-commit run --all-files
uv build
```

设置 `SO101_DATASET_PYTHON=/path/to/lerobot-env/bin/python` 可启用独立 LeRobot 环境的
数据集写入集成测试；可选依赖未安装时相关测试会跳过。安装步骤、测试范围和历史结果见
[VALIDATION.md](VALIDATION.md)，功能变更见 [CHANGELOG.md](CHANGELOG.md)。普通软件检查
无需连接电机或真实相机。个人相机配置、校准文件、运行日志和原始数据不应提交到 Git。

## 10. 常见问题

| 问题 | 排查方法 |
| --- | --- |
| 新克隆后安装提示找不到 `../lerobot` | 使用 1.2 节的独立安装命令；源码开发模式才要求同级仓库 |
| `uv sync --locked` 提示锁文件需更新 | 检查同级 LeRobot 版本；已安装环境用 `--no-sync` 运行，独立安装见 1.2 节 |
| 没有显示窗口 | 确认使用 `--display` 或 GUI/遥操作入口，且当前终端连接可用桌面显示；纯 SSH 通常不能直接弹出本地窗口 |
| `libOSMesa.so` 缺失或 EGL 初始化失败 | 安装对应渲染库/驱动，或在 YAML 中选择可用后端；GUI 使用 GLFW |
| 相机调参提示没有 Tk | 检查当前 Python 的 `import tkinter`，系统 `python3-tk` 不一定适用于 uv/Conda Python |
| 保存 front 参数后画面没变 | 重启遥操作；普通预览和评测不自动读取个人配置 |
| 本机相机 YAML 损坏 | 调参窗口会提示，可重新调整并保存修复；遥操作会报错，避免静默使用其他参数 |
| Leader 串口或校准报错 | 核对 by-id 路径、访问权限、端口是否被占用、校准 ID 与当前硬件是否一致 |
| 窗口能动但没有数据 | `WAITING` 不采集，先按 Space；只保存满足任务规则的成功回合 |
| 退出前看到 `ERROR` | 查看终端及 `writer.log`，确认写入 Python 支持 v3.0 API 和编码依赖 |
| 回放提示缺少 `transitions.jsonl` | 使用评测回合目录；遥操作的 LeRobot 数据集应使用 LeRobot 工具读取 |
| 策略无法加载 | 检查 checkpoint、processor、动作单位、图像名称和 LeRobot 版本；示例路径需替换 |
