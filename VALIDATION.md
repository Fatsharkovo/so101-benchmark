# 验证记录

## 遥操作运动平滑复验（2026-09-17）

- 对无接触抬臂/放回轨迹（肩/肘/腕同时运动，30 Hz 目标、600 Hz 物理步）对比原控制、
  增加阻尼、降低刚度及目标插值。原参数 kp=998.22、kv=2.731；关闭接触后结果一致。
- 以 8 Hz 低通后的残差衡量运动期间高频波动：腕部 RMS 从约 0.0849° 降至 0.0017°；
  肩部从 0.0876° 降至 0.0027°。该轨迹的五关节力矩饱和采样比例从 15.17% 降至 0%。
  使用插值时运动关节最大 RMS 跟踪误差约 0.28°，旧控制约 0.219°。
- 选择保留 kp/kv，仅在遥操作默认配置中开启插值。测试验证高频波动下降超过 90%、
  RMS 跟踪误差小于 0.5°、停止后误差小于 0.2°，以及 R/Space 所用初始恢复后的确定性。
- 六任务固定场景分别在插值开/关下完成，12/12 通过；外壳零接触摩擦回归通过。
- 完整回归 **83 passed、6 skipped**，包含独立官方数据集写入。跳过三项桌面 GUI
  与三项可选依赖测试；本地脚本语法及 --help 通过，my_scripts 继续被 Git 忽略。
- 上述量化结果来自仿真合成轨迹，原始参数对比保存在本地 outputs/motion_tuning/。
- 随后通过本地脚本启动实体 Leader → 仿真三窗口，启用目标插值；用户实际抬起/放回
  复验后反馈“已经恢复正常”。该反馈确认本次试操作中的改善，不代表所有姿态的量化验证。

## 木纹桌面与灰白机械臂复验（2026-09-17）

- 从 ambientCG 下载 Wood049 1K-PNG 原始颜色贴图，确认 CC0-1.0 授权并记录来源/SHA256。
- 六任务 overview 渲染及叠放 front/wrist 预览通过，灰白打印件、深色舵机、木纹桌面可见。
- 全套回归 **74 passed、6 skipped**；包含六任务脚本物理基线和独立 LeRobot 数据集写入。
  本轮未开启三项桌面 GUI 测试，另三项需可选 LeRobot/gRPC 依赖。
- 第二舵机外壳实际接触仍为零摩擦/仅法向约束；XML/Python 场景与相机检查通过。
- 独立解包 wheel 中 XML 与 Python 场景均可加载 1024×1024 木纹并渲染；Ruff 检查通过。
- 本轮不操作实体机械臂；预览图保存在本地 `outputs/wood_table_validation/`。

## 矽递腕部支架替换复验（2026-09-17）

- 变更后全套回归：73 passed、6 skipped，包含官方数据集写入与六任务固定场景基线。
  随后新增共享支架/光学开口检查，场景对齐测试 10 passed。
- 第二舵机外壳/安装座仍为 friction=0、condim=1、priority=2；高摩擦物体实际接触
  仍仅产生法向约束。关节 frictionloss 和上臂摩擦保持原值。
- XML/Python 场景共用同一支架与镜头参数；相机光轴正反方向均不被分段碰撞盒遮挡。
- STEP 的两个安装孔约 8.1 mm 间距，对齐夹爪 +Y 安装面；渲染检查显示多段折弯支架，
  wrist 图像包含夹爪和操作区域，初始姿态没有支架接触。
- Gmsh 4.15.2 从固定来源 STEP 重建的二进制 STL 与随包文件 SHA256 完全一致。
- 三项桌面 GUI 测试通过；独立解包 wheel 可加载新增 XML 和支架网格并渲染。
- 光心位置与 12 g 支架质量仍为文档中记录的估算；本轮没有做实物光学或质量标定。

## Front 相机调参窗口复验（2026-09-17）

- 全套测试：**73 passed、6 skipped**，包括独立 LeRobot 环境的官方数据集写入测试。
  其中三项 GUI 测试改在 GLFW 桌面下单独运行，**3 passed**；其余跳过项需要可选
  LeRobot/gRPC 依赖。
- 七项参数均验证即时改变 front 图像，保持模型、渲染器及完整物理状态；完整朝向转换
  覆盖正负 90° 俯角。wrist 相机不受个人 front 参数影响。
- 验证保存、重新打开、修改后覆盖保存、恢复、非法输入、损坏文件恢复、保存失败保留旧文件，
  以及关闭时保存/放弃/取消。新遥操作配置使用最新文件，已有会话与重置保持启动参数。
- 检查实际相机位置、朝向和 fovy 进入有效配置与场景快照；XML 和 Python 场景一致。
- 桌面截图确认中文标签、全部控件和保存按钮可见，预览保持宽高比；兼容当前 Tk 的中文字体。
- wheel 构建后在独立解包目录导入新模块、加载场景网格并渲染成功。
- 本轮仅使用仿真，没有连接实体 Leader 或真实相机；测试配置均使用临时路径。

```bash
env -u PYTHONPATH MUJOCO_GL=egl SO101_DATASET_PYTHON=/path/to/lerobot-env/bin/python \
  uv run --no-sync pytest tests -q
env -u PYTHONPATH MUJOCO_GL=glfw SO101_TEST_GUI=1 \
  uv run --no-sync pytest tests/test_camera_tuner_gui.py tests/test_teleop_viewer.py -q
uvx pre-commit run --all-files
```

## 相机可视化与第二舵机外壳复验（2026-09-17）

- 全套测试：51 passed、5 skipped；其中 GUI 在 GLFW 显示下单独运行，1 passed。
  其他跳过项为可选 LeRobot/gRPC 和本轮未启用的独立数据集写入测试。
- 验证第二舵机的肩部外壳/安装座摩擦参数为零，并用高摩擦球体实际产生碰撞：
  接触维度仍为 1（仅法向），长臂摩擦及 shoulder_lift 的 frictionloss 保持原值。
- 主窗口相机标记跟随真实 front 光心；开启后增加 11 个仅用于显示的几何元素，
  front/wrist 图像数组不变，窗口模型重置测试通过。
- 预览确认相机外壳、视野框和光轴可见；首次视角覆盖机械臂和外部相机。
- Ruff、格式检查及 pre-commit 通过。既有六任务固定场景物理基线均通过。

## 遥操作采集与 XML 场景复验（2026-09-16，当前版本）

- 相机默认位置 `[0.30, 0, 0.35]` 米，下俯 60°；六个原生 MJCF 入口可直接编译，
  运行时加载根目录 `task_scenes/`，XML 编辑、配置覆盖和评分尺寸同步通过测试。
- 等待试操作不计入回合；Space 恢复完整初始状态后采集；失败/超时/R 丢弃；
  仅保存确认后增加计数。主窗口沿用原顶部信息行，追加计数和 `Space: start recording`。
- 全套测试 **51 passed, 4 skipped**：其中 GUI 测试在 GLFW 下单独运行并通过，
  其余三个跳过项需要 uv 环境中未安装的可选 LeRobot/gRPC 依赖。
- GLFW 测试通过：连续重建模型后，三窗口原生 ID、用户窗口大小及主视角均保留；
  Space 的 REPEAT 事件不触发开始，PRESS 才触发。
- 使用已有 LeRobot 0.6.0 解释器完成官方 v3.0 写入、丢弃、退出收尾与重新读取，
  检查状态/实际动作/时间戳及 front/wrist 视频帧对应。
- 默认 640×480、30 Hz 的三窗口端到端验证连续采集 **2 个成功叠放回合**；
  自动保存、计数 0→1→2、原窗口内重置及等待下一次 Space 均通过；
  官方读取器重新读取 **568 帧、两路 640×480 视频、30 Hz** 成功。
  该验证使用纯仿真脚本动作，本轮没有连接实体 Leader，也没有改写硬件校准。
- 六任务各 seed 0–9，20 秒上限：**59/60 成功**，唯一失败仍为黄色入盘 seed 7
  抓取滑脱后 IK 补偿不可达，与上一版本一致；六个固定场景基线全部成功。
- wheel 构建及从独立解包目录加载六任务场景/网格通过；Ruff 和 pre-commit 检查通过。

`uv lock --check --offline` 在本工作区仍提示需要更新锁文件；用未修改的 `origin/main`
配置与锁文件复验也得到同样结果，属于原有本地 LeRobot checkout 元数据差异。
本次不修改依赖版本，测试与运行使用已有 uv 环境及 `--no-sync`。

本地验证产物在 `outputs/teleop_validation/`，不随代码提交。
自动化命令（`SO101_DATASET_PYTHON` 指向已有 LeRobot v3.0 写入环境）：

```bash
env -u PYTHONPATH MUJOCO_GL=egl SO101_DATASET_PYTHON=/path/to/lerobot-env/bin/python \
  uv run --no-sync pytest tests -q
env -u PYTHONPATH MUJOCO_GL=glfw SO101_TEST_GUI=1 \
  uv run --no-sync pytest tests/test_teleop_viewer.py -q
uv run --no-sync ruff check src tests tools
uv run --no-sync ruff format --check src tests tools
uvx pre-commit run --all-files
```

历史章节中的旧相机参数、未实现采集/回放等边界描述仅对应其当时版本。

## 实物参数对齐复验（2026-09-16，上一版本）

- wrist_roll 零位映射改为仿真角度 = Leader 角度 - 90°，夹爪与相机支架整体转向前方。
  已验证正反转换、正向角增量和默认支架位置。更新后运行实体 Leader → 仿真遥操作，
  三个窗口均确认可见，叠放任务在第 461 个控制步判定成功；这不替代完整的实机标定测量。
- front 相机位于 `[0.35, 0, 0.30]` 米，向下朝向基座方向 45°，垂直视场角 60°。
- 所有方块默认质量 10 g；浅蓝圆角方盘外边长 100 mm，圆角半径暂取 12 mm，
  盘底/盘沿厚度均为 2 mm，盘沿高出底面 6 mm。碰撞、入盘判定、脚本放置高度同步更新。
- 全套测试 **35 passed, 3 skipped**，Ruff 检查与格式检查通过。
- 六任务各 seed 0–9 的脚本物理基线 **59/60 成功**。黄色入盘 seed 7 抓取滑脱后，
  脚本 IK 补偿目标不可达；其余五项任务均 10/10。这是脚本基线结果，未评测学习模型。
  [结果](outputs/physical_alignment/thin_blue_plate/results.json)；
  [更新后的预览](outputs/physical_alignment/thin_blue_plate/preview.png)。

本节 `outputs/` 链接指向本地验证产物，不随代码提交；校准文件、运行日志、数据集和
模型权重也不纳入提交。复验命令：

```bash
env -u PYTHONPATH MUJOCO_GL=egl uv run --no-sync pytest tests -q
uv run --no-sync ruff check src tests tools
uv run --no-sync ruff format --check src tests tools
env -u PYTHONPATH MUJOCO_GL=egl uv run --no-sync so101-bench eval \
  --config configs/smoke.yaml --episodes 10 --seed 0
```

评测入口遇到运行错误会提前结束；要复现全部 60 个回合，需分别运行各任务，并在
黄色任务 seed 7 的已知错误后单独运行 seed 8–9。

## 本机场景优化复验（2026-09-16）

六项任务统一增加浅蓝渐变天空、明亮照明、无限视觉地面，并收拢默认方块布局。
默认每轴位置扰动缩小至 ±5 mm，朝向扰动缩小至 ±5°；叠放中心间距改为 9 cm。
以下为更新后的复验，后续章节保留原开发环境的历史记录。

- Python 3.12.13、MuJoCo 3.3.7，uv 独立环境；渲染使用 EGL。
- `env -u PYTHONPATH MUJOCO_GL=egl uv run --no-sync pytest tests -q`：
  **27 passed, 3 skipped**。跳过项需要未安装的 LeRobot/gRPC。
- `ruff check src tests tools`、`ruff format --check src tests tools`：通过。
- 六任务各 seed 0–9、20 秒上限、默认布局随机化：脚本物理基线 **60/60 成功**。
  [原始结果](outputs/scene_refresh/baseline/20260916T131436.998122Z_sync_scripted/results.json)。
- 六场景三视角均完成渲染：[预览总览](outputs/scene_refresh/all_tasks.png)。

本次未重新评测学习模型。无限地面仅参与显示，桌面接触及跌落失败判定保持原样。

验证日期：2026-09-16。平台：Linux、Python 3.12.13、MuJoCo 3.3.7；真实模型测试使用
RTX 5090 D v2。开发虚拟环境复用了现有 `soarm101` 的 PyTorch/LeRobot；另外创建了
不继承系统包的临时环境验证独立 wheel。没有操作实体机器人，也没有重置已有部署服务。

## 自动化与安装包

| 检查 | 结果 | 记录 |
| --- | --- | --- |
| 本项目完整测试 | 30 passed | [日志](outputs/validation/benchmark_tests.log) |
| 干净环境安装 wheel 后运行测试 | 27 passed，3 skipped | [日志](outputs/validation/clean_package_tests.log) |
| Ruff 检查和格式检查 | 全部通过，24 个 Python 文件 | `ruff check src tests tools` / `ruff format --check src tests tools` |
| 依赖锁检查 | 通过，93 个包解析一致 | `uv lock --check --offline` |
| sdist / wheel 构建 | 通过，含六项任务 YAML、机器人 XML、网格和原许可证 | [日志](outputs/validation/package_build.log) |
| LeRobot 相关测试 | 39 passed | [日志](outputs/validation/lerobot_focused_tests_external.log) |

干净环境跳过的三项需要可选的 LeRobot/gRPC，仿真、成功判定、视频、回放和实时队列测试
均运行通过。Gymnasium 提示动作空间宜归一化；本环境有意采用与 SO-101 对齐的绝对角度
和 0–100 夹爪目标，保留该提示。

自动化覆盖：固定 seed 与五色场景一致性、目标隔离、误放纠正、接触/抬升/释放/稳定判定、
叠放支撑、六项任务物理基线、关节转换与非法动作、外部 YAML/Python 任务扩展、随机化
组独立性、Gymnasium 接口、视频帧与状态对应和回放、LeRobot 插件发现/向量环境、
仿真 Robot 动作接口、数据特征契约，以及动作过期、跨回合拒绝、断供和实时统计。
实时断供单元测试采用可控时钟，避免渲染速度影响其预设的延迟条件。

LeRobot 相关测试命令（在 `../lerobot` 下运行）：

```bash
../so101_benchmark/.venv/bin/python -m pytest \
  tests/envs/test_dispatch.py tests/processor/test_pi05_processor.py \
  tests/async_inference/test_policy_server.py -q
```

也尝试了 LeRobot 完整测试集。补充数据集依赖后，仍有两个收集错误：TorchCodec 0.11.0
加载时缺少 `libnppicc.so.13`，SARM 测试缺少 `pydantic`；20 项跳过，未进入完整执行。
因此**不宣称上游完整测试通过**。见 [完整测试日志](outputs/validation/lerobot_full_tests_retry.log)。
没有为修复无关依赖而修改 LeRobot 源码；其原有 PolicyServer 修改保持原状。

## 物理脚本基线

使用 `configs/smoke.yaml`，默认布局随机化，seed 0–9，每回合上限 20 秒仿真时间：

| 任务 | 成功回合 |
| --- | --- |
| 红块入盘 | 10 / 10 |
| 蓝块入盘 | 10 / 10 |
| 绿块入盘 | 10 / 10 |
| 黄块入盘 | 10 / 10 |
| 橙块入盘 | 10 / 10 |
| 蓝块叠红块 | 10 / 10 |

合计 **60 / 60**。脚本使用真实物体位置、IK 和夹爪物理接触，回合内不传送物体；仍需
满足同样的抬升、支撑、脱离机械臂和连续 1 秒稳定条件。这验证环境可完成性，不是学习
模型成绩，也不代表任意随机化范围均可完成。

[逐回合原始结果](outputs/oracle_validation_v2/20260916T050335.846945Z_sync_scripted/results.json)。
早期调试输出保留在 `outputs/oracle_validation/`，以上数字仅来自修正后的 `v2`。

```bash
uv run --no-sync so101-bench eval --config configs/smoke.yaml --episodes 10 --seed 0
```

## 真实 π0.5 模型

检查点：`../deploy/checkpoints/full/004000/pretrained_model`，引用已有权重。
验证包括 LeRobot 保存的 pre/postprocessor、图像/状态输入、动作反归一化及仿真闭环。

| 项目 | 本地同步 | 远程实时 seed 0 | 远程实时 seed 1 |
| --- | --- | --- | --- |
| 控制步数 | 150 | 150 | 150 |
| 仿真时间 | 5 s | 5 s | 5 s |
| 回合控制墙钟时间 | 0.695 s | 4.970 s | 4.971 s |
| 回合准备时间 | 4.003 s | 154.421 s | 1.519 s |
| 过期动作丢弃 | 0 | 32 | 30 |
| 断供步数 | 0 | 0 | 0 |
| 控制超时计数 | 0 | 3 | 0 |
| 运行错误 | 无 | 无 | 无 |
| 任务成功 | 否，超时 | 否，超时 | 否，超时 |

这些短回合测试仅验证推理链路，没有证明模型学会叠放。本地同步模式不限制墙钟速度；
远程实时速度比约为 1.006，接近目标 30 Hz。结束步不再额外休眠，因此五秒回合的
墙钟时间略少于五秒。准备时间不计入控制回合，远程首回合包含服务器加载模型。

本地首段推理 2.645 秒，后续两段 0.137 / 0.152 秒；远程第二回合各段约
0.176–0.197 秒。完整耗时数组保存在原始结果中：

- [本地结果](outputs/model_validation/20260916T050428.677360Z_sync_local/results.json)
- [远程结果](outputs/remote_model_validation/20260916T050928.424546Z_realtime_remote/results.json)
- [远程服务日志](outputs/validation/remote_model.log)

远程测试通过 `tools/validate_remote_model.py` 启动绑定随机 loopback 端口的临时
PolicyServer，连续执行两个回合以验证重置与复用，结束后关闭该实例。已有部署服务
未收到 `Ready` 请求。

π0 和 SmolVLA 已提供适配和配置模板，但尚无匹配的 SO-101 检查点，**未做真实权重验证**。

## 显示、录像和遥操作边界

OSMesa 无窗口渲染、EGL 模型运行和 GLFW 三视角窗口均已验证。窗口在临时 Xvfb 上
绘制并截图，包含第三人称、front 和 wrist；[实际截图](docs/three_views.png)。
临时显示服务已关闭。本地模型回合的三路 320×240 视频均确认有 150 帧；自动化另验证
三路视频与 transition 数量一致，以及状态回放输出。

实体 Leader 尚未接入。已验证仿真 Follower 接受六关节动作字典，并产出兼容
LeRobotDataset 的特征和帧字段；这不等同于已经完成实体遥操作采集或实际数据集写入
验证。采集交互与录制 UI 留待后续实现，可复用现有 Robot 和 EpisodeSink 接口。

回放要求保留任务定义与项目版本；已保存 `scene.xml` 供审计，但当前回放从任务定义
重建模型，不保证跨版本回放。
