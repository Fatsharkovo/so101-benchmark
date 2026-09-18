# Changelog

## 2026-09-18 — 共享场景与任务大类

- 五个入盘 XML 合并为 place_in_plate.xml；叠块采用 stack_cubes.xml 双槽模板，颜色
  按 ID 字母序绑定，反向任务在同 seed 下保持同一布局。几何默认值统一从 XML 读取。
- 两份大类 YAML 集中共享场景、判定参数和指令模板，子任务只声明差异。新增橙放绿、
  红放蓝、绿放黄、黄放蓝，与蓝放红组成五项叠块任务；all 现在包含十项任务。
- 通用 StackCubes 取代蓝红专用场景构造，保留 StackBlueOnRed 别名、旧任务 ID、
  旧格式外部 YAML 与项目加载器对旧内置 XML 名称的兼容。计分初始化移到 reset。
- list 新增 --group-by-family；采集元数据包含大类与展开后的任务定义，历史完整 XML
  回放不会重新绑定模板。Python 自定义场景复用 common.xml，删除重复环境默认值。
- 保留当前随机范围、盘子物理参数、成功判定阈值、相机、运动插值和阴影修复。

## 2026-09-18 — 修复阴影边界截断

- 共享 XML 和 Python 场景的 `visual/map.shadowscale` 设为 1.1，将阴影视锥从默认
  54° 扩至 99°，覆盖 90° 主灯照射范围，消除机械臂伸向边缘时影子的直线截断。
  保留灯光位置、方向、亮度及物理参数。

## 2026-09-18 — 可移动盘子与遥操作随机布局

- 五个入盘 XML 场景及 Python 场景统一为盘沿高出盘底 10 mm、总高 12 mm，保留 10 cm
  浅蓝色圆角外形和 2 mm 底厚/壁厚。增加自由关节，盘子质量暂设 50 g，可由接触推拖。
- 盘底使用圆角外形的基本几何组合提供接触，避免移动薄网格与桌面接触时的数值抖动；
  摩擦保持 0.8，第二舵机外壳零接触摩擦设置不变。
- 遥操作布局开启每轴 ±15 mm、方块偏航 ±8° 随机化，保留可达半径及间距拒绝采样。
  成功回合后重新采样，R/Space 仍恢复本回合初始状态（包括盘子）。
- 入盘判定放宽为稳定 0.5 秒、线速度 <2 cm/s、角速度 <0.3 rad/s、边缘容差 2 mm；
  包含盘子速度，但允许机械臂轻触盘沿，仍要求目标松开且受盘底支撑。盘子掉落报告失败。
  脚本控制器规划时读取盘子当前位姿。历史 XML 中无自由关节的盘子仍按原来的固定方式加载。

## 2026-09-17 — 遥操作运动目标平滑

- 新增 `sim.interpolate_actions`，在每个控制周期内按物理步线性过渡关节目标，减少运动
  阶跃引发的前臂/腕部振荡；使用已有 ctrl 作为起点，重置不会沿用旧回合目标。
- 遥操作默认改用 `configs/teleop.yaml` 并开启插值；其他评测配置默认保持原控制方式。
- 保留舵机刚度、阻尼和力矩上限，保留第二舵机外壳及安装座零接触摩擦。记录的 action
  为周期最终控制目标，元数据包含插值设置。

## 木纹方向与机械臂白度微调

- 桌面木纹顺时针旋转 90°，保留原始贴图供历史场景回放。
- 打印件与支架由 RGB 0.82 提亮到 0.92，保留轻微灰感及柔和高光。

## 2026-09-17 — 木纹桌面与灰白机械臂

- 六场景桌面采用 ambientCG Wood049 的 CC0 橡木颜色贴图，随包提供，运行时无需联网。
- 打印件、夹爪及腕部支架改为柔和灰白色，降低高光，保留深色舵机及方块语义颜色。
- XML 与 Python 场景同步，支持纹理资源路径解析和独立 wheel 加载；物理参数不变，
  第二舵机外壳及安装座继续保持零接触摩擦。

## 2026-09-17 — 矽递教程版本腕部相机支架

- 将腕部支架替换为官方教程链接 STEP 中的 SO-ARM101_CAMERA_MOUNT，保留机械臂主体、
  硬夹爪及腕部零位映射；原支架资产保留供历史场景回放使用。
- 新增共享的 `task_scenes/wrist_camera.xml`，同步可视网格、分段碰撞体及 wrist 相机。
  按安装面调整光轴，镜头位置采用明确记录的估算，fovy 保持 65°。
- 保留第二舵机外壳/安装座零接触摩擦，front 参数及本机配置保持原值。
- 记录独立来源、转换参数、光心/质量假设和可重建 STL 的转换工具。

## Front 调参预览夹爪闭合

- 相机调参窗口使用闭合夹爪作为预览姿态，便于对齐实物；遥操作及任务默认姿态保持原值。

## 2026-09-17 — 独立 Front 相机调参窗口

- 新增 `so101-bench camera-tune`，提供保持宽高比的实时仿真预览、七项滑条与精确数值输入，
  无需连接硬件，调参不重建模型、不推进机械臂物理状态。
- 个人配置保存到 XDG 配置目录，可重新打开、继续修改、反复覆盖保存；提供恢复上次保存、
  恢复场景默认、未保存关闭提示，原子保存失败时保留旧文件。
- 遥操作启动自动读取本机 front 参数，固定到本次会话，重置保持一致；wrist 不受影响。
- 统一 XML/Python 场景相机覆盖逻辑，支持四元数和每相机 fixed 随机化开关；有效配置与
  场景快照记录实际使用的位置、完整朝向与 fovy。评测和回放不隐式使用个人配置。

## 相机视场角调试更新

- 按最新调试要求，front fovy 改为 45°，wrist 恢复原值 65°；位置和下俯角保持不变。

## 2026-09-17 — 对齐商家提供的相机视场角

- front 的默认垂直视场角 fovy 改为 86°，wrist 改为 90°；按用户提供的商家参数设置。
- 同步 XML、Python 自定义场景默认值及姿态回归测试，保持相机位置和朝向不变。
- 主窗口 front 视野框自动使用新的有效 fovy。

## 2026-09-17 — 相机调试显示与第二舵机外壳接触

- 遥操作主窗口显示 front 相机的示意外壳、光心、光轴和视野框，跟随实际相机姿态；
  不参与物理碰撞，不影响采集图像，可用 `sim.cameras.front.show_pose` 关闭。
- 第二舵机外壳及安装座（shoulder）的两个碰撞体设为无摩擦接触；使用 condim=1、
  priority=2 保证与普通物体及夹爪接触时不被另一侧摩擦覆盖。保留长臂、关节摩擦和阻尼。
- 补充相机 XML 下俯角与 fovy 的区别、外壳位置映射及碰撞边界说明。

## 2026-09-17 — front 相机距离调整

- front 相机水平距离由 30 cm 改为 45 cm，保持高度 35 cm、下俯 60°、垂直视场角 60°。
- 同步更新 XML 默认值、Python 自定义场景默认值和相机姿态回归测试。

## 2026-09-16 — 遥操作采集与 XML 场景

- 初次进入及重置后等待 Space；等待时允许试操作，Space 恢复完整初始场景后开始计时采集。
- 保留三个 GLFW 窗口和用户主视角；成功自动保存并重置，失败、超时和手动中断丢弃回合。
- 在主窗口原顶部信息行追加 `Saved episodes: N` 和 `Space: start recording`；
  仅在写入进程确认保存后计数，重置不清零。
- 独立 LeRobot 环境以官方 v3.0 格式保存成功回合的状态、实际动作、front/wrist 视频；
  队列满或写入失败时显示错误并停止采集。退出完成已保存回合的写入收尾。
- 遥操作接入现有 YAML；`sim.episode_seconds` 控制单回合，`--seconds` 仅限制整个会话。
- 六任务运行时直接加载根目录 `task_scenes` XML；保留 Python 自定义任务兼容，
  从有效 XML 提取评分尺寸，回放优先读取采集时的场景快照。
- front 相机更新为基座前方 30 cm、高 35 cm、向下 60°；保留 wrist 映射和 10 g 方块。

## Unreleased

### Added

- Physical SO-101 Leader to simulator teleoperation through `tools/teleoperate.py`, with separate
  overview/front/wrist windows, reset and exit controls, a session time limit, and local debug summaries.
  The Leader reader can use a separate Python environment with existing LeRobot/Feetech dependencies.
- Configurable rounded-square or circular plates, including base/wall thickness and rim height.
- Regression coverage for wrist mapping, camera pose, cube mass, and rounded-square containment.

### Changed

- Default wrist-roll mapping now uses `sim_angle = leader_angle - 90°`; observations use the inverse
  conversion. Existing checkpoints must use matching joint calibration settings.
- Front camera defaults to 35 cm in front of the base and 30 cm above the table, pitched down 45°
  with a 60° vertical field of view.
- All six tasks use brighter lighting, a gradient sky, infinite visual ground, and compact layouts.
  Layout jitter is now ±5 mm per axis and ±5° yaw; stacking cubes start 9 cm apart.
- Cubes weigh 10 g. The default plate is light blue with a 100 mm outer side, 12 mm corner radius,
  2 mm base/walls, and a rim 6 mm above the base. Collision geometry, containment checks, and scripted
  placement height follow these dimensions. Old benchmark scores are not directly comparable.

### Fixed

- Visible GLFW windows explicitly override the hidden-window hint left by MuJoCo offscreen rendering.

### Validation and known limitations

- Automated tests: 35 passed, 3 skipped (optional LeRobot/gRPC dependencies); Ruff checks passed.
- Scripted baseline: 59/60 successes across six tasks and seeds 0–9. Yellow placement seed 7 loses
  the grasp and produces an unreachable IK correction. Learned policies were not re-evaluated.
- Physical Leader to simulation: three visible windows and one successful stacking episode observed.
  Dataset recording UI remains unimplemented; this tool does not control a physical Follower.
