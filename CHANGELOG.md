# Changelog

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
