# Changelog

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
