# 验证记录

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
