# UAV Vision Tracking Control System

这是一个面向无人机视觉跟踪任务的 Python 工程。系统由 YOLO 感知进程、MAVLink 遥测链路、融合层、飞行模式控制层和总控编排层组成，当前主要支持：

- YOLO + ByteTrack 目标检测与主目标选择。
- MAVLink2 遥测接入、状态缓存、动作命令和连续控制命令发送。
- 感知目标与飞控/云台状态融合。
- 斜视接近 `APPROACH_TRACK`。
- 正上方悬停 `OVERHEAD_HOLD`。
- 统一命令限幅、平滑和 dry-run 安全出口。
- curses 终端 UI 与手动控制命令。

## 架构概览

```text
yolo_app/                    telemetry_link/
YOLO + ByteTrack             MAVLink2 + state cache + command sender
        | UDP JSON                    |
        v                             v
fusion/ ------------------------------------------------+
PerceptionTarget + DroneState + GimbalState -> FusedState |
                                                           v
flight_modes/common/input_adapter.py -> FlightModeInput
                                                           v
app/mission_manager.py -> active mode
                                                           v
flight_modes/<mode>/ -> raw FlightCommand
                                                           v
flight_modes/common/command_shaper.py -> shaped FlightCommand
                                                           v
flight_modes/common/executor.py -> telemetry_link.LinkManager
                                                           v
MAVLink control / gimbal commands
```

更详细的边界说明见 [docs/architecture.md](docs/architecture.md)，完整接口见 [docs/interfaces.md](docs/interfaces.md)。

## 目录结构

```text
app/              系统入口、服务编排、任务状态机、健康检查
flight_modes/     飞行模式和通用控制命令出口
fusion/           感知与遥测融合
telemetry_link/   MAVLink2 通讯、状态缓存、命令队列和发送
yolo_app/         YOLO + ByteTrack 感知进程
uav_ui/           终端 UI 与人工命令分发
config/           新架构配置入口
tests/            单元测试
docs/             架构、接口、运行、安全和开发规则
```

## 安装环境

建议使用两个 conda 环境：控制环境和 YOLO 环境。

```bash
conda create -n uav-control python=3.10 -y
conda activate uav-control
pip install pymavlink pyyaml pytest
```

```bash
conda create -n yolo python=3.11 -y
conda activate yolo
pip install ultralytics opencv-python pyyaml
```

如果使用 GPU，请按本机 CUDA 版本安装合适的 PyTorch。模型文件建议放在：

```text
~/models/best.pt
```

大型 `.pt`、日志、缓存文件不建议提交到 Git。

更完整的安装说明见 [docs/install.md](docs/install.md)。

## 快速运行

### 1. 启动 YOLO

```bash
conda activate yolo
cd ~/uav_project/src/yolo_app
python main.py
```

YOLO 默认通过 UDP JSON 输出主目标。控制端默认监听 `0.0.0.0:5005`，两边端口需要一致。

### 2. app dry-run，不连飞控

```bash
conda activate uav-control
cd ~/uav_project/src
python -m app.main --send-commands false
```

### 3. app 连接 telemetry，但不发控制

```bash
python -m app.main --connect-telemetry --send-commands false
```

### 4. app 连接 telemetry 并打开终端 UI

```bash
python -m app.main --connect-telemetry --ui --send-commands false
```

### 5. SITL 中实发控制

只在 SITL 或已确认安全的实机环境中使用：

```bash
python -m app.main --connect-telemetry --force-mode APPROACH_TRACK --send-commands true
```

更完整的运行手册见 [docs/running.md](docs/running.md)。

## 测试

```bash
cd ~/uav_project/src
python -m pytest -q
```

当前测试覆盖 input adapter、command shaper、approach mode、overhead mode、mission manager。

## 安全默认值

- `config/app.yaml` 中 `executor.send_commands` 默认应保持 `false`。
- 不传 `--connect-telemetry` 时，新 app 不连接 MAVLink。
- 不传 `--send-commands true` 时，不应向飞控发送连续控制命令。
- 所有 flight mode 输出必须经过 `CommandShaper` 和 `FlightCommandExecutor`。
- flight mode 禁止直接调用 MAVLink 或 `LinkManager`。

实机前请阅读 [docs/safety.md](docs/safety.md)。

## 文档索引

- [docs/new_session_checklist.md](docs/new_session_checklist.md)：每次开启新 AI 会话前必读。
- [docs/architecture.md](docs/architecture.md)：模块职责、依赖方向和边界。
- [docs/interfaces.md](docs/interfaces.md)：核心 dataclass、接口和单位。
- [docs/ai_development_rules.md](docs/ai_development_rules.md)：给 AI/开发者的修改规则。
- [docs/running.md](docs/running.md)：常用启动方式。
- [docs/install.md](docs/install.md)：环境和依赖安装。
- [docs/configuration.md](docs/configuration.md)：配置文件说明。
- [docs/control_flow.md](docs/control_flow.md)：从 YOLO 到 MAVLink 的数据流。
- [docs/safety.md](docs/safety.md)：安全边界和实机 checklist。
- [docs/sitl_test_plan.md](docs/sitl_test_plan.md)：SITL 测试顺序。
