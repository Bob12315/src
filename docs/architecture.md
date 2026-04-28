# 架构边界

本文定义各模块职责、允许依赖和禁止事项。后续改代码时优先遵守本文，再看具体实现。

## 总体原则

```text
mission_manager 决定“现在做什么”
flight_modes 决定“这件事怎么飞”
command_shaper 决定“命令是否过猛”
executor 决定“怎么交给 telemetry_link”
telemetry_link 决定“怎么发 MAVLink”
```

任何模式输出都必须经过：

```text
FlightMode.update()
  -> FlightCommand raw
  -> CommandShaper.update()
  -> FlightCommand shaped
  -> FlightCommandExecutor.execute()
  -> telemetry_link.LinkManager
```

## app/

职责：

- `main.py`：总入口，只解析参数、加载配置、创建 `SystemRunner`。
- `system_runner.py`：系统主循环，串联服务、融合、任务状态机、飞行模式和执行器。
- `mission_manager.py`：任务状态机，选择 `IDLE`、`APPROACH_TRACK`、`OVERHEAD_HOLD` 等 active mode。
- `service_manager.py`：启动和停止 YOLO UDP 接收、telemetry link、fusion manager。
- `health_monitor.py`：判断 vision/drone/gimbal/fusion/control 健康状态。
- `debug_runtime.py`：强制模式、强制通道开关和 dry-run 调试覆盖。
- `app_config.py`：统一加载 `config/*.yaml`。
- `mode_registry.py`：注册和获取 flight mode 实例。

允许依赖：

- `fusion`
- `telemetry_link`
- `flight_modes`
- `uav_ui`
- `config/*.yaml`

禁止事项：

- 不写具体 PID/P 控制公式。
- 不直接构造 MAVLink message。
- 不直接调用 pymavlink。
- 不绕过 `CommandShaper` 和 `FlightCommandExecutor` 发送控制。

## flight_modes/

职责：

- 定义飞行模式接口。
- 将 `FlightModeInput` 转成 `FlightCommand`。
- 按模式拆分控制逻辑，例如斜视接近、正上方悬停。
- 提供通用输入适配、命令限幅、命令执行出口。

允许依赖：

- `fusion.models.FusedState` 只应被 `common/input_adapter.py` 使用。
- `telemetry_link.models.ControlCommand` 只应被 `common/executor.py` 使用。

禁止事项：

- mode 里不得直接 import `telemetry_link.link_manager`。
- mode 里不得直接发送 MAVLink。
- mode 里不得读取 YAML。
- mode 里不得启动线程、socket 或 UI。
- mode 里不得决定全局任务流程跳转，任务跳转属于 `app/mission_manager.py`。

## flight_modes/common/

职责：

- `types.py`：公开 `FlightModeInput`、`FlightCommand`、`FlightModeStatus`。
- `input_adapter.py`：`FusedState -> FlightModeInput`。
- `command_shaper.py`：统一限幅、slew rate、disabled 通道归零。
- `executor.py`：将 shaped `FlightCommand` 交给 `telemetry_link`。
- `debug_config.py`：flight mode 层调试配置。

禁止事项：

- `command_shaper.py` 不应知道具体 mission 阶段。
- `executor.py` 不应计算控制律。
- `input_adapter.py` 不应发送命令。

## fusion/

职责：

- 接收 YOLO 主目标、DroneState、GimbalState。
- 输出统一 `FusedState`。
- 计算相机误差、机体系误差、数据有效性和控制允许状态。

允许依赖：

- `telemetry_link.models`
- `yolo_app` 输出协议对应的数据结构，当前通过 `PerceptionTarget` 表达。

禁止事项：

- 不决定任务阶段。
- 不生成控制命令。
- 不发送 MAVLink。
- 不启动 YOLO 或 telemetry 服务。

## telemetry_link/

职责：

- 建立 MAVLink2 连接。
- 维护 `DroneState`、`GimbalState`、`LinkStatus`。
- 管理连续控制命令、云台速率命令和 action 命令队列。
- 封装 ArduPilot/MAVLink 发送细节。
- 断线自动重连。

允许依赖：

- `pymavlink`
- Python 标准库
- `uav_ui` 仅限独立 `telemetry_link.main` 的终端 UI。

禁止事项：

- 不读取 YOLO UDP。
- 不计算目标跟踪控制律。
- 不依赖 `flight_modes` 或 `app`。
- 不决定 `APPROACH_TRACK` / `OVERHEAD_HOLD`。

## yolo_app/

职责：

- 读取视频源。
- 调用 Ultralytics YOLO 官方 tracking。
- 使用 ByteTrack。
- 维护主目标。
- 通过 UDP JSON 输出当前主目标。
- 接收目标选择相关的简单命令。

禁止事项：

- 不连接 MAVLink。
- 不读取 telemetry 状态。
- 不生成飞控速度或云台控制命令。

## uav_ui/

职责：

- 终端状态展示。
- 人工命令输入。
- 将人工命令分发给 `telemetry_link` 或 YOLO command client。

禁止事项：

- 不直接解析 YOLO 图像。
- 不直接计算 flight mode 控制律。
- 不绕过 `telemetry_link` 发送 MAVLink。

## config/

职责：

- `app.yaml`：运行时、mission、executor。
- `flight_modes.yaml`：input adapter、各控制器参数、shaper。
- `telemetry.yaml`：MAVLink 连接和消息频率。
- `debug.yaml`：强制模式和通道覆盖。

禁止事项：

- 不把大模型路径、视频源和 telemetry 混在一个没有边界的配置里。
- bool 值必须写 `true/false`，不要写字符串或拼错。

## tests/

职责：

- 覆盖纯逻辑和接口契约。
- 优先测试 input adapter、mission manager、flight mode、command shaper。

禁止事项：

- 单元测试不应要求真实飞控。
- 单元测试不应要求 GPU、相机或 YOLO 模型。
