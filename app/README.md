# app

`app/` 是系统编排层，不写具体控制算法，也不直接构造 MAVLink 消息。

## 职责

- 加载总配置。
- 启动/停止服务。
- 调用 fusion。
- 适配 `FusedState -> FlightModeInput`。
- 运行 health monitor。
- 运行 mission manager。
- 调用 active flight mode。
- 调用 command shaper 和 executor。
- 挂接 UI。

## 主要文件

- `main.py`：入口。
- `system_runner.py`：主循环。
- `service_manager.py`：YOLO UDP、telemetry、fusion 服务管理。
- `mission_manager.py`：任务状态机。
- `health_monitor.py`：数据健康状态。
- `app_config.py`：配置加载。
- `debug_runtime.py`：强制模式和通道覆盖。
- `mode_registry.py`：flight mode 注册。

## 禁止事项

- 不写控制公式。
- 不直接 import pymavlink。
- 不绕过 `FlightCommandExecutor`。
- 不在 mission manager 里计算速度。
