# flight_modes

`flight_modes/` 是飞行模式控制层。它接收 `FlightModeInput`，输出 `FlightCommand`。

## 职责

- 实现不同飞行模式。
- 维护模式内部小状态。
- 计算 raw `FlightCommand`。
- 通过 common 层统一限幅、平滑和执行。

## 目录

- `base_mode.py`：统一接口。
- `common/types.py`：公开输入、命令、状态 dataclass。
- `common/input_adapter.py`：`FusedState -> FlightModeInput`。
- `common/command_shaper.py`：限幅和平滑。
- `common/executor.py`：交给 `telemetry_link` 发送。
- `approach_track/`：斜视接近。
- `overhead_hold/`：正上方悬停。
- `corridor_follow/`：航道巡航占位/扩展。

## 禁止事项

- flight mode 不得直接调用 MAVLink。
- flight mode 不得读取 YAML。
- flight mode 不得启动线程、socket 或 UI。
- flight mode 不决定全局任务阶段。

## 新增模式

1. 新建 `flight_modes/<mode>/mode.py` 和 `config.py`。
2. 实现 `FlightMode` 接口。
3. 在 `app/mode_registry.py` 注册。
4. 在 `app/mission_manager.py` 添加任务切换条件。
5. 添加测试。
