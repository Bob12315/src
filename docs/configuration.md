# 配置说明

新架构默认读取 `config/` 下的配置。旧 `control/config.yaml` 保留用于旧入口兼容。

## config/app.yaml

运行时、mission 和 executor 配置。

重要项：

```yaml
runtime:
  yolo_udp_ip: "0.0.0.0"
  yolo_udp_port: 5005
  loop_hz: 20.0
  perception_timeout_sec: 1.0
  ui_enabled: false
  connect_telemetry: false
  start_yolo_udp: true
  start_send_commands: false

executor:
  send_commands: false
```

说明：

- `connect_telemetry`：默认 false；命令行 `--connect-telemetry` 可打开。
- `start_yolo_udp`：是否监听 YOLO UDP。
- `send_commands`：默认必须为 false；实发时必须显式打开。
- `run_seconds`：自动退出秒数，适合 smoke test。

mission 相关：

- `initial_mode`：初始任务阶段。
- `overhead_entry_*`：进入正上方悬停的条件。
- `overhead_exit_target_size_drop`：退出 overhead 的条件。
- `require_*`：各通道放行条件。

## config/flight_modes.yaml

飞行模式和通用控制参数。

主要分区：

- `input_adapter`：dt、age、低通滤波、target stable。
- `gimbal`：云台控制参数。
- `body`：机体横移和偏航参数。
- `approach`：前向接近参数。
- `shaper`：最终命令限幅和 slew rate。

实机初期建议降低：

```yaml
shaper:
  max_vx: 0.3
  max_vy: 0.3
  max_yaw_rate: 0.3
  max_gimbal_yaw_rate: 0.4
  max_gimbal_pitch_rate: 0.4
```

## config/telemetry.yaml

MAVLink 连接、消息频率、超时和 UI 配置。

常用项：

```yaml
data_source: sitl
active_source: sitl

sitl:
  connection_type: tcp
  tcp_host: 127.0.0.1
  tcp_port: 5762

real:
  connection_type: serial
  serial_port: /dev/ttyUSB0
  baudrate: 57600
```

说明：

- SITL 端口需要和实际 `sim_vehicle.py` 输出一致。
- 实机串口和波特率需要按硬件修改。
- `control_send_rate_hz` 控制连续命令最高发送频率。
- `request_message_intervals` 为 true 时会请求常用 MAVLink 消息频率。

## config/debug.yaml

调试覆盖：

```yaml
force_mode: null
enable_gimbal: null
enable_body: null
enable_approach: null
dry_run: true
```

说明：

- `force_mode` 可强制 active mode，例如 `APPROACH_TRACK`。
- `enable_*` 为 null 表示不覆盖，为 true/false 表示强制开关。
- `dry_run` 是调试语义，实际是否发送仍以 executor/send_commands 为准。

## yolo_app/config.yaml

YOLO 感知配置，包含：

- 模型路径。
- 视频源。
- UDP 输出目标。
- 目标选择策略。
- 显示和保存选项。

注意保持 UDP 端口与 `config/app.yaml` 一致。

## bool 配置规则

必须使用 YAML 原生 bool：

```yaml
true
false
```

不要写：

```yaml
"true"
"false"
ture
```

新 loader 对错误 bool 应明确报错，避免实机时误解配置。
