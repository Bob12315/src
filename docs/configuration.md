# 配置说明

新架构默认读取 `config/` 下的配置。旧 `control/config.yaml` 保留用于旧入口兼容。

## config/app.yaml

进程运行、服务开关和 executor 安全出口。

```yaml
runtime:
  yolo_udp_ip: "0.0.0.0"
  yolo_udp_port: 5005
  loop_hz: 20.0
  perception_timeout_sec: 1.0
  print_rate_hz: 2.0
  require_gimbal_feedback: true
  run_seconds: null
  log_level: INFO

services:
  ui_enabled: false
  connect_telemetry: false
  start_yolo_udp: true

executor:
  send_commands: false
```

- `connect_telemetry`：默认 false；命令行 `--connect-telemetry` 可打开。
- `ui_enabled`：默认 false；命令行 `--ui` 可打开。
- `start_yolo_udp`：是否监听 YOLO UDP。
- `send_commands`：默认必须为 false；实发时必须显式打开。
- `run_seconds`：自动退出秒数，适合 smoke test。

## config/mission.yaml

任务状态机、模式切换条件和正常恢复策略。

```yaml
initial_mode: "APPROACH_TRACK"
auto_switch_enabled: true

freshness:
  max_vision_age_s: 0.3
  max_drone_age_s: 0.3
  max_gimbal_age_s: 0.3

transitions:
  approach_track_to_overhead_hold:
    target_size_thresh: 10.0
    gimbal_pitch_rad: -1.5707963267948966
    gimbal_pitch_tol_rad: 0.20
    gimbal_yaw_tol_rad: 0.15
    hold_s: 0.5
  overhead_hold_to_approach_track:
    target_size_drop: 0.06

recovery:
  lost_target:
    recenter_gimbal_enabled: true
    recenter_after_s: 10.0
    recenter_pitch_deg: 0.0
    recenter_yaw_deg: 0.0
```

- `initial_mode`：任务状态机启动后的默认模式。
- `auto_switch_enabled`：是否允许自动切换模式。
- `freshness`：各数据源最大允许年龄。
- `transitions`：模式间切换阈值。
- `recovery.lost_target`：丢目标后的正常恢复动作。

## config/flight_modes.yaml

飞行模式和通用控制参数。

主要分区：

- `input_adapter`：dt、age、低通滤波、target stable。
- `approach_track.gates`：斜视接近各控制通道放行条件。
- `approach_track.gimbal`：斜视接近云台控制参数。
- `approach_track.body`：斜视接近横移和机体偏航参数。
- `approach_track.forward`：斜视接近前向速度参数。
- `overhead_hold.gates`：正上方悬停各控制通道放行条件。
- `overhead_hold.gimbal`：正上方悬停云台角度目标。
- `overhead_hold.lateral`：正上方悬停横向平移参数。
- `overhead_hold.longitudinal`：正上方悬停前后平移参数。
- `shaper`：最终命令限幅和 slew rate。

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

- SITL 端口需要和实际 `sim_vehicle.py` 输出一致。
- 实机串口和波特率需要按硬件修改。
- `control_send_rate_hz` 控制连续命令最高发送频率。
- `request_message_intervals` 为 true 时会请求常用 MAVLink 消息频率。

## yolo_app/config.yaml

YOLO 感知配置，包含模型路径、视频源、UDP 输出目标、目标选择策略、显示和保存选项。

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
