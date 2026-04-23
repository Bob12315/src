# Telemetry Link

本目录实现的是一个独立运行的飞控数传通讯适配层，不是 ROS2 节点，也不依赖 YOLO、控制器或飞控参数配置界面。

它的定位是一个基础设施层服务模块，负责：

- 建立与飞控的 MAVLink2 通信
- 接收并解析飞控状态流
- 接收并解析云台反馈状态
- 缓存最新飞控状态与云台状态
- 统一发送控制命令和一次性动作命令
- 对收发进行线程隔离、统一调度和限频

它不负责感知融合。`perception + telemetry_link -> FusedState` 的组合逻辑应放在独立的 `fusion/` 层。

当前版本已支持多数据源：

- `real`
- `sitl`
- `dual`

但无论连接几路，对外始终只暴露一个 `active_source`，并且控制命令只发给当前 `active_source`。

## 1. 这不是 ROS2 节点

请明确：

- 这个模块不是 `ros2 run ...`
- 不使用 `rclpy`
- 不发布或订阅 ROS2 topic
- 它应该直接在终端中运行

这是一个独立进程内的链路服务模块，目标是给上层提供两个基础能力：

- 最新飞控状态
- 最新云台反馈状态
- 统一命令发送接口

## 2. 模块职责

整体数据流如下：

```text
real source (Serial / UDP / TCP)
-> pymavlink
-> TelemetryReceiver
-> real StateCache

sitl source (Serial / UDP / TCP)
-> pymavlink
-> TelemetryReceiver
-> sitl StateCache

Action / Control Commands
-> active source selector
-> active source CommandQueue
-> active source CommandSender
-> active source Flight Controller
```

目录结构：

```text
telemetry_link/
  ├── main.py
  ├── config.yaml
  ├── config.py
  ├── link_manager.py
  ├── telemetry_receiver.py
  ├── command_sender.py
  ├── state_cache.py
  ├── command_queue.py
  ├── mavlink_client.py
  ├── models.py
  ├── rate_controller.py
  ├── utils.py
  └── README.md
```

## 3. 线程模型

线程职责被严格分离：

- 主线程：启动服务、打印状态日志、处理退出信号
- 每个 source 各自拥有接收线程 `TelemetryReceiver`
- 每个 source 各自拥有发送线程 `CommandSender`
- 每个 source 各自拥有独立状态缓存
- 主线程只对外暴露当前 `active_source` 对应的状态

关键约束：

- 只有 `LinkManager` 持有链路对象的生命周期控制权
- 外部模块不能直接访问 `pymavlink` 连接对象
- 接收线程不发命令
- 发送线程不做复杂状态计算
- 连续控制命令不排长队，只保留最新一条
- 即使 `data_source=dual`，控制命令也只会发给 `active_source`

## 4. ArduPilot 与 MAVLink2 说明

本模块默认面向 ArduPilot，并假设飞控数传口使用 MAVLink2。

### 4.1 串口协议

在 ArduPilot 中：

- `SERIALx_PROTOCOL = 2` 表示该串口使用 MAVLink2
- `SERIALx_BAUD` 控制该串口波特率

因此，电脑侧配置的波特率必须与飞控对应串口的 `SERIALx_BAUD` 匹配。

### 4.2 遥测流默认行为

ArduPilot 在启用 MAVLink 的链路上，通常会主动持续发送遥测消息，而不是必须等电脑逐条请求才发。

这意味着：

- 电脑侧接收线程应该持续读消息
- 不应假设所有状态都需要通过 request 才能拿到

### 4.3 流速控制

ArduPilot 的 MAVLink 消息流速可以通过飞控参数进行控制：

- 常见是 `MAVx_*` 或 `SRx_*` 相关参数
- 这些参数控制某条 MAVLink 链路上不同类型消息的发送速率

需要特别注意：

- `MAVx_* / SRx_*` 对应的是“第几个启用 MAVLink 的端口”
- 它们不是简单按 `SERIALx` 数字做机械一一对应
- 实际对应关系要结合当前启用的 MAVLink 端口来确认

还要提醒：

- Mission Planner、QGroundControl 等地面站有可能覆盖流速设置
- 因此你在飞控侧配置好的流速，接上其他地面站后可能被改掉

### 4.4 电脑侧请求消息频率

虽然飞控通常会主动发送遥测流，但电脑侧仍然可以使用：

- `MAV_CMD_SET_MESSAGE_INTERVAL`

来请求某一类消息的发送频率。

本模块已支持把这类请求作为一次性动作命令统一发送。

## 5. MVP 功能

当前版本已实现：

- 支持 `data_source=real / sitl / dual`
- 支持 `active_source=real / sitl`
- 支持串口连接
- 支持 UDP 连接
- 支持 TCP 连接，推荐用于 ArduPilot SITL
- `wait_heartbeat()` 获取 `target_system / target_component`
- 接收并解析常用状态消息
- 线程安全状态缓存
- 连续控制命令与一次性动作命令分流
- 统一发送线程
- 控制命令限频
- 心跳超时判断
- 运行时切换 `active_source`

已解析的消息包括：

- `HEARTBEAT`
- `ATTITUDE`
- `GLOBAL_POSITION_INT`
- `LOCAL_POSITION_NED`
- `VFR_HUD`
- `SYS_STATUS`
- `GPS_RAW_INT`
- `RC_CHANNELS`
- `MOUNT_STATUS`
- `GIMBAL_DEVICE_ATTITUDE_STATUS`

## 6. 数据结构

### 6.1 `DroneState`

至少包含：

- `timestamp`
- `connected`
- `stale`
- `mode`
- `armed`
- `control_allowed`
- `roll`
- `pitch`
- `yaw`
- `roll_rate`
- `pitch_rate`
- `yaw_rate`
- `vx`
- `vy`
- `vz`
- `velocity_valid`
- `velocity_source`
- `velocity_quality`
- `attitude_valid`
- `altitude_valid`
- `battery_valid`
- `relative_alt_valid`
- `local_position_valid`
- `altitude`
- `relative_altitude`
- `lat`
- `lon`
- `global_position_valid`
- `battery_voltage`
- `battery_remaining`
- `gps_fix_type`
- `satellites_visible`

当前约束：

- 速度只来自 `LOCAL_POSITION_NED.vx/vy/vz`
- `velocity_source` 固定为 `ekf`
- `velocity_quality` 根据 `gps_fix_type` 推导
- 断链或 `stale=true` 时，可以保留最后值用于日志，但对应 `*_valid` 必须为 `false`

### 6.2 `GimbalState`

包含：

- `timestamp`
- `gimbal_valid`
- `yaw`
- `pitch`
- `roll`
- `source_msg_type`
- `last_update_time`

重要约束：

- 云台姿态必须来自 MAVLink 实际反馈
- 不能把发送出去的云台命令值当成当前云台角度
- 若同时有 `MOUNT_STATUS` 和 `GIMBAL_DEVICE_ATTITUDE_STATUS`，优先新版反馈

### 6.3 `LinkStatus`

包含：

- `connected`
- `reconnecting`
- `last_rx_time`
- `last_tx_time`
- `last_heartbeat_time`
- `heartbeat_timeout_sec`
- `rx_timeout_sec`
- `target_system`
- `target_component`

## 7. 配置说明

配置文件位于 [config.yaml](/home/level6/uav_project/src/telemetry_link/config.yaml)。

当前使用多数据源结构：

```yaml
data_source: real   # real / sitl / dual
active_source: real # real / sitl

sitl:
  connection_type: tcp
  tcp_host: 127.0.0.1
  tcp_port: 5760

real:
  connection_type: serial
  serial_port: /dev/ttyUSB0
  baudrate: 115200
```

核心参数：

- `data_source`：`real`、`sitl` 或 `dual`
- `active_source`：`real` 或 `sitl`
- `real.connection_type`：`serial`、`udp` 或 `tcp`
- `sitl.connection_type`：`serial`、`udp` 或 `tcp`
- `real.serial_port` / `real.baudrate`
- `sitl.tcp_host` / `sitl.tcp_port`
- `control_send_rate_hz`：连续控制命令发送频率
- `action_cmd_retries`：动作命令失败后重试次数
- `heartbeat_timeout_sec`：心跳超时判定阈值
- `reconnect_interval_sec`：重连间隔
- `request_message_intervals`：是否启动时主动请求常用消息频率
- `state_udp_enabled`：是否向本地 UDP 广播最新状态
- `state_udp_ip` / `state_udp_port`：广播给 fusion 等上层进程的状态地址

推荐双源结构示例：

```yaml
data_source: dual
active_source: real

sitl:
  connection_type: tcp
  tcp_host: 127.0.0.1
  tcp_port: 5762

real:
  connection_type: serial
  serial_port: /dev/ttyUSB0
  baudrate: 115200
```

## 8. 运行方式

### 8.1 安装依赖

```bash
pip install pymavlink pyyaml
```

### 8.2 串口连接真实飞控

```bash
cd /home/level6/uav_project/src/telemetry_link
python3 main.py \
  --data-source real \
  --active-source real \
  --real-connection-type serial \
  --real-serial-port /dev/ttyUSB0 \
  --real-baudrate 115200
```

### 8.3 使用 UDP 连接 SITL

```bash
python3 main.py \
  --data-source sitl \
  --active-source sitl \
  --sitl-connection-type udp \
  --sitl-udp-mode udpin \
  --sitl-udp-host 0.0.0.0 \
  --sitl-udp-port 14550
```

### 8.4 使用 TCP 连接 ArduPilot SITL

```bash
python3 main.py \
  --data-source sitl \
  --active-source sitl \
  --sitl-connection-type tcp \
  --sitl-tcp-host 127.0.0.1 \
  --sitl-tcp-port 5760
```

### 8.5 同时连接 SITL 和真实飞控

```bash
python3 main.py \
  --data-source dual \
  --active-source real \
  --real-connection-type serial \
  --real-serial-port /dev/ttyUSB0 \
  --real-baudrate 115200 \
  --sitl-connection-type tcp \
  --sitl-tcp-host 127.0.0.1 \
  --sitl-tcp-port 5762
```

此时：

- `real` 和 `sitl` 都会建立独立链路
- 对外只暴露 `active_source` 对应的 `DroneState`
- 控制命令也只发给 `active_source`
- 不做数据融合
- 不会同时给两个飞控发控制命令

## 9. SITL / Gazebo 使用方式

当前仿真环境如果是：

- ArduPilot SITL
- Gazebo

并且 SITL 日志中出现类似：

```text
SERIAL0 on TCP port 5760
SERIAL1 on TCP port 5762
SERIAL2 on TCP port 5763
```

那么这些才是 telemetry_link 应连接的 MAVLink 端口。

### 9.1 推荐方式 A：SITL 不带 MAVProxy

先启动 SITL：

```bash
sim_vehicle.py -v ArduCopter --no-mavproxy
```

然后启动 telemetry_link：

```bash
python3 main.py \
  --data-source sitl \
  --active-source sitl \
  --sitl-connection-type tcp \
  --sitl-tcp-host 127.0.0.1 \
  --sitl-tcp-port 5760
```

### 9.2 方式 B：保留 MAVProxy

如果 `sim_vehicle.py` 默认带 MAVProxy，那么 `5760` 可能已被占用。

这时可以尝试：

- `5762`
- `5763`

例如：

```bash
python3 main.py \
  --data-source sitl \
  --active-source sitl \
  --sitl-connection-type tcp \
  --sitl-tcp-host 127.0.0.1 \
  --sitl-tcp-port 5762
```

但前提是：

- 该端口对应的 `SERIALx_PROTOCOL` 配置为 MAVLink
- 推荐使用 `SERIALx_PROTOCOL = 2`，即 MAVLink2

### 9.3 重要提醒：不要连接 Gazebo 的 JSON 端口

Gazebo 可能还会暴露自己的控制接口，例如：

- `9002`

这个端口通常不是 MAVLink 端口。

telemetry_link 不应该连接：

- `9002`

telemetry_link 连接的应当是：

- SITL 的 MAVLink TCP 端口，例如 `5760 / 5762 / 5763`

## 10. 对外接口说明

统一入口是 `LinkManager`：

- `get_latest_drone_state()`
- `get_latest_gimbal_state()`
- `get_latest_state()`，兼容旧接口，等价于 `get_latest_drone_state()`
- `get_link_status()`
- `get_source_state("real" | "sitl")`
- `get_source_gimbal_state("real" | "sitl")`
- `get_source_link_status("real" | "sitl")`
- `get_active_source()`
- `switch_active_source("real" | "sitl")`
- `submit_control_command(...)`
- `submit_action_command(...)`
- `set_mode(...)`
- `arm()`
- `disarm()`
- `send_gimbal_angle(...)`
- `send_gimbal_rate(...)`
- `send_velocity_command(...)`
- `send_yaw_rate_command(...)`
- `stop_control()`

运行时切换 `active_source` 也已支持，当前版本通过标准输入命令完成：

```text
switch_source real
switch_source sitl
mode GUIDED
mode STABILIZE
arm
disarm
gimbal -10 15
gimbal -10 15 0
gimbal_rate -5 10
gimbal_rate 0 15 lock
```

例如程序运行中直接在终端输入：

```text
switch_source sitl
```

切换时：

- 不重启进程
- 不做数据融合
- 只切换当前对外暴露状态与命令发送目标

### 10.1 终端发送命令

当前版本支持直接在 `telemetry_link` 运行终端里输入命令。

支持的命令如下：

- `switch_source real`
- `switch_source sitl`
- `mode <MODE_NAME>`
- `arm`
- `disarm`
- `takeoff <altitude_m>`
- `land`
- `body_vel <forward_mps> <right_mps> <down_mps>`
- `yaw_rate <rad_per_sec>`
- `stop`
- `gimbal <pitch_deg> <yaw_deg> [roll_deg]`
- `gimbal_rate <pitch_rate_deg_s> <yaw_rate_deg_s> [follow|lock]`

完整示例如下：

```text
switch_source real
switch_source sitl

mode GUIDED
mode STABILIZE
mode LOITER
mode ALT_HOLD
mode LAND
mode RTL

arm
disarm
takeoff 3
takeoff 5
land

body_vel 1 0 0
body_vel -1 0 0
body_vel 0 -1 0
body_vel 0 1 0
body_vel 0 0 -0.5
body_vel 0 0 0.5
yaw_rate 0.3
yaw_rate -0.3
stop

gimbal 0 0
gimbal -10 15
gimbal -10 15 0
gimbal 20 0 0
gimbal 0 30 0
gimbal 40 40 40
gimbal -20 -30 0

gimbal_rate -5 0
gimbal_rate 0 8
gimbal_rate -5 8
gimbal_rate 0 8 lock
```

说明：

- `switch_source real`：切换当前 active source 到真机链路
- `switch_source sitl`：切换当前 active source 到 SITL 链路
- `mode GUIDED` 会调用 `LinkManager.set_mode("GUIDED")`
- `mode STABILIZE / LOITER / ALT_HOLD / LAND / RTL` 的行为与飞控当前模式映射一致
- `arm` / `disarm` 会走一次性动作命令队列
- `takeoff <altitude_m>` 会发送 `MAV_CMD_NAV_TAKEOFF`
- `land` 会发送 `MAV_CMD_NAV_LAND`
- `body_vel` 使用机体系 `BODY_NED` 发送速度命令
- `yaw_rate` 使用机体系 `BODY_NED` 发送偏航角速度命令
- `stop` 会发送全零速度和全零偏航角速度
- `gimbal` 命令单位为角度 `deg`
- `gimbal` 的格式是 `pitch yaw [roll]`
- 如果不填 `roll`，默认按 `0.0`
- `gimbal_rate` 命令单位为角速度 `deg/s`
- `gimbal_rate` 的格式是 `pitch_rate yaw_rate [follow|lock]`
- `gimbal_rate` 内部会转换成 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`
- 如果不填 yaw 模式，默认按 `follow`

按命令分类举例：

`switch_source` 示例：

```text
switch_source real
switch_source sitl
```

`mode` 示例：

```text
mode GUIDED
mode STABILIZE
mode LOITER
mode ALT_HOLD
mode LAND
mode RTL
```

`arm / disarm` 示例：

```text
arm
disarm
```

`takeoff / land` 示例：

```text
takeoff 3
takeoff 5
land
```

这些命令可理解为：

- `takeoff 3`：起飞到 3 米
- `takeoff 5`：起飞到 5 米
- `land`：执行降落

`body_vel / yaw_rate / stop` 示例：

```text
body_vel 1 0 0
body_vel -1 0 0
body_vel 0 -1 0
body_vel 0 1 0
body_vel 0 0 -0.5
body_vel 0 0 0.5
yaw_rate 0.3
yaw_rate -0.3
stop
```

按“无人机前后左右上下”理解的运动示例：

```text
body_vel 1 0 0
body_vel -1 0 0
body_vel 0 -1 0
body_vel 0 1 0
body_vel 0 0 -0.5
body_vel 0 0 0.5
stop
```

这些命令可理解为：

- `body_vel 1 0 0`：向前飞
- `body_vel -1 0 0`：向后飞
- `body_vel 0 -1 0`：向左飞
- `body_vel 0 1 0`：向右飞
- `body_vel 0 0 -0.5`：向上飞
- `body_vel 0 0 0.5`：向下飞
- `stop`：停住

方向约定：

- `forward_mps > 0`：前进
- `forward_mps < 0`：后退
- `right_mps > 0`：向右
- `right_mps < 0`：向左
- `down_mps > 0`：向下
- `down_mps < 0`：向上

偏航控制示例：

```text
yaw_rate 0.3
yaw_rate -0.3
stop
```

可理解为：

- `yaw_rate 0.3`：持续向右转
- `yaw_rate -0.3`：持续向左转
- `stop`：停止转动

`gimbal` 示例：

```text
gimbal 0 0
gimbal -10 15
gimbal -10 15 0
gimbal 20 0 0
gimbal 0 30 0
gimbal 40 40 40
gimbal -20 -30 0
```

各个 `gimbal` 示例含义：

- `gimbal 0 0`：将 pitch/yaw 目标置到 0 度，roll 默认 0
- `gimbal -10 15`：pitch=-10 度，yaw=15 度，roll 默认 0
- `gimbal -10 15 0`：显式指定 roll=0
- `gimbal 20 0 0`：只改 pitch
- `gimbal 0 38 0`：只改 yaw
- `gimbal 40 40 40`：pitch/yaw/roll 全部给 40 度
- `gimbal -20 -30 30`：给负角度目标

按“前后左右”理解的云台示例：

```text
gimbal 0 0
gimbal -10 0
gimbal 10 0
gimbal 0 -10
gimbal 0 10
gimbal -10 -10
gimbal -10 10
gimbal 10 -10
gimbal 10 10
```

这些命令可理解为：

- `gimbal 0 0`：回中
- `gimbal -10 0`：向前
- `gimbal 10 0`：向后
- `gimbal 0 -10`：向左
- `gimbal 0 10`：向右
- `gimbal -10 -10`：左前
- `gimbal -10 10`：右前
- `gimbal 10 -10`：左后
- `gimbal 10 10`：右后

角度方向约定：

- `pitch < 0`：向前
- `pitch > 0`：向后
- `yaw < 0`：向左
- `yaw > 0`：向右

`gimbal_rate` 示例：

```text
gimbal_rate -5 0
gimbal_rate 5 0
gimbal_rate 0 -8
gimbal_rate 0 8
gimbal_rate -5 8
gimbal_rate 0 8 lock
gimbal_rate 0 0
```

这些命令可理解为：

- `gimbal_rate -5 0`：云台持续向前俯仰
- `gimbal_rate 5 0`：云台持续向后俯仰
- `gimbal_rate 0 -8`：云台持续向左偏航
- `gimbal_rate 0 8`：云台持续向右偏航，采用 follow 模式
- `gimbal_rate -5 8`：同时俯仰和偏航
- `gimbal_rate 0 8 lock`：云台以 yaw lock 模式持续向右偏航
- `gimbal_rate 0 0`：发送零角速度，停止连续转动

角速度方向约定：

- `pitch_rate < 0`：向前俯仰
- `pitch_rate > 0`：向后俯仰
- `yaw_rate < 0`：向左偏航
- `yaw_rate > 0`：向右偏航
- `follow`：yaw 相对机体，机体转动时云台 yaw 跟随
- `lock`：yaw 相对地理系锁定，机体转动时云台 yaw 尽量保持不变

调试时建议观察两类日志：

- 入队日志，例如 `mode command queued mode=GUIDED`
- 发送日志，例如 `sending action command=set_mode mode=GUIDED`

如果只看到入队日志，没有看到发送日志，通常说明链路还没连上或旧进程还没重启。

### 10.2 云台命令发送

当前版本已支持通过 `LinkManager` 发送云台角度命令：

```python
manager.send_gimbal_angle(
    pitch=-10.0,
    yaw=15.0,
    roll=0.0,
)
```

说明：

- 命令单位为角度 `deg`
- 当前实现与 `uav_bridge_ap` 一致，走 `MAV_CMD_DO_MOUNT_CONTROL`
- 默认 `mount_mode=2`，即 `MAV_MOUNT_MODE_MAVLINK_TARGETING`
- 这是“命令”，不是“反馈”
- 当前云台实际姿态仍应以 `MOUNT_STATUS` 或 `GIMBAL_DEVICE_ATTITUDE_STATUS` 为准

当前版本也支持通过 `LinkManager` 发送云台角速度命令：

```python
manager.send_gimbal_rate(
    yaw_rate=0.20,
    pitch_rate=-0.10,
    yaw_lock=False,
)
```

说明：

- `send_gimbal_rate()` 的输入单位仍为 `rad/s`，便于和现有控制器输出保持一致
- 发送时会按官方 MAVLink 定义转换为 `deg/s`
- 底层使用 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`
- 速率控制场景下，`param1/param2` 会发送 `NaN`，只使用 `param3/param4`
- `yaw_lock=False` 对应文档中的 body-frame / follow
- `yaw_lock=True` 对应文档中的 earth-frame / lock
- `gimbal_device_id` 默认使用 `0`

本实现参考官方文档：

- ArduPilot Dev: <https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html>
- MAVLink Common: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW>

## 11. 为什么要统一收发调度

这是本模块最重要的工程约束之一。

原因如下：

- 数传链路是共享带宽
- 飞控会主动发送遥测流
- 电脑侧还会继续发送控制与动作命令
- 如果多个模块自己抢着写链路，会导致带宽混乱、频率失控和调试困难

因此，本模块采用了如下策略：

- 接收线程与发送线程独立
- 所有发送都必须经过 `CommandSender`
- 连续控制命令只保留最新值
- 发送频率通过 `RateController` 统一限频
- 双源模式下也只对 `active_source` 发控制

## 12. 常见问题

### 12.1 为什么收不到消息

常见原因：

- 飞控串口不是 `SERIALx_PROTOCOL = 2`
- 波特率不匹配
- 串口设备写错
- SITL 的 UDP 地址或端口写错
- 被其他地面站占用

### 12.2 SITL 里为什么 5760 连不上

常见原因：

- `sim_vehicle.py` 已带 MAVProxy，占用了 5760
- 你应该改连 `5762` 或 `5763`
- 但要确认对应的 `SERIALx_PROTOCOL` 确实是 MAVLink

### 12.3 为什么不能连 9002

因为 `9002` 这类端口通常是 Gazebo 的 JSON 控制接口，不是 MAVLink 端口。

telemetry_link 只应连接 MAVLink 端口。

### 12.4 为什么不让两个飞控同时收控制命令

因为当前阶段目标只是：

- 双链路并存
- 单 active source
- 单源控制

当前版本明确不做：

- 数据融合
- 自动切换
- 双飞控同步
- 双飞控同时控制

## 13. 当前边界

本模块当前只负责数传通讯适配层，不负责：

- ROS2 节点
- YOLO
- Fusion
- 控制器
- 飞控参数配置界面
- 高层任务逻辑
- 多源数据融合

后续即使上层接入 ROS2 或视觉模块，也应继续把这个目录作为独立链路服务层保留。
