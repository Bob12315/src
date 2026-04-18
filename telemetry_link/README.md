# Telemetry Link

本目录实现的是一个独立运行的飞控数传通讯适配层，不是 ROS2 节点，也不依赖 YOLO、控制器或飞控参数配置界面。

它的定位是一个基础设施层服务模块，负责：

- 建立与飞控的 MAVLink2 通信
- 接收并解析飞控状态流
- 缓存最新飞控状态
- 统一发送控制命令和一次性动作命令
- 对收发进行线程隔离、统一调度和限频

## 1. 这不是 ROS2 节点

请明确：

- 这个模块不是 `ros2 run ...`
- 不使用 `rclpy`
- 不发布或订阅 ROS2 topic
- 它应该直接在终端中运行

这是一个独立进程内的链路服务模块，目标是给上层提供两个基础能力：

- 最新飞控状态
- 统一命令发送接口

## 2. 模块职责

整体数据流如下：

```text
Serial / UDP
-> pymavlink
-> TelemetryReceiver
-> StateCache

Action / Control Commands
-> CommandQueue
-> CommandSender
-> pymavlink
-> Flight Controller
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
- 接收线程 `TelemetryReceiver`：持续 `recv_match()`，解析 MAVLink 消息，更新最新状态缓存
- 发送线程 `CommandSender`：从命令队列中统一取命令并限频发送

关键约束：

- 只有 `LinkManager` 持有链路对象的生命周期控制权
- 外部模块不能直接访问 `pymavlink` 连接对象
- 接收线程不发命令
- 发送线程不做复杂状态计算
- 连续控制命令不排长队，只保留最新一条

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

- 支持串口连接
- 支持 UDP 连接，方便 SITL
- `wait_heartbeat()` 获取 `target_system / target_component`
- 接收并解析常用状态消息
- 线程安全状态缓存
- 连续控制命令与一次性动作命令分流
- 统一发送线程
- 控制命令限频
- 心跳超时判断

已解析的消息包括：

- `HEARTBEAT`
- `ATTITUDE`
- `GLOBAL_POSITION_INT`
- `LOCAL_POSITION_NED`
- `VFR_HUD`
- `SYS_STATUS`
- `GPS_RAW_INT`
- `RC_CHANNELS`

## 6. 数据结构

### 6.1 `DroneState`

至少包含：

- `timestamp`
- `connected`
- `armed`
- `mode`
- `roll`
- `pitch`
- `yaw`
- `vx`
- `vy`
- `vz`
- `altitude`
- `relative_alt`
- `lat`
- `lon`
- `battery_voltage`
- `battery_remaining`
- `gps_fix_type`
- `satellites_visible`

### 6.2 `LinkStatus`

包含：

- `connected`
- `last_rx_time`
- `last_tx_time`
- `heartbeat_timeout_sec`
- `target_system`
- `target_component`

### 6.3 `ControlCommand`

连续控制类命令，例如：

- 速度控制
- 偏航角速度控制
- 停止控制

它们不会积压成长队列，只保留最新值。

### 6.4 `ActionCommand`

一次性动作命令，例如：

- `set_mode`
- `arm`
- `disarm`
- `request_message_interval`

它们走优先级队列，可以重试。

## 7. 配置说明

配置文件位于 [config.yaml](//telemetry_link/config.yaml)。

核心参数：

- `connection_type`：`serial` 或 `udp`
- `serial_port`：串口设备路径
- `baudrate`：串口波特率
- `udp_mode`：`udpin` 或 `udpout`
- `udp_host`：UDP 地址
- `udp_port`：UDP 端口
- `control_send_rate_hz`：连续控制命令发送频率
- `action_cmd_retries`：动作命令失败后重试次数
- `heartbeat_timeout_sec`：心跳超时判定阈值
- `reconnect_interval_sec`：重连间隔
- `request_message_intervals`：是否启动时主动请求常用消息频率

## 8. 运行方式

### 8.1 安装依赖

```bash
pip install pymavlink pyyaml
```

### 8.2 串口连接飞控
ls /dev/ttyUSB*

```bash
cd /home/level6/uav_project/src/telemetry_link
python3 main.py \
  --connection-type serial \
  --serial-port /dev/ttyUSB0 \
  --baudrate 115200
```

### 8.3 使用 UDP 连接 SITL

监听 SITL 发来的 UDP：

```bash
python3 main.py \
  --connection-type udp \
  --udp-mode udpin \
  --udp-host 0.0.0.0 \
  --udp-port 14550
```

主动发往指定 UDP 目标：

```bash
python3 main.py \
  --connection-type udp \
  --udp-mode udpout \
  --udp-host 127.0.0.1 \
  --udp-port 14550
```

## 9. 对外接口说明

统一入口是 `LinkManager`：

- `get_latest_state()`
- `get_link_status()`
- `is_connected()`
- `submit_control_command(...)`
- `submit_action_command(...)`
- `set_mode(...)`
- `arm()`
- `disarm()`
- `send_velocity_command(...)`
- `send_yaw_rate_command(...)`
- `stop_control()`

外部业务不应直接调用 `pymavlink` 的连接对象。

## 10. 为什么要统一收发调度

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

## 11. 常见问题

### 11.1 为什么收不到消息

常见原因：

- 飞控串口不是 `SERIALx_PROTOCOL = 2`
- 波特率不匹配
- 串口设备写错
- SITL 的 UDP 地址或端口写错
- 被其他地面站占用

### 11.2 为什么模式或消息频率和预期不一致

常见原因：

- 其他地面站覆盖了流速设置
- `MAVx_* / SRx_*` 设置的是另一条链路
- 飞控当前模式不支持某些外部控制命令

### 11.3 为什么不让多个模块直接发 MAVLink

因为这样会导致：

- 发送频率不可控
- 链路带宽被打爆
- 控制命令和动作命令互相干扰
- 很难定位谁发了什么

所以必须统一走命令队列和发送线程。

## 12. 当前边界

本模块当前只负责数传通讯适配层，不负责：

- ROS2 节点
- YOLO
- 控制器
- 飞控参数配置界面
- 高层任务逻辑

后续即使上层接入 ROS2 或视觉模块，也应继续把这个目录作为独立链路服务层保留。
