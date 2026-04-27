# UAV UI

`uav_ui/` 是无人机项目的共享终端 UI 模块。它不直接管理 MAVLink 连接，也不直接运行控制器，只负责把已有运行时对象显示出来，并把用户输入的命令分发到对应模块。

当前主要入口：

- [terminal_ui.py](/home/level6/uav_project/src/uav_ui/terminal_ui.py)：curses 终端界面
- [ui_commands.py](/home/level6/uav_project/src/uav_ui/ui_commands.py)：UI 层命令分发
- [control_switches.py](/home/level6/uav_project/src/uav_ui/control_switches.py)：control controller 运行时开关
- [yolo_command_client.py](/home/level6/uav_project/src/uav_ui/yolo_command_client.py)：向 `yolo_app` 发送目标切换 UDP 命令

## 启动方式

### 跟随 control 启动

当 [telemetry_link/config.yaml](/home/level6/uav_project/src/telemetry_link/config.yaml) 中：

```yaml
ui_enabled: true
```

运行：

```bash
python -m control
```

会启动 control 主循环，同时打开终端 UI。control 循环在后台线程运行，UI 在主线程接管终端。

### 跟随 telemetry_link 启动

也可以只启动 telemetry link UI：

```bash
python -m telemetry_link.main
```

此模式只显示 telemetry 状态和 MAVLink 手动命令，不显示 control 输出命令，也不支持 controller 运行时开关。

## 界面区域

当前 UI 分为三块：

- `Latest telemetry`：显示最新无人机状态、姿态、速度、位置、电池、云台状态和链路状态
- `Manual commands`：显示手动输入命令的执行结果
- `Control output`：control 启动 UI 时显示 control 对无人机输出的 shaped command

底部输入框可以直接输入命令，按 Enter 发送。

常用按键：

- `Enter`：发送当前命令
- `Up / Down`：浏览命令历史
- `Esc` 或 `Ctrl-C`：退出 UI
- `quit` 或 `exit`：退出 UI

## 手动飞控命令

这些命令会走 `telemetry_link.command_dispatcher`，最终通过 `LinkManager` 进入 MAVLink 发送链路：

```text
arm
disarm
land
stop
mode GUIDED
takeoff 5
body_vel 5 0 0
yaw_rate 0.2
gimbal -20 0
gimbal_rate 0 20
switch_source real
switch_source sitl
```

更多格式见 [telemetry_link/command_dispatcher.py](/home/level6/uav_project/src/telemetry_link/command_dispatcher.py)。

## 目标切换命令

control 启动 UI 时支持：

```text
target next
target prev
target lock 7
target unlock
```

这些命令会通过 UDP 发给 `yolo_app.command_receiver`。

默认读取 [yolo_app/config.yaml](/home/level6/uav_project/src/yolo_app/config.yaml) 中：

```yaml
command_enabled: true
command_ip: "0.0.0.0"
command_port: 5006
```

如果 `command_ip` 是 `0.0.0.0`，UI 会自动改发到 `127.0.0.1`。

也可以启动 control 时指定 yolo 配置：

```bash
python -m control --yolo-config /path/to/yolo_app/config.yaml
```

发送 JSON 格式：

```json
{"action": "switch_next"}
{"action": "switch_prev"}
{"action": "unlock_target"}
{"action": "lock_target", "track_id": 7}
```

## Controller 运行时开关

control 启动 UI 时支持运行时开关：

```text
controller gimbal on
controller gimbal off
controller gimbal toggle

controller body on
controller body off
controller body toggle

controller approach on
controller approach off
controller approach toggle

controller all on
controller all off
controller all toggle
```

这些命令只影响当前运行中的 control，不会修改 [control/config.yaml](/home/level6/uav_project/src/control/config.yaml)。

初始值来自：

```yaml
runtime:
  enable_gimbal_controller: true
  enable_body_controller: true
  enable_approach_controller: true
```

UI 的 `Control output` 第一行会显示当前状态：

```text
Controllers G=ON B=ON A=OFF SEND=ON
```

## Control 发送开关

control 启动 UI 时也支持运行时开关是否真的下发 control 命令：

```text
control send on
control send off
control send toggle
```

这些命令只影响当前运行中的 control，不会修改配置文件。

初始值来自 [control/config.yaml](/home/level6/uav_project/src/control/config.yaml)：

```yaml
executor:
  send_commands: true
```

当 `SEND=OFF` 时，control 仍然会正常计算内部 shaped command，但不会调用 `ControlExecutor` 下发，也会清空 `telemetry_link` 中保留的连续 control/gimbal_rate 队列。UI 的 `Control output` 只显示一条静态 `DRY continuous command sending disabled` 提示，不再刷新命令流水。

## Control Output

control 启动 UI 时，`Control output` 会显示最近的 shaped command：

```text
12:34:56 vx=0.100 vy=0.000 yaw=0.000 gimbal=(0.120,-0.050) en=G1 B0 A1 active=True valid=True
```

字段含义：

- `vx / vy / yaw`：机体速度与偏航角速度命令
- `gimbal=(yaw_rate,pitch_rate)`：云台角速度命令
- `en=G/B/A`：gimbal/body/approach 三路 controller 是否放行
- `active`：当前 shaped command 是否活跃
- `valid`：当前 shaped command 是否有效

## 设计边界

`uav_ui` 只做显示和命令分发：

- 不创建 MAVLink 连接
- 不直接访问飞控串口或 UDP/TCP 链路
- 不保存 controller 开关到配置文件
- 不解析 YOLO 画面或 track 列表

具体执行仍由对应模块完成：

- MAVLink 命令由 `telemetry_link` 执行
- 目标切换由 `yolo_app` 执行
- controller 开关由 `control` 主循环读取
