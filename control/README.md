# Control Layer

`control/` 当前实现的是控制层输入适配、模式调度、局部云台视觉伺服与接近控制部分，不是完整控制器，也不是 ROS2 节点。

它的职责是：

- 把 `fusion/` 输出的 `FusedState` 转成控制层统一输入 `ControlInput`
- 根据 `ControlInput` 生成控制阶段摘要与各子控制器的 enable/gating 信号
- 在被上层允许时，根据图像误差生成局部云台 yaw / pitch 角速度命令
- 在被上层允许时，根据机体系横向误差与云台偏航角生成机体 `vy / yaw_rate` 命令
- 在被上层允许时，根据目标尺度生成机体前向速度命令
- 为后续控制器提供更稳定、边界更清晰的上游输入与调度边界

当前目录已包含：

- [control_input.py](/home/level6/uav_project/src/control/control_input.py)
- [control_mode.py](/home/level6/uav_project/src/control/control_mode.py)
- [gimbal_controller.py](/home/level6/uav_project/src/control/gimbal_controller.py)
- [body_controller.py](/home/level6/uav_project/src/control/body_controller.py)
- [approach_controller.py](/home/level6/uav_project/src/control/approach_controller.py)
- [command_shaper.py](/home/level6/uav_project/src/control/command_shaper.py)
- [control_executor.py](/home/level6/uav_project/src/control/control_executor.py)

## 1. 模块定位

`control_input.py` 是 stateful adapter，不是 controller。

`control_mode.py` 是控制层调度/模式判定模块，不是 controller。

`gimbal_controller.py` 是局部视觉伺服快环 controller，不是 mode manager，也不是 body / approach controller。

`body_controller.py` 是机体横向对齐 + 航向对正 controller，不是 mode manager，也不是 gimbal / approach controller。

`approach_controller.py` 是局部接近 controller，不是 mode manager，也不是 gimbal / body controller。

`command_shaper.py` 是控制输出整形与统一收口层，不是 mode manager，不是 controller，也不是 MAVLink sender。

`control_executor.py` 是控制层最后一层执行桥接模块，不是 mode manager，不是 controller，也不是 MAVLink sender。

`control_input.py` 只做：

- 输入预处理
- 状态提取
- 数据清洗
- 轻量一阶低通滤波
- 跟踪连续性判断
- source age 计算

`control_input.py` 不做：

- 状态机
- PID
- 控制律
- `vx / vy / yaw_rate` 输出
- saturation / limit
- fail-safe 策略
- MAVLink 发送逻辑

`control_mode.py` 只做：

- mode summary 判定
- gimbal/body/approach enable 生成
- 输入有效性组合
- staleness gating
- hold / inhibit reason 输出

`control_mode.py` 不做：

- 状态机
- PID
- 控制律
- `vx / vy / yaw_rate` 输出
- saturation / limit
- fail-safe 策略
- MAVLink 发送逻辑

`gimbal_controller.py` 只做：

- 基于 `ex_cam / ey_cam` 的局部控制律
- 生成云台 `yaw_rate / pitch_rate` 命令
- 小 deadband 抑制中心抖动
- 输出限幅
- 最小必要的输入有效性保护
- 可选的小量导数项

`gimbal_controller.py` 不做：

- mode 判定
- enable_gimbal 策略决策
- body 控制
- approach 控制
- `vx / vy / yaw_rate` 机体控制
- `target_size` 处理
- fail-safe 策略
- MAVLink 发送逻辑
- PX4 / gimbal manager 配置逻辑

`body_controller.py` 只做：

- 基于 `ex_body` 的局部横向控制律，生成机体系 `vy` 命令
- 基于 `gimbal_yaw` 的局部航向控制律，生成机体 `yaw_rate` 命令
- 小 deadband 抑制横移 / 转向抖动
- 输出限幅
- 最小必要的输入有效性保护
- 可选的小量导数项

`body_controller.py` 不做：

- mode 判定
- enable_body 策略决策
- gimbal 控制
- approach 控制
- `vx` 控制
- `ey_body -> vx` 映射
- `target_size / bbox_size` 处理
- fail-safe 策略
- MAVLink 发送逻辑
- command shaper 全局整形

`approach_controller.py` 只做：

- 基于 `target_size` 的局部控制律
- 生成机体系前向 `vx` 命令
- 期望距离附近的小 deadband 抑制
- 前进 / 后退分方向限幅
- 最小必要的输入有效性保护
- 可选的小量导数项

`approach_controller.py` 不做：

- mode 判定
- enable_approach 策略决策
- gimbal 控制
- body 控制
- `vy / yaw_rate` 控制
- `ex_body / ey_body` 到 `vx` 的映射
- 使用 `ey_body` 生成 `vx`
- 使用 `ex_body` 生成 `vx`
- `target_size` 提取
- fail-safe 策略
- MAVLink 发送逻辑
- command shaper 全局整形

`command_shaper.py` 只做：

- 合并 `GimbalCommand / BodyCommand / ApproachCommand`
- 根据 `ControlModeOutput.enable_*` 决定各通道是否采纳
- 对最终 `vx / vy / yaw_rate / gimbal_*` 做统一限幅
- 基于上一帧输出做基础 slew rate limit / 平滑
- 对 `None / invalid / inactive / disabled` 的通道统一归零
- 输出可被后续发送层直接消费的 `ShapedCommand`

`command_shaper.py` 不做：

- mode 判定
- PID
- gimbal/body/approach 控制律
- `target_size` 提取
- `track_switched / target_stable` 计算
- fail-safe 状态机
- MAVLink 打包发送

`control_executor.py` 只做：

- 消费 `ShapedCommand`
- 拆分机体命令与云台命令
- 复用 `telemetry_link` 已有发送接口
- 做非常薄的一层字段适配
- 做最小必要的空值/接口可用性检查
- 记录简单执行日志

`control_executor.py` 不做：

- MAVLink 连接创建
- MAVLink 打包细节
- 发送线程管理
- 限频/调度
- 限幅/平滑
- mode 判定
- PID / 控制律
- target / bbox / error 逻辑

## 2. 控制层输入链路

当前推荐的数据链路是：

```text
yolo_app + telemetry_link
-> fusion.FusionManager
-> FusedState
-> control.ControlInputAdapter
-> ControlInput
-> control.ControlModeManager
-> ControlModeOutput
-> control.GimbalController
-> GimbalCommand
-> control.BodyController
-> BodyCommand
-> control.ApproachController
-> ApproachCommand
-> control.CommandShaper
-> ShapedCommand
-> control.ControlExecutor
-> telemetry_link
```

重要边界：

- 控制层应依赖 `FusedState`，不要自己重新拼 `YOLO + telemetry_link`
- 控制器应依赖 `ControlInput`，不要在控制器内部重复做一遍输入清洗和滤波
- `ControlInputAdapter` 只负责“输入变干净”，不负责“控制怎么打”
- `ControlModeManager` 只负责“该不该放行”，不负责“该打多少”
- `GimbalController` 只负责“该转多少”，不负责“该不该启用”
- `BodyController` 只负责“该横移多少和转多少”，不负责“该不该启用”
- `ApproachController` 只负责“该前进多少”，不负责“该不该启用”
- `CommandShaper` 只负责“最后怎么合并和整形”，不负责“控制律怎么来”
- `ControlExecutor` 只负责“怎么桥接到 telemetry_link”，不负责“协议怎么发”

## 3. `ControlInput`

`ControlInput` 是控制层统一输入数据结构，包含以下几类信息：

### 3.1 时间

- `timestamp`
- `dt`

### 3.2 有效性

- `fused_valid`
- `target_valid`
- `target_locked`
- `vision_valid`
- `drone_valid`
- `gimbal_valid`
- `control_allowed`

### 3.3 跟踪连续性

- `track_id`
- `track_switched`
- `target_stable`
- `tracking_state`

### 3.4 已滤波误差

- `ex_cam`
- `ey_cam`
- `ex_body`
- `ey_body`

### 3.5 已滤波云台角

- `gimbal_yaw`
- `gimbal_pitch`

### 3.6 目标尺寸

- `target_size`
- `target_size_valid`

### 3.7 source age

- `fusion_age_s`
- `vision_age_s`
- `drone_age_s`
- `gimbal_age_s`

## 4. 核心规则

### 4.1 `dt`

`dt` 使用 `fused.timestamp` 作为主时间基准：

```text
dt = current_fused_timestamp - last_fused_timestamp
```

实现约束：

- 第一帧返回安全默认值
- 对负值、零值、异常大值做基本保护
- 不依赖 ROS 时间

### 4.2 `track_switched`

仅当“上一帧和当前帧的 `track_id` 都有效且不同”时，`track_switched=True`。

不计为切换的情况：

- `None -> 某个 id`
- `某个 id -> None`
- 无效 `track_id` 参与的变化

在当前工程里，`FusedState.track_id < 0` 会被视为无效。

### 4.3 `target_stable`

`target_stable` 只表达“输入目标是否已稳定连续”，定义为：

```text
target_valid
AND target_locked
AND track_id 有效
AND 同一个 track_id 持续时间 > stable_hold_s
```

当前默认：

- `stable_hold_s = 0.3`

重置条件：

- `track_id` 变化
- `target_valid=False`
- `target_locked=False`

### 4.4 `target_size`

统一输出一个给下游 approach controller 使用的尺寸量：

- 优先 `bbox_h`
- 若 `bbox_h` 无效，则回退 `sqrt(bbox_area)`
- 若都无效，则 `target_size_valid=False`

这里不做任何接近控制逻辑。

### 4.5 低通滤波

当前适配器对这些字段做轻量一阶低通：

- `ex_cam`
- `ey_cam`
- `ex_body`
- `ey_body`
- `gimbal_yaw`
- `gimbal_pitch`
- `target_size`

滤波是标准一阶低通，按 `dt` 更新，目标是抑制抖动，不引入复杂观测器或策略逻辑。

### 4.6 source age

使用统一当前时钟计算：

```text
fusion_age_s = now - fused.timestamp
vision_age_s = now - fused.perception_timestamp
drone_age_s = now - fused.drone_timestamp
gimbal_age_s = now - fused.gimbal_timestamp
```

实现约束：

- 对无效时间戳做保护
- age 至少非负
- 当前实现默认使用 Python 当前时钟 `time.time()`

## 5. 使用方式

```python
from control.control_input import ControlInputAdapter
from control.approach_controller import ApproachController
from control.body_controller import BodyController
from control.command_shaper import CommandShaper
from control.gimbal_controller import GimbalController
from control.control_mode import ControlModeManager
from fusion.models import FusedState

adapter = ControlInputAdapter()
mode_manager = ControlModeManager()
gimbal_controller = GimbalController()
body_controller = BodyController()
approach_controller = ApproachController()
command_shaper = CommandShaper()

control_input = adapter.adapt(fused_state)
mode_output = mode_manager.update(control_input)
gimbal_command = gimbal_controller.update(
    control_input,
    task_mode=mode_output.task_mode,
    enabled=mode_output.enable_gimbal,
)
body_command = body_controller.update(
    control_input,
    task_mode=mode_output.task_mode,
    enabled=mode_output.enable_body,
)
approach_command = approach_controller.update(
    control_input,
    task_mode=mode_output.task_mode,
    enabled=mode_output.enable_approach,
)
shaped_command = command_shaper.update(
    mode=mode_output,
    gimbal_cmd=gimbal_command,
    body_cmd=body_command,
    approach_cmd=approach_command,
    dt=control_input.dt,
)
```

如需在模式切换、链路重连或控制器重启后清空内部状态：

```python
adapter.reset()
mode_manager.reset()
gimbal_controller.reset()
body_controller.reset()
approach_controller.reset()
command_shaper.reset()
```

如果已经有运行中的 `telemetry_link` 服务对象，还可以继续接执行桥接层：

```python
from control.control_executor import ControlExecutor

control_executor = ControlExecutor(telemetry_link=link_manager)
control_executor.execute(shaped_command)
```

如需在执行层重绑 transport/service：

```python
control_executor.set_telemetry_link(link_manager)
# 或
control_executor.update_transport(link_manager)
```

如果需要调整滤波强度或稳定保持时间，可传入配置：

```python
from control.control_input import ControlInputAdapter, ControlInputAdapterConfig

config = ControlInputAdapterConfig(
    stable_hold_s=0.3,
    ex_cam_tau_s=0.08,
    ey_cam_tau_s=0.08,
    ex_body_tau_s=0.08,
    ey_body_tau_s=0.08,
    gimbal_yaw_tau_s=0.10,
    gimbal_pitch_tau_s=0.10,
    target_size_tau_s=0.12,
)
adapter = ControlInputAdapter(config=config)
```

## 6. `ShapedCommand`

`ShapedCommand` 是控制层统一收口后的最终控制命令，当前包含：

- `vx_cmd`
- `vy_cmd`
- `yaw_rate_cmd`
- `gimbal_yaw_rate_cmd`
- `gimbal_pitch_rate_cmd`
- `enable_gimbal`
- `enable_body`
- `enable_approach`
- `active`
- `valid`

语义约定：

- `vx_cmd` 仅来自 `ApproachCommand`
- `vy_cmd / yaw_rate_cmd` 仅来自 `BodyCommand`
- `gimbal_yaw_rate_cmd / gimbal_pitch_rate_cmd` 仅来自 `GimbalCommand`
- `mode.enable_* = False` 时，对应通道不会被采纳
- 对应命令为 `None`、或 `valid=False`、或 `active=False` 时，对应通道视为 `0`
- `valid=True` 表示 unified command 结构完整，可供后续发送层直接消费
- `active=True` 表示当前控制链路被放行，或仍有尚未完全收敛到零的整形输出

## 7. `CommandShaper` 核心规则

### 7.1 合并规则

当前实现严格保持单一来源：

- `ApproachCommand` 只影响 `vx_cmd`
- `BodyCommand` 只影响 `vy_cmd / yaw_rate_cmd`
- `GimbalCommand` 只影响 `gimbal_yaw_rate_cmd / gimbal_pitch_rate_cmd`

也就是说：

- `gimbal_controller.py` 不参与 `vx / vy / yaw_rate`
- `approach_controller.py` 不参与 `vy / yaw_rate`
- `body_controller.py` 不参与 `vx`

### 7.2 disabled / invalid 处理

每个通道独立处理，不会因为某一个 controller 无效就让整条链路一起失效。

例如：

- `body_cmd=None` 时，`vy_cmd / yaw_rate_cmd = 0`
- 但如果 `approach` 仍被放行且输出有效，`vx_cmd` 仍可继续工作

### 7.3 最终限幅与平滑

`CommandShaper` 会对最终输出再次做统一限幅，即使各 controller 内部已经做过局部限幅，这里仍保留系统级最后一道保护。

当前默认对以下通道做 slew rate limit：

- `vx_cmd`
- `vy_cmd`
- `yaw_rate_cmd`
- `gimbal_yaw_rate_cmd`
- `gimbal_pitch_rate_cmd`

调用接口为：

```python
shaped_command = command_shaper.update(
    mode=mode_output,
    gimbal_cmd=gimbal_command,
    body_cmd=body_command,
    approach_cmd=approach_command,
    dt=control_input.dt,
)
```

说明：

- `dt` 由外部传入，推荐直接复用 `ControlInput.dt`
- 当 `smooth_to_zero_when_disabled=True` 时，通道被禁用后会按 slew limit 平滑收敛到 `0`
- 当 `smooth_to_zero_when_disabled=False` 时，通道被禁用后会立即输出 `0`

## 8. `ControlExecutor`

`ControlExecutor` 是控制层最后一层执行桥接模块。

它的输入是：

- `ShapedCommand`
- 已存在的 `telemetry_link` 发送接口对象

它的核心职责是：

- 消费最终 shaped 输出，不重新计算控制量
- 将 `vx_cmd / vy_cmd / yaw_rate_cmd` 映射到机体控制发送接口
- 将 gimbal shaped 字段映射到云台发送接口
- 在不新增协议细节的前提下完成最薄一层适配

推荐主接口：

```python
control_executor.execute(shaped_command)
```

并提供：

- `reset()`
- `set_telemetry_link(...)`
- `update_transport(...)`

### 8.1 机体命令消费规则

`ControlExecutor` 当前从 `ShapedCommand` 读取：

- `vx_cmd`
- `vy_cmd`
- `yaw_rate_cmd`

优先复用 `telemetry_link` 的统一连续控制入口：

- `submit_control_command(...)`

若只有较高层 body velocity 接口可用，则退回：

- `send_velocity_command(...)`

### 8.2 云台命令消费规则

`ControlExecutor` 会按 `ShapedCommand` 当前实际字段适配。

当前仓库里的 `command_shaper.py` 输出的是：

- `gimbal_yaw_rate_cmd`
- `gimbal_pitch_rate_cmd`

也就是说，当前控制层输出的是云台角速度命令，不是角度命令。

如果后续 `ShapedCommand` 变为角度型字段，例如：

- `gimbal_yaw_cmd`
- `gimbal_pitch_cmd`

那么执行层应桥接到 `telemetry_link.send_gimbal_angle(...)`。

### 8.3 当前仓库里的实际适配边界

截至当前实现，`telemetry_link` 已公开的相关接口主要是：

- `submit_control_command(...)`
- `send_velocity_command(...)`
- `send_yaw_rate_command(...)`
- `send_gimbal_angle(...)`
- `stop_control()`

当前版本已通过 `telemetry_link.send_gimbal_rate(...)` 提供云台连续速率接口。

因此：

- `ControlExecutor` 会优先尝试调用 `send_gimbal_rate(...)`
- 若下游 transport 尚未实现该接口，执行层仍会记录日志并跳过 gimbal rate 下发
- 当前仓库的 `telemetry_link` 已补齐该接口，并在发送端把速率命令转换为连续小步进角度控制

### 8.4 `execute()` 的最小策略

当前执行逻辑保持很薄：

1. `cmd is None` 时直接返回
2. `cmd.valid=False` 时直接返回
3. `telemetry_link` 不存在时记录日志并返回
4. body 通道按 `enable_body / enable_approach` 或“仍有非零整形输出”决定是否继续下发
5. gimbal 通道按 `enable_gimbal` 或“仍有非零整形输出”决定是否继续下发
6. 捕获发送异常并记录日志，不在这里做复杂恢复状态机

### 8.5 `reset()`

`reset()` 当前只做执行层最小重置：

- 清空最近一次 body/gimbal 执行记录
- 清空最近一次异常记录
- 若 `telemetry_link` 已提供 `stop_control()`，则复用它发送零 body 控制
- 若 `telemetry_link` 已提供 `send_gimbal_rate()`，则发送零云台角速度命令

执行层当前不负责云台复位到固定角度的策略，也不负责自动重连。

## 9. `GimbalCommand`

`GimbalCommand` 是局部云台视觉伺服输出，当前包含：

- `yaw_rate_cmd`
- `pitch_rate_cmd`
- `active`
- `valid`

语义约定：

- 当前第一版输出的是角速度命令，不是角度命令
- 默认按控制层角速度语义使用弧度制，即 `rad/s`
- `active=True` 表示控制器当前正在主动输出控制
- `valid=True` 表示本次输出在语义上可直接用于下游执行

## 10. `GimbalController` 核心规则

### 10.1 基本控制律

默认第一版采用 `P` 控制：

```text
yaw_rate_cmd   = yaw_sign   * kp_yaw   * ex_cam
pitch_rate_cmd = pitch_sign * kp_pitch * ey_cam
```

说明：

- `ex_cam -> yaw`
- `ey_cam -> pitch`
- 不实现积分项
- `yaw_sign / pitch_sign` 用于匹配图像误差方向与云台运动方向

### 10.2 导数项

当前支持可选导数项，但默认关闭：

```text
d_ex = (ex_cam - last_ex_cam) / dt
d_ey = (ey_cam - last_ey_cam) / dt
```

说明：

- 默认 `use_derivative=False`
- `dt` 会做最小值保护，避免除零与异常放大
- `reset()` 后第一次进入有效控制时，导数输出强制为 `0`

### 10.3 deadband

当前实现包含小 deadband，用于抑制中心附近抖动：

- `abs(ex_cam) < deadband_x` 时，yaw 通道误差视为 `0`
- `abs(ey_cam) < deadband_y` 时，pitch 通道误差视为 `0`

这属于 controller 内部 anti-jitter 机制，不属于 mode 级 gating。

### 10.4 输出限幅

当前输出会在 controller 内部直接限幅：

- `yaw_rate_cmd` 限制到 `[-max_yaw_rate, +max_yaw_rate]`
- `pitch_rate_cmd` 限制到 `[-max_pitch_rate, +max_pitch_rate]`

### 10.5 最小必要保护

当前实现只保留最小必要保护。当以下任一条件不满足时，输出零命令：

- `enabled=False`
- `target_valid=False`
- `vision_valid=False`
- `gimbal_valid=False`
- `ex_cam / ey_cam` 非有限数

此时：

- `active=False`
- `valid=False`

控制器不会在这里实现复杂 mode 判定、staleness 策略或 fail-safe。

## 11. `GimbalControllerConfig`

`GimbalControllerConfig` 当前集中管理这些参数：

- `kp_yaw`
- `kp_pitch`
- `kd_yaw`
- `kd_pitch`
- `use_derivative`
- `deadband_x`
- `deadband_y`
- `max_yaw_rate`
- `max_pitch_rate`
- `yaw_sign`
- `pitch_sign`
- `dt_min`

示例：

```python
from control.gimbal_controller import GimbalController, GimbalControllerConfig

config = GimbalControllerConfig(
    kp_yaw=1.5,
    kp_pitch=1.5,
    use_derivative=False,
    deadband_x=0.01,
    deadband_y=0.01,
    max_yaw_rate=1.0,
    max_pitch_rate=1.0,
    yaw_sign=1.0,
    pitch_sign=1.0,
)
gimbal_controller = GimbalController(config=config)
```

## 12. `ControlModeOutput`

`ControlModeOutput` 是控制层调度输出，当前包含：

### 12.1 mode summary

- `mode_name`
- `task_mode`

### 12.2 controller enables

- `enable_gimbal`
- `enable_body`
- `enable_approach`

### 12.3 status summary

- `has_target`
- `target_stable`
- `control_ready`

### 12.4 debug / hold

- `hold_reason`

当前 `task_mode` 用于切换 controller 语义：

- `APPROACH_TRACK`
- `OVERHEAD_HOLD`

当前 `mode_name` 仍用于概括 enable 摘要；在 `OVERHEAD_HOLD` 任务下会额外输出：

- `OVERHEAD_HOLD`
- `OVERHEAD_RECOVERY`

其余情况下 `mode_name` 由三个 `enable_*` 反推：

- 三个都不放行时为 `IDLE`
- 仅 `gimbal` 放行时为 `TRACKING`
- `gimbal + body` 放行时为 `ALIGNING`
- `gimbal + body + approach` 放行时为 `APPROACHING`

## 13. `ApproachCommand`

`ApproachCommand` 是局部接近控制输出，当前包含：

- `vx_cmd`
- `active`
- `valid`

语义约定：

- 当前第一版只输出机体系前向速度命令，不输出 `vy / yaw_rate`
- 默认沿用项目现有速度语义，`vx` 为 body-frame 速度
- `active=True` 表示控制器当前正在主动输出前进或后退控制
- `valid=True` 表示本次输出在语义上可直接用于下游执行

## 14. `ApproachController` 核心规则

### 14.1 基本控制律

默认第一版采用 `P` 控制：

```text
size_error = target_size_ref - target_size
vx_cmd = vx_sign * kp_vx * size_error
```

说明：

- `target_size < target_size_ref` 时，目标看起来更远，`size_error > 0`，应倾向于前进
- `target_size > target_size_ref` 时，目标看起来更近，`size_error < 0`，应倾向于减速、停止或轻微后退
- 当前控制的是“距离代理量闭环”，不是绝对米制距离闭环
- 不实现积分项
- `vx_sign` 用于匹配尺度误差方向与机体前向速度正方向
- 当前实现先计算 `size_error`，再对前向控制通道应用 deadband

### 14.2 导数项

当前支持可选导数项，但默认关闭：

```text
d_size_error = (size_error - last_size_error) / dt
```

说明：

- 默认 `use_derivative=False`
- `dt` 会做最小值保护，避免除零与异常放大
- `reset()` 后第一次进入有效控制时，导数输出强制为 `0`
- 当前实现使用 `size_error` 的帧间变化计算导数
- 当 deadband 后的控制误差为 `0` 时，默认不再叠加 `D` 项，避免目标附近来回抖动

### 14.3 deadband

当前实现包含小 deadband，用于抑制期望距离附近的前后抖动：

- `abs(size_error) < deadband_size` 时，前向通道误差视为 `0`

这属于 controller 内部 anti-jitter 机制，不属于 mode 级 gating。

### 14.4 输出限幅

当前输出会在 controller 内部直接限幅：

- 前进速度最大不超过 `max_forward_vx`
- 若 `allow_backward=False`，则 `vx_cmd >= 0`
- 若 `allow_backward=True`，则 `vx_cmd >= -max_backward_vx`

### 14.5 最小必要保护

当前实现只保留最小必要保护。当以下任一条件不满足时，输出零命令：

- `enabled=False`
- `target_valid=False`
- `vision_valid=False`
- `drone_valid=False`
- `target_size_valid=False`
- `target_size <= min_valid_target_size`
- `target_size` 非有限数

此时：

- `active=False`
- `valid=False`

控制器不会在这里实现复杂 mode 判定、staleness 策略或 fail-safe。

### 14.6 职责边界

`ApproachController` 当前严格只围绕 `target_size -> vx_cmd` 工作：

- 不使用 `ey_body` 生成 `vx_cmd`
- 不使用 `ex_body` 生成 `vx_cmd`
- 不输出 `vy_cmd`
- 不输出 `yaw_rate_cmd`
- 不判断 `TRACKING / ALIGNING / APPROACHING` 模式
- 不实现 `enable_approach` 策略

## 15. `ApproachControllerConfig`

`ApproachControllerConfig` 当前集中管理这些参数：

- `target_size_ref`
- `kp_vx`
- `kd_vx`
- `use_derivative`
- `deadband_size`
- `max_forward_vx`
- `max_backward_vx`
- `vx_sign`
- `allow_backward`
- `min_valid_target_size`
- `dt_min`

示例：

```python
from control.approach_controller import ApproachController, ApproachControllerConfig

config = ApproachControllerConfig(
    target_size_ref=0.35,
    kp_vx=1.0,
    use_derivative=False,
    deadband_size=0.02,
    max_forward_vx=0.8,
    max_backward_vx=0.2,
    vx_sign=1.0,
    allow_backward=False,
)
approach_controller = ApproachController(config=config)
```

## 16. `ControlModeManager` gating 规则

### 16.1 `gimbal`

默认第一版：

```text
enable_gimbal =
    control_allowed
    AND fused_valid
    AND vision_valid
    AND vision_age_s <= max_vision_age_s
    AND target_valid
```

说明：

- `gimbal` 放行条件最宽松
- 默认不要求 `target_stable`
- 默认不强制要求 `gimbal_age_s` 新鲜，但已预留配置项

### 16.2 `body`

默认第一版：

```text
enable_body =
    control_allowed
    AND fused_valid
    AND vision_valid
    AND vision_age_s <= max_vision_age_s
    AND drone_valid
    AND drone_age_s <= max_drone_age_s
    AND target_valid
    AND target_locked
```

说明：

- `body` 比 `gimbal` 更严格
- 当前默认 `require_target_locked_for_body=True`
- 第一版不因为 `track_switched` 单独抑制 `body`

### 16.3 `approach`

默认第一版：

```text
enable_approach =
    control_allowed
    AND fused_valid
    AND vision_valid
    AND vision_age_s <= max_vision_age_s
    AND drone_valid
    AND drone_age_s <= max_drone_age_s
    AND target_valid
    AND target_locked
    AND target_stable
    AND NOT track_switched
    AND target_size_valid
    AND abs(gimbal_yaw) <= yaw_align_thresh_rad
```

说明：

- `approach` 放行条件最严格
- `target_stable=False` 时不允许前进
- `track_switched=True` 时当前实现也会抑制 `approach`
- 偏航未对齐时默认返回 `yaw_not_aligned`

### 16.4 `hold_reason`

当前实现会在 `body` 或 `approach` 未放行时，尽量输出一个主阻塞原因，例如：

- `control_not_allowed`
- `fusion_invalid`
- `vision_invalid`
- `vision_stale`
- `drone_invalid`
- `drone_stale`
- `no_target`
- `target_not_locked`
- `track_switched`
- `target_not_stable`
- `target_size_invalid`
- `yaw_not_aligned`

## 16. 参数配置

`ControlModeConfig` 当前集中管理这些阈值和开关：

- `max_vision_age_s`
- `max_drone_age_s`
- `max_gimbal_age_s`
- `yaw_align_thresh_rad`
- `require_target_locked_for_body`
- `require_target_stable_for_approach`
- `require_yaw_aligned_for_approach`
- `require_gimbal_fresh_for_gimbal`

示例：

```python
from control.control_mode import ControlModeConfig, ControlModeManager

config = ControlModeConfig(
    max_vision_age_s=0.3,
    max_drone_age_s=0.3,
    yaw_align_thresh_rad=0.35,
    require_target_locked_for_body=True,
    require_target_stable_for_approach=True,
)
mode_manager = ControlModeManager(config=config)
```

## 17. 当前实现说明

`control_input.py` 直接复用了：

- `fusion.models.FusedState`

没有在控制层重定义一套长期使用的 `FusedState`。

这意味着如果后续 `fusion/` 补充字段，控制层应优先在适配器里做语义映射，而不是复制一份新的上游状态结构。

`control_mode.py` 直接复用了：

- `control.control_input.ControlInput`

没有在模式层重定义一套长期使用的控制输入结构。

这意味着后续如果 `ControlInput` 增加更明确的 ready / age / alignment 字段，模式层应优先复用这些字段，而不是在模式层内部重复造一套并行语义。

`gimbal_controller.py` 直接复用了：

- `control.control_input.ControlInput`

没有在云台控制器内部重复定义输入适配、mode summary 或执行链路配置。

这意味着后续如果 `ControlInput` 增加更明确的图像误差语义或云台状态字段，局部控制器应优先直接复用，而不是在控制器里再复制一套上游语义。

`approach_controller.py` 直接复用了：

- `control.control_input.ControlInput`

没有在接近控制器内部重复定义 mode 逻辑、目标尺度提取或执行链路配置。

这意味着后续如果 `ControlInput` 增加更明确的距离代理量语义或接近相关状态字段，局部控制器应优先直接复用，而不是在控制器里再复制一套上游语义。
