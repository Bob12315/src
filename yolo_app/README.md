# YOLO App

本目录只负责无人机项目中的 YOLO 感知与目标跟踪层，不包含 ROS2、飞控控制器或其他上层模块。

本实现严格遵循以下原则：

- 保留官方 Ultralytics `model.track(...)` 主流程
- 使用官方 ByteTrack 配置 `tracker="bytetrack.yaml"`
- 不重写 ByteTrack 底层
- 不引入 ROS2
- 通过 UDP(JSON) 与后续系统通信
- 在官方 tracking 输出外层增加目标管理、UDP 输出和调试标注

## 1. 功能概述

当前实现的流程是：

```text
Video Source
-> Ultralytics YOLO official track(...)
-> ByteTrack official tracker
-> Track list with track_id
-> TargetManager
-> UdpPublisher(JSON)
-> Annotator
```

系统完成的功能包括：

- 读取视频源
- 调用官方 YOLO tracking
- 输出带 `track_id` 的目标
- 自动选择当前主目标
- 锁定目标
- 切换目标
- 处理目标丢失
- 通过 UDP 发送当前唯一主目标
- 在画面中显示所有目标与当前锁定目标

## 2. 目录结构

```text
yolo_app/
  ├── main.py
  ├── config.yaml
  ├── config.py
  ├── video_source.py
  ├── tracker_runner.py
  ├── target_manager.py
  ├── udp_publisher.py
  ├── command_receiver.py
  ├── annotator.py
  ├── models.py
  ├── utils.py
  └── README.md
```

各模块职责：

- `main.py`：主入口，负责把各个模块串起来。
- `config.yaml`：默认配置文件。
- `config.py`：加载配置文件并支持命令行覆盖。
- `video_source.py`：统一处理 `/dev/videoX`、UDP 端口、RTSP、本地视频文件输入。
- `tracker_runner.py`：官方 tracking 调用封装层，只负责 `model.track(...)` 和结果解析。
- `target_manager.py`：负责主目标自动选择、锁定、切换、丢失计数。
- `udp_publisher.py`：按固定 JSON 协议输出当前唯一主目标。
- `command_receiver.py`：接收简单 UDP 控制命令。
- `annotator.py`：负责显示普通框、锁定框、状态信息、准星和辅助线。
- `models.py`：统一数据结构。
- `utils.py`：辅助函数。

## 3. 运行前准备

### 3.1 Python 环境

建议在单独的 conda 环境中运行本目录。

至少需要安装：

- Python  3.11
- `ultralytics`
- `opencv-python`
- `pyyaml`

示例：

```bash
conda create -n yolo python=3.10 -y
conda activate yolo
pip install ultralytics opencv-python pyyaml
```

如果你要用 GPU，请确保本机 PyTorch/CUDA 环境与 `ultralytics` 对应版本可用。

### 3.2 模型准备

本项目支持自定义训练模型 `.pt`。

例如：

```text
/home/level6/models/best.pt
```

你只需要在 `config.yaml` 或命令行里设置：

```yaml
model_path: "/home/level6/models/best.pt"
```

### 3.3 视频源准备

支持以下几类输入：

- `/dev/video0`
- `/dev/video1`
- UDP 端口号，例如 `5600`
- RTSP 地址
- 本地视频文件，例如 `demo.mp4`
- 完整 GStreamer pipeline

推荐填写方式：

- 如果是视频设备，直接填 `/dev/videoX`
- 如果是 UDP H264/RTP 输入，直接填端口号，例如 `5600`

也就是说：

- `video` 就填 `/dev/videoX`
- `udp` 就填端口号

程序内部会自动判断并接入。

## 4. 主流程说明

程序每一帧的执行流程如下：

1. `video_source.py` 读取一帧图像，产生 `frame / frame_id / timestamp`
2. `tracker_runner.py` 调用官方 `YOLO(model_path).track(...)`
3. 官方 ByteTrack 返回带 `track_id` 的目标
4. `tracker_runner.py` 将官方结果转换成统一 `Track` 列表
5. `command_receiver.py` 读取外部命令，例如切换目标或强制锁定
6. `target_manager.py` 根据当前 tracks 维护唯一主目标
7. `udp_publisher.py` 将当前主目标按固定 JSON 协议发出
8. `annotator.py` 在图像上绘制普通框、锁定框、状态信息和准星
9. 根据配置决定是否显示窗口以及是否保存视频

## 5. 快速启动

先进入目录：

```bash
cd /home/level6/uav_project/src/yolo_app
```

### 5.1 直接使用默认配置启动

```bash
python3 main.py
```

默认读取：

- `config.yaml`
- `source=/dev/video0`
- `tracker=bytetrack.yaml`

### 5.2 使用命令行覆盖配置启动

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --tracker bytetrack.yaml \
  --udp-ip 127.0.0.1 \
  --udp-port 5005
```

### 5.3 使用 RTSP 启动

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source rtsp://127.0.0.1:8554/live
```

### 5.4 使用本地视频文件启动

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /home/level6/videos/test.mp4
```

### 5.4.1 使用 UDP 端口输入启动

如果你的输入来自本机某个 UDP 端口，例如：

- `127.0.0.1:5600`

现在推荐直接这样启动：

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source 5600
```

或者在 [config.yaml](/home/level6/uav_project/src/yolo_app/config.yaml) 中写：

```yaml
source: "5600"
```

程序内部会自动完成：

```text
UDP port
-> internal system-python GStreamer helper
-> H264 decode
-> raw frame pipe
-> YOLO
```

### 5.4.2 如需手动指定完整 GStreamer pipeline

如果你的图传或本地转发输出是 GStreamer 可解析的视频流，例如：

- UDP + RTP + H264

可以直接把完整 pipeline 作为 `source` 传入。

本项目当前已经在本机验证通过下面这个例子：

```text
udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false
```

对应启动命令：

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source "udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false"
```

如果你希望直接写到配置文件中，可以在 [config.yaml](/home/level6/uav_project/src/yolo_app/config.yaml) 中这样改：

```yaml
source: "udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false"
```

说明：

- `udpsrc port=5600`：监听本机 5600 端口
- `application/x-rtp,media=video,encoding-name=H264,payload=96`：声明该 UDP 数据是 RTP/H264
- `rtph264depay`：去掉 RTP 封装
- `avdec_h264`：H264 解码
- `videoconvert`：转换成 OpenCV 更容易接收的图像格式
- `appsink drop=true sync=false`：把数据送给 OpenCV，并尽量降低延迟

注意：

- 这一路输入不是简单的 `udp://127.0.0.1:5600`
- 而是通过 GStreamer pipeline 显式告诉 OpenCV 如何解封装和解码
- 如果直接写成 `udp://127.0.0.1:5600`，当前环境下不一定能识别成视频流

当前机器已经确认：

- 系统 Python 环境带 GStreamer 支持
- 上述 pipeline 可以成功打开并读到 `(480, 640, 3)` 图像帧
- `yolo` conda 环境中的 OpenCV 没有 GStreamer 支持
- 因此普通 UDP 端口模式会通过内部 helper 完成低延迟桥接
- 完整 GStreamer pipeline 仍然兼容，但更推荐直接写端口号

### 5.5 保存结果视频

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /home/level6/videos/test.mp4 \
  --save-video true \
  --save-path /home/level6/uav_project/output/track_result.mp4
```

### 5.6 不显示窗口，仅做后台推理与 UDP 输出

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --show false
```

## 6. 配置文件说明

默认配置文件是 [config.yaml](/home/level6/uav_project/src/yolo_app/config.yaml)。

当前主要参数说明如下：

- `model_path`：YOLO 模型 `.pt` 路径
- `source`：视频源，可为 `/dev/video0`、UDP 端口号、RTSP、本地视频文件，或完整 GStreamer pipeline
- `img_size`：YOLO 推理尺寸
- `conf_thres`：检测置信度阈值
- `iou_thres`：NMS IOU 阈值
- `tracker`：tracker 配置，默认 `bytetrack.yaml`
- `device`：运行设备，例如 `0`、`cpu`
- `classes`：类别过滤列表，留空表示不过滤
- `udp_ip`：当前主目标 JSON 的发送 IP
- `udp_port`：当前主目标 JSON 的发送端口
- `selection_mode`：自动选目标策略，可选 `center`、`biggest`、`class`
- `target_class`：当 `selection_mode=class` 时优先选择的类别名
- `max_lost_frames`：锁定目标允许丢失的最大帧数
- `show`：是否显示窗口
- `save_video`：是否保存标注后视频
- `save_path`：保存视频路径
- `line_width`：普通框线宽
- `show_all_tracks`：是否绘制所有可见 track
- `command_enabled`：是否启用命令接收
- `command_ip`：命令监听 IP
- `command_port`：命令监听端口
- `window_name`：显示窗口名称

## 7. 自动选目标逻辑

在没有锁定目标时，系统会自动从当前 tracks 中选一个主目标。

支持三种策略：

### 7.1 `center`

选择离画面中心最近的目标。

示例：

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --selection-mode center
```

### 7.2 `biggest`

选择面积最大的目标。

示例：

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --selection-mode biggest
```

### 7.3 `class`

优先选择指定类别中的目标，然后在该类别中选最靠近中心的目标。

示例：

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --selection-mode class \
  --target-class person
```

## 8. 锁定、切换与丢失处理

### 8.1 锁定目标

系统选中一个目标后，会记录该目标的 `locked_track_id`。

后续优先跟踪这个 ID，不会每帧重新选目标。

### 8.2 目标丢失

如果当前 `locked_track_id` 在本帧没有出现：

- `lost_count += 1`
- 状态切到 `lost`

如果它重新出现：

- `lost_count = 0`
- 状态恢复为 `locked`

如果：

```text
lost_count >= max_lost_frames
```

则视为真正丢失，系统解锁该目标。

### 8.3 切换目标

支持以下控制命令：

- `lock_target(track_id)`
- `switch_next()`
- `switch_prev()`
- `unlock_target()`

可见目标排序方式为：

- 按中心点 `cx` 从左到右排序

## 9. UDP 输出协议

每一帧都会通过 UDP 发送“当前唯一主目标”。

即使没有有效目标，也会发送一条 `target_valid=false` 的消息。

### 9.1 有目标时

```json
{
  "timestamp": 1710000000.123,
  "frame_id": 1052,
  "target_valid": true,
  "tracking_state": "locked",
  "track_id": 7,
  "class_id": 0,
  "class_name": "person",
  "confidence": 0.93,
  "cx": 314.2,
  "cy": 241.8,
  "w": 88.1,
  "h": 135.4,
  "ex": -0.018,
  "ey": 0.012,
  "image_width": 640,
  "image_height": 480,
  "lost_count": 0
}
```

### 9.2 无目标时

```json
{
  "timestamp": 1710000001.100,
  "frame_id": 1053,
  "target_valid": false,
  "tracking_state": "lost",
  "track_id": -1,
  "class_id": -1,
  "class_name": "",
  "confidence": 0.0,
  "cx": 0.0,
  "cy": 0.0,
  "w": 0.0,
  "h": 0.0,
  "ex": 0.0,
  "ey": 0.0,
  "image_width": 640,
  "image_height": 480,
  "lost_count": 5
}
```

### 9.3 `ex / ey` 定义

归一化误差计算方式固定为：

```text
ex = (cx - image_width / 2) / (image_width / 2)
ey = (cy - image_height / 2) / (image_height / 2)
```

## 10. 如何接收 UDP 数据

你可以用任意 UDP 接收程序读取当前主目标 JSON。

例如用 Python 临时监听：

```bash
python3 - <<'PY'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
print("listening on udp 5005...")
while True:
    data, addr = sock.recvfrom(65535)
    print(addr, data.decode("utf-8"))
PY
```

## 11. 如何发送控制命令

当前版本支持通过 UDP 发送简单 JSON 命令给 `command_receiver.py`。

默认命令监听端口是：

```text
5006
```

### 11.1 切到下一个目标

```bash
python3 - <<'PY'
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(json.dumps({"action": "switch_next"}).encode(), ("127.0.0.1", 5006))
PY
```

### 11.2 切到上一个目标

```bash
python3 - <<'PY'
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(json.dumps({"action": "switch_prev"}).encode(), ("127.0.0.1", 5006))
PY
```

### 11.3 强制锁定指定 track_id

```bash
python3 - <<'PY'
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(json.dumps({"action": "lock_target", "track_id": 7}).encode(), ("127.0.0.1", 5006))
PY
```

### 11.4 解锁当前目标

```bash
python3 - <<'PY'
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(json.dumps({"action": "unlock_target"}).encode(), ("127.0.0.1", 5006))
PY
```

## 12. 标注显示规则

当前画面标注规则如下：

### 12.1 普通目标

- 细框
- 标签格式：`class_name #track_id conf`

### 12.2 当前锁定目标

- 粗框
- 特殊颜色
- 标签格式：`LOCKED class_name #track_id conf`
- 目标中心画点
- 从图像中心到目标中心画辅助线

### 12.3 左上角状态信息

显示内容：

- `tracking_state`
- `locked_track_id`
- `lost_count`
- `ex`
- `ey`

### 12.4 图像中心

- 画准星

## 13. 常见运行示例

### 场景 1：USB 图传接收器接到 `/dev/video0`

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --selection-mode center \
  --udp-ip 127.0.0.1 \
  --udp-port 5005 \
  --show true
```

### 场景 2：只想跟踪 `person`

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --selection-mode class \
  --target-class person
```

### 场景 3：不看画面，只输出 UDP 给后续模块

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /dev/video0 \
  --show false \
  --save-video false
```

### 场景 4：回放本地视频调算法

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source /home/level6/videos/test.mp4 \
  --show true \
  --save-video true \
  --save-path /home/level6/uav_project/output/test_track.mp4
```

### 场景 5：使用本机 5600 端口的 GStreamer UDP H264 流

```bash
python3 main.py \
  --model-path /home/level6/models/best.pt \
  --source "udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false" \
  --show true
```

### 场景 6：把 GStreamer 流写入配置文件后直接启动

先修改 [config.yaml](/home/level6/uav_project/src/yolo_app/config.yaml)：

```yaml
source: "udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false"
```

然后启动：

```bash
cd /home/level6/uav_project/src/yolo_app
python3 main.py
```

## 14.1 GStreamer 使用建议

如果你后面继续使用 UDP H264 图传，建议优先采用 GStreamer pipeline 方式，而不是 `udp://...` 方式。

原因：

- 能显式指定流格式
- 更适合 RTP/H264 输入
- 更容易控制低延迟行为
- 当前机器已经实测可用
- 当前代码已经对 GStreamer pipeline 做了显式后端选择，比默认 `VideoCapture(...)` 更稳

如果后续你的图传不是 RTP/H264，而是别的封装格式，需要相应调整 pipeline 中间部分，但 `appsink` 结尾的总体结构通常不变。

## 14. 当前限制

当前版本是 MVP，重点是把官方 tracking 主流程接通并提供稳定业务接口。

当前未包含：

- ROS2 集成
- 飞控通信
- 控制器
- 多目标任务策略
- GUI 控制面板
- 更复杂的命令鉴权与反馈机制

## 15. 开发与联调建议

建议联调顺序如下：

1. 先用本地视频文件验证模型和 tracking 是否正常
2. 再切换到 `/dev/video*` 或 RTSP 实时视频
3. 再打开 UDP 接收程序检查 JSON 是否符合预期
4. 再使用命令接口测试 `switch_next / lock_target / unlock_target`
5. 最后再对接后续 ROS2 模块

## 16. 核心实现边界再说明

本目录中最重要的边界是：

- `tracker_runner.py` 不是自写 tracker
- 它只是在官方 `Ultralytics + ByteTrack` 输出外层做一层解析封装
- 主目标逻辑全部在 `target_manager.py`
- 对外通信全部走 UDP(JSON)

如果后续继续扩展，也应保持这个边界，不要把 ByteTrack 底层逻辑重新实现一遍。
