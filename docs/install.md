# 安装说明

建议分两个环境：控制环境和 YOLO 环境。

## 控制环境

```bash
conda create -n uav-control python=3.10 -y
conda activate uav-control
pip install pymavlink pyyaml pytest
```

用途：

- `app`
- `flight_modes`
- `fusion`
- `telemetry_link`
- `uav_ui`
- `tests`

验证：

```bash
cd ~/uav_project/src
python -m app.main --help
python -m telemetry_link.main --help
python -m pytest -q
```

## YOLO 环境

```bash
conda create -n yolo python=3.10 -y
conda activate yolo
pip install ultralytics opencv-python pyyaml
```

如果使用 GPU，请按本机 CUDA 版本安装合适的 PyTorch。可以先确认：

```bash
python - <<'PY'
import torch
print(torch.__version__)
print(torch.cuda.is_available())
PY
```

## 模型文件

建议路径：

```text
~/models/best.pt
```

然后在 `yolo_app/config.yaml` 中配置：

```yaml
model_path: "~/models/best.pt"
```

不建议把 `.pt` 模型提交到 Git。

## 可选依赖

如果要运行视频源、GStreamer 或特定相机，可能还需要系统包。具体取决于硬件和视频输入方式。

## 快速验证

控制环境：

```bash
conda activate uav-control
cd ~/uav_project/src
python -m app.main --no-yolo-udp --run-seconds 1 --send-commands false
```

YOLO 环境：

```bash
conda activate yolo
cd ~/uav_project/src/yolo_app
python main.py
```
