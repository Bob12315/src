# 运行手册

以下命令默认工作目录为：

```bash
cd ~/uav_project/src
```

## 1. 启动 YOLO

YOLO 建议在独立 conda 环境中运行：

```bash
conda activate yolo
cd ~/uav_project/src/yolo_app
python main.py
```

确认 `yolo_app/config.yaml` 中 UDP 输出端口与 `config/app.yaml` 一致。控制端默认监听：

```yaml
runtime:
  yolo_udp_ip: "0.0.0.0"
  yolo_udp_port: 5005
```

## 2. app dry-run，不连接飞控

```bash
conda activate uav-control
cd ~/uav_project/src
python -m app.main --send-commands false
```

自动退出 smoke test：

```bash
python -m app.main --no-yolo-udp --run-seconds 2 --send-commands false
```

## 3. app 连接 telemetry，但不发控制

```bash
python -m app.main --connect-telemetry --send-commands false
```

如果 SITL/飞控尚未启动，telemetry 会在后台重连，app 主循环仍会运行。

## 4. app + UI

```bash
python -m app.main --connect-telemetry --ui --send-commands false
```

这是 curses 终端 UI，不是网页 GUI。当前 UI 依赖 `LinkManager`，所以需要 `--connect-telemetry`。

## 5. 独立 telemetry 服务

```bash
python -m telemetry_link.main --config config/telemetry.yaml
```

打开 telemetry UI：

```bash
python -m telemetry_link.main --config config/telemetry.yaml --ui
```

## 6. SITL 低风险顺序

1. 启动 SITL。
2. 启动 YOLO 或用 `--no-yolo-udp` 做空输入测试。
3. 运行：

```bash
python -m app.main --connect-telemetry --ui --send-commands false
```

4. 确认状态正常后，只测某个模式：

```bash
python -m app.main --connect-telemetry --force-mode APPROACH_TRACK --send-commands false
```

5. 确认 raw/shaped 命令方向正确后，再打开实发：

```bash
python -m app.main --connect-telemetry --force-mode APPROACH_TRACK --send-commands true
```

## 7. 常用组合

只看 YOLO/fusion/control 计算，不连飞控：

```bash
python -m app.main --send-commands false
```

连飞控但不发命令：

```bash
python -m app.main --connect-telemetry --send-commands false
```

强制 overhead dry-run：

```bash
python -m app.main --connect-telemetry --force-mode OVERHEAD_HOLD --send-commands false
```

禁用 YOLO UDP：

```bash
python -m app.main --no-yolo-udp --run-seconds 2 --send-commands false
```

## 8. 排查提示

没有 UI：

- 确认是否传了 `--ui`。
- 确认是否传了 `--connect-telemetry`。

收不到 YOLO：

- 检查 `yolo_app/config.yaml` 的 UDP 目标 IP/端口。
- 检查 `config/app.yaml` 的 `yolo_udp_port`。
- 本机运行时建议 YOLO 发到 `127.0.0.1:5005`。

telemetry 连接不上：

- SITL 默认检查 `config/telemetry.yaml` 的 `sitl.tcp_port`。
- 当前默认是 `127.0.0.1:5762`。
- 没开 SITL 时看到 reconnect warning 是正常的。
