from __future__ import annotations

from collections.abc import Callable

from command_dispatcher import CommandResult, dispatch_text_command
from link_manager import LinkManager

from uav_ui.control_switches import ControlRuntimeSwitches, ControlSwitchSnapshot
from uav_ui.yolo_command_client import YoloCommandClient


def build_ui_command_handler(
    manager: LinkManager,
    *,
    controller_switches: ControlRuntimeSwitches | None = None,
    yolo_client: YoloCommandClient | None = None,
) -> Callable[[str], CommandResult]:
    def _handle(command: str) -> CommandResult:
        own_result = _dispatch_ui_command(
            command,
            controller_switches=controller_switches,
            yolo_client=yolo_client,
        )
        if own_result is not None:
            return own_result
        return dispatch_text_command(manager, command)

    return _handle


def _dispatch_ui_command(
    command: str,
    *,
    controller_switches: ControlRuntimeSwitches | None,
    yolo_client: YoloCommandClient | None,
) -> CommandResult | None:
    parts = command.strip().split()
    if not parts:
        return CommandResult(False, "empty command")

    root = parts[0].lower()
    if root in {"controller", "controllers"}:
        return _dispatch_controller_command(parts, controller_switches)
    if root == "control":
        return _dispatch_control_command(parts, controller_switches)
    if root == "target":
        return _dispatch_target_command(parts, yolo_client)
    return None


def _dispatch_controller_command(
    parts: list[str],
    controller_switches: ControlRuntimeSwitches | None,
) -> CommandResult:
    if controller_switches is None:
        return CommandResult(False, "controller switching is not available in this UI")
    if len(parts) != 3:
        return CommandResult(False, "format: controller <gimbal|body|approach|all> <on|off|toggle>")
    name = parts[1].lower()
    action = parts[2].lower()
    if name not in {"gimbal", "body", "approach", "all"}:
        return CommandResult(False, "controller name must be gimbal, body, approach, or all")
    if action in {"on", "enable", "enabled", "1", "true"}:
        snapshot = controller_switches.set_controller(name, True)
    elif action in {"off", "disable", "disabled", "0", "false"}:
        snapshot = controller_switches.set_controller(name, False)
    elif action in {"toggle", "tog"}:
        snapshot = controller_switches.toggle_controller(name)
    else:
        return CommandResult(False, "controller action must be on, off, or toggle")
    return CommandResult(True, f"controllers {format_controller_snapshot(snapshot)}")


def _dispatch_control_command(
    parts: list[str],
    controller_switches: ControlRuntimeSwitches | None,
) -> CommandResult:
    if controller_switches is None:
        return CommandResult(False, "control runtime switching is not available in this UI")
    if len(parts) != 3 or parts[1].lower() not in {"send", "send_commands", "commands"}:
        return CommandResult(False, "format: control send <on|off|toggle>")
    action = parts[2].lower()
    if action in {"on", "enable", "enabled", "1", "true"}:
        snapshot = controller_switches.set_send_commands(True)
    elif action in {"off", "disable", "disabled", "0", "false"}:
        snapshot = controller_switches.set_send_commands(False)
    elif action in {"toggle", "tog"}:
        snapshot = controller_switches.toggle_send_commands()
    else:
        return CommandResult(False, "control send action must be on, off, or toggle")
    return CommandResult(True, f"control send_commands={'ON' if snapshot.send_commands else 'OFF'}")


def _dispatch_target_command(
    parts: list[str],
    yolo_client: YoloCommandClient | None,
) -> CommandResult:
    if yolo_client is None:
        return CommandResult(False, "target switching is not available in this UI")
    if len(parts) < 2:
        return CommandResult(False, "format: target <next|prev|lock <track_id>|unlock>")
    action = parts[1].lower()
    try:
        if action == "next":
            yolo_client.send("switch_next")
            return CommandResult(True, "target switch_next sent")
        if action in {"prev", "previous"}:
            yolo_client.send("switch_prev")
            return CommandResult(True, "target switch_prev sent")
        if action == "unlock":
            yolo_client.send("unlock_target")
            return CommandResult(True, "target unlock_target sent")
        if action == "lock":
            if len(parts) != 3:
                return CommandResult(False, "format: target lock <track_id>")
            track_id = int(parts[2])
            yolo_client.send("lock_target", track_id=track_id)
            return CommandResult(True, f"target lock_target sent track_id={track_id}")
    except ValueError:
        return CommandResult(False, "target lock track_id must be an integer")
    except Exception as exc:
        return CommandResult(False, f"target command failed: {exc}")
    return CommandResult(False, "target action must be next, prev, lock, or unlock")


def format_controller_snapshot(snapshot: ControlSwitchSnapshot) -> str:
    return (
        f"G={'ON' if snapshot.gimbal else 'OFF'} "
        f"B={'ON' if snapshot.body else 'OFF'} "
        f"A={'ON' if snapshot.approach else 'OFF'} "
        f"SEND={'ON' if snapshot.send_commands else 'OFF'}"
    )
