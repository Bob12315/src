# AI PID Tuning Guide For `debug_gimbal.py`

This document is written for AI agents that need to tune the PID parameters of [debug_gimbal.py](/home/level6/uav_project/src/control/debug/debug_gimbal.py:1).

## Goal

Tune the gimbal so that it:

- moves quickly toward the target when the target is off center
- slows down near the image center
- avoids visible oscillation near center
- does not keep drifting after the target is centered
- stops cleanly when the target is lost or invalid

## Control Style

`debug_gimbal.py` is intentionally designed in an OpenMV-like style:

- use image error directly
- compute yaw/pitch rate commands every loop
- send gimbal rate through `send_gimbal_rate(...)`

This is not an angle planner. It is a direct visual servo controller.

## Input Interface

The controller reads data through the existing control debug pipeline:

- `read_control_input(...)`
- `ControlInputAdapter`
- `ControlModeManager`

The fields that matter most for PID tuning are:

- `control_input.ex_cam`
  horizontal normalized image error
- `control_input.ey_cam`
  vertical normalized image error
- `control_input.dt`
  loop delta time in seconds
- `control_input.target_valid`
- `control_input.vision_valid`
- `control_input.gimbal_valid`
- `control_input.gimbal_yaw`
- `control_input.gimbal_pitch`

Expected error meaning:

- target centered: `ex_cam ~= 0`, `ey_cam ~= 0`
- target on one side: corresponding error grows in magnitude

## Output Interface

The controller sends:

- `link_manager.send_gimbal_rate(yaw_rate=..., pitch_rate=...)`

Units:

- internal output units are `rad/s`
- `telemetry_link` converts these to the AP gimbal-rate command path

## Main Tunable Args

Yaw axis:

- `--pid-yaw-kp`
- `--pid-yaw-ki`
- `--pid-yaw-kd`
- `--pid-yaw-max-rate`
- `--pid-yaw-derivative-limit`
- `--center-hold-yaw-threshold`

Pitch axis:

- `--pid-pitch-kp`
- `--pid-pitch-ki`
- `--pid-pitch-kd`
- `--pid-pitch-max-rate`
- `--pid-pitch-derivative-limit`
- `--center-hold-pitch-threshold`

Shared:

- `--axis both|yaw|pitch`
- `--deadband`
- `--pid-integral-limit`
- `--respect-control-gating`
- `--dry-run`

## Parameter Meaning

`kp`

- primary response strength
- larger `kp` makes tracking faster
- too large causes overshoot and shaking

`ki`

- removes small steady-state bias
- useful when target stays slightly off-center
- too large causes slow oscillation and drift buildup

`kd`

- damps fast changes
- helps reduce overshoot
- too large makes motion noisy or hesitant

`deadband`

- small image errors inside this region are forced to zero
- useful for reducing jitter
- too large leaves visible center error

`center-hold-*`

- when error is very small, explicitly hold that axis at zero rate
- stronger than `deadband` for preventing center hunting

`pid-integral-limit`

- clamps the integral term
- protects against windup when the target is lost or motion saturates

`pid-*-max-rate`

- hard output limit
- use this to cap aggressiveness even if PID output gets large

`pid-*-derivative-limit`

- clamps derivative spikes from noisy detections or frame jumps

## Recommended Tuning Order

Always tune one axis at a time.

Recommended order:

1. Tune yaw first
2. Tune pitch second
3. Add a little `ki` only after `kp` and `kd` are stable

## Safe Starting Method

1. Start with `ki = 0`
2. Start with small `kd`
3. Increase `kp` until the axis becomes responsive
4. If overshoot appears, add a little `kd`
5. Only then add a small `ki` if there is steady center bias

Good practical approach:

1. Set `--pid-yaw-ki 0` and `--pid-pitch-ki 0`
2. Raise `kp` until tracking is fast but slightly too lively
3. Add `kd` until overshoot becomes acceptable
4. Add a very small `ki` only if center bias remains
5. Revisit `deadband` and `center-hold-*` last

## Symptom To Action Map

If the target returns too slowly:

- increase `kp`
- possibly increase `max-rate`

If the target overshoots badly:

- reduce `kp`
- increase `kd`
- reduce `max-rate`

If the target shakes near center:

- increase `deadband`
- increase `center-hold-*`
- reduce `kp`
- reduce `kd` if noise amplification is obvious

If the target settles with a constant offset:

- add a small `ki`
- or slightly reduce deadband / center-hold threshold if they are too large

If the controller slowly drifts after long tracking:

- reduce `ki`
- reduce `pid-integral-limit`

If commands spike on detection jumps:

- reduce `kd`
- reduce derivative limit

If large errors are corrected too softly but center behavior is good:

- increase `max-rate`
- then, if needed, slightly increase `kp`

Current debug-script default max rates are set 3x higher than the original safe
starting point:

- yaw max rate: `3.3 rad/s`
- pitch max rate: `2.55 rad/s`

## Practical Notes For This Script

The script has two gating modes:

- default debug mode:
  only requires valid vision, target, and gimbal feedback
- `--respect-control-gating`:
  follows the main pipeline gating logic

For early PID tuning, prefer the default debug mode because it is simpler.

Use `--dry-run` first when validating sign and log behavior.

Use `--axis yaw` while tuning yaw and `--axis pitch` while tuning pitch. The inactive
axis is forced to zero at the sender, so one axis can be evaluated without the other
axis moving the target.

## What To Watch In Logs

The script prints:

- `ex_cam`
- `ey_cam`
- `dt`
- `gimbal_yaw`
- `gimbal_pitch`
- `yaw_rate_cmd`
- `pitch_rate_cmd`
- mode and hold reason

When tuning, look for:

- error decreases after command is applied
- command sign changes sensibly around center
- command returns near zero near center
- no repeated large sign flips near center

## Sign Check

Before tuning gains deeply, confirm sign is correct.

Correct behavior:

- when target stays to one side, command should move the gimbal so that the error magnitude shrinks over time

Wrong-sign indicators:

- target moves farther from center after command
- error magnitude grows immediately
- command saturates and tracking gets worse

If sign is wrong, do not tune PID first. Fix axis sign in config:

- `runtime.control_cfg.gimbal.yaw_sign`
- `runtime.control_cfg.gimbal.pitch_sign`

## Suggested Test Sequence

1. Hold the vehicle still
2. Move target left/right only and tune yaw
3. Move target up/down only and tune pitch
4. Test diagonal motion
5. Test target entering from image edge
6. Test short target loss and reacquire

## Minimal Example Command

```bash
python -m control.debug.debug_gimbal \
  --axis yaw \
  --pid-yaw-kp 3.2 \
  --pid-yaw-ki 0.00 \
  --pid-yaw-kd 0.18 \
  --pid-pitch-kp 1.8 \
  --pid-pitch-ki 0.00 \
  --pid-pitch-kd 0.12 \
  --pid-yaw-max-rate 3.3 \
  --pid-pitch-max-rate 2.55
```

## AI Editing Rules

When an AI updates PID values in `debug_gimbal.py` or runtime launch commands:

- change as few parameters as possible per iteration
- record what symptom motivated each change
- tune yaw and pitch separately unless both clearly share the same issue
- prefer changing `kp`, `kd`, `max-rate`, and deadband before adding more `ki`
- do not treat oscillation as a reason to blindly increase `ki`
- if behavior is unstable, reduce aggressiveness first

## Success Criteria

A good tune should satisfy all of these:

- target recenters quickly
- no sustained oscillation near center
- little or no steady-state bias
- commands go near zero when centered
- target loss does not leave continued motion
