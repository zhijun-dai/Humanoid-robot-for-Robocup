# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RoboCup Humanoid competition — a biped robot follows a black line (white background) on a ~6m closed-loop track, detects QR codes (1-6) to trigger actions, and crosses a red obstacle bar. Chinese-language project. Uses OpenMV H7+ for vision, Webots R2025a for simulation.

## Key Architecture

### OpenMV Camera (vision processing, MicroPython)
- `OpenMV_flash/main.py` — **deployment** entry: line follow + QR detect + protocol V2 (copied to OpenMV U盘)
- `OpenMV_flash/protocol_v2.py` — comms protocol (shared with sim)
- `OpenMV_flash/line_follow_params.json` — **real-robot** params (sim_opencv_distort: false)
- `CVpart/main/main_webots_aligned.py` — source "master" copy aligned with Webots controller
- `CVpart/main/main1.py` — lighter 3-ROI reference implementation
- `CVpart/main/protocol_v2.py` — protocol V2 master source

### Webots Simulation (CPython)
- `Webots/worlds/Robocup.wbt` — simulation world
- `Webots/controllers/line_follow_transfer/line_follow_transfer.py` — main controller (Supervisor for camera shake + lens)
- `Webots/controllers/line_follow_transfer/uart_sink.py` — pluggable byte sink (file/UDP/multi)
- `Webots/controllers/line_follow_transfer/protocol_v2.py` — same protocol, CPython compat

### Configuration (single source of truth)
- **`line_follow_params.json`** (repo root) — ALL parameters: camera, ROI, PID, thresholds, shake, protocol, QR. Both Webots and OpenMV scripts hard-code a search path pointing here. Do NOT casually move it.
- `OpenMV_flash/line_follow_params.json` — deployment copy (subset, no sim-only keys)
- `config/field/场地参数基线.json` — field CAD baseline dimensions
- `config/presets/*.json` — read-only golden snapshots (cp to root to use)
- `config/README.md` — details on config layering

### Communication Protocol V2
- Frame: `0x55 0xAA VER MSG FLAGS SEQ TS_MS(4) LEN PAYLOAD CRC16(2)`
- CRC: CCITT-FALSE (poly=0x1021)
- Key messages: heartbeat (0x01, 10Hz), line_ctrl (0x02, 10Hz), QR_event (0x10), robot_state (0x81)
- line_ctrl payload: mode_u8, route_u8 (1=go, 2=left, 3=right, 4=slight-left), conf_u8, lost_u8, ex_mm_i16, ang_cdeg_i16, v/w reserved
- Spec: `docs/vision_main_protocol_v2.md`, `docs/motor_protocol_v2_p1.md`

### Line Following Algorithm
- Grayscale QQVGA (160x120), Otsu adaptive threshold
- Multiple horizontal scanlines per ROI detecting black/white transitions
- Camera projection model: pitch + height → cm-per-px per row via pinhole
- Detection modes: 3-band (down/mid/up with weights), or simple bottom + assist, or Nx ROI
- Bottom center lock: symmetry-based validation from bottom rows
- Pixel-domain fusion: near_err + far lookahead + curve slope + angle error
- Dual-mode PID (straight vs curve) → nonlinear tanh steer → discrete route_u8
- Shake robustness: diff RMS tracking → adaptive smoothing/Kd/rate-limit
- Startup settling phase (25 frames) with graduated confidence

### QR Detection (integrated in main loop)
- Frame-skipping (every N frames), ROI crop + 2x upscale, lens_corr + retry without
- Multi-frame stability (3 frames) + configurable cooldown (~3.2s) + optional ACK
- Mapped actions: 1=举左手, 2=举右手, 3=抬左腿, 4=抬右腿, 5=举双手, 6=左右摇头

## Key Scripts

| Script | Purpose |
|--------|---------|
| `scripts/auto_tune_webots_params.py` | Black-box parameter search via Webots (--trials, --run-seconds, --apply-best) |
| `scripts/run_shake_eval.py` | Run shake evaluation in Webots |
| `scripts/evaluate_line_follow_log.py` | Grade telemetry log quality |
| `scripts/iterate_line_follow_params.py` | Analyze telemetry to recommend param changes |
| `scripts/calibrate_camera_opencv.py` | Camera intrinsic/extrinsic calibration |
| `scripts/check_camera_consistency.py` | Check calibration sanity |
| `scripts/apply_calibration_to_line_follow_params.py` | Save calibration to params JSON |

## Important Workflows

### Simulation loop
1. Edit `line_follow_params.json` (ROI, PID, thresholds, shake, etc.)
2. Launch Webots world `Webots/worlds/Robocup.wbt` (reads params at startup)
3. Controller writes log lines to stdout + optionally to `generated/` via env `LINE_FOLLOW_LOG_FILE`
4. Run `scripts/evaluate_line_follow_log.py` or `scripts/iterate_line_follow_params.py` on logs

### Parameter auto-tuning
```
python scripts/auto_tune_webots_params.py --trials 24 --run-seconds 16 --apply-best
```

### Deploy to real robot
Copy 3 files to OpenMV U盘 root (alongside boot.py):
- `OpenMV_flash/main.py`
- `OpenMV_flash/protocol_v2.py`
- `OpenMV_flash/line_follow_params.json`

### Sync param changes to OpenMV
Edit `line_follow_params.json` (root) → verify in Webots → copy relevant keys to `OpenMV_flash/line_follow_params.json`. Do NOT copy `sim_opencv_distort: true` to the OpenMV copy.

## Running Tests
```
python -m pytest tests/test_protocol_v2.py -v
python tests/test_protocol_v2.py
```

## Conventions
- All parameters loaded via `_cfg_get(SHARED_CFG, "dotted.path", default)` — JSON config drives behavior, not hardcoded constants (except the _cfg_get itself)
- Telemetry logs use regex `TELEMETRY_RE` for structured parsing (shared between auto-tune, evaluate, iterate scripts)
- Two parallel codebases share algorithm logic: MicroPython (OpenMV_flash) + CPython (Webots controller). Keep `protocol_v2.py` in sync between `CVpart/main/` and `OpenMV_flash/`
- `main_webots_aligned.py` and `line_follow_transfer.py` are designed to be "aligned" — same algorithm, different pixel APIs (sensor vs raw BGRA)
- OpenMV runs on grayscale (no red detection) unless `red_detect_on_grayscale` is explicitly enabled
