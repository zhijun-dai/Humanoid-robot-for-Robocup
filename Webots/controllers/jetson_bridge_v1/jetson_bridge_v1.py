"""Webots 控制器 — V1 (warp + V0 全套检测机制)"""
from __future__ import annotations
import json, math, os, sys, time, atexit
import numpy as np

_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson_vision"))
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Supervisor
import cv2
from line_detector_v1_warp import LineDetector

robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())
MAX_SEC = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 45))

camera = robot.getDevice("camera_ext")
camera.enable(TIMESTEP)
W, H = camera.getWidth(), camera.getHeight()

ld = LineDetector(cam_w=W, cam_h=H, cam_height_cm=38, cam_pitch_deg=45, cam_vfov_deg=43.6)

def _env_or_cfg(key, default):
    env_key = "JETSON_" + key.upper().replace(".", "_")
    if env_key in os.environ:
        return float(os.environ[env_key])
    return float(default)

# 双模式 PID（直道/弯道）
KP = _env_or_cfg("pid.straight.kp", 0.5)
KI = _env_or_cfg("pid.straight.ki", 0.004)
KD = _env_or_cfg("pid.straight.kd", 0.10)
KP_C = _env_or_cfg("pid.curve.kp", 1.05)
KI_C = _env_or_cfg("pid.curve.ki", 0.008)
KD_C = _env_or_cfg("pid.curve.kd", 0.18)
CURVE_SWITCH_DEG = 15.0   # heading 超过 15° 切弯道 PID
RIGHT_TURN_SCALE = 0.65    # 右转不对称修正
I_CLAMP = 60.0
STEER_SAT = _env_or_cfg("steer.sat", 45.0)
STEER_SCALE = _env_or_cfg("steer.scale", 0.9)
BASE_SPEED = 6.4
STEER_W = 0.04
MAX_SPEED = 6.28

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

pid = {"integral": 0.0, "last_err": 0.0, "last_steer": 0.0,
       "lost_frames": 0, "smoothed_err": 0.0, "curve_mode": 0}
_start_t = None

_log_fh = None
try:
    _log_fh = open(os.path.join(_CTRL_DIR, "..", "..", "..", "generated", "jetson_bridge_v1_log.txt"), "w")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        _log_fh.write(msg + "\n")
        _log_fh.flush()

_log(f"V1 [warp+band-scan] camera={W}x{H}")

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()

    raw = camera.getImage()
    if raw is None:
        continue

    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    dev_px, heading_deg, conf, vis, dbg = ld.process(bgr)

    steer = 0.0
    fused_err = 0.0
    LOST_HOLD = 6
    LOST_SEARCH = 26.0

    if dev_px is not None and conf > 0.08:
        pid["lost_frames"] = 0
        dt = TIMESTEP / 1000.0

        near_norm = dev_px / 80.0
        fused_err = -near_norm
        if abs(heading_deg) < 15:
            fused_err += 0.015 * (-heading_deg / 45.0)

        SMOOTH_ALPHA = 0.75 if conf > 0.4 else 0.88
        pid["smoothed_err"] = SMOOTH_ALPHA * pid["smoothed_err"] + (1.0 - SMOOTH_ALPHA) * fused_err

        if abs(pid["last_steer"]) < STEER_SAT * 0.8:
            pid["integral"] += pid["smoothed_err"] * dt
        pid["integral"] = max(-I_CLAMP, min(I_CLAMP, pid["integral"]))

        if conf < 0.25:
            pid["integral"] *= 0.85

        derr = (pid["smoothed_err"] - pid["last_err"]) / max(dt, 1e-3)
        pid["last_err"] = pid["smoothed_err"]

        # 双模式 PID：弯道时切更高 KP/KD（旧代码逻辑）
        curve_mode = abs(heading_deg) >= CURVE_SWITCH_DEG
        pid["curve_mode"] = int(curve_mode)
        if curve_mode:
            kp_e, ki_e, kd_e = KP_C, KI_C, KD_C
        else:
            kp_e, ki_e, kd_e = KP, KI, KD

        pid_out = kp_e * pid["smoothed_err"] + ki_e * pid["integral"] + kd_e * derr
        steer = STEER_SAT * math.tanh(pid_out / STEER_SCALE)

        max_ds = 14.0 * dt * 30
        ds = steer - pid["last_steer"]
        if abs(ds) > max_ds:
            steer = pid["last_steer"] + max_ds * (1 if ds > 0 else -1)
    else:
        pid["lost_frames"] = pid.get("lost_frames", 0) + 1
        lost_n = pid["lost_frames"]
        if lost_n <= LOST_HOLD:
            steer = pid["last_steer"] * 0.85
        else:
            phase = (lost_n // 8) % 2
            steer = LOST_SEARCH if phase == 0 else -LOST_SEARCH
            pid["integral"] *= 0.5
        pid["smoothed_err"] *= 0.9
    pid["last_steer"] = steer

    speed_scale = 1.0 if conf > 0.3 else (0.75 if conf > 0.1 else 0.6)
    spd = BASE_SPEED * speed_scale

    delta = max(-2.8, min(2.8, steer * STEER_W))
    left.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd - delta)))
    right.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd + delta)))

    t = robot.getTime()
    if int(t * 4) != int((t - TIMESTEP / 1000.0) * 4):
        _log(f"t={t:.1f}s steer={steer:.1f} dev={dev_px}px head={heading_deg}deg conf={conf:.2f} err={fused_err:.2f} curve={pid['curve_mode']}")

    if t - (_start_t or 0) > MAX_SEC:
        _log(f"Done. {t:.1f}s elapsed.")
        robot.simulationQuit()
        break

if _log_fh:
    _log_fh.close()
os._exit(0)
