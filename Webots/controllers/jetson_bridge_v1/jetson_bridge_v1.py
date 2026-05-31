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
# CURVE_SWITCH 已移至检测器, 控制器用 dbg["curve_mode"]
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

    smoothed_err, heading_deg, conf, vis, dbg = ld.process(bgr)

    steer = 0.0
    LOST_HOLD = 6
    LOST_SEARCH = 26.0

    # 重捕获重置
    bl_valid = dbg.get("bottom_lock_valid", False)
    if bl_valid and not pid.get("last_bl_valid", False):
        pid["integral"] = 0.0
        pid["last_err"] = 0.0
    pid["last_bl_valid"] = bl_valid

    if smoothed_err is not None and conf > 0.08:
        pid["lost_frames"] = 0
        dt = TIMESTEP / 1000.0

        # 检测器输出已融合+平滑: near+far+curve+angle → fused → EMA

        if abs(pid["last_steer"]) < STEER_SAT * 0.8:
            pid["integral"] += smoothed_err * dt
        pid["integral"] = max(-I_CLAMP, min(I_CLAMP, pid["integral"]))

        if conf < 0.25:
            pid["integral"] *= 0.85

        derr = (smoothed_err - pid["last_err"]) / max(dt, 1e-3)
        pid["last_err"] = smoothed_err

        # 双模式 PID：用检测器输出的 curve_mode
        curve_mode = dbg.get("curve_mode", 0)
        pid["curve_mode"] = int(curve_mode)
        if curve_mode:
            kp_e, ki_e, kd_e = KP_C, KI_C, KD_C
        else:
            kp_e, ki_e, kd_e = KP, KI, KD

        pid_out = kp_e * smoothed_err + ki_e * pid["integral"] + kd_e * derr
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
        smoothed_err = smoothed_err * 0.9 if smoothed_err else 0.0
    pid["last_steer"] = steer

    speed_scale = 1.0 if conf > 0.3 else (0.75 if conf > 0.1 else 0.6)
    spd = BASE_SPEED * speed_scale

    delta = max(-2.8, min(2.8, steer * STEER_W))
    left.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd - delta)))
    right.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd + delta)))

    t = robot.getTime()
    if int(t * 4) != int((t - TIMESTEP / 1000.0) * 4):
        _log(f"t={t:.1f}s steer={steer:.1f} err={smoothed_err:.3f} head={heading_deg}deg conf={conf:.2f} curve={pid['curve_mode']}")

    if t - (_start_t or 0) > MAX_SEC:
        _log(f"Done. {t:.1f}s elapsed.")
        robot.simulationQuit()
        break

if _log_fh:
    _log_fh.close()
os._exit(0)
