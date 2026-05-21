"""Webots 控制器 — V2 抛物线拟合版

与 jetson_bridge 完全相同的 PID/控制逻辑，仅 LineDetector 换成 v2 抛物线。
"""
from __future__ import annotations
import json, math, os, sys, time, atexit, random
import numpy as np

_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson_vision"))
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Robot, Supervisor
import cv2

from line_detector_v2_parabola import LineDetector


def _cfg_get(cfg, path, default):
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur

def _load_cfg():
    for p in [
        os.path.join(_CTRL_DIR, "line_follow_params.json"),
        os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "line_follow_params.json")),
        os.path.abspath("line_follow_params.json"),
    ]:
        try:
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}

CFG = _load_cfg()
# ─── Webots 相机参数对齐（world 文件: Pose 0.38m, pitch 45°, FOV 0.9793rad）───
CAM_PITCH = 45.0  # rotation 0 1 0 0.785398 ≈ 45°
CAM_HEIGHT = 38.0
CAM_VFOV = 43.6    # HFOV=56.13° → VFOV=2·atan(tan(28.07°)×240/320)≈43.6°
TRACK_W_CM = 35.5

def _env_or_cfg(cfg, key, default):
    env_key = "JETSON_" + key.upper().replace(".", "_")
    if env_key in os.environ:
        return float(os.environ[env_key])
    return float(_cfg_get(cfg, key, default))

KP = _env_or_cfg(CFG, "pid.straight.kp", 0.35)
KI = _env_or_cfg(CFG, "pid.straight.ki", 0.012)
KD = _env_or_cfg(CFG, "pid.straight.kd", 0.08)
I_CLAMP = float(_cfg_get(CFG, "pid.i_clamp", 60.0))
KP_C = _env_or_cfg(CFG, "pid.curve.kp", 0.55)
KI_C = _env_or_cfg(CFG, "pid.curve.ki", 0.006)
KD_C = _env_or_cfg(CFG, "pid.curve.kd", 0.10)
STEER_SAT = _env_or_cfg(CFG, "steer.sat", 45.0)
STEER_SCALE = _env_or_cfg(CFG, "steer.scale", 1.0)
BASE_SPEED = float(_cfg_get(CFG, "webots.base_speed", 3.2))
STEER_W = float(_cfg_get(CFG, "webots.steer_to_wheel", 0.04))
MAX_SPEED = float(_cfg_get(CFG, "webots.max_speed", 6.28))

robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())
MAX_SEC = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 45))

camera = robot.getDevice("camera_ext")
camera.enable(TIMESTEP)
W, H = camera.getWidth(), camera.getHeight()

# 相机抖动暂关闭
# CAM_SHAKE_PITCH = 0.025
# CAM_SHAKE_YAW = 0.015

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

ld = LineDetector(cam_height_cm=CAM_HEIGHT, cam_pitch_deg=CAM_PITCH,
                  cam_vfov_deg=CAM_VFOV, cam_w=W, cam_h=H, th_offset=6)

pid = {"integral": 0.0, "last_err": 0.0, "last_steer": 0.0,
       "lost_frames": 0, "smoothed_err": 0.0}
_start_t = None

_log_fh = None
try:
    _log_fh = open(os.path.join(_CTRL_DIR, "..", "..", "..", "generated", "jetson_bridge_v2_log.txt"), "w")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        _log_fh.write(msg + "\n")
        _log_fh.flush()

_log(f"V2 [parabola] camera={W}x{H} pitch={CAM_PITCH}deg height={CAM_HEIGHT}cm")
_log(f"  PID kp={KP} ki={KI} kd={KD}  parabola fit")

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()

    raw = camera.getImage()
    if raw is None:
        continue

    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    dev_px, heading_deg, conf, vis, dbg = ld.process(bgr)

    # 前 3 帧保存鸟瞰图调试
    t = robot.getTime()
    if not hasattr(ld, '_dbg_snap_cnt'): ld._dbg_snap_cnt = 0
    if t > 2.0 and ld._dbg_snap_cnt < 3:
        path = os.path.join(_CTRL_DIR, "..", "..", "..", "generated",
                            f"v2_dbg_{ld._dbg_snap_cnt}.png")
        cv2.imwrite(path, vis)
        _log(f"  [debug] saved {path}")
        ld._dbg_snap_cnt += 1

    steer = 0.0
    fused_err = 0.0
    LOST_HOLD = 6
    LOST_SEARCH = 26.0

    if dev_px is not None and conf > 0.08:
        pid["lost_frames"] = 0
        dt = TIMESTEP / 1000.0

        near_norm = dev_px / 80.0
        fused_err = -near_norm
        # 朝向只给一点点前馈（抛物线朝向噪声较大）
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

        pid_out = KP * pid["smoothed_err"] + KI * pid["integral"] + KD * derr
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
        _log(f"t={t:.1f}s steer={steer:.1f} dev={dev_px}px head={heading_deg}deg conf={conf:.2f} err={fused_err:.2f}")

    if t - (_start_t or 0) > MAX_SEC:
        _log(f"Done. {t:.1f}s elapsed.")
        robot.simulationQuit()
        break

if _log_fh:
    _log_fh.close()
