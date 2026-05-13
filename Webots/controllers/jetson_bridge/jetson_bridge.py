"""Webots 控制器 — 使用 jetson_vision 模块（仿真+真机同一套代码）

与 OpenMV 的 line_follow_transfer 并列存在，不影响旧方案。
"""
from __future__ import annotations
import json, math, os, sys, time, atexit
import numpy as np

# ── 路径设置：让 Webots 找到 jetson_vision 模块 ──
_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson_vision"))
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Robot  # type: ignore
import cv2

from qr_detector import QRDetector
from line_detector import LineDetector


# ── 配置加载（与旧方案同一份 line_follow_params.json） ──
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

CAM_PITCH = float(_cfg_get(CFG, "camera.pitch_deg", 45.0))
CAM_HEIGHT = float(_cfg_get(CFG, "camera.height_cm", 40.0))
TRACK_W_CM = 35.0  # 赛道宽 350mm

# PID（沿用旧参数，因为控制输入还是归一化偏差）
KP = float(_cfg_get(CFG, "pid.straight.kp", 0.70))
KI = float(_cfg_get(CFG, "pid.straight.ki", 0.015))
KD = float(_cfg_get(CFG, "pid.straight.kd", 0.12))
I_CLAMP = float(_cfg_get(CFG, "pid.i_clamp", 60.0))
KP_C = float(_cfg_get(CFG, "pid.curve.kp", 1.05))
KI_C = float(_cfg_get(CFG, "pid.curve.ki", 0.008))
KD_C = float(_cfg_get(CFG, "pid.curve.kd", 0.18))

BASE_SPEED = float(_cfg_get(CFG, "webots.base_speed", 3.2))
STEER_W = float(_cfg_get(CFG, "webots.steer_to_wheel", 0.04))
MAX_SPEED = float(_cfg_get(CFG, "webots.max_speed", 6.28))

# ── 初始化 ──
robot = Robot()
TIMESTEP = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera_ext")
camera.enable(TIMESTEP)
W, H = camera.getWidth(), camera.getHeight()

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

# ── jetson_vision 检测器 ──
qr = QRDetector(stable_frames=1, cooldown_ms=2000, min_edge_px=20, max_edge_px=300, debug=True)
ld = LineDetector(cam_height_cm=CAM_HEIGHT, cam_pitch_deg=CAM_PITCH, cam_w=W, cam_h=H, track_width_cm=TRACK_W_CM)

# ── PID 状态 ──
pid = {"integral": 0.0, "last_err": 0.0, "last_steer": 0.0, "smoothed_err": 0.0}
_start_t = None

# ── 日志 ──
_log_fh = None
try:
    _log_fh = open(os.path.join(_CTRL_DIR, "..", "..", "..", "generated", "jetson_bridge_log.txt"), "w")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        _log_fh.write(msg + "\n")
        _log_fh.flush()

_log(f"[jetson_bridge] camera={W}x{H} pitch={CAM_PITCH}deg height={CAM_HEIGHT}cm")
_log(f"  PID kp={KP} ki={KI} kd={KD}  line_det: warp 160x200")

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()
    raw = camera.getImage()
    if raw is None:
        continue

    # BGRA -> BGR（和真机 USB 摄像头一样）
    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    # ── 巡线 ──
    dev_px, conf, vis, dbg = ld.process(bgr)

    # 归一化偏差 + PID
    steer = 0.0
    curve = False
    if dev_px is not None and conf > 0.12:
        err = -dev_px / 80.0
        pid["smoothed_err"] = 0.65 * pid["smoothed_err"] + 0.35 * err
        dt = TIMESTEP / 1000.0
        pid["integral"] += err * dt
        pid["integral"] = max(-I_CLAMP, min(I_CLAMP, pid["integral"]))
        derr = (err - pid["last_err"]) / max(dt, 1e-3)
        pid["last_err"] = err
        kp, ki, kd = (KP_C, KI_C, KD_C) if curve else (KP, KI, KD)
        steer = 70.0 * math.tanh((kp * err + ki * pid["integral"] + kd * derr) / 22.0)
    else:
        steer = pid["last_steer"] * 0.85
    pid["last_steer"] = steer

    # 速度 + 差速
    spd = BASE_SPEED
    delta = max(-2.8, min(2.8, steer * STEER_W))
    left.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd - delta)))
    right.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd + delta)))

    # ── QR 检测 ──
    action, qdbg = qr.update(bgr)

    # ── 红条检测 ──
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255)) | cv2.inRange(hsv, (160, 80, 80), (180, 255, 255))
    red_r = cv2.countNonZero(mask) / (W * H)

    # ── 日志（4Hz） ──
    t = robot.getTime()
    if int(t * 4) != int((t - TIMESTEP / 1000.0) * 4):
        _log(f"t={t:.1f}s steer={steer:.1f} dev={dev_px:.1f}px conf={conf:.2f} red={red_r:.3f}")

    # 超时退出
    if t - (_start_t or 0) > 45:
        _log(f"Done. {t:.1f}s elapsed.")
        break

if _log_fh:
    _log_fh.close()
