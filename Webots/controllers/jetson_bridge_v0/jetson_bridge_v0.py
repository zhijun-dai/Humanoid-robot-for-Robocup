"""Webots 控制器 — V0 (line_follow_transfer OpenCV 移植)"""
from __future__ import annotations
import json, math, os, sys, time, atexit
import numpy as np

_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson_vision"))
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Supervisor
import cv2
from line_detector_v0_opencv import LineDetector

robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())
MAX_SEC = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 45))

camera = robot.getDevice("camera_ext")
camera.enable(TIMESTEP)
W, H = camera.getWidth(), camera.getHeight()

ld = LineDetector(cam_w=W, cam_h=H)

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

_start_t = None

_log_fh = None
try:
    _log_fh = open(os.path.join(_CTRL_DIR, "..", "..", "..", "generated", "jetson_bridge_v0_log.txt"), "w")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        _log_fh.write(msg + "\n")
        _log_fh.flush()

_log(f"V0 [line_follow_transfer port] camera={W}x{H}")

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()

    raw = camera.getImage()
    if raw is None:
        continue

    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    dev_px, heading_deg, conf, vis, dbg = ld.process(bgr)

    t = robot.getTime()
    if int(t * 4) != int((t - TIMESTEP / 1000.0) * 4):
        _log(f"t={t:.1f}s dev={dev_px}px head={heading_deg}deg conf={conf:.2f}")

    if t - (_start_t or 0) > MAX_SEC:
        _log(f"Done. {t:.1f}s elapsed.")
        robot.simulationQuit()
        break

if _log_fh:
    _log_fh.close()
os._exit(0)
