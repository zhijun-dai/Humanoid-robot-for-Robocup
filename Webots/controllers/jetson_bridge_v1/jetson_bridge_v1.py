"""Webots controller -- V1 (faithful port of line_follow_transfer with V1 detector)

Ports every control mechanism from:
  Webots/controllers/line_follow_transfer/line_follow_transfer.py
adapted to use line_detector_v1_warp.py for vision processing.

Key features:
  - Dual-mode PID (straight vs curve) with JSON config + env var overrides
  - Pixel-domain fusion with bottom lock blending
  - Shake robustness (diff RMS, adaptive alpha/Kd/rate-limit)
  - Speed control layers (steer slowdown, lost scale, startup ramp, blockers)
  - Lost line recovery (hold + alternating search)
  - Camera shake injection (Supervisor)
  - Configurable via env vars (JETSON_PID_*, LINE_FOLLOW_BASE_SPEED, etc.)
"""

import json
import math
import os
import sys
import atexit
import random

import numpy as np
import cv2

# ---- Path setup ----
_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson_vision"))
_V1_DIR = os.path.join(_JETSON_DIR, "v1_production")
if _V1_DIR not in sys.path:
    sys.path.insert(0, _V1_DIR)
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Supervisor
from line_detector_v1_warp import LineDetector


# ═══════════════════════════════════════════════════════════════════════
# Config loading (mirrors line_follow_transfer.py)
# ═══════════════════════════════════════════════════════════════════════

def _cfg_get(cfg, path, default):
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _load_shared_cfg():
    candidates = [
        os.path.join(_CTRL_DIR, "line_follow_params.json"),
        os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "line_follow_params.json")),
        "line_follow_params.json",
    ]
    for p in candidates:
        try:
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


SHARED_CFG = _load_shared_cfg()


# ═══════════════════════════════════════════════════════════════════════
# Parameters
# ═══════════════════════════════════════════════════════════════════════

# PID (from JSON config; overridable by JETSON_PID_* env vars)
KP_STRAIGHT = float(os.environ.get(
    "JETSON_PID_STRAIGHT_KP", _cfg_get(SHARED_CFG, "pid.straight.kp", 0.83)))
KI_STRAIGHT = float(os.environ.get(
    "JETSON_PID_STRAIGHT_KI", _cfg_get(SHARED_CFG, "pid.straight.ki", 0.004)))
KD_STRAIGHT = float(os.environ.get(
    "JETSON_PID_STRAIGHT_KD", _cfg_get(SHARED_CFG, "pid.straight.kd", 0.095)))
KP_CURVE = float(os.environ.get(
    "JETSON_PID_CURVE_KP", _cfg_get(SHARED_CFG, "pid.curve.kp", 0.98)))
KI_CURVE = float(os.environ.get(
    "JETSON_PID_CURVE_KI", _cfg_get(SHARED_CFG, "pid.curve.ki", 0.002)))
KD_CURVE = float(os.environ.get(
    "JETSON_PID_CURVE_KD", _cfg_get(SHARED_CFG, "pid.curve.kd", 0.115)))
I_CLAMP = float(_cfg_get(SHARED_CFG, "pid.i_clamp", 60.0))

# Steer
STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 45.0))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 0.6))

# Lost recovery
LOST_HOLD_FRAMES = int(_cfg_get(SHARED_CFG, "lost.hold_frames", 6))
LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 11.0))
LOST_PREFER_LEFT = bool(_cfg_get(SHARED_CFG, "lost.prefer_left", True))

# Speed
BASE_SPEED = float(os.environ.get(
    "LINE_FOLLOW_BASE_SPEED", _cfg_get(SHARED_CFG, "webots.base_speed", 6.28)))
STEER_TO_WHEEL = float(_cfg_get(SHARED_CFG, "webots.steer_to_wheel", 0.0184))
MAX_SPEED = float(_cfg_get(SHARED_CFG, "webots.max_speed", 6.28))
SPEED_STEER_SLOWDOWN = float(_cfg_get(SHARED_CFG, "webots.speed_steer_slowdown", 0.62))
SPEED_LOST_SCALE = float(_cfg_get(SHARED_CFG, "webots.speed_lost_scale", 0.92))
SPEED_MIN = float(_cfg_get(SHARED_CFG, "webots.speed_min", 2.6))

# Fusion / Pixel-domain gains (user-specified defaults as per spec)
PIX_LOOKAHEAD_GAIN = float(os.environ.get(
    "JETSON_PIX_LOOKAHEAD_GAIN", _cfg_get(SHARED_CFG, "webots.pixel_lookahead_gain", 0.0)))
PIX_CURVE_GAIN = float(os.environ.get(
    "JETSON_PIX_CURVE_GAIN", _cfg_get(SHARED_CFG, "webots.pixel_curve_gain", 0.0)))
PIX_ANGLE_GAIN = float(os.environ.get(
    "JETSON_PIX_ANGLE_GAIN", _cfg_get(SHARED_CFG, "webots.pixel_angle_gain", 0.06)))

# Curve mode detection
CURVE_SWITCH_PX = float(_cfg_get(SHARED_CFG, "webots.curve_switch_px", 20.0))
RIGHT_TURN_SCALE = float(_cfg_get(SHARED_CFG, "webots.right_turn_scale", 1.0))
LEFT_CURVE_OUTWARD_GAIN = float(_cfg_get(SHARED_CFG, "webots.left_curve_outward_gain", 0.35))
LEFT_CURVE_OUTWARD_PX = float(_cfg_get(SHARED_CFG, "webots.left_curve_outward_px", 6.0))

# Startup / transient
STARTUP_SETTLE_FRAMES = int(_cfg_get(SHARED_CFG, "roi.startup_settle_frames", 60))
STARTUP_SPEED_SCALE = float(_cfg_get(SHARED_CFG, "roi.startup_speed_scale", 0.45))
STARTUP_LOST_BIAS_FREE = bool(_cfg_get(SHARED_CFG, "roi.startup_lost_bias_free", True))

# Bottom lock
BOTTOM_LOCK_BLEND = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_blend", 0.20))
BOTTOM_LOCK_CONF_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_conf_penalty", 0.45))
BOTTOM_LOCK_SPEED_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_speed_penalty", 0.12))
BOTTOM_LOCK_SYM_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_sym_tol_px", 12.0))
LOCK_REACQUIRE_RESET = bool(_cfg_get(SHARED_CFG, "roi.lock_reacquire_reset", True))

# Fusion smoothing
SMOOTH_ALPHA = float(_cfg_get(SHARED_CFG, "fusion.smooth_alpha", 0.74))

# Shake robustness
ROBUST_CFG = _cfg_get(SHARED_CFG, "shake_robust", {}) or {}
ROBUST_ENABLE = bool(ROBUST_CFG.get("enable", True))
ROBUST_DIFF_WINDOW = int(ROBUST_CFG.get("diff_window", 5))
ROBUST_DIFF_RMS_TRIGGER_PX = float(ROBUST_CFG.get("diff_rms_trigger_px", 6.0))
ROBUST_ALPHA_HIGH = float(ROBUST_CFG.get("alpha_high", 0.96))
ROBUST_BOTTOM_LOCK_BLEND_SCALE = float(ROBUST_CFG.get("bottom_lock_blend_scale", 2.2))
ROBUST_KD_SHAKE_SCALE = float(ROBUST_CFG.get("kd_shake_scale", 0.37))
ROBUST_STEER_RATE_LIMIT = float(ROBUST_CFG.get("steer_rate_limit_per_frame", 5.0))
ROBUST_DECAY_FRAMES = int(ROBUST_CFG.get("decay_frames", 14))

# Camera shake injection
SHAKE_CFG = _cfg_get(SHARED_CFG, "shake", {}) or {}
SHAKE_ENABLE = bool(SHAKE_CFG.get("enable", False))
_env_shake = os.environ.get("LINE_FOLLOW_SHAKE", "")
if _env_shake.strip().lower() in ("1", "true", "on", "yes"):
    SHAKE_ENABLE = True
elif _env_shake.strip().lower() in ("0", "false", "off", "no"):
    SHAKE_ENABLE = False
SHAKE_WARMUP_FRAMES = int(SHAKE_CFG.get("warmup_frames", 30))
SHAKE_SEED = int(SHAKE_CFG.get("seed", 17))
SHAKE_DEF = str(SHAKE_CFG.get("camera_pose_def", "CAM_POSE"))
_shake_yaw = SHAKE_CFG.get("yaw", {}) or {}
_shake_pitch = SHAKE_CFG.get("pitch", {}) or {}
_shake_roll = SHAKE_CFG.get("roll", {}) or {}
_shake_xy = SHAKE_CFG.get("xy", {}) or {}
SHAKE_YAW_FREQ = float(_shake_yaw.get("freq_hz", 1.6))
SHAKE_YAW_AMP_RAD = math.radians(float(_shake_yaw.get("amp_deg", 6.0)))
SHAKE_YAW_PHASE_JIT = float(_shake_yaw.get("phase_jit_rad", 0.4))
SHAKE_PITCH_FREQ = float(_shake_pitch.get("freq_hz", 7.0))
SHAKE_PITCH_AMP_RAD = math.radians(float(_shake_pitch.get("amp_deg", 2.5)))
SHAKE_PITCH_NOISE_RAD = math.radians(float(_shake_pitch.get("noise_amp_deg", 1.0)))
SHAKE_ROLL_FREQ = float(_shake_roll.get("freq_hz", 0.7))
SHAKE_ROLL_AMP_RAD = math.radians(float(_shake_roll.get("amp_deg", 3.0)))
SHAKE_XY_NOISE_M = float(_shake_xy.get("noise_cm", 1.5)) / 100.0

# Misc
CAMERA_DEVICE_NAME = str(_cfg_get(SHARED_CFG, "webots.camera.device_name", "camera_ext"))
MAX_SEC = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 45))


# ═══════════════════════════════════════════════════════════════════════
# Utility functions
# ═══════════════════════════════════════════════════════════════════════

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def euler_zyx_to_axis_angle(yaw_rad, pitch_rad, roll_rad):
    """Convert intrinsic ZYX euler angles to Webots SFRotation
    (axis_x, axis_y, axis_z, angle). Mirrors line_follow_transfer.py lines 525-547.
    """
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    qw_clamped = max(-1.0, min(1.0, qw))
    angle = 2.0 * math.acos(qw_clamped)
    s = math.sqrt(max(1e-12, 1.0 - qw_clamped * qw_clamped))
    if s < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    return (qx / s, qy / s, qz / s, angle)


def apply_camera_shake(t_sec, frame_idx, pose_node, base_trs, rng):
    """Inject biped-style head shake on the camera Pose. Mirrors
    line_follow_transfer.py lines 550-577.
    """
    if (not SHAKE_ENABLE) or (pose_node is None) or (frame_idx < SHAKE_WARMUP_FRAMES):
        return
    yaw = SHAKE_YAW_AMP_RAD * math.sin(
        2.0 * math.pi * SHAKE_YAW_FREQ * t_sec
        + SHAKE_YAW_PHASE_JIT * (rng.random() - 0.5)
    )
    pitch = (
        SHAKE_PITCH_AMP_RAD * math.sin(2.0 * math.pi * SHAKE_PITCH_FREQ * t_sec)
        + SHAKE_PITCH_NOISE_RAD * (rng.random() - 0.5) * 2.0
    )
    roll = SHAKE_ROLL_AMP_RAD * math.sin(2.0 * math.pi * SHAKE_ROLL_FREQ * t_sec)
    ax, ay, az, ang = euler_zyx_to_axis_angle(yaw, pitch, roll)
    try:
        rot_field = pose_node.getField("rotation")
        if rot_field is not None:
            rot_field.setSFRotation([ax, ay, az, ang])
    except Exception:
        pass
    if SHAKE_XY_NOISE_M > 0.0:
        try:
            trs_field = pose_node.getField("translation")
            if trs_field is not None:
                nx = (rng.random() - 0.5) * 2.0 * SHAKE_XY_NOISE_M
                ny = (rng.random() - 0.5) * 2.0 * SHAKE_XY_NOISE_M
                trs_field.setSFVec3f(
                    [base_trs[0] + nx, base_trs[1] + ny, base_trs[2]]
                )
        except Exception:
            pass


def nonlinear_map(v):
    """Map PID output to steer via tanh nonlinearity. Mirrors
    line_follow_transfer.py line 1355."""
    return STEER_SAT * math.tanh(v / STEER_SCALE)


def _single_band_mask(mask):
    """True if only one band (0x1=down, 0x2=mid, 0x4=up) has data."""
    return mask in (0x1, 0x2, 0x4)


def pid_step(err, dt, state, curve_mode=False, kd_scale=1.0):
    """Dual-mode PID step. Mirrors line_follow_transfer.py lines 1359-1372."""
    if not curve_mode:
        kp, ki, kd = KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT
    else:
        kp, ki, kd = KP_CURVE, KI_CURVE, KD_CURVE

    kd = kd * float(kd_scale)
    state["integral"] += err * dt
    state["integral"] = clamp(state["integral"], -I_CLAMP, I_CLAMP)

    derr = (err - state["last_err"]) / max(dt, 1e-3)
    state["last_err"] = err

    return kp * err + ki * state["integral"] + kd * derr


# ═══════════════════════════════════════════════════════════════════════
# Camera lens/FOV setup
# ═══════════════════════════════════════════════════════════════════════

def _setup_camera(robot, camera, cfg):
    """Set camera FOV from calibration fx; clear lens to pinhole
    (V1 detector expects pinhole input)."""
    cal = _cfg_get(cfg, "camera.calibration", None)
    if not isinstance(cal, dict):
        return
    try:
        cam_node = robot.getFromDevice(camera)
        if cam_node is None:
            return
        # Clear lens to pinhole -- V1 detector does its own warp
        cam_node.getField("lens").importSFNodeFromString(
            "Lens { center 0.5 0.5 "
            "radialCoefficients [ 0 0 ] "
            "tangentialCoefficients [ 0 0 ] }"
        )
    except Exception:
        pass

    if not cal.get("webots_sync_fov_from_fx", True):
        return
    fx = float(cal.get("fx", 0.0))
    if fx <= 0.0:
        return
    img_w_f = float(camera.getWidth())
    fov_h = 2.0 * math.atan(img_w_f / (2.0 * fx))
    ov = float(cal.get("webots_fov_overscan", 1.0))
    if ov > 0.0:
        fov_h *= ov
    if fov_h <= 0.0 or fov_h >= math.pi:
        return
    try:
        cam_node = robot.getFromDevice(camera)
        if cam_node is not None:
            cam_node.getField("fieldOfView").setSFFloat(fov_h)
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════
# Logging
# ═══════════════════════════════════════════════════════════════════════

_log_fh = None
_LOG_PATH = ""
try:
    _log_dir = os.path.abspath(
        os.path.join(_CTRL_DIR, "..", "..", "..", "generated")
    )
    os.makedirs(_log_dir, exist_ok=True)
    _LOG_PATH = os.path.join(_log_dir, "jetson_bridge_v1_log.txt")
    _log_fh = open(_LOG_PATH, "w", encoding="utf-8")
except Exception:
    pass


def _log(msg):
    print(msg)
    if _log_fh:
        try:
            _log_fh.write(msg + "\n")
            _log_fh.flush()
        except Exception:
            pass


atexit.register(lambda: _log_fh.close() if _log_fh else None)


# ═══════════════════════════════════════════════════════════════════════
# Robot setup
# ═══════════════════════════════════════════════════════════════════════

robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())

camera = robot.getDevice(CAMERA_DEVICE_NAME)
if camera is None:
    raise RuntimeError("No camera device found: %s" % CAMERA_DEVICE_NAME)

_setup_camera(robot, camera, SHARED_CFG)
camera.enable(TIMESTEP)

W, H = camera.getWidth(), camera.getHeight()

# Camera shake pose node
CAM_POSE_NODE = None
CAM_POSE_BASE_TRS = (0.0, 0.0, 0.38)
try:
    CAM_POSE_NODE = robot.getFromDef(SHAKE_DEF)
except Exception:
    CAM_POSE_NODE = None
if CAM_POSE_NODE is not None:
    try:
        trs_field = CAM_POSE_NODE.getField("translation")
        if trs_field is not None:
            base_v = trs_field.getSFVec3f()
            CAM_POSE_BASE_TRS = (
                float(base_v[0]), float(base_v[1]), float(base_v[2])
            )
    except Exception:
        pass
SHAKE_RNG = random.Random(SHAKE_SEED)

# Line detector (V1 warp) -- use camera params from JSON config
cam_height_cm = float(_cfg_get(SHARED_CFG, "camera.height_cm", 40.0))
cam_pitch_deg = float(_cfg_get(SHARED_CFG, "camera.pitch_deg", 45.0))
cam_vfov_deg = float(_cfg_get(SHARED_CFG, "camera.vfov_deg", 43.6))
ld = LineDetector(
    cam_w=W, cam_h=H,
    cam_height_cm=cam_height_cm,
    cam_pitch_deg=cam_pitch_deg,
    cam_vfov_deg=cam_vfov_deg,
)
BIRD_W = ld.bird_w       # 160
IMG_CX = ld.center_x     # 80

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ═══════════════════════════════════════════════════════════════════════
# Controller state (mirrors line_follow_transfer.py state dict)
# ═══════════════════════════════════════════════════════════════════════

ctrl = {
    "integral": 0.0,
    "last_err": 0.0,
    "last_steer": 0.0,
    "smoothed_err": 0.0,
    "lost_frames": 0,
    "last_bottom_lock_valid": False,
    "near_err_history": [],
    "shake_active_frames": 0,
    "diff_rms_px": 0.0,
    "last_print": -1.0,
}

_start_t = None

_log("jetson_bridge_v1: faithful port of line_follow_transfer, "
     "camera=%dx%d timestep=%dms log=%s"
     % (W, H, TIMESTEP, _LOG_PATH))
_log("  PID straight: KP=%.3f KI=%.4f KD=%.3f"
     % (KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT))
_log("  PID curve:    KP=%.3f KI=%.4f KD=%.3f"
     % (KP_CURVE, KI_CURVE, KD_CURVE))
_log("  Steer: SAT=%.1f SCALE=%.2f RIGHT_TURN_SCALE=%.2f"
     % (STEER_SAT, STEER_SCALE, RIGHT_TURN_SCALE))
_log("  Fusion: lookahead=%.3f curve=%.3f angle=%.3f"
     % (PIX_LOOKAHEAD_GAIN, PIX_CURVE_GAIN, PIX_ANGLE_GAIN))
_log("  Speed: base=%.2f min=%.2f steer_slow=%.2f lost_scale=%.2f"
     % (BASE_SPEED, SPEED_MIN, SPEED_STEER_SLOWDOWN, SPEED_LOST_SCALE))
_log("  Lost: hold=%d search=%.1f curve_switch_px=%.1f"
     % (LOST_HOLD_FRAMES, LOST_SEARCH_TURN, CURVE_SWITCH_PX))
_log("  Shake robust: enable=%d alpha_hi=%.2f lock_blend_scale=%.2f "
     "kd_scale=%.2f rate_limit=%.1f"
     % (ROBUST_ENABLE, ROBUST_ALPHA_HIGH, ROBUST_BOTTOM_LOCK_BLEND_SCALE,
        ROBUST_KD_SHAKE_SCALE, ROBUST_STEER_RATE_LIMIT))
_log("  Camera: height=%.1fcm pitch=%.1fdeg vfov=%.2fdeg bird=%dx%d"
     % (cam_height_cm, cam_pitch_deg, cam_vfov_deg, BIRD_W, ld.bird_h))


# ═══════════════════════════════════════════════════════════════════════
# Main loop
# ═══════════════════════════════════════════════════════════════════════

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()

    t_sec = robot.getTime()
    frame_idx = ctrl.get("frame_idx", 0) + 1
    ctrl["frame_idx"] = frame_idx

    # ---- Camera shake injection ----
    apply_camera_shake(
        t_sec, frame_idx, CAM_POSE_NODE, CAM_POSE_BASE_TRS, SHAKE_RNG
    )

    # ---- Grab frame ----
    raw = camera.getImage()
    if raw is None:
        continue

    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    # ---- Run V1 detector ----
    dev_px, heading_deg, conf, vis, dbg = ld.process(bgr)

    # ---- Read detector outputs ----
    n_roi = int(dbg.get("n_roi_results", 0))
    near_err_px = float(dbg.get("near_err_px", 0.0))     # post-hist-blend, pre-lock
    far_err_px_val = float(dbg.get("far_err_px", 0.0))    # raw far band error
    angle_err = float(dbg.get("angle_err_deg", 0.0))
    avg_conf = float(dbg.get("avg_conf", 0.0))
    band_mask = int(dbg.get("band_mask", 0))
    red_block_score = float(dbg.get("red_block_score", 0.0))
    black_block_score = float(dbg.get("black_block_score", 0.0))
    bottom_pair_ratio = float(dbg.get("bottom_pair_ratio", 0.0))
    bottom_sym_err_px = float(dbg.get("bottom_sym_err_px", 0.0))
    bottom_lock_valid = bool(dbg.get("bottom_lock_valid", False))
    center_lock_quality = float(dbg.get("center_lock_quality", 0.0))
    startup_frames = int(dbg.get("startup_frames", 0))
    far_dist_cm = float(dbg.get("far_dist_cm", 0.0))
    black_th = int(dbg.get("black_th", 0))

    startup_active = (
        (STARTUP_SETTLE_FRAMES > 0)
        and (startup_frames < STARTUP_SETTLE_FRAMES)
    )

    # Per-frame variables (initialized for both branches)
    steer = 0.0
    curve_mode = 0
    turn_gate = 0.0
    near_err_blended = near_err_px
    far_err_px_blended = far_err_px_val
    conf_for_control = avg_conf

    # ---- Lock reacquisition reset (mirrors lines 1674-1678) ----
    if (LOCK_REACQUIRE_RESET and bottom_lock_valid
            and (not ctrl["last_bottom_lock_valid"])):
        ctrl["integral"] = 0.0
        ctrl["last_err"] = 0.0
        ctrl["smoothed_err"] *= 0.35
    ctrl["last_bottom_lock_valid"] = bottom_lock_valid

    # ═════════════════════════════════════════════════════════════════
    # TRACKING BRANCH (mirrors lines 1688-1801)
    # ═════════════════════════════════════════════════════════════════
    if n_roi > 0:
        ctrl["lost_frames"] = 0

        # --- Shake robust: use previous-frame shake state ---
        shake_active = ROBUST_ENABLE and (ctrl["shake_active_frames"] > 0)
        alpha_eff = ROBUST_ALPHA_HIGH if shake_active else SMOOTH_ALPHA
        lock_blend_scale = (
            ROBUST_BOTTOM_LOCK_BLEND_SCALE if shake_active else 1.0
        )
        kd_scale_eff = ROBUST_KD_SHAKE_SCALE if shake_active else 1.0

        # --- Bottom lock fusion (mirrors lines 1723-1727) ---
        # near_err_px from V1 is post-historical-blend, pre-lock-fusion
        near_err_blended = near_err_px
        if bottom_pair_ratio > 0.0:
            lock_gain = (
                (BOTTOM_LOCK_BLEND * lock_blend_scale)
                * (0.55 + 0.45 * center_lock_quality)
            )
            lock_gain = clamp(lock_gain, 0.0, 0.95)
            near_err_blended = (
                (1.0 - lock_gain) * near_err_blended
                + lock_gain * bottom_sym_err_px
            )

        # --- Compute curve ---
        far_err_px_blended = far_err_px_val
        curve_px = far_err_px_blended - near_err_blended

        near_norm = near_err_blended / max(0.5 * BIRD_W, 1.0)
        far_norm = far_err_px_blended / max(0.5 * BIRD_W, 1.0)
        curve_norm = curve_px / max(0.5 * BIRD_W, 1.0)

        # Confidence already includes lock penalty from V1 detector
        conf_for_control = avg_conf

        # --- Curve mode detection (mirrors lines 1760-1766) ---
        turn_gate = clamp(
            abs(curve_px) / max(CURVE_SWITCH_PX, 1.0), 0.0, 1.0
        )
        curve_mode_force = abs(curve_px) >= CURVE_SWITCH_PX
        if _single_band_mask(band_mask):
            curve_mode_force = curve_mode_force or (abs(angle_err) >= 8.0)
        if ((not bottom_lock_valid) and (bottom_pair_ratio > 0.0)
                and (abs(bottom_sym_err_px) > BOTTOM_LOCK_SYM_TOL_PX)):
            curve_mode_force = True
        curve_mode = 1 if curve_mode_force else 0

        # --- Pixel-domain fusion (mirrors lines 1768-1775) ---
        fused_err = -near_norm
        fused_err += PIX_LOOKAHEAD_GAIN * (-far_norm)
        fused_err += PIX_CURVE_GAIN * (-curve_norm)
        fused_err += PIX_ANGLE_GAIN * (-angle_err / 45.0)
        if curve_px < -LEFT_CURVE_OUTWARD_PX:
            fused_err += LEFT_CURVE_OUTWARD_GAIN * curve_norm

        # --- Smoothing (mirrors line 1777) ---
        ctrl["smoothed_err"] = (
            alpha_eff * ctrl["smoothed_err"]
            + (1.0 - alpha_eff) * fused_err
        )

        # --- PID step (mirrors lines 1779-1780) ---
        dt_sec = TIMESTEP / 1000.0
        pid_out = pid_step(
            ctrl["smoothed_err"], dt_sec, ctrl,
            curve_mode=curve_mode_force, kd_scale=kd_scale_eff,
        )

        # --- Nonlinear steer map (mirrors line 1780) ---
        steer = nonlinear_map(pid_out)

        # --- Right turn scale (mirrors lines 1781-1782) ---
        if steer < 0.0:
            steer *= RIGHT_TURN_SCALE

        # --- Low-confidence integral decay ---
        if avg_conf < 0.25:
            ctrl["integral"] *= 0.85

        # --- Shake diff RMS tracking (mirrors lines 1786-1797) ---
        hist = ctrl["near_err_history"]
        hist.append(float(near_err_blended))
        if len(hist) > ROBUST_DIFF_WINDOW + 1:
            del hist[0]
        if len(hist) >= 3:
            diffs = [hist[i] - hist[i - 1] for i in range(1, len(hist))]
            rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
            ctrl["diff_rms_px"] = rms
            if rms >= ROBUST_DIFF_RMS_TRIGGER_PX:
                ctrl["shake_active_frames"] = ROBUST_DECAY_FRAMES
            elif ctrl["shake_active_frames"] > 0:
                ctrl["shake_active_frames"] -= 1

    # ═════════════════════════════════════════════════════════════════
    # LOST BRANCH (mirrors lines 1802-1820)
    # ═════════════════════════════════════════════════════════════════
    else:
        ctrl["lost_frames"] += 1
        curve_mode = 1
        turn_gate = 1.0

        if ctrl["lost_frames"] <= LOST_HOLD_FRAMES:
            steer = ctrl["last_steer"] * 0.85
        else:
            if startup_active and STARTUP_LOST_BIAS_FREE:
                phase = (ctrl["lost_frames"] // 8) % 2
                steer = (
                    abs(LOST_SEARCH_TURN) if phase == 0
                    else -abs(LOST_SEARCH_TURN)
                )
            else:
                steer = (
                    abs(LOST_SEARCH_TURN) if LOST_PREFER_LEFT
                    else -abs(LOST_SEARCH_TURN)
                )

        # Integral decay during prolonged loss
        ctrl["integral"] *= 0.5
        ctrl["smoothed_err"] *= 0.9

    # ═════════════════════════════════════════════════════════════════
    # Speed control (mirrors lines 1822-1833)
    # ═════════════════════════════════════════════════════════════════

    steer_norm = abs(steer) / max(STEER_SAT, 1e-6)
    speed_scale = 1.0 - SPEED_STEER_SLOWDOWN * steer_norm
    if ctrl["lost_frames"] > 0:
        speed_scale *= SPEED_LOST_SCALE
    blocker_ratio = clamp(max(red_block_score, black_block_score), 0.0, 1.0)
    speed_scale *= (1.0 - 0.35 * blocker_ratio)
    speed_scale *= (
        1.0 - BOTTOM_LOCK_SPEED_PENALTY * (1.0 - center_lock_quality)
    )
    if STARTUP_SETTLE_FRAMES > 0 and startup_frames < STARTUP_SETTLE_FRAMES:
        warmup = startup_frames / float(STARTUP_SETTLE_FRAMES)
        speed_scale *= (
            STARTUP_SPEED_SCALE
            + (1.0 - STARTUP_SPEED_SCALE) * warmup
        )
    target_base_speed = clamp(
        BASE_SPEED * speed_scale, SPEED_MIN, BASE_SPEED
    )

    # ═════════════════════════════════════════════════════════════════
    # Steer rate limit (mirrors lines 1835-1839)
    # ═════════════════════════════════════════════════════════════════

    if ROBUST_ENABLE and ROBUST_STEER_RATE_LIMIT > 0.0:
        prev_steer = float(ctrl["last_steer"])
        d_steer = steer - prev_steer
        if abs(d_steer) > ROBUST_STEER_RATE_LIMIT:
            steer = prev_steer + math.copysign(
                ROBUST_STEER_RATE_LIMIT, d_steer
            )
    ctrl["last_steer"] = steer

    # ═════════════════════════════════════════════════════════════════
    # Motor commands (mirrors lines 1842-1847)
    # ═════════════════════════════════════════════════════════════════

    delta = clamp(steer * STEER_TO_WHEEL, -2.8, 2.8)
    left_speed = clamp(target_base_speed - delta, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(target_base_speed + delta, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # ═════════════════════════════════════════════════════════════════
    # Logging (mirrors line_follow_transfer telemetry format)
    # ═════════════════════════════════════════════════════════════════

    if t_sec - ctrl["last_print"] > 0.25:
        ctrl["last_print"] = t_sec
        _log(
            "th=%d steer=%.2f near=%.2fpx far=%.2fpx ang=%.2fdeg "
            "conf=%.3f lost=%d curve=%d "
            "L=%.3f R=%.3f spd=%.2f tg=%.3f bmask=%d "
            "red=%.2f blk=%.2f bp=%.2f sym=%.1f lock=%d cq=%.2f "
            "drms=%.2f sk=%d startup=%d n_roi=%d"
            % (
                black_th,
                steer,
                near_err_blended,
                far_err_px_blended,
                angle_err,
                conf_for_control,
                ctrl["lost_frames"],
                curve_mode,
                left_speed,
                right_speed,
                target_base_speed,
                turn_gate,
                band_mask,
                red_block_score,
                black_block_score,
                bottom_pair_ratio,
                bottom_sym_err_px,
                1 if bottom_lock_valid else 0,
                center_lock_quality,
                ctrl["diff_rms_px"],
                ctrl["shake_active_frames"],
                startup_frames,
                n_roi,
            )
        )

    # ═════════════════════════════════════════════════════════════════
    # Time limit
    # ═════════════════════════════════════════════════════════════════

    if t_sec - (_start_t or 0) > MAX_SEC:
        _log("Done. %.1fs elapsed (MAX_SEC=%.0f)." % (t_sec, MAX_SEC))
        robot.simulationQuit(0)
        break

# Cleanup
if _log_fh:
    _log_fh.close()
os._exit(0)
