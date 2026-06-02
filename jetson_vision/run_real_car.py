"""Standalone real-car controller — V1 detector + PID + 4-wheel differential drive.

USB camera → LineDetector → dual-mode PID + lost recovery → 4 wheel speeds (rad/s).
Optional serial output to MCU at 115200 baud, ~10 Hz.
"""

import math, os, sys, time, json, atexit
import numpy as np
import cv2

# ── Path to V1 detector ──
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
_V1_DIR = os.path.join(_SCRIPT_DIR, "v1_production")
if _V1_DIR not in sys.path:
    sys.path.insert(0, _V1_DIR)

from line_detector_v1_warp import LineDetector


def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v


# ═══════════════════════════════════════════════════════════════════════
# Config — all overridable via env vars
# ═══════════════════════════════════════════════════════════════════════

# Camera
CAM_IDX  = int(os.environ.get("CAM_IDX", "1"))
CAM_W    = 1280
CAM_H    = 720

# Detector
CAM_HEIGHT_CM  = float(os.environ.get("CAM_HEIGHT_CM",  "40.0"))
CAM_PITCH_DEG  = float(os.environ.get("CAM_PITCH_DEG",  "45.0"))
CAM_VFOV_DEG   = float(os.environ.get("CAM_VFOV_DEG",   "44.0"))

# PID — dual-mode (straight / curve)
KP_S = float(os.environ.get("JETSON_PID_STRAIGHT_KP", "0.83"))
KI_S = float(os.environ.get("JETSON_PID_STRAIGHT_KI", "0.004"))
KD_S = float(os.environ.get("JETSON_PID_STRAIGHT_KD", "0.095"))
KP_C = float(os.environ.get("JETSON_PID_CURVE_KP",    "0.98"))
KI_C = float(os.environ.get("JETSON_PID_CURVE_KI",    "0.002"))
KD_C = float(os.environ.get("JETSON_PID_CURVE_KD",    "0.115"))
I_MAX  = float(os.environ.get("JETSON_PID_I_CLAMP",   "60.0"))
PID_DT  = float(os.environ.get("JETSON_PID_DT",       "0.033"))  # nominal 30 FPS

# Steer
STEER_SAT = float(os.environ.get("JETSON_STEER_SAT",   "45.0"))
STEER_SCL = float(os.environ.get("JETSON_STEER_SCALE", "0.6"))
RATE_LIM  = float(os.environ.get("JETSON_STEER_RATE_LIMIT", "5.0"))

# Speed (real car — cm/s)
BASE_SPD  = float(os.environ.get("REAL_CAR_SPEED",       "10.0"))
MIN_SPD   = float(os.environ.get("REAL_CAR_MIN_SPEED",   "5.0"))
LOST_SPD  = float(os.environ.get("REAL_CAR_LOST_SCALE",  "0.92"))

# Lost recovery
HOLD_FRAMES = int(os.environ.get("LOST_HOLD_FRAMES", "6"))
SRCH_TURN   = float(os.environ.get("LOST_SEARCH_TURN", "11.0"))

# Differential drive — real car
ST2WHL       = float(os.environ.get("REAL_CAR_ST2WHL", "0.1"))       # steer→轮速差(cm/s)
WHEEL_RADIUS = float(os.environ.get("REAL_CAR_WHEEL_RADIUS", "3.0")) # cm

# Serial
SERIAL_PORT  = os.environ.get("SERIAL_PORT", "COM10")
SERIAL_BAUD  = int(os.environ.get("SERIAL_BAUD", "115200"))
SERIAL_ENABLED = True   # 启动即开串口, 's' 键切换

# Misc
MAX_SEC = float(os.environ.get("REAL_CAR_MAX_SEC", "0"))  # 0 = no limit
PRINT_INTERVAL = 0.5  # seconds between console prints


# ═══════════════════════════════════════════════════════════════════════
# Serial setup (optional)
# ═══════════════════════════════════════════════════════════════════════

_ser = None
_serial_first_sent = False

def _serial_open():
    global _ser
    try:
        import serial
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        print(f"[serial] opened {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        print(f"[serial] WARNING: cannot open {SERIAL_PORT}: {e}")
        _ser = None

def _serial_close():
    global _ser
    if _ser is not None:
        try:
            _ser.close()
        except Exception:
            pass
        _ser = None

def _serial_send(frame: bytes):
    """Send binary frame to MCU. Returns True on success."""
    global _ser, SERIAL_ENABLED, _serial_first_sent
    if not SERIAL_ENABLED or _ser is None:
        return False
    try:
        n = _ser.write(frame)
        if not _serial_first_sent:
            _serial_first_sent = True
            print(f"[serial] first frame sent ({n} bytes): {frame.hex().upper()}")
        return True
    except Exception as e:
        print(f"[serial] write error: {e}")
        _serial_close()
        return False

atexit.register(_serial_close)


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

def main():
    global SERIAL_ENABLED

    # ── Camera ──
    cap = cv2.VideoCapture(CAM_IDX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index {CAM_IDX}")
        sys.exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {CAM_IDX}: requested {CAM_W}x{CAM_H}, got {actual_w}x{actual_h}")

    # ── Detector ──
    ld = LineDetector(actual_w, actual_h,
        cam_height_cm=CAM_HEIGHT_CM,
        cam_pitch_deg=CAM_PITCH_DEG,
        cam_vfov_deg=CAM_VFOV_DEG)

    # ── State ──
    integral      = 0.0
    last_err      = 0.0
    last_steer    = 0.0
    last_lock_ok  = False
    last_print_t  = -99.0
    last_serial_t = 0.0
    t0            = None

    print(f"run_real_car: {actual_w}x{actual_h}  "
          f"KP_s={KP_S:.3f} KP_c={KP_C:.3f}  "
          f"speed={BASE_SPD:.1f} cm/s  st2whl={ST2WHL:.2f}  wheel_r={WHEEL_RADIUS:.1f}cm  "
          f"max_sec={MAX_SEC:.0f}")
    print("Keys: 'q'=quit  's'=toggle serial")

    while True:
        t = time.time()
        if t0 is None:
            t0 = t

        # ── Grab frame ──
        ok, bgr = cap.read()
        if not ok:
            print("[WARN] frame grab failed, retrying...")
            time.sleep(0.01)
            continue

        # ── Detector ──
        _dev, _hdg, conf, _vis, dbg = ld.process(bgr)
        err     = float(dbg.get("fused_err", 0.0))
        curve   = bool(dbg.get("curve_mode", False))
        lost    = int(dbg.get("lost_frames", 0))
        lock_ok = bool(dbg.get("bottom_lock_valid", False))

        # ── Reacquisition reset ──
        if lock_ok and not last_lock_ok:
            integral = 0.0
            last_err = 0.0
        last_lock_ok = lock_ok

        # ── PID (dual-mode) or lost recovery ──
        if lost == 0:
            kp, ki, kd = (KP_C, KI_C, KD_C) if curve else (KP_S, KI_S, KD_S)
            integral += err * PID_DT
            integral = clamp(integral, -I_MAX, I_MAX)
            derr = (err - last_err) / max(PID_DT, 1e-3)
            last_err = err
            pid_out = kp * err + ki * integral + kd * derr
            steer = STEER_SAT * math.tanh(pid_out / STEER_SCL)
            if conf < 0.25:
                integral *= 0.85
        else:
            if lost <= HOLD_FRAMES:
                steer = last_steer * 0.85
            else:
                phase = (lost // 8) % 2
                steer = abs(SRCH_TURN) if phase == 0 else -abs(SRCH_TURN)
            integral *= 0.5

        # ── Steer rate limit ──
        ds = steer - last_steer
        if abs(ds) > RATE_LIM:
            steer = last_steer + math.copysign(RATE_LIM, ds)
        last_steer = steer

        # ── Speed control ──
        spd = BASE_SPD
        if lost > 0:
            spd *= LOST_SPD
        if conf < 0.5:
            spd *= (0.5 + 0.5 * conf)
        spd = max(spd, MIN_SPD)

        # ── Convert to 4 wheel speeds (rad/s) ──
        # steer 直接当轮速差, 和仿真一样: delta = steer × STEER_TO_WHEEL
        delta    = steer * ST2WHL
        v_right  = spd + delta                     # cm/s at right wheels
        v_left   = spd - delta                     # cm/s at left wheels

        fl = v_left  / WHEEL_RADIUS  # rad/s
        fr = v_right / WHEEL_RADIUS
        rl = fl
        rr = fr

        # ── Serial output (~10 Hz), binary protocol ──
        if t - last_serial_t >= 0.1:
            last_serial_t = t
            # rad/s → int8_t: ×10, clamp [-127,127], 0=stop, 正=forward
            def _rad2byte(v):
                return max(-127, min(127, int(v * 10.0))) & 0xFF
            frame = bytes([0xFF,
                           _rad2byte(fr), _rad2byte(fl),
                           _rad2byte(rr), _rad2byte(rl),
                           0xEE])
            _serial_send(frame)

        # ── Console: 打印串口帧（10Hz） ──
        if t - last_print_t > PRINT_INTERVAL and t - last_serial_t < 0.15:
            last_print_t = t
            print(f"{frame[0]:02X}{frame[1]:02X}{frame[2]:02X}{frame[3]:02X}{frame[4]:02X}{frame[5]:02X}")

        # ── Display ──
        # Build overlay on the vis image returned by detector
        vis = _vis.copy() if _vis is not None else bgr.copy()
        if vis.shape[:2] != (actual_h, actual_w):
            vis = cv2.resize(vis, (actual_w, actual_h))

        def _put(s, y, color=(0, 255, 0)):
            cv2.putText(vis, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        _put(f"steer:{steer:+.1f}  curve:{1 if curve else 0}  lost:{lost}", 25)
        _put(f"spd:{spd:.1f} cm/s  conf:{conf:.2f}", 50)
        _put(f"FL:{fl:+.1f}  FR:{fr:+.1f}  RL:{rl:+.1f}  RR:{rr:+.1f} rad/s", 75)
        _put(f"SERIAL:{'ON' if SERIAL_ENABLED else 'OFF'}", 100, (255, 255, 0))
        _put("Q=quit S=toggle_serial", 125, (200, 200, 200))

        cv2.imshow("run_real_car", vis)

        # ── Keyboard ──
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            SERIAL_ENABLED = not SERIAL_ENABLED
            if SERIAL_ENABLED and _ser is None:
                _serial_open()
            print(f"[key] serial {'ON' if SERIAL_ENABLED else 'OFF'}")

        # ── Time limit ──
        if MAX_SEC > 0 and (t - t0) > MAX_SEC:
            print(f"Done. {t - t0:.1f}s elapsed (MAX_SEC={MAX_SEC:.0f}).")
            break

    # ── Cleanup ──
    cap.release()
    cv2.destroyAllWindows()
    _serial_close()
    print("Exited.")


if __name__ == "__main__":
    main()
