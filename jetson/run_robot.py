"""Jetson 机器人控制器 — USB 相机 + V1 检测 + 一步前瞻 + 协议V2 LINE_CTRL。

链路: USB相机 → LineDetector → 一步前瞻(弯道切线补偿) → route/ex/ang
      → LINE_CTRL(10Hz) + HEARTBEAT(10Hz) → 主控步态

弯道切线问题: 双足每步离散, 一步之内沿当前航向切线漂移 L_step·sin(ang)。
一步前瞻把"这一步将积累的横向偏差"提前算进指令:
    eff_err = fused_err + PREVIEW_GAIN · L_step · sin(ang)
弯道时 eff_err 提前加大 → route 提前切向 + ex_mm 加大 → 主控转角力度加大。

机器人步长与测试车不同, 用 env 覆盖 (STEP_LEN_CM 等, 机器人侧标定)。
"""

import math, os, sys, time, struct
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
from shape_detector import ShapeDetector
from protocol_v2 import (VisionProtocolV2, StreamParser, MSG_ROBOT_STATE,
                         MSG_LINE_CTRL, MSG_HEARTBEAT)


def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v


# ═══════════════════════════════════════════════════════════════════════
# Config — all overridable via env vars
# ═══════════════════════════════════════════════════════════════════════

# Camera
CAM_IDX  = int(os.environ.get("CAM_IDX", "0"))
CAM_W    = 1280
CAM_H    = 720

# Detector
CAM_HEIGHT_CM  = float(os.environ.get("CAM_HEIGHT_CM",  "40.0"))
CAM_PITCH_DEG  = float(os.environ.get("CAM_PITCH_DEG",  "45.0"))
CAM_VFOV_DEG   = float(os.environ.get("CAM_VFOV_DEG",   "56.2"))

# ── 机器人步态参数 (机器人侧标定, 与测试车不同) ──
STEP_LEN_CM    = float(os.environ.get("STEP_LEN_CM",      "10.0"))  # 一步前进距离 cm
STEP_TIME_S    = float(os.environ.get("STEP_TIME_S",      "0.4"))   # 一步时长 s (日志/未来用)
PREVIEW_GAIN   = float(os.environ.get("PREVIEW_GAIN",     "1.0"))   # 一步前瞻增益
DEADBAND_CM    = float(os.environ.get("ROUTE_DEADBAND_CM",  "1.5")) # |err|<=此值 → GO
LEFT_THRESH_CM = float(os.environ.get("ROUTE_LEFT_THRESH_CM", "2.0")) # err<负此值 → LEFT, 否则 SLIGHT_LEFT

# PID — dual-mode (straight / curve), 输出连续纠偏量 (cm)
KP_S = float(os.environ.get("JETSON_PID_STRAIGHT_KP", "0.83"))
KI_S = float(os.environ.get("JETSON_PID_STRAIGHT_KI", "0.004"))
KD_S = float(os.environ.get("JETSON_PID_STRAIGHT_KD", "0.095"))
KP_C = float(os.environ.get("JETSON_PID_CURVE_KP",    "0.78"))
KI_C = float(os.environ.get("JETSON_PID_CURVE_KI",    "0.002"))
KD_C = float(os.environ.get("JETSON_PID_CURVE_KD",    "0.16"))
I_MAX  = float(os.environ.get("JETSON_PID_I_CLAMP",   "60.0"))
PID_DT = float(os.environ.get("JETSON_PID_DT",        "0.033"))

# Lost recovery
HOLD_FRAMES = int(os.environ.get("LOST_HOLD_FRAMES", "6"))
SRCH_TURN   = float(os.environ.get("LOST_SEARCH_TURN", "11.0"))

# Serial
SERIAL_PORT  = os.environ.get("SERIAL_PORT", "COM10")
SERIAL_BAUD  = int(os.environ.get("SERIAL_BAUD", "115200"))

# Protocol
CTRL_HZ      = float(os.environ.get("CTRL_HZ", "10.0"))
MODE_LINE_FOLLOW = 1
MODE_LOST_SEARCH = 2

# Misc
MAX_SEC = float(os.environ.get("REAL_CAR_MAX_SEC", "0"))
PRINT_INTERVAL = 0.5


# ═══════════════════════════════════════════════════════════════════════
# Serial
# ═══════════════════════════════════════════════════════════════════════

_ser = None

def _serial_open():
    global _ser
    try:
        import serial
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        print(f"[serial] opened {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        print(f"[serial] WARNING: cannot open {SERIAL_PORT}: {e}")
        _ser = None

def _serial_send(frame: bytes) -> bool:
    if _ser is None:
        return False
    try:
        _ser.write(frame)
        return True
    except Exception as e:
        print(f"[serial] write error: {e}")
        return False

def _serial_read(parser):
    if _ser is None:
        return
    try:
        n = _ser.in_waiting
        if n > 0:
            chunk = _ser.read(n)
            for fr in parser.feed(chunk):
                if fr.msg_type == MSG_ROBOT_STATE:
                    print(f"[rx] ROBOT_STATE seq={fr.seq} payload={fr.payload.hex()}")
                elif fr.msg_type == 0x80:
                    print(f"[rx] ACK seq={fr.seq} payload={fr.payload.hex()}")
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════
# 控制映射
# ═══════════════════════════════════════════════════════════════════════

def steer_to_route(steer_cm):
    """steer > 0: 线偏右 → 右转. 离散档: 1=go 2=left 3=right 4=slight_left"""
    if abs(steer_cm) <= DEADBAND_CM:
        return 1
    if steer_cm > 0:
        return 3
    if steer_cm < -LEFT_THRESH_CM:
        return 2
    return 4


def quantize(v, step):
    if step <= 1:
        return int(v)
    return int(round(float(v) / float(step)) * float(step))


def main():
    # ── Camera ──
    cap = cv2.VideoCapture(CAM_IDX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index {CAM_IDX}")
        sys.exit(1)

    for _ in range(3):
        ok, _ = cap.read()
        if ok:
            break
        time.sleep(0.05)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if actual_w <= 0 or actual_h <= 0:
        actual_w, actual_h = CAM_W, CAM_H
    print(f"Camera {CAM_IDX}: requested {CAM_W}x{CAM_H}, got {actual_w}x{actual_h}")

    # ── Detectors ──
    ld = LineDetector(actual_w, actual_h,
        cam_height_cm=CAM_HEIGHT_CM,
        cam_pitch_deg=CAM_PITCH_DEG,
        cam_vfov_deg=CAM_VFOV_DEG)
    sd = ShapeDetector(stable_frames=3, cooldown_ms=3200)

    # ── State ──
    integral   = 0.0
    last_err   = 0.0
    last_steer = 0.0
    last_curve = False
    last_print_t = -99.0
    last_frame_t = 0.0
    last_ctrl_t  = 0.0
    last_hb_t    = 0.0
    t0 = None

    proto = VisionProtocolV2()
    parser = StreamParser()

    # 图卡动作状态机
    SM_DRIVE, SM_ACTION = 0, 1
    sm_mode = SM_DRIVE
    sm_action_start = 0.0
    sm_action = 0
    sm_frame_count = 0

    print(f"run_robot: {actual_w}x{actual_h}  step_len={STEP_LEN_CM:.1f}cm  "
          f"preview={PREVIEW_GAIN:.2f}  KP_s={KP_S:.3f} KP_c={KP_C:.3f}  ctrl={CTRL_HZ:.0f}Hz")
    print("Keys: 'q'=quit")

    _serial_open()

    while True:
        t = time.time()
        if t0 is None:
            t0 = t
        dt = t - last_frame_t if last_frame_t > 0 else PID_DT
        dt = clamp(dt, 0.01, 0.2)
        last_frame_t = t

        # ── Grab frame ──
        ok, bgr = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        # ── Detector ──
        _dev, _hdg, conf, _vis, dbg = ld.process(bgr)
        err     = float(dbg.get("fused_err", 0.0))
        angle_err = float(dbg.get("angle_err_deg", 0.0))
        curve   = bool(dbg.get("curve_mode", False))
        lost    = int(dbg.get("lost_frames", 0))
        lock_ok = bool(dbg.get("bottom_lock_valid", False))

        # ── 图卡检测 (隔帧) ──
        if sm_mode == SM_DRIVE:
            sm_frame_count += 1
            if sm_frame_count % 3 == 0:
                shape_action, _ = sd.update(bgr)
                if shape_action is not None:
                    sm_mode = SM_ACTION
                    sm_action = shape_action
                    sm_action_start = t
                    print(f"[shape] >>> 图卡 action={shape_action}，停车做动作")

        if lock_ok and sm_mode == SM_DRIVE:
            integral = 0.0 if last_curve else integral
        last_curve = curve

        # ── 图卡动作状态：停车+动作3秒 ──
        if sm_mode == SM_ACTION:
            if t - sm_action_start >= 3.0:
                sm_mode = SM_DRIVE
                integral = 0.0
                last_err = 0.0
                print(f"[shape] 动作{sm_action}完成，恢复巡线")
            steer = 0.0
        elif lost == 0:
            # ── PID (dual-mode) ──
            kp, ki, kd = (KP_C, KI_C, KD_C) if curve else (KP_S, KI_S, KD_S)
            integral += err * dt
            integral = clamp(integral, -I_MAX, I_MAX)
            derr = (err - last_err) / max(dt, 1e-3)
            last_err = err
            steer = kp * err + ki * integral + kd * derr

            # ── 一步前瞻: 补偿一步之内沿切线的横向漂移 ──
            steer += PREVIEW_GAIN * STEP_LEN_CM * math.sin(math.radians(angle_err))
        else:
            # ── Lost: 保持历史转向 / 搜索 ──
            if lost <= HOLD_FRAMES:
                steer = last_steer * 0.85
            else:
                phase = (lost // 8) % 2
                steer = SRCH_TURN if phase == 0 else -SRCH_TURN
            integral *= 0.5

        steer = clamp(steer, -50.0, 50.0)
        last_steer = steer

        # ── 量化 → LINE_CTRL ──
        route_u8 = steer_to_route(steer)
        mode_u8 = MODE_LOST_SEARCH if lost > 0 else MODE_LINE_FOLLOW
        conf_u8 = clamp(quantize(conf * 100.0, 5), 0, 100)
        lost_u8 = 1 if lost > 0 else 0
        ex_mm = quantize(round(steer * 10.0), 10)          # 预测步末横向偏差 mm
        ang_cdeg = quantize(round(angle_err * 100.0), 100) # 航向误差 cdeg

        n_ms = int(t * 1000)
        if n_ms - last_ctrl_t >= 1000.0 / CTRL_HZ:
            _serial_send(proto.build_line_ctrl(
                mode_u8=mode_u8, conf_u8=conf_u8, lost_u8=lost_u8,
                route_u8=route_u8, ex_mm_i16=ex_mm, ang_cdeg_i16=ang_cdeg,
                v_cmd_mmps_i16=0, w_cmd_mradps_i16=0, ts_ms=n_ms))
            last_ctrl_t = n_ms
        if n_ms - last_hb_t >= 100:
            _serial_send(proto.build_heartbeat(mode_u8, ts_ms=n_ms))
            last_hb_t = n_ms

        _serial_read(parser)

        # ── Console print ──
        if t - last_print_t > PRINT_INTERVAL:
            last_print_t = t
            print(f"route={route_u8} ex={ex_mm}mm ang={ang_cdeg}cdeg conf={conf_u8} "
                  f"lost={lost} curve={int(curve)} err={err:+.1f}cm ang={angle_err:+.1f}deg")

        # ── Display ──
        frame_disp = bgr.copy()
        cv2.putText(frame_disp, f"route={route_u8} ex={ex_mm}mm ang={ang_cdeg}cdeg conf={conf_u8}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame_disp, f"err={err:+.1f}cm ang={angle_err:+.1f}deg curve={int(curve)} lost={lost}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame_disp, "Q=quit", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
        cv2.imshow("1.Original", frame_disp)

        if "bird" in dbg and dbg["bird"] is not None:
            bird_bgr = cv2.cvtColor(dbg["bird"], cv2.COLOR_GRAY2BGR)
            cv2.imshow("2.Warp (birdseye)", cv2.resize(bird_bgr, (320, 400), interpolation=cv2.INTER_NEAREST))
        if "binary_raw" in dbg and dbg["binary_raw"] is not None:
            b_raw = cv2.cvtColor(dbg["binary_raw"], cv2.COLOR_GRAY2BGR)
            cv2.imshow("3.Adaptive (binary)", cv2.resize(b_raw, (320, 400), interpolation=cv2.INTER_NEAREST))
        if _vis is not None:
            cv2.imshow("4.Close+Fit", cv2.resize(_vis, (320, 400), interpolation=cv2.INTER_NEAREST))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if MAX_SEC > 0 and (t - t0) > MAX_SEC:
            print(f"Done. {t - t0:.1f}s elapsed (MAX_SEC={MAX_SEC:.0f}).")
            break

    cap.release()
    cv2.destroyAllWindows()
    if _ser is not None:
        try:
            _ser.close()
        except Exception:
            pass
    print("Exited.")


if __name__ == "__main__":
    main()
