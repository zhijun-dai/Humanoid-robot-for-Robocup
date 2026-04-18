from controller import Robot
import json
import math
import os


def _cfg_get(cfg, path, default):
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _load_shared_cfg():
    base_dir = os.path.dirname(__file__)
    candidates = [
        os.path.join(base_dir, "line_follow_params.json"),
        os.path.abspath(os.path.join(base_dir, "..", "..", "..", "line_follow_params.json")),
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

# Camera geometry
CAM_PITCH_DEG = float(_cfg_get(SHARED_CFG, "camera.pitch_deg", 45.0))
CAM_HEIGHT_CM = float(_cfg_get(SHARED_CFG, "camera.height_cm", 40.0))
CAM_VFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.vfov_deg", 43.3))
CAM_HFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.hfov_deg", 56.1))
ROW_DIST_MIN_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_min_cm", 6.0))
ROW_DIST_MAX_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_max_cm", 300.0))

# Threshold and polarity
TH_OFFSET = int(_cfg_get(SHARED_CFG, "threshold.offset", 8))
TH_MIN = int(_cfg_get(SHARED_CFG, "threshold.min", 25))
TH_MAX = int(_cfg_get(SHARED_CFG, "threshold.max", 120))
DARK_MARGIN = int(_cfg_get(SHARED_CFG, "threshold.dark_margin", 10))
TRACK_COLOR_MODE = str(_cfg_get(SHARED_CFG, "threshold.track_color", "auto")).lower()

# Two-line constraints
MIN_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_line_width", 2))
MAX_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_line_width", 95))
MIN_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_track_width", 12))
MAX_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_track_width", 200))
LANE_WIDTH_INIT_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_init_px", 70.0))
LANE_WIDTH_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_tol_px", 60.0))
MAX_CENTER_JUMP_PX = float(_cfg_get(SHARED_CFG, "roi.max_center_jump_px", 80.0))
MIN_PAIR_LINES = int(_cfg_get(SHARED_CFG, "roi.min_pair_lines", 2))
MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.min_pair_ratio", 0.25))
CONF_MIN = float(_cfg_get(SHARED_CFG, "roi.conf_min", 0.08))

# Four ROI quarters: bottom / mid-low / mid-up / top
ROI_BOTTOM_START = float(_cfg_get(SHARED_CFG, "roi.bottom_quarter_start_ratio", 0.75))
ROI_MID_LOW_START = float(_cfg_get(SHARED_CFG, "roi.mid_low_quarter_start_ratio", 0.50))
ROI_MID_UP_START = float(_cfg_get(SHARED_CFG, "roi.mid_up_quarter_start_ratio", 0.25))
ROI_TOP_START = float(_cfg_get(SHARED_CFG, "roi.top_quarter_start_ratio", 0.00))

ROWS_BOTTOM = int(_cfg_get(SHARED_CFG, "roi.rows_bottom_quarter", _cfg_get(SHARED_CFG, "roi.band_rows_down", 14)))
ROWS_MID_LOW = int(_cfg_get(SHARED_CFG, "roi.rows_mid_low_quarter", _cfg_get(SHARED_CFG, "roi.band_rows_mid", 12)))
ROWS_MID_UP = int(_cfg_get(SHARED_CFG, "roi.rows_mid_up_quarter", _cfg_get(SHARED_CFG, "roi.band_rows_up", 10)))
ROWS_TOP = int(_cfg_get(SHARED_CFG, "roi.rows_top_quarter", 8))

STEP_BOTTOM = int(_cfg_get(SHARED_CFG, "roi.step_bottom_quarter", _cfg_get(SHARED_CFG, "roi.band_step_down", 2)))
STEP_MID_LOW = int(_cfg_get(SHARED_CFG, "roi.step_mid_low_quarter", _cfg_get(SHARED_CFG, "roi.band_step_mid", 2)))
STEP_MID_UP = int(_cfg_get(SHARED_CFG, "roi.step_mid_up_quarter", _cfg_get(SHARED_CFG, "roi.band_step_up", 2)))
STEP_TOP = int(_cfg_get(SHARED_CFG, "roi.step_top_quarter", 2))

BOTTOM_SYM_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_sym_tol_px", 12.0))
CURVE_SWITCH_PX = float(_cfg_get(SHARED_CFG, "webots.curve_switch_px", 20.0))

# Simple steering and speed control
STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 45.0))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 0.6))
STEER_GAIN_STRAIGHT = float(_cfg_get(SHARED_CFG, "webots.simple_gain_straight", 1.00))
STEER_GAIN_CURVE = float(_cfg_get(SHARED_CFG, "webots.simple_gain_curve", 1.25))
CURVE_FEEDFORWARD_GAIN = float(_cfg_get(SHARED_CFG, "webots.simple_curve_feedforward_gain", 0.35))

BASE_SPEED = float(_cfg_get(SHARED_CFG, "webots.base_speed", 6.28))
STEER_TO_WHEEL = float(_cfg_get(SHARED_CFG, "webots.steer_to_wheel", 0.0184))
MAX_SPEED = float(_cfg_get(SHARED_CFG, "webots.max_speed", 6.28))
SPEED_MIN = float(_cfg_get(SHARED_CFG, "webots.speed_min", 3.8))
SPEED_STEER_SLOWDOWN = float(_cfg_get(SHARED_CFG, "webots.speed_steer_slowdown", 0.62))
SPEED_LOST_SCALE = float(_cfg_get(SHARED_CFG, "webots.speed_lost_scale", 0.90))
STRAIGHT_SPEED_SCALE = float(_cfg_get(SHARED_CFG, "webots.simple_straight_speed_scale", 1.00))
CURVE_SPEED_SCALE = float(_cfg_get(SHARED_CFG, "webots.simple_curve_speed_scale", 0.82))

LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 11.0))
LOST_RECOVER_GAIN = float(_cfg_get(SHARED_CFG, "lost.recover_gain", 0.55))

CAMERA_DEVICE_NAME = str(_cfg_get(SHARED_CFG, "webots.camera.device_name", "camera_ext"))


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def median(vals):
    n = len(vals)
    if n == 0:
        return None
    s = sorted(vals)
    m = n // 2
    if n & 1:
        return s[m]
    return 0.5 * (s[m - 1] + s[m])


def stdev(vals):
    n = len(vals)
    if n < 2:
        return 0.0
    mu = sum(vals) / n
    var = 0.0
    for v in vals:
        d = v - mu
        var += d * d
    return math.sqrt(var / (n - 1))


def grayscale_from_bgra(raw, w, x, y):
    idx = (y * w + x) * 4
    b = raw[idx]
    g = raw[idx + 1]
    r = raw[idx + 2]
    return (114 * b + 587 * g + 299 * r) // 1000


def otsu_threshold(raw, w, h):
    hist = [0] * 256
    total = 0
    step_y = 4
    step_x = 4

    y = 0
    while y < h:
        x = 0
        while x < w:
            g = grayscale_from_bgra(raw, w, x, y)
            hist[g] += 1
            total += 1
            x += step_x
        y += step_y

    if total == 0:
        return 64

    sum_all = 0
    i = 0
    while i < 256:
        sum_all += i * hist[i]
        i += 1

    sum_b = 0
    w_b = 0
    max_var = -1.0
    best_t = 64

    t = 0
    while t < 256:
        w_b += hist[t]
        if w_b == 0:
            t += 1
            continue
        w_f = total - w_b
        if w_f == 0:
            break
        sum_b += t * hist[t]
        m_b = sum_b / w_b
        m_f = (sum_all - sum_b) / w_f
        d = m_b - m_f
        var_between = w_b * w_f * d * d
        if var_between > max_var:
            max_var = var_between
            best_t = t
        t += 1

    return best_t


def detect_track_is_dark(raw, w, h, black_th):
    if TRACK_COLOR_MODE == "dark":
        return True
    if TRACK_COLOR_MODE == "light":
        return False

    dark = 0
    light = 0
    step_y = max(1, h // 20)
    step_x = max(1, w // 20)

    y = 0
    while y < h:
        x = 0
        while x < w:
            g = grayscale_from_bgra(raw, w, x, y)
            if g <= black_th:
                dark += 1
            else:
                light += 1
            x += step_x
        y += step_y

    # Track is usually thinner than background, so dark<=light is a robust default.
    return dark <= light


def pixel_is_track(g, black_th, track_is_dark):
    if track_is_dark:
        return g <= max(0, black_th - DARK_MARGIN)
    return g >= min(255, black_th + DARK_MARGIN)


def collect_track_runs_on_row(raw, y, x0, x1, black_th, img_w, track_is_dark):
    runs = []
    run_start = -1

    x = x0
    while x <= x1:
        g = grayscale_from_bgra(raw, img_w, x, y)
        is_track = pixel_is_track(g, black_th, track_is_dark)
        if is_track and run_start < 0:
            run_start = x
        elif (not is_track) and run_start >= 0:
            run_end = x - 1
            run_w = run_end - run_start + 1
            if run_w >= MIN_LINE_WIDTH and run_w <= MAX_LINE_WIDTH:
                runs.append((run_start, run_end))
            run_start = -1
        x += 1

    if run_start >= 0:
        run_end = x1
        run_w = run_end - run_start + 1
        if run_w >= MIN_LINE_WIDTH and run_w <= MAX_LINE_WIDTH:
            runs.append((run_start, run_end))

    return runs


def choose_pair_center_from_runs(runs, hint_center, lane_width_hint, x0, x1):
    if len(runs) < 2:
        return None

    best = None
    best_score = 1e9

    i = 0
    while i < len(runs):
        left_c = 0.5 * (runs[i][0] + runs[i][1])
        j = i + 1
        while j < len(runs):
            right_c = 0.5 * (runs[j][0] + runs[j][1])
            lane_w = right_c - left_c
            if lane_w >= MIN_TRACK_WIDTH and lane_w <= MAX_TRACK_WIDTH:
                center = 0.5 * (left_c + right_c)
                if center >= x0 and center <= x1 and abs(center - hint_center) <= (MAX_CENTER_JUMP_PX * 2.0):
                    width_err = 0.0
                    if lane_width_hint > 0:
                        width_err = abs(lane_w - lane_width_hint)
                        if width_err > (LANE_WIDTH_TOL_PX * 2.0):
                            j += 1
                            continue
                    score = abs(center - hint_center) + 0.5 * width_err
                    if score < best_score:
                        best_score = score
                        best = {
                            "center_px": float(center),
                            "lane_width_px": float(lane_w),
                        }
            j += 1
        i += 1

    return best


def scan_two_line_band(
    raw,
    black_th,
    img_w,
    img_h,
    track_is_dark,
    y_start_ratio,
    y_end_ratio,
    max_rows,
    row_step,
    hint_center,
    lane_width_hint,
):
    x0 = 0
    x1 = img_w - 1
    y_start = int(clamp(y_start_ratio * img_h, 0, img_h - 1))
    y_end = int(clamp(y_end_ratio * img_h, 0, img_h - 1))
    if y_end < y_start:
        y_end = y_start

    row_step = max(1, row_step)
    max_rows = max(1, max_rows)

    centers = []
    widths = []
    ys = []

    rows_done = 0
    last_center = float(hint_center)
    last_width = float(lane_width_hint)
    y = y_start

    while y <= y_end and rows_done < max_rows:
        runs = collect_track_runs_on_row(raw, y, x0, x1, black_th, img_w, track_is_dark)
        pair = choose_pair_center_from_runs(runs, last_center, last_width, x0, x1)
        if pair is not None:
            centers.append(pair["center_px"])
            widths.append(pair["lane_width_px"])
            ys.append(y)
            last_center = pair["center_px"]
            last_width = pair["lane_width_px"]

        rows_done += 1
        y += row_step

    if len(centers) < max(2, MIN_PAIR_LINES):
        return None

    center_px = float(median(centers))
    lane_width_px = float(median(widths))
    y_med = int(median(ys)) if ys else y_start
    pair_ratio = len(centers) / float(max(1, rows_done))
    center_std = stdev(centers)
    conf = pair_ratio * (1.0 - clamp(center_std / max(MAX_CENTER_JUMP_PX, 1e-6), 0.0, 1.0))

    return {
        "center_px": center_px,
        "lane_width_px": lane_width_px,
        "pair_ratio": pair_ratio,
        "conf": clamp(conf, 0.0, 1.0),
        "y_med": y_med,
    }


def nonlinear_map(v):
    return STEER_SAT * math.tanh(v / max(STEER_SCALE, 1e-6))


def build_camera_lut(img_w, img_h):
    row_distance_cm = [0.0] * img_h
    row_cm_per_px = [0.0] * img_h
    half_v = CAM_VFOV_DEG * 0.5
    half_h = CAM_HFOV_DEG * 0.5

    y = 0
    while y < img_h:
        v_deg = ((img_h * 0.5 - y) / (img_h * 0.5)) * half_v
        ray_deg = CAM_PITCH_DEG + v_deg
        if ray_deg < 1.0:
            ray_deg = 1.0

        z_cm = CAM_HEIGHT_CM / math.tan(math.radians(ray_deg))
        z_cm = clamp(z_cm, ROW_DIST_MIN_CM, ROW_DIST_MAX_CM)
        row_distance_cm[y] = z_cm
        row_cm_per_px[y] = (2.0 * z_cm * math.tan(math.radians(half_h))) / max(1.0, float(img_w))
        y += 1

    return row_distance_cm, row_cm_per_px


def band_valid(band):
    if band is None:
        return False
    return (band["conf"] >= CONF_MIN) and (band["pair_ratio"] >= MIN_PAIR_RATIO)


def band_err_px(band, img_cx):
    if band is None:
        return None
    return float(band["center_px"] - img_cx)


def first_non_none(*vals):
    for v in vals:
        if v is not None:
            return v
    return None


robot = Robot()
timestep = int(robot.getBasicTimeStep())

try:
    camera = robot.getDevice(CAMERA_DEVICE_NAME)
except Exception:
    camera = None

if camera is None:
    raise RuntimeError("No camera device found: %s" % CAMERA_DEVICE_NAME)

camera.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

img_w = camera.getWidth()
img_h = camera.getHeight()
img_cx = img_w // 2
row_distance_cm, row_cm_per_px = build_camera_lut(img_w, img_h)

last_print = -1.0

print("line_follow_transfer(simple_4roi): camera=%dx%d, timestep=%dms" % (img_w, img_h, timestep))

while robot.step(timestep) != -1:
    raw = camera.getImage()
    if raw is None:
        continue

    black_th = clamp(otsu_threshold(raw, img_w, img_h) + TH_OFFSET, TH_MIN, TH_MAX)
    track_is_dark = detect_track_is_dark(raw, img_w, img_h, black_th)

    b0 = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        ROI_BOTTOM_START,
        1.0,
        ROWS_BOTTOM,
        STEP_BOTTOM,
        img_cx,
        LANE_WIDTH_INIT_PX,
    )

    hint1_center = b0["center_px"] if b0 is not None else float(img_cx)
    hint1_width = b0["lane_width_px"] if b0 is not None else LANE_WIDTH_INIT_PX
    b1 = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        ROI_MID_LOW_START,
        ROI_BOTTOM_START,
        ROWS_MID_LOW,
        STEP_MID_LOW,
        hint1_center,
        hint1_width,
    )

    hint2_center = b1["center_px"] if b1 is not None else hint1_center
    hint2_width = b1["lane_width_px"] if b1 is not None else hint1_width
    b2 = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        ROI_MID_UP_START,
        ROI_MID_LOW_START,
        ROWS_MID_UP,
        STEP_MID_UP,
        hint2_center,
        hint2_width,
    )

    hint3_center = b2["center_px"] if b2 is not None else hint2_center
    hint3_width = b2["lane_width_px"] if b2 is not None else hint2_width
    b3 = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        ROI_TOP_START,
        ROI_MID_UP_START,
        ROWS_TOP,
        STEP_TOP,
        hint3_center,
        hint3_width,
    )

    v0 = band_valid(b0)
    v1 = band_valid(b1)
    v2 = band_valid(b2)
    v3 = band_valid(b3)

    e0 = band_err_px(b0, img_cx) if v0 else None
    e1 = band_err_px(b1, img_cx) if v1 else None
    e2 = band_err_px(b2, img_cx) if v2 else None
    e3 = band_err_px(b3, img_cx) if v3 else None

    base_e = first_non_none(e0, e1, e2, e3)
    if base_e is None:
        base_e = 0.0

    if e0 is None:
        e0 = base_e
    if e1 is None:
        e1 = e0
    if e2 is None:
        e2 = e1

    curve01 = e1 - e0
    curve12 = e2 - e1
    curve_px = 0.65 * curve01 + 0.35 * curve12

    main_valid_count = int(v0) + int(v1) + int(v2)
    bottom_sym_ok = v0 and (abs(e0) <= BOTTOM_SYM_TOL_PX)
    bottom_sym_bad = v0 and (not bottom_sym_ok)

    if main_valid_count == 0:
        mode_id = 2  # lost
    elif (abs(curve_px) >= CURVE_SWITCH_PX) or bottom_sym_bad:
        mode_id = 1  # curve
    else:
        mode_id = 0  # straight

    if mode_id == 0:
        target_err_px = 0.70 * e0 + 0.20 * e1 + 0.10 * e2
        err_norm = target_err_px / max(0.5 * img_w, 1.0)
        steer = nonlinear_map(STEER_GAIN_STRAIGHT * err_norm)
    elif mode_id == 1:
        if v0:
            target_err_px = 0.50 * e0 + 0.30 * e1 + 0.20 * e2 + CURVE_FEEDFORWARD_GAIN * curve_px
        else:
            target_err_px = 0.65 * e1 + 0.35 * e2 + CURVE_FEEDFORWARD_GAIN * curve_px
        err_norm = target_err_px / max(0.5 * img_w, 1.0)
        steer = nonlinear_map(STEER_GAIN_CURVE * err_norm)
    else:
        target_err_px = base_e
        if abs(curve_px) > 1.0:
            steer = clamp(math.copysign(abs(LOST_SEARCH_TURN), curve_px), -STEER_SAT, STEER_SAT)
        else:
            err_norm = target_err_px / max(0.5 * img_w, 1.0)
            steer = clamp(
                LOST_RECOVER_GAIN * err_norm * STEER_SAT,
                -0.5 * abs(LOST_SEARCH_TURN),
                0.5 * abs(LOST_SEARCH_TURN),
            )

    steer_norm = abs(steer) / max(STEER_SAT, 1e-6)

    if mode_id == 0:
        speed_scale = STRAIGHT_SPEED_SCALE * (1.0 - 0.35 * SPEED_STEER_SLOWDOWN * steer_norm)
    elif mode_id == 1:
        speed_scale = CURVE_SPEED_SCALE * (1.0 - 0.65 * SPEED_STEER_SLOWDOWN * steer_norm)
    else:
        speed_scale = SPEED_LOST_SCALE * 0.75

    target_base_speed = clamp(BASE_SPEED * speed_scale, SPEED_MIN, BASE_SPEED)

    delta = clamp(steer * STEER_TO_WHEEL, -2.8, 2.8)
    left_speed = clamp(target_base_speed - delta, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(target_base_speed + delta, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    conf_vals = []
    if v0:
        conf_vals.append(float(b0["conf"]))
    if v1:
        conf_vals.append(float(b1["conf"]))
    if v2:
        conf_vals.append(float(b2["conf"]))
    avg_conf = sum(conf_vals) / float(len(conf_vals)) if conf_vals else 0.0

    y_ref = b2["y_med"] if v2 else (b1["y_med"] if v1 else (b0["y_med"] if v0 else img_h - 1))
    y_ref = int(clamp(y_ref, 0, img_h - 1))
    dist_cm = row_distance_cm[y_ref]
    err_cm = target_err_px * row_cm_per_px[img_h - 1]

    now = robot.getTime()
    if now - last_print > 0.25:
        last_print = now
        mode_txt = "S" if mode_id == 0 else ("C" if mode_id == 1 else "L")
        print(
            "th=%d mode=%s ex=%.2fcm e0=%.1f e1=%.1f e2=%.1f cv=%.1f sym=%d conf=%.2f z=%.2fcm steer=%.2f L=%.2f R=%.2f"
            % (
                black_th,
                mode_txt,
                err_cm,
                e0,
                e1,
                e2,
                curve_px,
                1 if bottom_sym_ok else 0,
                avg_conf,
                dist_cm,
                steer,
                left_speed,
                right_speed,
            )
        )
