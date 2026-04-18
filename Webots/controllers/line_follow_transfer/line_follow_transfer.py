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

# Camera projection (for approximate cm logging/control)
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
NEAR_CONF_TARGET = float(_cfg_get(SHARED_CFG, "roi.near_conf_target", 0.30))
NEAR_PAIR_TARGET = float(_cfg_get(SHARED_CFG, "roi.near_pair_target", 0.45))
NEAR_TRUST_FLOOR = float(_cfg_get(SHARED_CFG, "roi.near_trust_floor", 0.25))

# Three vertical bands
BAND_NEAR_START = float(_cfg_get(SHARED_CFG, "roi.band_down_start_ratio", 2.0 / 3.0))
BAND_MID_START = float(_cfg_get(SHARED_CFG, "roi.band_mid_start_ratio", 1.0 / 3.0))
BAND_FAR_START = float(_cfg_get(SHARED_CFG, "roi.band_up_start_ratio", 0.0))
BAND_ROWS_NEAR = int(_cfg_get(SHARED_CFG, "roi.band_rows_down", 16))
BAND_ROWS_MID = int(_cfg_get(SHARED_CFG, "roi.band_rows_mid", 14))
BAND_ROWS_FAR = int(_cfg_get(SHARED_CFG, "roi.band_rows_up", 12))
BAND_STEP_NEAR = int(_cfg_get(SHARED_CFG, "roi.band_step_down", 2))
BAND_STEP_MID = int(_cfg_get(SHARED_CFG, "roi.band_step_mid", 2))
BAND_STEP_FAR = int(_cfg_get(SHARED_CFG, "roi.band_step_up", 2))

# Bottom lock
BOTTOM_LOCK_ENABLE = bool(_cfg_get(SHARED_CFG, "roi.bottom_lock_enable", True))
BOTTOM_LOCK_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_start_ratio", 0.80))
BOTTOM_LOCK_ROWS = int(_cfg_get(SHARED_CFG, "roi.bottom_lock_rows", 12))
BOTTOM_LOCK_STEP = int(_cfg_get(SHARED_CFG, "roi.bottom_lock_step", 2))
BOTTOM_LOCK_MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_min_pair_ratio", 0.55))
BOTTOM_LOCK_SYM_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_sym_tol_px", 12.0))
BOTTOM_LOCK_BLEND = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_blend", 0.55))
BOTTOM_LOCK_CONF_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_conf_penalty", 0.45))
BOTTOM_LOCK_SPEED_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_speed_penalty", 0.12))
BOTTOM_LOCK_HEAVY_FACTOR = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_heavy_factor", 1.35))
BOTTOM_LOCK_CURVE_EXTRA = float(
    _cfg_get(SHARED_CFG, "roi.bottom_lock_curve_extra", _cfg_get(SHARED_CFG, "roi.bottom_lock_left_turn_extra", 0.22))
)
BOTTOM_LOCK_MAX_GAIN = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_max_gain", 0.95))

# Startup stabilization
STARTUP_SETTLE_FRAMES = int(_cfg_get(SHARED_CFG, "roi.startup_settle_frames", 25))
STARTUP_SPEED_SCALE = float(_cfg_get(SHARED_CFG, "roi.startup_speed_scale", 0.75))
LOCK_REACQUIRE_RESET = bool(_cfg_get(SHARED_CFG, "roi.lock_reacquire_reset", True))

# Control gains
SMOOTH_ALPHA = float(_cfg_get(SHARED_CFG, "fusion.smooth_alpha", 0.65))
PIX_LOOKAHEAD_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_lookahead_gain", 0.08))
PIX_CURVE_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_curve_gain", 0.22))
CURVE_SWITCH_PX = float(_cfg_get(SHARED_CFG, "webots.curve_switch_px", 20.0))

KP_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kp", 0.9))
KI_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.ki", 0.004))
KD_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kd", 0.08))
KP_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kp", 1.05))
KI_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.ki", 0.002))
KD_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kd", 0.10))
I_CLAMP = float(_cfg_get(SHARED_CFG, "pid.i_clamp", 60.0))

STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 45.0))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 0.6))
STARTUP_STEER_MAX_DELTA = float(_cfg_get(SHARED_CFG, "webots.startup_steer_max_delta", 5.5))
RUN_STEER_MAX_DELTA = float(_cfg_get(SHARED_CFG, "webots.run_steer_max_delta", 14.0))
STARTUP_INTEGRAL_FREEZE_CONF = float(_cfg_get(SHARED_CFG, "webots.startup_integral_freeze_conf", 0.42))

# Lost-line and speed
LOST_HOLD_FRAMES = int(_cfg_get(SHARED_CFG, "lost.hold_frames", 6))
LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 11.0))
LOST_PREFER_LEFT = bool(_cfg_get(SHARED_CFG, "lost.prefer_left", True))
LOST_SEARCH_FOLLOW_LAST_STEER = bool(_cfg_get(SHARED_CFG, "lost.search_follow_last_steer", True))

BASE_SPEED = float(_cfg_get(SHARED_CFG, "webots.base_speed", 9.42))
STEER_TO_WHEEL = float(_cfg_get(SHARED_CFG, "webots.steer_to_wheel", 0.0184))
MAX_SPEED = float(_cfg_get(SHARED_CFG, "webots.max_speed", 9.42))
SPEED_LOST_SCALE = float(_cfg_get(SHARED_CFG, "webots.speed_lost_scale", 0.92))
SPEED_MIN = float(_cfg_get(SHARED_CFG, "webots.speed_min", 7.2))
SPEED_STEER_SLOWDOWN = float(_cfg_get(SHARED_CFG, "webots.speed_steer_slowdown", 0.35))

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


def startup_warmup_ratio(startup_frames):
    if STARTUP_SETTLE_FRAMES <= 0:
        return 1.0, False
    if startup_frames >= STARTUP_SETTLE_FRAMES:
        return 1.0, False
    return clamp(startup_frames / float(STARTUP_SETTLE_FRAMES), 0.0, 1.0), True


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
                    score = abs(center - hint_center) + 0.6 * width_err
                    if score < best_score:
                        best_score = score
                        best = {
                            "center_px": center,
                            "lane_width_px": lane_w,
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
    lane_widths = []
    ys = []

    rows_done = 0
    last_center = hint_center
    last_width = lane_width_hint
    y = y_start
    while y <= y_end and rows_done < max_rows:
        runs = collect_track_runs_on_row(raw, y, x0, x1, black_th, img_w, track_is_dark)
        pair = choose_pair_center_from_runs(runs, last_center, last_width, x0, x1)
        if pair is not None:
            centers.append(float(pair["center_px"]))
            lane_widths.append(float(pair["lane_width_px"]))
            ys.append(y)
            last_center = float(pair["center_px"])
            last_width = float(pair["lane_width_px"])
        rows_done += 1
        y += row_step

    if len(centers) < max(2, MIN_PAIR_LINES):
        return None

    center_px = float(median(centers))
    lane_width_px = float(median(lane_widths))
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


def make_bottom_lock_result(
    valid,
    quality,
    pair_ratio,
    center_px,
    center_err_px,
    symmetry_abs_px,
):
    return {
        "valid": bool(valid),
        "quality": float(quality),
        "pair_ratio": float(pair_ratio),
        "center_px": float(center_px),
        "center_err_px": float(center_err_px),
        "symmetry_abs_px": float(symmetry_abs_px),
    }


def detect_bottom_lock(raw, black_th, img_w, img_h, img_cx, track_is_dark, lane_width_hint):
    if not BOTTOM_LOCK_ENABLE:
        return make_bottom_lock_result(True, 1.0, 1.0, img_cx, 0.0, 0.0)

    x0 = 0
    x1 = img_w - 1
    y_start = int(clamp(BOTTOM_LOCK_START_RATIO * img_h, 0, img_h - 1))
    row_step = max(1, BOTTOM_LOCK_STEP)

    pair_rows = 0
    rows_done = 0
    centers = []

    y = y_start
    while y < img_h and rows_done < max(1, BOTTOM_LOCK_ROWS):
        runs = collect_track_runs_on_row(raw, y, x0, x1, black_th, img_w, track_is_dark)
        pair = choose_pair_center_from_runs(runs, img_cx, lane_width_hint, x0, x1)
        if pair is not None:
            pair_rows += 1
            centers.append(float(pair["center_px"]))

        rows_done += 1
        y += row_step

    if rows_done <= 0 or not centers:
        return make_bottom_lock_result(False, 0.0, 0.0, img_cx, 0.0, img_w)

    pair_ratio = pair_rows / float(rows_done)
    center_px = float(median(centers))
    center_err_px = center_px - float(img_cx)
    symmetry_abs_px = abs(center_err_px)

    pair_q = clamp(
        (pair_ratio - BOTTOM_LOCK_MIN_PAIR_RATIO) / max(1.0 - BOTTOM_LOCK_MIN_PAIR_RATIO, 1e-6),
        0.0,
        1.0,
    )
    sym_q = 1.0 - clamp(symmetry_abs_px / max(BOTTOM_LOCK_SYM_TOL_PX * 2.0, 1e-6), 0.0, 1.0)
    quality = clamp(0.65 * pair_q + 0.35 * sym_q, 0.0, 1.0)
    valid = (pair_ratio >= BOTTOM_LOCK_MIN_PAIR_RATIO) and (symmetry_abs_px <= BOTTOM_LOCK_SYM_TOL_PX)

    return make_bottom_lock_result(
        valid,
        quality,
        pair_ratio,
        center_px,
        center_err_px,
        symmetry_abs_px,
    )


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


def nonlinear_map(v):
    return STEER_SAT * math.tanh(v / max(STEER_SCALE, 1e-6))


def pid_step(err, dt, state, curve_mode=False):
    if curve_mode:
        kp, ki, kd = KP_CURVE, KI_CURVE, KD_CURVE
    else:
        kp, ki, kd = KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT

    state["integral"] += err * dt
    state["integral"] = clamp(state["integral"], -I_CLAMP, I_CLAMP)

    derr = (err - state["last_err"]) / max(dt, 1e-3)
    state["last_err"] = err
    return kp * err + ki * state["integral"] + kd * derr


def limit_steer_slew(steer, last_steer, startup_active, startup_warmup):
    max_delta = RUN_STEER_MAX_DELTA
    if startup_active:
        max_delta = STARTUP_STEER_MAX_DELTA + (RUN_STEER_MAX_DELTA - STARTUP_STEER_MAX_DELTA) * startup_warmup
    max_delta = max(0.5, max_delta)
    return clamp(steer, last_steer - max_delta, last_steer + max_delta)


def compute_target_base_speed(steer, lost_frames, center_lock_quality, startup_active, startup_warmup):
    steer_norm = abs(steer) / max(STEER_SAT, 1e-6)
    speed_scale = 1.0 - SPEED_STEER_SLOWDOWN * steer_norm
    if lost_frames > 0:
        speed_scale *= SPEED_LOST_SCALE
    speed_scale *= (1.0 - BOTTOM_LOCK_SPEED_PENALTY * (1.0 - center_lock_quality))
    if startup_active:
        speed_scale *= (STARTUP_SPEED_SCALE + (1.0 - STARTUP_SPEED_SCALE) * startup_warmup)
    return clamp(BASE_SPEED * speed_scale, SPEED_MIN, BASE_SPEED)


def compute_near_trust(near_conf, near_pair_ratio):
    conf_q = clamp((near_conf - CONF_MIN) / max(NEAR_CONF_TARGET - CONF_MIN, 1e-6), 0.0, 1.0)
    pair_q = clamp((near_pair_ratio - MIN_PAIR_RATIO) / max(NEAR_PAIR_TARGET - MIN_PAIR_RATIO, 1e-6), 0.0, 1.0)
    return clamp(0.5 * conf_q + 0.5 * pair_q, NEAR_TRUST_FLOOR, 1.0)


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

state = {
    "integral": 0.0,
    "last_err": 0.0,
    "smoothed_err": 0.0,
    "last_steer": 0.0,
    "lost_frames": 0,
    "last_print": -1.0,
    "track_dark_score": 0,
    "last_lane_center_x": float(img_cx),
    "last_lane_width_px": float(LANE_WIDTH_INIT_PX),
    "last_base_err_cm": 0.0,
    "last_far_dist_cm": 0.0,
    "startup_frames": 0,
    "last_bottom_lock_valid": False,
}

print("line_follow_transfer(simple): camera=%dx%d, timestep=%dms" % (img_w, img_h, timestep))

while robot.step(timestep) != -1:
    state["startup_frames"] += 1
    startup_warmup, startup_active = startup_warmup_ratio(state["startup_frames"])

    raw = camera.getImage()
    if raw is None:
        continue

    black_th = clamp(otsu_threshold(raw, img_w, img_h) + TH_OFFSET, TH_MIN, TH_MAX)
    track_dark_candidate = detect_track_is_dark(raw, img_w, img_h, black_th)
    if track_dark_candidate:
        state["track_dark_score"] = int(clamp(state["track_dark_score"] + 1, -6, 6))
    else:
        state["track_dark_score"] = int(clamp(state["track_dark_score"] - 1, -6, 6))
    track_is_dark = state["track_dark_score"] >= 0

    near_hint_center = state["last_lane_center_x"]
    near_hint_width = state["last_lane_width_px"]

    near = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        BAND_NEAR_START,
        1.0,
        BAND_ROWS_NEAR,
        BAND_STEP_NEAR,
        near_hint_center,
        near_hint_width,
    )

    mid_hint_center = near["center_px"] if near is not None else near_hint_center
    mid_hint_width = near["lane_width_px"] if near is not None else near_hint_width
    mid = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        BAND_MID_START,
        BAND_NEAR_START,
        BAND_ROWS_MID,
        BAND_STEP_MID,
        mid_hint_center,
        mid_hint_width,
    )

    far_hint_center = mid["center_px"] if mid is not None else mid_hint_center
    far_hint_width = mid["lane_width_px"] if mid is not None else mid_hint_width
    far = scan_two_line_band(
        raw,
        black_th,
        img_w,
        img_h,
        track_is_dark,
        BAND_FAR_START,
        BAND_MID_START,
        BAND_ROWS_FAR,
        BAND_STEP_FAR,
        far_hint_center,
        far_hint_width,
    )

    bottom_lock = detect_bottom_lock(
        raw,
        black_th,
        img_w,
        img_h,
        img_cx,
        track_is_dark,
        state["last_lane_width_px"],
    )

    bottom_pair_ratio = float(bottom_lock.get("pair_ratio", 0.0))
    bottom_sym_err_px = float(bottom_lock.get("center_err_px", 0.0))
    center_lock_quality = float(bottom_lock.get("quality", 0.0))
    bottom_lock_valid = bool(bottom_lock.get("valid", False))

    if LOCK_REACQUIRE_RESET and bottom_lock_valid and (not state["last_bottom_lock_valid"]):
        state["integral"] = 0.0
        state["last_err"] = 0.0
        state["smoothed_err"] *= 0.35
    state["last_bottom_lock_valid"] = bottom_lock_valid

    steer = 0.0
    base_err_px = state["last_lane_center_x"] - img_cx
    base_err_cm = state["last_base_err_cm"]
    far_dist_cm = state["last_far_dist_cm"]
    curve_px = 0.0
    avg_conf = 0.0
    curve_mode = 1
    turn_gate = 1.0

    valid_near = near is not None and near["conf"] >= CONF_MIN and near["pair_ratio"] >= MIN_PAIR_RATIO
    valid_mid = mid is not None and mid["conf"] >= CONF_MIN
    valid_far = far is not None and far["conf"] >= CONF_MIN

    if valid_near:
        state["lost_frames"] = 0

        near_err_px_raw = float(near["center_px"] - img_cx)
        near_err_px = near_err_px_raw
        near_err_cm = near_err_px * row_cm_per_px[img_h - 1]

        far_ref = far if valid_far else (mid if valid_mid else near)
        far_err_px = float(far_ref["center_px"] - img_cx)
        curve_px_raw = far_err_px - near_err_px_raw

        if bottom_pair_ratio > 0.0:
            lock_gain = BOTTOM_LOCK_BLEND * (0.55 + 0.45 * center_lock_quality)
            # Symmetric bottom-lock strengthening: no left/right special-casing.
            lock_gain *= (1.0 + (BOTTOM_LOCK_HEAVY_FACTOR - 1.0) * center_lock_quality)
            turn_strength = clamp(abs(curve_px_raw) / max(CURVE_SWITCH_PX, 1.0), 0.0, 1.0)
            lock_gain += BOTTOM_LOCK_CURVE_EXTRA * turn_strength * center_lock_quality
            lock_valid_gate = 1.0 if bottom_lock_valid else 0.55
            lock_gain *= lock_valid_gate
            if startup_active:
                lock_gain = clamp(lock_gain + 0.20 * (1.0 - startup_warmup), 0.0, BOTTOM_LOCK_MAX_GAIN)
            else:
                lock_gain = clamp(lock_gain, 0.0, BOTTOM_LOCK_MAX_GAIN)
            near_err_px = (1.0 - lock_gain) * near_err_px_raw + lock_gain * bottom_sym_err_px
            near_err_cm = near_err_px * row_cm_per_px[img_h - 1]

        curve_px = far_err_px - near_err_px
        turn_gate = clamp(abs(curve_px) / max(CURVE_SWITCH_PX, 1.0), 0.0, 1.0)

        conf_list = [near["conf"]]
        if valid_mid:
            conf_list.append(mid["conf"])
        if valid_far:
            conf_list.append(far["conf"])
        avg_conf = sum(conf_list) / float(len(conf_list))
        if not bottom_lock_valid:
            avg_conf *= (1.0 - BOTTOM_LOCK_CONF_PENALTY * (1.0 - center_lock_quality))
            avg_conf = clamp(avg_conf, 0.0, 1.0)

        norm_den = max(0.5 * img_w, 1.0)
        near_norm = near_err_px / norm_den
        far_norm = far_err_px / norm_den
        curve_norm = curve_px / norm_den
        near_trust = compute_near_trust(float(near["conf"]), float(near["pair_ratio"]))

        fused_err = -(near_trust * near_norm + (1.0 - near_trust) * far_norm)
        fused_err += PIX_LOOKAHEAD_GAIN * (-far_norm)
        fused_err += PIX_CURVE_GAIN * (-curve_norm)

        state["smoothed_err"] = SMOOTH_ALPHA * state["smoothed_err"] + (1.0 - SMOOTH_ALPHA) * fused_err
        if startup_active and avg_conf < STARTUP_INTEGRAL_FREEZE_CONF:
            state["integral"] *= 0.80

        dt = timestep / 1000.0
        curve_mode_force = abs(curve_px) >= CURVE_SWITCH_PX
        pid_out = pid_step(state["smoothed_err"], dt, state, curve_mode_force)
        steer = nonlinear_map(pid_out)
        steer = limit_steer_slew(steer, state["last_steer"], startup_active, startup_warmup)

        state["last_steer"] = steer
        state["last_lane_center_x"] = clamp(float(img_cx + near_err_px), 0.0, float(img_w - 1))
        state["last_lane_width_px"] = clamp(float(near["lane_width_px"]), float(MIN_TRACK_WIDTH), float(MAX_TRACK_WIDTH))
        state["last_base_err_cm"] = near_err_cm

        y_far = int(far_ref.get("y_med", img_h - 1))
        y_far = int(clamp(y_far, 0, img_h - 1))
        far_dist_cm = row_distance_cm[y_far]
        state["last_far_dist_cm"] = far_dist_cm

        base_err_px = near_err_px
        base_err_cm = near_err_cm
        curve_mode = 1 if curve_mode_force else 0
    else:
        state["lost_frames"] += 1
        base_err_px = state["last_lane_center_x"] - img_cx
        base_err_cm = state["last_base_err_cm"]
        far_dist_cm = state["last_far_dist_cm"]
        avg_conf = 0.0
        curve_mode = 1
        turn_gate = 1.0

        if state["lost_frames"] <= LOST_HOLD_FRAMES:
            steer = state["last_steer"] * 0.85
        else:
            if LOST_SEARCH_FOLLOW_LAST_STEER and abs(state["last_steer"]) > 1.0:
                search_mag = abs(LOST_SEARCH_TURN)
                if state["last_steer"] >= 0.0:
                    steer = search_mag
                else:
                    steer = -search_mag
            else:
                steer = abs(LOST_SEARCH_TURN) if LOST_PREFER_LEFT else -abs(LOST_SEARCH_TURN)

    target_base_speed = compute_target_base_speed(
        steer,
        state["lost_frames"],
        center_lock_quality,
        startup_active,
        startup_warmup,
    )

    delta = clamp(steer * STEER_TO_WHEEL, -2.8, 2.8)
    left_speed = clamp(target_base_speed - delta, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(target_base_speed + delta, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    now = robot.getTime()
    if now - state["last_print"] > 0.25:
        state["last_print"] = now
        print(
            "th=%d steer=%.2f ex=%.2fcm z=%.2fcm tg=%.2f mode=%d conf=%.2f lost=%d L=%.2f R=%.2f expx=%.1f bp=%.2f sym=%.1f lock=%d cq=%.2f"
            % (
                black_th,
                steer,
                base_err_cm,
                far_dist_cm,
                turn_gate,
                curve_mode,
                avg_conf,
                state["lost_frames"],
                left_speed,
                right_speed,
                base_err_px,
                bottom_pair_ratio,
                bottom_sym_err_px,
                1 if bottom_lock_valid else 0,
                center_lock_quality,
            )
        )
