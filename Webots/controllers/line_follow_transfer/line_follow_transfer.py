from controller import Robot
import math
import os
import json


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

# Core parameters migrated from CVpart/main/main1.py
CAM_PITCH_DEG = float(_cfg_get(SHARED_CFG, "camera.pitch_deg", 30.0))
CAM_HEIGHT_CM = float(_cfg_get(SHARED_CFG, "camera.height_cm", 17.0))
CAM_VFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.vfov_deg", 52.0))
CAM_HFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.hfov_deg", 70.0))
ROW_DIST_MIN_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_min_cm", 6.0))
ROW_DIST_MAX_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_max_cm", 300.0))

TURN_START_DIST_CM = float(_cfg_get(SHARED_CFG, "turn.start_dist_cm", 42.0))
TURN_END_DIST_CM = float(_cfg_get(SHARED_CFG, "turn.end_dist_cm", 16.0))
TURN_ERR_CM = float(_cfg_get(SHARED_CFG, "turn.err_cm", 3.5))

BASE_FRAME_W = int(_cfg_get(SHARED_CFG, "base_frame.width", 160))
BASE_FRAME_H = int(_cfg_get(SHARED_CFG, "base_frame.height", 120))
_ROIS_DEFAULT = [
    [0, 84, 160, 30, 0.20],
    [0, 54, 160, 28, 0.55],
    [0, 24, 160, 24, 0.25],
]
_ROIS_PX = _cfg_get(SHARED_CFG, "roi.rois_px", _ROIS_DEFAULT)
ROI_RATIOS = []
for r in _ROIS_PX:
    if not isinstance(r, (list, tuple)) or len(r) < 5:
        continue
    x, y, w, h, wt = r[0], r[1], r[2], r[3], r[4]
    ROI_RATIOS.append((
        float(x) / float(BASE_FRAME_W),
        float(y) / float(BASE_FRAME_H),
        float(w) / float(BASE_FRAME_W),
        float(h) / float(BASE_FRAME_H),
        float(wt),
    ))
if not ROI_RATIOS:
    ROI_RATIOS = [
        (0.0, 0.70, 1.0, 0.25, 0.20),
        (0.0, 0.45, 1.0, 0.23, 0.55),
        (0.0, 0.20, 1.0, 0.20, 0.25),
    ]

SCAN_LINES_PER_ROI = int(_cfg_get(SHARED_CFG, "roi.scan_lines_per_roi", 5))
MIN_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_track_width", 12))
MAX_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_track_width", 220))
MIN_PAIR_LINES = int(_cfg_get(SHARED_CFG, "roi.min_pair_lines", 2))
MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.min_pair_ratio", 0.4))

TH_OFFSET = int(_cfg_get(SHARED_CFG, "threshold.offset", 8))
TH_MIN = int(_cfg_get(SHARED_CFG, "threshold.min", 25))
TH_MAX = int(_cfg_get(SHARED_CFG, "threshold.max", 120))
DARK_MARGIN = int(_cfg_get(SHARED_CFG, "threshold.dark_margin", 12))

MIN_VALID_LINES = int(_cfg_get(SHARED_CFG, "roi.min_valid_lines", 2))
WIDTH_STD_MAX = float(_cfg_get(SHARED_CFG, "roi.width_std_max", 14))
CONF_MIN = float(_cfg_get(SHARED_CFG, "roi.conf_min", 0.12))
SMOOTH_ALPHA = float(_cfg_get(SHARED_CFG, "fusion.smooth_alpha", 0.65))
CURVE_GAIN = float(_cfg_get(SHARED_CFG, "fusion.curve_gain", 0.25))
ANGLE_GAIN = float(_cfg_get(SHARED_CFG, "fusion.angle_gain", 0.22))
MIN_WEIGHT = float(_cfg_get(SHARED_CFG, "fusion.min_weight", 0.10))
LOOKAHEAD_GAIN = float(_cfg_get(SHARED_CFG, "fusion.lookahead_gain", 0.35))
TRACK_COLOR_MODE = str(_cfg_get(SHARED_CFG, "threshold.track_color", "auto")).lower()
MIN_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_line_width", 2))
MAX_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_line_width", 80))
LANE_WIDTH_INIT_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_init_px", 70.0))
LANE_WIDTH_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_tol_px", 40.0))
MAX_CENTER_JUMP_PX = float(_cfg_get(SHARED_CFG, "roi.max_center_jump_px", 55.0))

CURVE_SWITCH_CM = float(_cfg_get(SHARED_CFG, "pid.curve_switch_cm", 4.5))
KP_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kp", 0.70))
KI_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.ki", 0.015))
KD_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kd", 0.12))
KP_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kp", 1.05))
KI_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.ki", 0.008))
KD_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kd", 0.18))
I_CLAMP = float(_cfg_get(SHARED_CFG, "pid.i_clamp", 60.0))

STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 70.0))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 22.0))
LOST_HOLD_FRAMES = int(_cfg_get(SHARED_CFG, "lost.hold_frames", 6))
LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 26.0))
LOST_PREFER_LEFT = bool(_cfg_get(SHARED_CFG, "lost.prefer_left", True))

BASE_SPEED = float(_cfg_get(SHARED_CFG, "webots.base_speed", 3.2))
STEER_TO_WHEEL = float(_cfg_get(SHARED_CFG, "webots.steer_to_wheel", 0.040))
MAX_SPEED = float(_cfg_get(SHARED_CFG, "webots.max_speed", 6.28))
SPEED_TURN_SLOWDOWN = float(_cfg_get(SHARED_CFG, "webots.speed_turn_slowdown", 0.45))
SPEED_LOST_SCALE = float(_cfg_get(SHARED_CFG, "webots.speed_lost_scale", 0.75))
SPEED_MIN = float(_cfg_get(SHARED_CFG, "webots.speed_min", 0.6))
SPEED_STEER_SLOWDOWN = float(_cfg_get(SHARED_CFG, "webots.speed_steer_slowdown", 0.20))
PIX_LOOKAHEAD_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_lookahead_gain", 0.0))
PIX_CURVE_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_curve_gain", 0.0))
PIX_ANGLE_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_angle_gain", 0.06))
CURVE_SWITCH_PX = float(_cfg_get(SHARED_CFG, "webots.curve_switch_px", 18.0))
RIGHT_TURN_SCALE = float(_cfg_get(SHARED_CFG, "webots.right_turn_scale", 0.65))
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


def line_fit(ys, xs):
    n = len(xs)
    if n < 2:
        return 0.0, xs[0]
    mean_y = sum(ys) / n
    mean_x = sum(xs) / n
    num = 0.0
    den = 0.0
    for i in range(n):
        dy = ys[i] - mean_y
        num += dy * (xs[i] - mean_x)
        den += dy * dy
    if den == 0:
        return 0.0, mean_x
    a = num / den
    b = mean_x - a * mean_y
    return a, b


def grayscale_from_bgra(raw, w, x, y):
    idx = (y * w + x) * 4
    b = raw[idx]
    g = raw[idx + 1]
    r = raw[idx + 2]
    return (114 * b + 587 * g + 299 * r) // 1000


def otsu_threshold(raw, w, h):
    hist = [0] * 256
    step_y = 4
    step_x = 4
    total = 0

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


def build_rois(img_w, img_h):
    rois = []
    for rx, ry, rw, rh, w in ROI_RATIOS:
        x = int(rx * img_w)
        y = int(ry * img_h)
        ww = int(rw * img_w)
        hh = int(rh * img_h)
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x + ww > img_w:
            ww = img_w - x
        if y + hh > img_h:
            hh = img_h - y
        rois.append((x, y, ww, hh, w))
    return rois


def build_camera_lut(img_w, img_h, img_cx):
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
        row_cm_per_px[y] = (2.0 * z_cm * math.tan(math.radians(half_h))) / img_w
        y += 1

    return row_distance_cm, row_cm_per_px


def px_to_ground_cm(x, y, img_h, img_cx, row_cm_per_px, row_distance_cm):
    if y < 0:
        y = 0
    elif y >= img_h:
        y = img_h - 1
    x_cm = (x - img_cx) * row_cm_per_px[y]
    z_cm = row_distance_cm[y]
    return x_cm, z_cm


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

    # Track line is usually the minority class in the full frame.
    return dark <= light


def pixel_is_track(g, black_th, track_is_dark):
    if track_is_dark:
        return g <= max(0, black_th - DARK_MARGIN)
    return g >= min(255, black_th + DARK_MARGIN)


def find_lr_edges_on_row(raw, y, x0, x1, black_th, img_w, track_is_dark, hint_x, lane_width_hint):
    hint = int(clamp(hint_x, x0, x1))

    left = -1
    x = hint
    while x >= x0:
        g = grayscale_from_bgra(raw, img_w, x, y)
        if pixel_is_track(g, black_th, track_is_dark):
            left = x
            break
        x -= 1

    right = -1
    x = hint
    while x <= x1:
        g = grayscale_from_bgra(raw, img_w, x, y)
        if pixel_is_track(g, black_th, track_is_dark):
            right = x
            break
        x += 1

    if left < 0 or right < 0:
        left = -1
        right = -1

    if left >= 0 and right >= 0:
        lane_w = right - left
        if lane_w < MIN_TRACK_WIDTH or lane_w > MAX_TRACK_WIDTH:
            left = -1
            right = -1

    if left >= 0 and right >= 0:
        center = 0.5 * (left + right)
        if abs(center - hint_x) > MAX_CENTER_JUMP_PX:
            left = -1
            right = -1

    if left >= 0 and right >= 0 and lane_width_hint > 0 and abs(lane_w - lane_width_hint) > LANE_WIDTH_TOL_PX:
        left = -1
        right = -1

    if left >= 0 and right >= 0:
        return left, right

    # Fallback: find all dark runs in this row and pick the pair closest to hint and width hint.
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
            w = run_end - run_start + 1
            if w >= MIN_LINE_WIDTH and w <= MAX_LINE_WIDTH:
                runs.append((run_start, run_end))
            run_start = -1
        x += 1
    if run_start >= 0:
        run_end = x1
        w = run_end - run_start + 1
        if w >= MIN_LINE_WIDTH and w <= MAX_LINE_WIDTH:
            runs.append((run_start, run_end))

    if len(runs) < 2:
        return None

    best = None
    best_score = 1e9
    i = 0
    while i < len(runs):
        li = 0.5 * (runs[i][0] + runs[i][1])
        j = i + 1
        while j < len(runs):
            rj = 0.5 * (runs[j][0] + runs[j][1])
            lane_w = rj - li
            if lane_w >= MIN_TRACK_WIDTH and lane_w <= MAX_TRACK_WIDTH:
                center = 0.5 * (li + rj)
                if abs(center - hint_x) <= (MAX_CENTER_JUMP_PX * 1.8):
                    width_err = abs(lane_w - lane_width_hint) if lane_width_hint > 0 else 0.0
                    if lane_width_hint <= 0 or width_err <= (LANE_WIDTH_TOL_PX * 2.0):
                        score = abs(center - hint_x) + 0.7 * width_err
                        if score < best_score:
                            best_score = score
                            best = (int(li), int(rj))
            j += 1
        i += 1

    return best


def roi_midline(raw, roi, black_th, img_w, img_h, img_cx, row_cm_per_px, row_distance_cm, track_is_dark, hint_x, lane_width_hint):
    x, y, w, h, weight = roi
    x0 = x
    x1 = x + w - 1

    centers_cm = []
    centers_px = []
    lane_widths = []
    ys = []
    zs_cm = []

    step = h // (SCAN_LINES_PER_ROI + 1)
    if step < 1:
        step = 1

    i = 1
    last_center = hint_x
    last_width = lane_width_hint
    while i <= SCAN_LINES_PER_ROI:
        sy = y + i * step
        if sy >= img_h:
            sy = img_h - 1

        lr = find_lr_edges_on_row(raw, sy, x0, x1, black_th, img_w, track_is_dark, last_center, last_width)
        if lr is not None:
            left, right = lr
            lane_w = right - left
            center_px = 0.5 * (left + right)
            x_cm, z_cm = px_to_ground_cm(center_px, sy, img_h, img_cx, row_cm_per_px, row_distance_cm)
            centers_cm.append(x_cm)
            centers_px.append(center_px)
            zs_cm.append(z_cm)
            lane_widths.append(lane_w)
            ys.append(sy)
            last_center = center_px
            last_width = lane_w
        i += 1

    if len(centers_cm) < MIN_VALID_LINES:
        return None

    center_cm = median(centers_cm)
    center_px = median(centers_px)
    dist_cm = median(zs_cm)
    lane_width_px = median(lane_widths)
    width_std = stdev(lane_widths)
    conf = (len(centers_cm) / float(SCAN_LINES_PER_ROI)) * (1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0))

    a, _ = line_fit(ys, centers_px)
    angle = math.degrees(math.atan(a))

    return {
        "center_cm": center_cm,
        "center_px": center_px,
        "dist_cm": dist_cm,
        "lane_width_px": lane_width_px,
        "weight": weight,
        "angle": angle,
        "conf": conf,
    }


def nonlinear_map(v):
    return STEER_SAT * math.tanh(v / STEER_SCALE)


def pid_step(err, dt, state, curve_mode=False):
    if not curve_mode:
        kp, ki, kd = KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT
    else:
        kp, ki, kd = KP_CURVE, KI_CURVE, KD_CURVE

    state["integral"] += err * dt
    state["integral"] = clamp(state["integral"], -I_CLAMP, I_CLAMP)

    derr = (err - state["last_err"]) / max(dt, 1e-3)
    state["last_err"] = err

    return kp * err + ki * state["integral"] + kd * derr


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

rois = build_rois(img_w, img_h)
row_distance_cm, row_cm_per_px = build_camera_lut(img_w, img_h, img_cx)

state = {
    "integral": 0.0,
    "last_err": 0.0,
    "last_steer": 0.0,
    "smoothed_err": 0.0,
    "lost_frames": 0,
    "last_print": -1.0,
    "last_base_err": 0.0,
    "last_angle_err": 0.0,
    "last_far_dist": 0.0,
    "last_lane_center_x": float(img_cx),
    "last_lane_width_px": float(LANE_WIDTH_INIT_PX),
    "track_dark_score": 0,
}

max_speed = MAX_SPEED
print("line_follow_transfer: camera=%dx%d, timestep=%dms" % (img_w, img_h, timestep))

while robot.step(timestep) != -1:
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

    roi_results = []
    for roi in rois:
        res = roi_midline(
            raw,
            roi,
            black_th,
            img_w,
            img_h,
            img_cx,
            row_cm_per_px,
            row_distance_cm,
            track_is_dark,
            state["last_lane_center_x"],
            state["last_lane_width_px"],
        )
        if res is not None and res["conf"] >= CONF_MIN:
            roi_results.append(res)

    steer = 0.0
    base_err_cm = 0.0
    base_err_px = 0.0
    angle_err = 0.0
    far_dist_cm = 0.0
    turn_gate = 0.0
    curve_mode = 0
    avg_conf = 0.0

    if roi_results:
        score_total = 0.0
        for r in roi_results:
            score_total += r["weight"] * r["conf"]
        if score_total <= MIN_WEIGHT:
            roi_results = []

    if roi_results:
        state["lost_frames"] = 0

        near = min(roi_results, key=lambda r: r["dist_cm"])
        far = max(roi_results, key=lambda r: r["dist_cm"])

        near_err_cm = near["center_cm"]
        far_err_cm = far["center_cm"]
        near_err_px = near["center_px"] - img_cx
        far_err_px = far["center_px"] - img_cx
        far_dist_cm = far["dist_cm"]
        state["last_lane_center_x"] = clamp(float(near["center_px"]), 0.0, float(img_w - 1))
        state["last_lane_width_px"] = clamp(float(near["lane_width_px"]), float(MIN_TRACK_WIDTH), float(MAX_TRACK_WIDTH))

        base_err_cm = near_err_cm
        base_err_px = near_err_px
        curve_err = far_err_cm - near_err_cm
        curve_px = far_err_px - near_err_px
        angle_err = 0.5 * (near["angle"] + far["angle"])
        avg_conf = sum(r["conf"] for r in roi_results) / float(len(roi_results))

        near_norm = near_err_px / max(0.5 * img_w, 1.0)
        far_norm = far_err_px / max(0.5 * img_w, 1.0)
        curve_norm = curve_px / max(0.5 * img_w, 1.0)

        turn_gate = clamp(abs(curve_px) / max(CURVE_SWITCH_PX, 1.0), 0.0, 1.0)
        curve_mode_force = abs(curve_px) >= CURVE_SWITCH_PX
        curve_mode = 1 if curve_mode_force else 0

        # Pixel-domain center control: right-side line means negative steer (turn right).
        fused_err = -near_norm
        fused_err += PIX_LOOKAHEAD_GAIN * (-far_norm)
        fused_err += PIX_CURVE_GAIN * (-curve_norm)
        fused_err += PIX_ANGLE_GAIN * (-angle_err / 45.0)

        state["smoothed_err"] = SMOOTH_ALPHA * state["smoothed_err"] + (1.0 - SMOOTH_ALPHA) * fused_err
        dt = timestep / 1000.0
        pid_out = pid_step(state["smoothed_err"], dt, state, curve_mode_force)
        steer = nonlinear_map(pid_out)
        if steer < 0.0:
            steer *= RIGHT_TURN_SCALE
        state["last_steer"] = steer

        state["last_base_err"] = base_err_cm
        state["last_angle_err"] = angle_err
        state["last_far_dist"] = far_dist_cm
    else:
        state["lost_frames"] += 1
        base_err_cm = state["last_base_err"]
        base_err_px = state["last_lane_center_x"] - img_cx
        angle_err = state["last_angle_err"]
        far_dist_cm = state["last_far_dist"]
        avg_conf = 0.0
        turn_gate = 1.0
        curve_mode = 1
        if state["lost_frames"] <= LOST_HOLD_FRAMES:
            steer = state["last_steer"] * 0.85
        else:
            steer = abs(LOST_SEARCH_TURN) if LOST_PREFER_LEFT else -abs(LOST_SEARCH_TURN)

    steer_norm = abs(steer) / max(STEER_SAT, 1e-6)
    speed_scale = 1.0 - SPEED_STEER_SLOWDOWN * steer_norm
    if state["lost_frames"] > 0:
        speed_scale *= SPEED_LOST_SCALE
    target_base_speed = clamp(BASE_SPEED * speed_scale, SPEED_MIN, BASE_SPEED)

    delta = clamp(steer * STEER_TO_WHEEL, -2.8, 2.8)
    left_speed = clamp(target_base_speed - delta, -max_speed, max_speed)
    right_speed = clamp(target_base_speed + delta, -max_speed, max_speed)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    now = robot.getTime()
    if now - state["last_print"] > 0.25:
        state["last_print"] = now
        print(
            "th=%d steer=%.1f ex=%.2fcm ang=%.1fdeg z=%.1fcm tg=%.2f mode=%d conf=%.2f lost=%d L=%.2f R=%.2f expx=%.1f"
            % (
                black_th,
                steer,
                base_err_cm,
                angle_err,
                far_dist_cm,
                turn_gate,
                curve_mode,
                avg_conf,
                state["lost_frames"],
                left_speed,
                right_speed,
                base_err_px,
            )
        )
