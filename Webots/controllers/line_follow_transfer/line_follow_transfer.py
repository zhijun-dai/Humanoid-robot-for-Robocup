from controller import Robot
import math


# Core parameters migrated from CVpart/main/main1.py
CAM_PITCH_DEG = 30.0
CAM_HEIGHT_CM = 17.0
CAM_VFOV_DEG = 52.0
CAM_HFOV_DEG = 70.0
ROW_DIST_MIN_CM = 6.0
ROW_DIST_MAX_CM = 300.0

TURN_START_DIST_CM = 42.0
TURN_END_DIST_CM = 16.0
TURN_ERR_CM = 3.5

# ROI ratios mapped from the original 160x120 settings.
ROI_RATIOS = [
    (0.0, 0.70, 1.0, 0.25, 0.20),  # near
    (0.0, 0.45, 1.0, 0.23, 0.55),  # mid
    (0.0, 0.20, 1.0, 0.20, 0.25),  # far
]

SCAN_LINES_PER_ROI = 5
MIN_TRACK_WIDTH = 18
MAX_TRACK_WIDTH = 145

TH_OFFSET = 8
TH_MIN = 25
TH_MAX = 120

MIN_VALID_LINES = 3
WIDTH_STD_MAX = 14
CONF_MIN = 0.25
SMOOTH_ALPHA = 0.65
CURVE_GAIN = 0.25
ANGLE_GAIN = 0.22
MIN_WEIGHT = 0.10
LOOKAHEAD_GAIN = 0.35

CURVE_SWITCH_CM = 4.5
KP_STRAIGHT = 0.70
KI_STRAIGHT = 0.015
KD_STRAIGHT = 0.12
KP_CURVE = 1.05
KI_CURVE = 0.008
KD_CURVE = 0.18
I_CLAMP = 60.0

STEER_SAT = 70.0
STEER_SCALE = 22.0
LOST_HOLD_FRAMES = 6
LOST_SEARCH_TURN = 26.0

BASE_SPEED = 3.2
STEER_TO_WHEEL = 0.040


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


def find_lr_edges_on_row(raw, y, x0, x1, black_th, img_w):
    left = -1
    right = -1

    x = x0
    while x <= x1:
        if grayscale_from_bgra(raw, img_w, x, y) <= black_th:
            left = x
            break
        x += 1

    x = x1
    while x >= x0:
        if grayscale_from_bgra(raw, img_w, x, y) <= black_th:
            right = x
            break
        x -= 1

    if left < 0 or right < 0:
        return None

    width = right - left
    if width < MIN_TRACK_WIDTH or width > MAX_TRACK_WIDTH:
        return None
    return left, right


def roi_midline(raw, roi, black_th, img_w, img_h, img_cx, row_cm_per_px, row_distance_cm):
    x, y, w, h, weight = roi
    x0 = x
    x1 = x + w - 1

    centers_cm = []
    widths = []
    ys = []
    zs_cm = []

    step = h // (SCAN_LINES_PER_ROI + 1)
    if step < 1:
        step = 1

    i = 1
    while i <= SCAN_LINES_PER_ROI:
        sy = y + i * step
        if sy >= img_h:
            sy = img_h - 1

        lr = find_lr_edges_on_row(raw, sy, x0, x1, black_th, img_w)
        if lr is not None:
            l, r = lr
            center_px = 0.5 * (l + r)
            x_cm, z_cm = px_to_ground_cm(center_px, sy, img_h, img_cx, row_cm_per_px, row_distance_cm)
            centers_cm.append(x_cm)
            zs_cm.append(z_cm)
            widths.append(r - l)
            ys.append(sy)
        i += 1

    if len(centers_cm) < MIN_VALID_LINES:
        return None

    center_cm = median(centers_cm)
    dist_cm = median(zs_cm)
    width_std = stdev(widths)
    conf = (len(centers_cm) / float(SCAN_LINES_PER_ROI)) * (1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0))

    a, _ = line_fit(zs_cm, centers_cm)
    angle = math.degrees(math.atan(a))

    return {
        "center_cm": center_cm,
        "dist_cm": dist_cm,
        "weight": weight,
        "angle": angle,
        "conf": conf,
    }


def nonlinear_map(v):
    return STEER_SAT * math.tanh(v / STEER_SCALE)


def pid_step(err, far_err_cm, dt, state, curve_force=False):
    if (not curve_force) and (abs(far_err_cm) < CURVE_SWITCH_CM):
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

camera = robot.getDevice("camera")
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
}

max_speed = 6.28
print("line_follow_transfer: camera=%dx%d, timestep=%dms" % (img_w, img_h, timestep))

while robot.step(timestep) != -1:
    raw = camera.getImage()
    if raw is None:
        continue

    black_th = clamp(otsu_threshold(raw, img_w, img_h) + TH_OFFSET, TH_MIN, TH_MAX)

    roi_results = []
    for roi in rois:
        res = roi_midline(raw, roi, black_th, img_w, img_h, img_cx, row_cm_per_px, row_distance_cm)
        if res is not None and res["conf"] >= CONF_MIN:
            roi_results.append(res)

    steer = 0.0
    base_err_cm = 0.0
    angle_err = 0.0
    far_dist_cm = 0.0
    turn_gate = 0.0
    curve_mode = 0
    avg_conf = 0.0

    if roi_results:
        state["lost_frames"] = 0

        weighted_sum = 0.0
        weight_total = 0.0
        weight_nominal = 0.0
        angle_sum = 0.0
        angle_weight = 0.0

        for r in roi_results:
            err = r["center_cm"]
            w = r["weight"] * r["conf"]
            weighted_sum += err * w
            weight_total += w
            weight_nominal += r["weight"]
            angle_sum += r["angle"] * w
            angle_weight += w

        if weight_total <= MIN_WEIGHT:
            roi_results = []

    if roi_results:
        far = roi_results[-1]
        near = roi_results[0]
        far_err_cm = far["center_cm"]
        near_err_cm = near["center_cm"]
        far_dist_cm = far["dist_cm"]

        base_err_cm = weighted_sum / weight_total
        angle_err = angle_sum / angle_weight if angle_weight > 0 else 0.0
        avg_conf = (weight_total / weight_nominal) if weight_nominal > 0 else 0.0
        curve_err = far_err_cm - near_err_cm

        dist_norm = clamp((TURN_START_DIST_CM - far_dist_cm) / (TURN_START_DIST_CM - TURN_END_DIST_CM), 0.0, 1.0)
        err_norm = clamp(abs(far_err_cm) / TURN_ERR_CM, 0.0, 1.0)
        turn_gate = dist_norm * err_norm
        lookahead_gain_dyn = LOOKAHEAD_GAIN * (0.70 + 0.90 * turn_gate)
        curve_mode_force = (turn_gate > 0.35) or (abs(curve_err) > CURVE_SWITCH_CM)
        curve_mode = 1 if curve_mode_force else 0

        fused_err = base_err_cm
        fused_err += lookahead_gain_dyn * far_err_cm
        fused_err += CURVE_GAIN * curve_err
        fused_err += ANGLE_GAIN * angle_err

        state["smoothed_err"] = SMOOTH_ALPHA * state["smoothed_err"] + (1.0 - SMOOTH_ALPHA) * fused_err
        dt = timestep / 1000.0
        pid_out = pid_step(state["smoothed_err"], far_err_cm, dt, state, curve_mode_force)
        steer = nonlinear_map(pid_out)
        state["last_steer"] = steer
    else:
        state["lost_frames"] += 1
        if state["lost_frames"] <= LOST_HOLD_FRAMES:
            steer = state["last_steer"] * 0.85
        else:
            steer = LOST_SEARCH_TURN if state["last_steer"] >= 0 else -LOST_SEARCH_TURN

    delta = clamp(steer * STEER_TO_WHEEL, -2.8, 2.8)
    left_speed = clamp(BASE_SPEED - delta, -max_speed, max_speed)
    right_speed = clamp(BASE_SPEED + delta, -max_speed, max_speed)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    now = robot.getTime()
    if now - state["last_print"] > 0.25:
        state["last_print"] = now
        print(
            "th=%d steer=%.1f ex=%.2fcm ang=%.1fdeg z=%.1fcm tg=%.2f mode=%d conf=%.2f lost=%d L=%.2f R=%.2f"
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
            )
        )
