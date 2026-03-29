import sensor  # type: ignore
import image  # type: ignore
import time
import math
from pyb import UART, LED  # type: ignore


# ---------------------- 串口配置 ----------------------
UART_ID = 1
UART_BAUD = 9600
uart = UART(UART_ID, UART_BAUD)

ROUTE_GO = "1"
ROUTE_LEFT = "2"
ROUTE_RIGHT = "3"
ROUTE_SLIGHT_LEFT = "4"


# ---------------------- 相机与算法参数 ----------------------
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # 160x120，兼顾速度与细节。
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

IMG_W = 160
IMG_H = 120
IMG_CX = IMG_W // 2

# 相机几何模型（近似）：向下俯视约30度，建立像素行到地面距离映射。
CAM_PITCH_DEG = 30.0
CAM_HEIGHT_CM = 17.0
CAM_VFOV_DEG = 52.0
CAM_HFOV_DEG = 70.0
ROW_DIST_MIN_CM = 6.0
ROW_DIST_MAX_CM = 300.0

# 转弯触发距离模型：到达一定距离且偏移足够时提前入弯。
TURN_START_DIST_CM = 42.0
TURN_END_DIST_CM = 16.0
TURN_ERR_CM = 3.5

# 三层ROI：近端抑制抖动，中端主控，远端预判弯道。
ROIS = [
	(0, 84, 160, 30, 0.20),  # near
	(0, 54, 160, 28, 0.55),  # mid
	(0, 24, 160, 24, 0.25),  # far
]

SCAN_LINES_PER_ROI = 5
MIN_TRACK_WIDTH = 18
MAX_TRACK_WIDTH = 145

# 动态阈值参数：基于Otsu结果上移，适应光照变化。
TH_OFFSET = 8
TH_MIN = 25
TH_MAX = 120

# 预处理与稳定性增强。
USE_HISTEQ = True
USE_BINARY = False
USE_ERODE = True
ERODE_ITER = 1
ENABLE_DEBUG_DRAW = True

MIN_VALID_LINES = 3
WIDTH_STD_MAX = 14
CONF_MIN = 0.25
SMOOTH_ALPHA = 0.65
CURVE_GAIN = 0.25
ANGLE_GAIN = 0.22
MIN_WEIGHT = 0.10

# 前瞻增益：让远端信息提前参与控制。
LOOKAHEAD_GAIN = 0.35

# 双模式PID参数（直道/弯道）。
CURVE_SWITCH_CM = 4.5
KP_STRAIGHT = 0.70
KI_STRAIGHT = 0.015
KD_STRAIGHT = 0.12
KP_CURVE = 1.05
KI_CURVE = 0.008
KD_CURVE = 0.18
I_CLAMP = 60

# 非线性映射：避免边缘大误差导致过激转向。
STEER_SAT = 70
STEER_SCALE = 22.0
DEADBAND = 6

# 丢线恢复。
LOST_HOLD_FRAMES = 6
LOST_SEARCH_TURN = 26

# 输出节流。
SEND_INTERVAL_MS = 70
last_send_ms = 0


# ---------------------- 运行状态 ----------------------
integral_term = 0.0
last_err = 0.0
last_ms = time.ticks_ms()
last_steer = 0.0
lost_frames = 0
smoothed_err = 0.0

# 行映射查找表：每行对应的前向距离与横向厘米/像素比例。
row_distance_cm = [0.0] * IMG_H
row_cm_per_px = [0.0] * IMG_H


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
	return (s[m - 1] + s[m]) / 2


def stdev(vals):
	n = len(vals)
	if n < 2:
		return 0.0
	mean = sum(vals) / n
	var = 0.0
	for v in vals:
		var += (v - mean) * (v - mean)
	return math.sqrt(var / (n - 1))


def line_fit(ys, xs):
	# 拟合 x = a*y + b。
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


def build_camera_lut():
	half_v = CAM_VFOV_DEG * 0.5
	half_h = CAM_HFOV_DEG * 0.5

	y = 0
	while y < IMG_H:
		# 把图像行映射为相对光轴的垂向夹角（上正下负）。
		v_deg = ((IMG_H * 0.5 - y) / (IMG_H * 0.5)) * half_v
		ray_deg = CAM_PITCH_DEG + v_deg
		if ray_deg < 1.0:
			ray_deg = 1.0

		# 地面交点前向距离：z = h / tan(ray_angle)。
		z_cm = CAM_HEIGHT_CM / math.tan(math.radians(ray_deg))
		z_cm = clamp(z_cm, ROW_DIST_MIN_CM, ROW_DIST_MAX_CM)
		row_distance_cm[y] = z_cm

		# 该距离下每个像素对应的横向厘米比例。
		row_cm_per_px[y] = (2.0 * z_cm * math.tan(math.radians(half_h))) / IMG_W
		y += 1


def px_to_ground_cm(x, y):
	if y < 0:
		y = 0
	elif y >= IMG_H:
		y = IMG_H - 1
	x_cm = (x - IMG_CX) * row_cm_per_px[y]
	z_cm = row_distance_cm[y]
	return x_cm, z_cm


def now_ms():
	return time.ticks_ms()


def send_route_cmd(ch):
	uart.write(ch)


def adaptive_black_threshold(img):
	# 采用全图Otsu阈值，并做偏移，保留深色赛道线。
	th = img.get_histogram().get_threshold().value()
	return clamp(th + TH_OFFSET, TH_MIN, TH_MAX)


def pixel_is_black(pixel, black_th, binary_mode):
	if binary_mode:
		return pixel == 0
	return pixel <= black_th


def find_lr_edges_on_row(img, y, x0, x1, black_th, binary_mode):
	left = -1
	right = -1

	# 从左到右找左边界。
	x = x0
	while x <= x1:
		if pixel_is_black(img.get_pixel(x, y), black_th, binary_mode):
			left = x
			break
		x += 1

	# 从右到左找右边界。
	x = x1
	while x >= x0:
		if pixel_is_black(img.get_pixel(x, y), black_th, binary_mode):
			right = x
			break
		x -= 1

	if left < 0 or right < 0:
		return None

	width = right - left
	if width < MIN_TRACK_WIDTH or width > MAX_TRACK_WIDTH:
		return None
	return (left, right)


def roi_midline(img, roi, black_th, binary_mode, draw_color):
	x, y, w, h, weight = roi
	x0 = x
	x1 = x + w - 1

	lefts = []
	rights = []
	centers = []
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
		if sy >= IMG_H:
			sy = IMG_H - 1

		lr = find_lr_edges_on_row(img, sy, x0, x1, black_th, binary_mode)
		if lr is not None:
			l, r = lr
			lefts.append(l)
			rights.append(r)
			center_px = (l + r) / 2
			centers.append(center_px)
			x_cm, z_cm = px_to_ground_cm(center_px, sy)
			centers_cm.append(x_cm)
			zs_cm.append(z_cm)
			widths.append(r - l)
			ys.append(sy)
			if ENABLE_DEBUG_DRAW:
				img.draw_cross(l, sy, color=draw_color, size=3, thickness=1)
				img.draw_cross(r, sy, color=draw_color, size=3, thickness=1)
		i += 1

	if len(lefts) < MIN_VALID_LINES:
		return None

	l_med = median(lefts)
	r_med = median(rights)
	center = median(centers)
	center_cm = median(centers_cm)
	dist_cm = median(zs_cm)
	width_std = stdev(widths)
	conf = (len(centers) / SCAN_LINES_PER_ROI) * (1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0))

	# 画该ROI估计到的中位线点。
	cy = y + h // 2
	if ENABLE_DEBUG_DRAW:
		img.draw_cross(int(center), cy, color=draw_color, size=4, thickness=1)

	# 在地面坐标拟合中心线斜率，用于航向角误差。
	a, b = line_fit(zs_cm, centers_cm)
	angle = math.degrees(math.atan(a))

	return {
		"center": center,
		"center_cm": center_cm,
		"dist_cm": dist_cm,
		"left": l_med,
		"right": r_med,
		"weight": weight,
		"cy": cy,
		"angle": angle,
		"conf": conf,
		"count": len(centers),
		"width_std": width_std,
	}


def nonlinear_map(v):
	# S形映射，抑制大偏差时的过转。
	return STEER_SAT * math.tanh(v / STEER_SCALE)


def pid_step(err, far_err_cm, curve_force=False):
	global integral_term, last_err, last_ms

	now = now_ms()
	dt_ms = time.ticks_diff(now, last_ms)
	if dt_ms <= 0:
		dt_ms = 1
	dt = dt_ms / 1000.0
	last_ms = now

	if (not curve_force) and (abs(far_err_cm) < CURVE_SWITCH_CM):
		kp = KP_STRAIGHT
		ki = KI_STRAIGHT
		kd = KD_STRAIGHT
	else:
		kp = KP_CURVE
		ki = KI_CURVE
		kd = KD_CURVE

	integral_term += err * dt
	integral_term = clamp(integral_term, -I_CLAMP, I_CLAMP)

	derr = (err - last_err) / dt
	last_err = err

	return kp * err + ki * integral_term + kd * derr


def steer_to_cmd(steer):
	# steer > 0：赛道中线偏右，机器人应向右纠偏。
	if abs(steer) <= DEADBAND:
		return ROUTE_GO
	if steer > 0:
		return ROUTE_RIGHT
	if steer < -18:
		return ROUTE_LEFT
	return ROUTE_SLIGHT_LEFT


LED(1).on()
LED(2).on()
LED(3).on()
build_camera_lut()

while True:
	clock.tick()
	img = sensor.snapshot()
	avg_conf = 0.0
	angle_err = 0.0
	base_err_cm = 0.0
	far_dist_cm = 0.0
	turn_gate = 0.0
	curve_mode = 0

	if USE_HISTEQ:
		img = img.histeq()

	black_th = adaptive_black_threshold(img)
	binary_mode = False
	if USE_BINARY:
		img = img.binary([(0, black_th)], invert=False)
		binary_mode = True
		black_th = 0

	if USE_ERODE:
		img.erode(ERODE_ITER)

	draw_color = 1 if binary_mode else 255
	draw_dim = 1 if binary_mode else 80

	# 可视化中心参考线。
	if ENABLE_DEBUG_DRAW:
		img.draw_line(IMG_CX, 0, IMG_CX, IMG_H - 1, color=draw_dim)

	roi_results = []
	for roi in ROIS:
		x, y, w, h, _ = roi
		if ENABLE_DEBUG_DRAW:
			img.draw_rectangle(x, y, w, h, color=draw_dim)
		res = roi_midline(img, roi, black_th, binary_mode, draw_color)
		if res is not None and res["conf"] >= CONF_MIN:
			roi_results.append(res)

	if roi_results:
		lost_frames = 0

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
		else:
			# 使用最上方（最远）有效ROI作为前瞻误差。
			far = roi_results[-1]
			near = roi_results[0]
			far_err_cm = far["center_cm"]
			near_err_cm = near["center_cm"]
			far_dist_cm = far["dist_cm"]

			base_err_cm = weighted_sum / weight_total
			angle_err = angle_sum / angle_weight if angle_weight > 0 else 0.0
			avg_conf = (weight_total / weight_nominal) if weight_nominal > 0 else 0.0
			curve_err = far_err_cm - near_err_cm

			# 距离与偏差联合触发提前入弯。
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

			smoothed_err = (SMOOTH_ALPHA * smoothed_err) + ((1.0 - SMOOTH_ALPHA) * fused_err)
			pid_out = pid_step(smoothed_err, far_err_cm, curve_mode_force)
			steer = nonlinear_map(pid_out)
			last_steer = steer

	if not roi_results:
		# 丢线时短时保持历史转向，长时进入搜索。
		lost_frames += 1
		if lost_frames <= LOST_HOLD_FRAMES:
			steer = last_steer * 0.85
		else:
			if last_steer >= 0:
				steer = LOST_SEARCH_TURN
			else:
				steer = -LOST_SEARCH_TURN

	cmd = steer_to_cmd(steer)

	# 发送节流，避免串口刷爆主控。
	n = now_ms()
	if time.ticks_diff(n, last_send_ms) >= SEND_INTERVAL_MS:
		send_route_cmd(cmd)
		last_send_ms = n

	print(
		"th=%d steer=%.1f ex=%.1fcm ang=%.1fdeg z=%.1fcm tg=%.2f mode=%d conf=%.2f cmd=%s lost=%d fps=%.1f"
		% (black_th, steer, base_err_cm, angle_err, far_dist_cm, turn_gate, curve_mode, avg_conf, cmd, lost_frames, clock.fps())
	)
