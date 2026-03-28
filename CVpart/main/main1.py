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
ANGLE_GAIN = 0.55
MIN_WEIGHT = 0.10

# 前瞻增益：让远端信息提前参与控制。
LOOKAHEAD_GAIN = 0.35

# 双模式PID参数（直道/弯道）。
CURVE_SWITCH_PX = 12
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
	widths = []
	ys = []

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
			centers.append((l + r) / 2)
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
	width_std = stdev(widths)
	conf = (len(centers) / SCAN_LINES_PER_ROI) * (1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0))

	# 画该ROI估计到的中位线点。
	cy = y + h // 2
	if ENABLE_DEBUG_DRAW:
		img.draw_cross(int(center), cy, color=draw_color, size=4, thickness=1)

	# 拟合中心线斜率，用于角度误差。
	a, b = line_fit(ys, centers)
	angle = math.degrees(math.atan(a))

	return {
		"center": center,
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


def pid_step(err, far_err):
	global integral_term, last_err, last_ms

	now = now_ms()
	dt_ms = time.ticks_diff(now, last_ms)
	if dt_ms <= 0:
		dt_ms = 1
	dt = dt_ms / 1000.0
	last_ms = now

	if abs(far_err) < CURVE_SWITCH_PX:
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

while True:
	clock.tick()
	img = sensor.snapshot()

	# 可视化中心参考线。
	img.draw_line(IMG_CX, 0, IMG_CX, IMG_H - 1, color=100)

	black_th = adaptive_black_threshold(img)

	roi_results = []
	for roi in ROIS:
		x, y, w, h, _ = roi
		img.draw_rectangle(x, y, w, h, color=80)
		res = roi_midline(img, roi, black_th)
		if res is not None:
			roi_results.append(res)

	if roi_results:
		lost_frames = 0

		weighted_sum = 0.0
		weight_total = 0.0
		far_err = 0.0

		for r in roi_results:
			err = r["center"] - IMG_CX
			weighted_sum += err * r["weight"]
			weight_total += r["weight"]

		# 使用最上方（最远）有效ROI作为前瞻误差。
		far = roi_results[-1]
		far_err = far["center"] - IMG_CX

		base_err = weighted_sum / weight_total
		fused_err = base_err + LOOKAHEAD_GAIN * far_err

		pid_out = pid_step(fused_err, far_err)
		steer = nonlinear_map(pid_out)
		last_steer = steer
	else:
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
		"th=%d steer=%.1f cmd=%s lost=%d fps=%.1f"
		% (black_th, steer, cmd, lost_frames, clock.fps())
	)
