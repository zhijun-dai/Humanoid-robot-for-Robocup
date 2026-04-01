import math
import os
import time as _pytime


def env_float(name, default):
	raw = os.getenv(name)
	if raw is None or raw == "":
		return default
	try:
		return float(raw)
	except ValueError:
		return default


def env_int(name, default):
	raw = os.getenv(name)
	if raw is None or raw == "":
		return default
	try:
		return int(raw)
	except ValueError:
		return default


def env_bool(name, default):
	raw = os.getenv(name)
	if raw is None:
		return default
	raw = raw.strip().lower()
	if raw in ("1", "true", "yes", "on"):
		return True
	if raw in ("0", "false", "no", "off"):
		return False
	return default


class UART:
	def __init__(self, uart_id, baud):
		self.uart_id = uart_id
		self.baud = baud
		self.last = ""

	def write(self, data):
		self.last = data


class LED:
	def __init__(self, idx):
		self.idx = idx

	def on(self):
		return None


class SimClock:
	def __init__(self):
		self._last = _pytime.perf_counter()
		self._fps = 30.0

	def tick(self):
		now = _pytime.perf_counter()
		dt = now - self._last
		if dt <= 1e-6:
			dt = 1e-6
		self._fps = 1.0 / dt
		self._last = now

	def fps(self):
		return self._fps


class time:
	@staticmethod
	def clock():
		return SimClock()

	@staticmethod
	def ticks_ms():
		return int(_pytime.perf_counter() * 1000)

	@staticmethod
	def ticks_diff(now, prev):
		return now - prev


class FakeThreshold:
	def __init__(self, value):
		self._value = int(value)

	def value(self):
		return self._value


class FakeHistogram:
	def __init__(self, pixels):
		self.pixels = pixels

	def get_threshold(self):
		hist = [0] * 256
		for row in self.pixels:
			for p in row:
				hist[p] += 1

		total = 0
		sum_all = 0
		i = 0
		while i < 256:
			total += hist[i]
			sum_all += i * hist[i]
			i += 1

		sum_b = 0
		w_b = 0
		var_max = -1.0
		threshold = 64
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
			var_between = w_b * w_f * (m_b - m_f) * (m_b - m_f)
			if var_between > var_max:
				var_max = var_between
				threshold = t
			t += 1

		return FakeThreshold(threshold)


class FakeImage:
	def __init__(self, pixels):
		self.pixels = pixels
		self.h = len(pixels)
		self.w = len(pixels[0]) if self.h > 0 else 0

	def histeq(self):
		return self

	def get_histogram(self):
		return FakeHistogram(self.pixels)

	def binary(self, ranges, invert=False):
		low, high = ranges[0]
		y = 0
		while y < self.h:
			x = 0
			row = self.pixels[y]
			while x < self.w:
				v = row[x]
				inside = (low <= v <= high)
				if invert:
					inside = not inside
				row[x] = 0 if inside else 255
				x += 1
			y += 1
		return self

	def erode(self, iterations):
		if iterations <= 0:
			return self
		return self

	def draw_line(self, x0, y0, x1, y1, color=255):
		return None

	def draw_rectangle(self, x, y, w, h, color=255):
		return None

	def draw_cross(self, x, y, color=255, size=3, thickness=1):
		return None

	def get_pixel(self, x, y):
		if x < 0 or x >= self.w or y < 0 or y >= self.h:
			return 255
		return self.pixels[y][x]


class CameraSim:
	def __init__(self):
		self.x_cm = env_float("SIM_INIT_X_CM", 0.0)
		self.yaw_deg = env_float("SIM_INIT_YAW_DEG", 0.0)
		self.s_cm = 0.0
		self.t_s = 0.0
		self.amp_scale = env_float("SIM_LANE_AMP_SCALE", 1.0)
		self.yaw_rate_scale = env_float("SIM_YAW_RATE_SCALE", 1.0)

	def lane_center_cm(self, z_cm):
		u = self.s_cm + z_cm
		if u < 280.0:
			x = 0.0
		elif u < 760.0:
			x = 14.0 * math.sin((u - 280.0) / 85.0)
		elif u < 1200.0:
			x = 10.0 * math.sin((u - 760.0) / 60.0 + 0.9)
		else:
			x = 12.0 * math.sin(u / 90.0)
		x += 2.0 * math.sin(0.7 * self.t_s + 0.02 * z_cm)
		x *= self.amp_scale
		return x

	def step(self, steer_deg, speed_mps, dt):
		speed_mps = clamp(speed_mps, 0.05, 2.5)
		self.t_s += dt

		yaw_rate = (2.8 * steer_deg + 3.0 * math.sin(0.15 * self.t_s)) * self.yaw_rate_scale
		self.yaw_deg = clamp(self.yaw_deg + yaw_rate * dt, -22.0, 22.0)

		yaw_rad = math.radians(self.yaw_deg)
		v_cm_s = speed_mps * 100.0
		self.x_cm += v_cm_s * math.sin(yaw_rad) * dt
		self.s_cm += v_cm_s * max(math.cos(yaw_rad), 0.2) * dt
		self.yaw_deg *= 0.996

	def render(self):
		bg = 185
		dark = 28
		pixels = [[bg for _ in range(IMG_W)] for _ in range(IMG_H)]

		track_half_cm = 10.5
		y = 0
		while y < IMG_H:
			z_cm = row_distance_cm[y]
			cmpp = row_cm_per_px[y]
			if cmpp <= 1e-6:
				cmpp = 1e-6

			lane_world = self.lane_center_cm(z_cm)
			rel_center_cm = lane_world - self.x_cm + z_cm * math.tan(math.radians(self.yaw_deg))
			center_px = int(IMG_CX + rel_center_cm / cmpp)

			half_px = int(track_half_cm / cmpp)
			min_half = MIN_TRACK_WIDTH // 2
			max_half = MAX_TRACK_WIDTH // 2
			if half_px < min_half:
				half_px = min_half
			if half_px > max_half:
				half_px = max_half

			left = center_px - half_px
			right = center_px + half_px
			if left < 0:
				left = 0
			if right >= IMG_W:
				right = IMG_W - 1

			x = left
			while x <= right:
				pixels[y][x] = dark
				x += 1
			y += 1

		return FakeImage(pixels)


class FakeSensor:
	GRAYSCALE = 0
	QQVGA = (160, 120)

	def __init__(self):
		self.sim = CameraSim()
		self.ctrl_steer = 0.0
		self.ctrl_speed = 0.9
		self.dt = 1.0 / 30.0

	def reset(self):
		return None

	def set_pixformat(self, pix):
		return None

	def set_framesize(self, size):
		return None

	def skip_frames(self, time=1500):
		return None

	def set_auto_gain(self, enabled):
		return None

	def set_auto_whitebal(self, enabled):
		return None

	def apply_control(self, steer, speed):
		self.ctrl_steer = steer
		self.ctrl_speed = speed

	def snapshot(self):
		self.sim.step(self.ctrl_steer, self.ctrl_speed, self.dt)
		return self.sim.render()


sensor = FakeSensor()


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

# 速度控制（测试专用，不改变寻迹主链）。
SPEED_BASE = 1.20
SPEED_MIN = 0.25
SPEED_MAX = 1.50
SPEED_ERR_GAIN = 0.06
SPEED_ANG_GAIN = 0.02
SPEED_TURN_GAIN = 0.35
SPEED_LOST_PENALTY = 0.25


# ---------------------- 自动调参覆写（可选） ----------------------
LOOKAHEAD_GAIN = env_float("TUNE_LOOKAHEAD_GAIN", LOOKAHEAD_GAIN)
KP_STRAIGHT = env_float("TUNE_KP_STRAIGHT", KP_STRAIGHT)
KI_STRAIGHT = env_float("TUNE_KI_STRAIGHT", KI_STRAIGHT)
KD_STRAIGHT = env_float("TUNE_KD_STRAIGHT", KD_STRAIGHT)
KP_CURVE = env_float("TUNE_KP_CURVE", KP_CURVE)
KI_CURVE = env_float("TUNE_KI_CURVE", KI_CURVE)
KD_CURVE = env_float("TUNE_KD_CURVE", KD_CURVE)
STEER_SCALE = env_float("TUNE_STEER_SCALE", STEER_SCALE)
DEADBAND = env_float("TUNE_DEADBAND", DEADBAND)
CURVE_GAIN = env_float("TUNE_CURVE_GAIN", CURVE_GAIN)
ANGLE_GAIN = env_float("TUNE_ANGLE_GAIN", ANGLE_GAIN)
TURN_ERR_CM = env_float("TUNE_TURN_ERR_CM", TURN_ERR_CM)

SPEED_BASE = env_float("TUNE_SPEED_BASE", SPEED_BASE)
SPEED_MIN = env_float("TUNE_SPEED_MIN", SPEED_MIN)
SPEED_MAX = env_float("TUNE_SPEED_MAX", SPEED_MAX)
SPEED_ERR_GAIN = env_float("TUNE_SPEED_ERR_GAIN", SPEED_ERR_GAIN)
SPEED_ANG_GAIN = env_float("TUNE_SPEED_ANG_GAIN", SPEED_ANG_GAIN)
SPEED_TURN_GAIN = env_float("TUNE_SPEED_TURN_GAIN", SPEED_TURN_GAIN)
SPEED_LOST_PENALTY = env_float("TUNE_SPEED_LOST_PENALTY", SPEED_LOST_PENALTY)

CONF_MIN = env_float("TUNE_CONF_MIN", CONF_MIN)
TH_OFFSET = env_int("TUNE_TH_OFFSET", TH_OFFSET)
ENABLE_DEBUG_DRAW = env_bool("TUNE_ENABLE_DEBUG_DRAW", ENABLE_DEBUG_DRAW)


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


def speed_step(base_err_cm, angle_err, turn_gate, lost_count):
	v = SPEED_BASE
	v -= SPEED_ERR_GAIN * abs(base_err_cm)
	v -= SPEED_ANG_GAIN * abs(angle_err)
	v -= SPEED_TURN_GAIN * turn_gate
	if lost_count > 0:
		v -= SPEED_LOST_PENALTY
	return clamp(v, SPEED_MIN, SPEED_MAX)


LED(1).on()
LED(2).on()
LED(3).on()
build_camera_lut()

MAX_FRAMES = 900
MAX_FRAMES = env_int("TUNE_MAX_FRAMES", MAX_FRAMES)
PRINT_EVERY = env_int("TUNE_PRINT_EVERY", 1)
frame_id = 0

sum_abs_ex = 0.0
sum_abs_ang = 0.0
sum_speed = 0.0
sum_conf = 0.0
sum_turn_gate = 0.0
lost_count_total = 0
steer_sat_frames = 0

while True:
	frame_id += 1
	if MAX_FRAMES > 0 and frame_id > MAX_FRAMES:
		break

	clock.tick()
	img = sensor.snapshot()
	avg_conf = 0.0
	angle_err = 0.0
	base_err_cm = 0.0
	far_dist_cm = 0.0
	turn_gate = 0.0
	curve_mode = 0
	steer = 0.0

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
	speed_mps = speed_step(base_err_cm, angle_err, turn_gate, lost_frames)
	sensor.apply_control(steer, speed_mps)

	sum_abs_ex += abs(base_err_cm)
	sum_abs_ang += abs(angle_err)
	sum_speed += speed_mps
	sum_conf += avg_conf
	sum_turn_gate += turn_gate
	lost_count_total += 1 if lost_frames > 0 else 0
	if abs(steer) >= 0.9 * STEER_SAT:
		steer_sat_frames += 1

	# 发送节流，避免串口刷爆主控。
	n = now_ms()
	if time.ticks_diff(n, last_send_ms) >= SEND_INTERVAL_MS:
		send_route_cmd(cmd)
		last_send_ms = n

	if PRINT_EVERY <= 1 or (frame_id % PRINT_EVERY) == 0:
		print(
			"f=%d th=%d steer=%.1f v=%.2f ex=%.1fcm ang=%.1fdeg z=%.1fcm tg=%.2f mode=%d conf=%.2f cmd=%s lost=%d x=%.1f yaw=%.1f fps=%.1f"
			% (
				frame_id,
				black_th,
				steer,
				speed_mps,
				base_err_cm,
				angle_err,
				far_dist_cm,
				turn_gate,
				curve_mode,
				avg_conf,
				cmd,
				lost_frames,
				sensor.sim.x_cm,
				sensor.sim.yaw_deg,
				clock.fps(),
			)
		)


frames = frame_id if frame_id > 0 else 1
mae_ex = sum_abs_ex / frames
mae_ang = sum_abs_ang / frames
mean_speed = sum_speed / frames
mean_conf = sum_conf / frames
mean_turn_gate = sum_turn_gate / frames
lost_ratio = lost_count_total / frames
steer_sat_ratio = steer_sat_frames / frames

# 分数越低越好：偏差/丢线/打满惩罚，速度和置信度给奖励。
score = (
	mae_ex
	+ 0.32 * mae_ang
	+ 120.0 * lost_ratio
	+ 8.0 * steer_sat_ratio
	+ 1.8 * mean_turn_gate
	- 1.2 * mean_speed
	- 0.8 * mean_conf
)

print(
	"RESULT score=%.4f mae_ex=%.4f mae_ang=%.4f mean_speed=%.4f mean_conf=%.4f lost_ratio=%.4f sat_ratio=%.4f"
	% (score, mae_ex, mae_ang, mean_speed, mean_conf, lost_ratio, steer_sat_ratio)
)
