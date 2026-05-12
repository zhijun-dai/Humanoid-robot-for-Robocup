# qr_test_hardened.py — QR 专项测试（算法/参数与 main_webots_aligned.py 一致）
#
# 用法：OpenMV IDE 终端运行，观察实时输出。
# 输出内容：
#   启动 — 打印当前 QR 全部参数
#   每帧 — 检测到几号码 / 尺寸 / 哪种解码路径命中 / 帧率
#   触发 — 候选累积 → 稳定确认 → 发送（含首帧→发送延迟 ms）
#   定期 — 每秒统计摘要（帧数/检测次数/发送次数/平均帧率）
#
# 调试参数可在 line_follow_params.json → openmv_webots_aligned 中调整：
#   qr_patch_scale, qr_roi_height_frac, qr_lens_corr_strength,
#   qr_stable_frames, qr_send_cooldown_ms, qr_min_pixels, qr_max_pixels, qr_min_area

import sensor
import image
import time
from pyb import UART, LED

try:
	import ujson as json
except ImportError:
	import json


# ── 配置加载（与主程序同一套逻辑）──────────────────────────────
def _cfg_get(cfg, path, default):
	cur = cfg
	for key in path.split("."):
		if not isinstance(cur, dict) or key not in cur:
			return default
		cur = cur[key]
	return cur


def _load_shared_cfg():
	for p in (
		"line_follow_params.json",
		"../line_follow_params.json",
		"../../line_follow_params.json",
	):
		try:
			with open(p, "r") as f:
				return json.load(f)
		except Exception:
			pass
	return {}


SHARED_CFG = _load_shared_cfg()
OMV_WA = _cfg_get(SHARED_CFG, "openmv_webots_aligned", {}) or {}

# ── QR 参数（与主程序完全一致）──────────────────────────────────
QR_ENABLE = bool(OMV_WA.get("qr_enable", True))
QR_EVERY_N_FRAMES = max(1, int(OMV_WA.get("qr_every_n_frames", 3)))
QR_LENS_CORR = float(OMV_WA.get("qr_lens_corr_strength", 1.35))
QR_STABLE_FRAMES = max(1, int(OMV_WA.get("qr_stable_frames", 3)))
QR_COOLDOWN_MS = max(400, int(OMV_WA.get("qr_send_cooldown_ms", 3200)))
QR_RETRY_NO_LENS = bool(OMV_WA.get("qr_retry_without_lens_corr", True))
QR_MIN_PIXELS = int(OMV_WA.get("qr_min_pixels", 12))
QR_MAX_PIXELS = int(OMV_WA.get("qr_max_pixels", 300))
QR_MIN_AREA = int(OMV_WA.get("qr_min_area", 60))
QR_PATCH_SCALE = int(OMV_WA.get("qr_patch_scale", 3))
QR_ROI_W = float(OMV_WA.get("qr_roi_width_frac", 1.0))
QR_ROI_H = float(OMV_WA.get("qr_roi_height_frac", 0.55))
QR_ROI_ANCHOR = str(OMV_WA.get("qr_roi_y_anchor", "bottom"))
QR_HISTEQ = bool(OMV_WA.get("qr_decode_histeq", True))
QR_ACTION_MAP = {"1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6}

# 测试模式控制
SEND_UART = True              # 是否实际发送 UART（False=纯观察模式）
DRAW_DEBUG = True             # 是否在图像上画框和文字
LOG_EVERY_FRAME = True        # True=逐帧打日志，False=仅摘要

RESOLUTION = str(OMV_WA.get("qr_test_resolution", "QQVGA")).upper()
if RESOLUTION == "QVGA":
	SENSOR_FRAMESIZE = sensor.QVGA
elif RESOLUTION == "QQVGA":
	SENSOR_FRAMESIZE = sensor.QQVGA
else:
	SENSOR_FRAMESIZE = sensor.QQVGA


# ── 工具函数 ────────────────────────────────────────────────────
def clamp(v, lo, hi):
	if v < lo:
		return lo
	if v > hi:
		return hi
	return v


def now_ms():
	return time.ticks_ms()


# ── QR 管线（与主程序 _qr_image_for_decode / _qr_extract_payload / _qr_try_decode_from_base 完全一致）──

def _qr_image_for_decode(src):
	w0 = src.width()
	h0 = src.height()
	rw = clamp(QR_ROI_W, 0.2, 1.0)
	rh = clamp(QR_ROI_H, 0.2, 1.0)
	if rw >= 0.999 and rh >= 0.999:
		qimg = src.copy()
	else:
		roi_w = max(8, min(w0, int(w0 * rw + 0.5)))
		roi_h = max(8, min(h0, int(h0 * rh + 0.5)))
		rx = max(0, (w0 - roi_w) // 2)
		if QR_ROI_ANCHOR == "center":
			ry = max(0, (h0 - roi_h) // 2)
		else:
			ry = max(0, h0 - roi_h)
		qimg = src.copy(roi=(rx, ry, roi_w, roi_h))
	ps = max(1, min(QR_PATCH_SCALE, 4))
	if ps > 1:
		nw = max(8, qimg.width() * ps)
		nh = max(8, qimg.height() * ps)
		try:
			qimg = qimg.resize(nw, nh)
		except Exception:
			pass
	if QR_HISTEQ:
		try:
			qimg = qimg.histeq()
		except Exception:
			pass
	return qimg


def _qr_extract_payload(qimg):
	if qimg is None:
		return None, None
	try:
		qrs = qimg.find_qrcodes()
	except Exception:
		return None, None
	if not qrs:
		return None, None
	best = None
	best_area = 0
	best_dbg = None
	all_info = []
	for qr in qrs:
		pl = qr.payload().strip()
		ok = pl in QR_ACTION_MAP
		w, h = qr.w(), qr.h()
		all_info.append((pl, w, h, ok, qr.x(), qr.y()))
		if not ok:
			continue
		mx = max(w, h)
		mn = min(w, h)
		area = w * h
		if mx < QR_MIN_PIXELS or mx > QR_MAX_PIXELS:
			continue
		if mn <= 0:
			continue
		if area < QR_MIN_AREA:
			continue
		if area > best_area:
			best_area = area
			best = pl
			best_dbg = {"w": w, "h": h, "x": qr.x(), "y": qr.y(), "area": area, "all": all_info}
	return best, best_dbg


def _qr_try_decode_from_base(qbase):
	if qbase is None:
		return None, None
	order = []
	if QR_LENS_CORR > 0.01:
		order.append(True)
	order.append(False)
	for use_lens in order:
		if use_lens is False and len(order) > 1 and (not QR_RETRY_NO_LENS):
			break
		try:
			qx = qbase.copy()
			if use_lens:
				qx = qx.lens_corr(QR_LENS_CORR)
			pl, dbg = _qr_extract_payload(qx)
			if pl is not None:
				if dbg is not None:
					dbg["lens"] = use_lens
				return pl, dbg
		except Exception:
			pass
	return None, None


# ── 绘制调试框 ──────────────────────────────────────────────────
def draw_qr_debug(img, all_qr_info):
	"""在主图像上绘制所有检出二维码的框和标签。"""
	for pl, w, h, ok, qx, qy in all_qr_info:
		color = 255 if ok else 100
		try:
			img.draw_rectangle(qx, qy, w, h, color=color, thickness=2)
			label = "%s %dx%d" % (pl, w, h)
			img.draw_string(qx, max(0, qy - 10), label, color=color, scale=1)
		except Exception:
			pass


# ── 初始化硬件 ──────────────────────────────────────────────────
UART_ID = int(_cfg_get(SHARED_CFG, "output.uart.id", 1))
UART_BAUD = int(_cfg_get(SHARED_CFG, "output.uart.baud", 115200))
try:
	uart = UART(UART_ID, UART_BAUD, timeout_char=100)
except TypeError:
	uart = UART(UART_ID, UART_BAUD)

LED(1).on()
LED(2).on()
LED(3).off()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(SENSOR_FRAMESIZE)
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

IMG_W = sensor.width()
IMG_H = sensor.height()


# ── 打印启动信息 ────────────────────────────────────────────────
def fmt_bool(v):
	return "1" if v else "0"


print("")
print("=" * 60)
print("  QR Test — Hardened Pipeline  (algorithm mirrors main_webots_aligned.py)")
print("=" * 60)
print("  resolution     : %dx%d  %s" % (IMG_W, IMG_H, RESOLUTION))
print("  patch_scale    : %d" % QR_PATCH_SCALE)
print("  roi            : w=%.2f  h=%.2f  anchor=%s" % (QR_ROI_W, QR_ROI_H, QR_ROI_ANCHOR))
print("  histeq         : %s" % fmt_bool(QR_HISTEQ))
print("  lens_corr      : %.2f  (retry_no_lens=%s)" % (QR_LENS_CORR, fmt_bool(QR_RETRY_NO_LENS)))
print("  stable_frames  : %d" % QR_STABLE_FRAMES)
print("  cooldown_ms    : %d" % QR_COOLDOWN_MS)
print("  size_filter    : min=%dpx  max=%dpx  min_area=%d" % (QR_MIN_PIXELS, QR_MAX_PIXELS, QR_MIN_AREA))
print("  every_n_frames : %d" % QR_EVERY_N_FRAMES)
print("  send_uart      : %s" % fmt_bool(SEND_UART))
print("  draw_debug     : %s" % fmt_bool(DRAW_DEBUG))
if not SHARED_CFG:
	print("  WARNING: could not load line_follow_params.json — using defaults")
print("-" * 60)
print("  Ready. Point camera at a QR code (1-6).")
print("")
print("  LEGEND:  . = no QR   3! = detected 3   3* = candidate building")
print("           3>>> = SENT 3  via UART        3? = rejected by filter")
print("=" * 60)
print("")

# ── 运行时状态 ──────────────────────────────────────────────────
clock = time.clock()
frame_idx = 0
qr_tick = 0
last_qr_send_ms = None
qr_candidate = None
qr_candidate_count = 0
first_candidate_ms = None       # 候选首次出现的时间（测延迟）

# 统计
stat_total_frames = 0
stat_detect_frames = 0          # 有 QR 检出的帧
stat_sent_count = 0             # 发送次数
stat_reject_count = 0           # 被尺寸过滤的次数
last_stat_ms = now_ms()
stat_latency_ms = []            # 延迟样本

# ── 主循环 ──────────────────────────────────────────────────────
while True:
	clock.tick()
	frame_idx += 1
	stat_total_frames += 1
	n = now_ms()

	img = sensor.snapshot()

	# 跳帧（与主程序一致）
	qr_tick += 1
	if qr_tick % QR_EVERY_N_FRAMES != 0:
		if LOG_EVERY_FRAME:
			print(".", end="")
		continue

	# ── QR 解码（与主程序 100% 一致） ──
	t0 = time.ticks_us()
	qbase = _qr_image_for_decode(img)
	qr_pl, qr_dbg = _qr_try_decode_from_base(qbase)
	t_decode_us = time.ticks_diff(time.ticks_us(), t0)

	# ── 提取全部检出信息用于调试绘制 ──
	all_qr_info = []
	if qr_dbg is not None and "all" in qr_dbg:
		all_qr_info = qr_dbg["all"]
	if DRAW_DEBUG and all_qr_info:
		draw_qr_debug(img, all_qr_info)

	# ── 终端输出 ──
	if qr_pl is not None:
		stat_detect_frames += 1
		w = qr_dbg["w"] if qr_dbg else 0
		h = qr_dbg["h"] if qr_dbg else 0
		lens = qr_dbg.get("lens", "?") if qr_dbg else "?"
		lens_str = "corr" if lens is True else ("raw" if lens is False else str(lens))

		# 去重 + 稳定计数
		if qr_pl == qr_candidate:
			qr_candidate_count += 1
		else:
			qr_candidate = qr_pl
			qr_candidate_count = 1
			first_candidate_ms = n
			print("")
			print("  [%06d] NEW  pl=%s  %dx%d px  lens=%s  decode=%dus"
				% (frame_idx, qr_pl, w, h, lens_str, t_decode_us))

		# 稳定确认 → 发送
		sent = False
		if qr_candidate_count >= QR_STABLE_FRAMES:
			ready = (last_qr_send_ms is None
				or time.ticks_diff(n, last_qr_send_ms) >= QR_COOLDOWN_MS)
			if ready:
				latency = time.ticks_diff(n, first_candidate_ms) if first_candidate_ms else -1
				u = QR_ACTION_MAP[qr_pl]
				if SEND_UART:
					try:
						uart.write(bytes([u]))
					except Exception:
						pass
				last_qr_send_ms = n
				qr_candidate_count = 0
				stat_sent_count += 1
				stat_latency_ms.append(latency)
				sent = True
				LED(2).toggle()
				print("")
				print("  [%06d] >>> SEND action=%d (pl=%s)  latency=%dms  %dx%d px  lens=%s"
					% (frame_idx, u, qr_pl, latency, w, h, lens_str))
				print("")

		# 逐帧符号
		if not sent and LOG_EVERY_FRAME:
			if qr_candidate_count > 0:
				print("%s*" % qr_pl, end="")      # 候选累积中
			else:
				print("%s!" % qr_pl, end="")      # 刚检测到
	else:
		# 未检出有效 QR
		if qr_candidate is not None:
			# 之前有候选，现在丢了
			if qr_dbg is None and LOG_EVERY_FRAME:
				print("_", end="")                # 彻底没检出
			else:
				if LOG_EVERY_FRAME:
					print("?", end="")            # 有检出但被过滤
				stat_reject_count += 1
		else:
			if LOG_EVERY_FRAME:
				print(".", end="")
		qr_candidate = None
		qr_candidate_count = 0
		first_candidate_ms = None

	# ── 每秒统计摘要 ──
	if time.ticks_diff(n, last_stat_ms) >= 1000:
		elapsed_s = time.ticks_diff(n, last_stat_ms) / 1000.0
		fps = stat_total_frames / max(1, elapsed_s) if elapsed_s > 0 else 0
		detect_pct = 100.0 * stat_detect_frames / max(1, stat_total_frames)
		avg_lat = int(sum(stat_latency_ms) / max(1, len(stat_latency_ms)))
		print("")
		print("  --- stats: frames=%d  detect=%.1f%%  sent=%d  reject=%d  latency_avg=%dms  fps=%.1f ---"
			% (stat_total_frames, detect_pct, stat_sent_count, stat_reject_count, avg_lat, fps))
		# 重置每秒计数
		stat_total_frames = 0
		stat_detect_frames = 0
		stat_sent_count = 0
		stat_reject_count = 0
		stat_latency_ms = []
		last_stat_ms = n
