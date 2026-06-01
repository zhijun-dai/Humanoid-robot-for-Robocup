"""巡线模块 V0 — 严格算法移植 line_follow_transfer.py 至 OpenCV

无 IPM 鸟瞰变换（V0 基准）。从 repo 根目录读取 line_follow_params.json 获取所有参数。
移植 Webots 控制器的全部 14 个机制：
  1.  相机 LUT (cm-per-px, row distance)
  2.  Otsu 自适应阈值 (160x120 下采样)
  3.  赛道颜色检测 (auto/dark/light)
  4.  逐行边缘扫描（带 hint 引导）
  5.  游程对选择（评分排序）
  6.  单线推断（只看到一条边时补全）
  7.  三带扫描（down/mid/up 级联向下传递）
  8.  底部对称锁定（底部 20% 行）
  9.  cm 域坐标转换
  10. 像素域误差融合 (near/far/curve/angle)
  11. 抗抖动鲁棒层 (diff RMS 跟踪、自适应平滑/Kd/速率限制)
  12. 障碍物检测 (红色横杆 / 黑色交叉块)
  13. 置信度评估 (多因子复合)
  14. 底部四分位中线 + 辅助带 (simple_bottom_mode)
"""

import cv2
import numpy as np
import math
import json
import os


# ═══════════════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════════════

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
    """最小二乘直线拟合 x = a * y + b。返回 (a, b)。"""
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


def _cfg_get(cfg, path, default):
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _load_config():
    """从 repo 根目录加载 line_follow_params.json。"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(base_dir, "line_follow_params.json"),
        os.path.abspath(os.path.join(base_dir, "..", "line_follow_params.json")),
        "line_follow_params.json",
    ]
    for p in candidates:
        try:
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


# ═══════════════════════════════════════════════════════════════════════
# LineDetector 类
# ═══════════════════════════════════════════════════════════════════════

class LineDetector:
    def __init__(self, cam_w=320, cam_h=240):
        cfg = _load_config()

        # ── 相机参数 ──
        self._cam_pitch_deg = float(_cfg_get(cfg, "camera.pitch_deg", 30.0))
        self._cam_height_cm = float(_cfg_get(cfg, "camera.height_cm", 17.0))
        self._cam_vfov_deg = float(_cfg_get(cfg, "camera.vfov_deg", 52.0))
        self._cam_hfov_deg = float(_cfg_get(cfg, "camera.hfov_deg", 70.0))
        self._row_dist_min_cm = float(_cfg_get(cfg, "camera.row_dist_min_cm", 6.0))
        self._row_dist_max_cm = float(_cfg_get(cfg, "camera.row_dist_max_cm", 300.0))

        # ── 输入/处理分辨率 ──
        self._cam_w = int(cam_w)
        self._cam_h = int(cam_h)
        self._base_w = int(_cfg_get(cfg, "base_frame.width", 160))
        self._base_h = int(_cfg_get(cfg, "base_frame.height", 120))
        self._scale_x = self._cam_w / float(self._base_w)
        self._scale_y = self._cam_h / float(self._base_h)
        self._img_cx_full = self._cam_w // 2
        self._img_cx_base = self._base_w // 2

        # ── 阈值参数 ──
        self._th_offset = int(_cfg_get(cfg, "threshold.offset", 8))
        self._th_min = int(_cfg_get(cfg, "threshold.min", 25))
        self._th_max = int(_cfg_get(cfg, "threshold.max", 120))
        self._dark_margin = int(_cfg_get(cfg, "threshold.dark_margin", 12))
        self._track_color_mode = str(_cfg_get(cfg, "threshold.track_color", "auto")).lower()

        # ── ROI 通用参数 ──
        self._min_track_width = int(_cfg_get(cfg, "roi.min_track_width", 12))
        self._max_track_width = int(_cfg_get(cfg, "roi.max_track_width", 220))
        self._min_pair_ratio = float(_cfg_get(cfg, "roi.min_pair_ratio", 0.4))
        self._min_valid_lines = int(_cfg_get(cfg, "roi.min_valid_lines", 2))
        self._width_std_max = float(_cfg_get(cfg, "roi.width_std_max", 14))
        self._conf_min = float(_cfg_get(cfg, "roi.conf_min", 0.12))
        self._min_line_width = int(_cfg_get(cfg, "roi.min_line_width", 2))
        self._max_line_width = int(_cfg_get(cfg, "roi.max_line_width", 80))
        self._lane_width_init_px = float(_cfg_get(cfg, "roi.lane_width_init_px", 70.0))
        self._lane_width_tol_px = float(_cfg_get(cfg, "roi.lane_width_tol_px", 40.0))
        self._max_center_jump_px = float(_cfg_get(cfg, "roi.max_center_jump_px", 55.0))
        self._scan_lines_per_roi = int(_cfg_get(cfg, "roi.scan_lines_per_roi", 5))
        self._min_pair_lines = int(_cfg_get(cfg, "roi.min_pair_lines", 2))

        # ── Simple Bottom Mode ──
        self._simple_bottom_mode = bool(_cfg_get(cfg, "roi.simple_bottom_mode", True))
        self._bottom_start_ratio = float(_cfg_get(cfg, "roi.bottom_start_ratio", 0.75))
        self._bottom_rows = int(_cfg_get(cfg, "roi.bottom_rows", 14))
        self._bottom_step = int(_cfg_get(cfg, "roi.bottom_step", 2))
        self._single_line_conf = float(_cfg_get(cfg, "roi.single_line_conf", 0.35))
        self._assist_enable = bool(_cfg_get(cfg, "roi.assist_enable", True))
        self._assist_start_ratio = float(_cfg_get(cfg, "roi.assist_start_ratio", 0.50))
        self._assist_end_ratio = float(_cfg_get(cfg, "roi.assist_end_ratio", 0.75))
        self._assist_rows = int(_cfg_get(cfg, "roi.assist_rows", 12))
        self._assist_step = int(_cfg_get(cfg, "roi.assist_step", 2))

        # ── Three Band Mode ──
        self._three_band_mode = bool(_cfg_get(cfg, "roi.three_band_mode", True))
        self._band_down_start_ratio = float(_cfg_get(cfg, "roi.band_down_start_ratio", 2.0 / 3.0))
        self._band_mid_start_ratio = float(_cfg_get(cfg, "roi.band_mid_start_ratio", 1.0 / 3.0))
        self._band_up_start_ratio = float(_cfg_get(cfg, "roi.band_up_start_ratio", 0.0))
        self._band_rows_down = int(_cfg_get(cfg, "roi.band_rows_down", 16))
        self._band_rows_mid = int(_cfg_get(cfg, "roi.band_rows_mid", 14))
        self._band_rows_up = int(_cfg_get(cfg, "roi.band_rows_up", 12))
        self._band_step_down = int(_cfg_get(cfg, "roi.band_step_down", 2))
        self._band_step_mid = int(_cfg_get(cfg, "roi.band_step_mid", 2))
        self._band_step_up = int(_cfg_get(cfg, "roi.band_step_up", 2))
        self._band_weight_down = float(_cfg_get(cfg, "roi.band_weight_down", 0.58))
        self._band_weight_mid = float(_cfg_get(cfg, "roi.band_weight_mid", 0.30))
        self._band_weight_up = float(_cfg_get(cfg, "roi.band_weight_up", 0.12))

        # ── 障碍物检测 ──
        self._cross_black_run_ratio = float(_cfg_get(cfg, "roi.cross_black_run_ratio", 0.42))
        self._cross_black_cover_ratio = float(_cfg_get(cfg, "roi.cross_black_cover_ratio", 0.56))
        self._red_detect_enable = bool(_cfg_get(cfg, "roi.red_detect_enable", True))
        self._red_min_r = int(_cfg_get(cfg, "roi.red_min_r", 105))
        self._red_dom_margin = int(_cfg_get(cfg, "roi.red_dom_margin", 28))
        self._red_row_ratio = float(_cfg_get(cfg, "roi.red_row_ratio", 0.35))

        # ── 底部对称锁定 ──
        self._bottom_lock_enable = bool(_cfg_get(cfg, "roi.bottom_lock_enable", True))
        self._bottom_lock_start_ratio = float(_cfg_get(cfg, "roi.bottom_lock_start_ratio", 0.80))
        self._bottom_lock_rows = int(_cfg_get(cfg, "roi.bottom_lock_rows", 12))
        self._bottom_lock_step = int(_cfg_get(cfg, "roi.bottom_lock_step", 2))
        self._bottom_lock_min_pair_ratio = float(_cfg_get(cfg, "roi.bottom_lock_min_pair_ratio", 0.55))
        self._bottom_lock_sym_tol_px = float(_cfg_get(cfg, "roi.bottom_lock_sym_tol_px", 12.0))
        self._bottom_lock_blend = float(_cfg_get(cfg, "roi.bottom_lock_blend", 0.55))
        self._bottom_lock_conf_penalty = float(_cfg_get(cfg, "roi.bottom_lock_conf_penalty", 0.45))
        self._bottom_lock_speed_penalty = float(_cfg_get(cfg, "roi.bottom_lock_speed_penalty", 0.25))
        self._lock_reacquire_reset = bool(_cfg_get(cfg, "roi.lock_reacquire_reset", True))

        # ── 启动暂态 ──
        self._startup_settle_frames = int(_cfg_get(cfg, "roi.startup_settle_frames", 25))
        self._startup_speed_scale = float(_cfg_get(cfg, "roi.startup_speed_scale", 0.55))
        self._startup_conf_min_scale = float(_cfg_get(cfg, "roi.startup_conf_min_scale", 0.70))
        self._startup_min_weight_scale = float(_cfg_get(cfg, "roi.startup_min_weight_scale", 0.70))
        self._startup_force_simple_bottom = bool(_cfg_get(cfg, "roi.startup_force_simple_bottom", True))
        self._startup_lost_bias_free = bool(_cfg_get(cfg, "roi.startup_lost_bias_free", True))

        # ── 融合参数 ──
        self._smooth_alpha = float(_cfg_get(cfg, "fusion.smooth_alpha", 0.65))
        self._curve_gain = float(_cfg_get(cfg, "fusion.curve_gain", 0.25))
        self._angle_gain = float(_cfg_get(cfg, "fusion.angle_gain", 0.22))
        self._min_weight = float(_cfg_get(cfg, "fusion.min_weight", 0.10))
        self._lookahead_gain = float(_cfg_get(cfg, "fusion.lookahead_gain", 0.35))

        # ── 像素域增益 (webots.*→实际对应 fusion 里的远/曲线/角度系数) ──
        self._pix_lookahead_gain = float(_cfg_get(cfg, "webots.pixel_lookahead_gain", 0.0))
        self._pix_curve_gain = float(_cfg_get(cfg, "webots.pixel_curve_gain", 0.0))
        self._pix_angle_gain = float(_cfg_get(cfg, "webots.pixel_angle_gain", 0.06))
        self._curve_switch_px = float(_cfg_get(cfg, "webots.curve_switch_px", 18.0))
        self._left_curve_outward_gain = float(_cfg_get(cfg, "webots.left_curve_outward_gain", 0.35))
        self._left_curve_outward_px = float(_cfg_get(cfg, "webots.left_curve_outward_px", 6.0))

        # ── 抗抖动鲁棒层 ──
        robust_cfg = _cfg_get(cfg, "shake_robust", {}) or {}
        self._robust_enable = bool(robust_cfg.get("enable", True))
        self._robust_diff_window = int(robust_cfg.get("diff_window", 5))
        self._robust_diff_rms_trigger_px = float(robust_cfg.get("diff_rms_trigger_px", 6.0))
        self._robust_alpha_high = float(robust_cfg.get("alpha_high", 0.88))
        self._robust_bottom_lock_blend_scale = float(robust_cfg.get("bottom_lock_blend_scale", 1.5))
        self._robust_kd_shake_scale = float(robust_cfg.get("kd_shake_scale", 0.6))
        self._robust_decay_frames = int(robust_cfg.get("decay_frames", 8))

        # ── 构建相机 LUT (在 base 分辨率 160x120 上) ──
        self._row_dist_cm, self._row_cm_per_px = self._build_camera_lut(
            self._base_w, self._base_h
        )

        # ── 内部状态 ──
        self._state = {
            "smoothed_err": 0.0,
            "lost_frames": 0,
            "last_base_err": 0.0,
            "last_angle_err": 0.0,
            "last_far_dist": 0.0,
            "last_lane_center_x": float(self._img_cx_base),
            "last_lane_width_px": float(self._lane_width_init_px),
            "track_dark_score": 0,
            "last_band_mask": 0,
            "startup_frames": 0,
            "last_bottom_lock_valid": False,
            "near_err_history": [],
            "shake_active_frames": 0,
            "diff_rms_px": 0.0,
        }

    # ═══════════════════════════════════════════════════════════
    # 机制 1：相机 LUT
    # ═══════════════════════════════════════════════════════════

    def _build_camera_lut(self, img_w, img_h):
        row_distance_cm = [0.0] * img_h
        row_cm_per_px = [0.0] * img_h
        half_v = self._cam_vfov_deg * 0.5
        half_h = self._cam_hfov_deg * 0.5

        for y in range(img_h):
            v_deg = ((img_h * 0.5 - y) / (img_h * 0.5)) * half_v
            ray_deg = self._cam_pitch_deg + v_deg
            if ray_deg < 1.0:
                ray_deg = 1.0

            z_cm = self._cam_height_cm / math.tan(math.radians(ray_deg))
            z_cm = clamp(z_cm, self._row_dist_min_cm, self._row_dist_max_cm)
            row_distance_cm[y] = z_cm
            row_cm_per_px[y] = (2.0 * z_cm * math.tan(math.radians(half_h))) / img_w

        return row_distance_cm, row_cm_per_px

    # ═══════════════════════════════════════════════════════════
    # 机制 9：cm 域坐标转换
    # ═══════════════════════════════════════════════════════════

    def _px_to_ground_cm(self, x, y):
        img_h = self._base_h
        img_cx = self._img_cx_base
        if y < 0:
            y = 0
        elif y >= img_h:
            y = img_h - 1
        x_cm = (x - img_cx) * self._row_cm_per_px[y]
        z_cm = self._row_dist_cm[y]
        return x_cm, z_cm

    # ═══════════════════════════════════════════════════════════
    # 机制 2：Otsu 自适应阈值 (160x120 下采样)
    # ═══════════════════════════════════════════════════════════

    def _otsu_threshold(self, gray):
        """Otsu 自适应阈值——与原始代码算法一致 (手动实现，不依赖 cv2.threshold OTSU)。"""
        hist = [0] * 256
        h, w = gray.shape
        step_y = max(1, h // 30)
        step_x = max(1, w // 40)
        total = 0

        for y in range(0, h, step_y):
            for x in range(0, w, step_x):
                g = int(gray[y, x])
                hist[g] += 1
                total += 1

        if total == 0:
            return 64

        sum_all = sum(i * hist[i] for i in range(256))

        sum_b = 0
        w_b = 0
        max_var = -1.0
        best_t = 64

        for t in range(256):
            w_b += hist[t]
            if w_b == 0:
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

        return best_t

    # ═══════════════════════════════════════════════════════════
    # 机制 3：赛道颜色检测
    # ═══════════════════════════════════════════════════════════

    def _detect_track_is_dark(self, gray, black_th):
        if self._track_color_mode == "dark":
            return True
        if self._track_color_mode == "light":
            return False

        h, w = gray.shape
        dark = 0
        light = 0
        step_y = max(1, h // 20)
        step_x = max(1, w // 20)

        for y in range(0, h, step_y):
            for x in range(0, w, step_x):
                g = int(gray[y, x])
                if g <= black_th:
                    dark += 1
                else:
                    light += 1

        return dark <= light

    # ═══════════════════════════════════════════════════════════
    # 像素分类辅助
    # ═══════════════════════════════════════════════════════════

    def _pixel_is_track(self, g, black_th, track_is_dark):
        if track_is_dark:
            return g <= max(0, black_th - self._dark_margin)
        return g >= min(255, black_th + self._dark_margin)

    def _pixel_is_red(self, bgr_small, x, y):
        if not self._red_detect_enable:
            return False
        # bgr_small[y, x] = [B, G, R]
        r = int(bgr_small[y, x, 2])
        g = int(bgr_small[y, x, 1])
        b = int(bgr_small[y, x, 0])
        return (r >= self._red_min_r) and ((r - g) >= self._red_dom_margin) and ((r - b) >= self._red_dom_margin)

    # ═══════════════════════════════════════════════════════════
    # 机制 12：障碍物检测 (红色横杆 / 黑色交叉块)
    # ═══════════════════════════════════════════════════════════

    def _detect_row_blocker(self, gray, bgr_small, y, x0, x1, black_th, track_is_dark):
        total = max(1, x1 - x0 + 1)
        track_count = 0
        red_count = 0
        longest_track_run = 0
        cur_run = 0

        for x in range(x0, x1 + 1):
            g = int(gray[y, x])
            is_track = self._pixel_is_track(g, black_th, track_is_dark)
            if is_track:
                track_count += 1
                cur_run += 1
                if cur_run > longest_track_run:
                    longest_track_run = cur_run
            else:
                cur_run = 0

            if self._pixel_is_red(bgr_small, x, y):
                red_count += 1

        red_ratio = red_count / float(total)
        cover_ratio = track_count / float(total)
        run_ratio = longest_track_run / float(total)

        red_block = red_ratio >= self._red_row_ratio
        black_block = (
            run_ratio >= self._cross_black_run_ratio
            and cover_ratio >= self._cross_black_cover_ratio
        )
        return red_block, black_block

    # ═══════════════════════════════════════════════════════════
    # 机制 5：游程收集
    # ═══════════════════════════════════════════════════════════

    def _collect_track_runs_on_row(self, gray, y, x0, x1, black_th, track_is_dark):
        runs = []
        run_start = -1
        for x in range(x0, x1 + 1):
            g = int(gray[y, x])
            is_track = self._pixel_is_track(g, black_th, track_is_dark)
            if is_track and run_start < 0:
                run_start = x
            elif (not is_track) and run_start >= 0:
                run_end = x - 1
                w = run_end - run_start + 1
                if self._min_line_width <= w <= self._max_line_width:
                    runs.append((run_start, run_end))
                run_start = -1
        if run_start >= 0:
            run_end = x1
            w = run_end - run_start + 1
            if self._min_line_width <= w <= self._max_line_width:
                runs.append((run_start, run_end))
        return runs

    # ═══════════════════════════════════════════════════════════
    # 机制 4：逐行边沿扫描（带 hint 引导）
    # ═══════════════════════════════════════════════════════════

    def _find_lr_edges_on_row(self, gray, y, x0, x1, black_th, track_is_dark,
                              hint_x, lane_width_hint):
        hint = int(clamp(hint_x, x0, x1))

        left = -1
        for x in range(hint, x0 - 1, -1):
            g = int(gray[y, x])
            if self._pixel_is_track(g, black_th, track_is_dark):
                left = x
                break

        right = -1
        for x in range(hint, x1 + 1):
            g = int(gray[y, x])
            if self._pixel_is_track(g, black_th, track_is_dark):
                right = x
                break

        if left < 0 or right < 0:
            left = -1
            right = -1

        if left >= 0 and right >= 0:
            lane_w = right - left
            if lane_w < self._min_track_width or lane_w > self._max_track_width:
                left = -1
                right = -1

        if left >= 0 and right >= 0:
            center = 0.5 * (left + right)
            if abs(center - hint_x) > self._max_center_jump_px:
                left = -1
                right = -1

        if (
            left >= 0
            and right >= 0
            and lane_width_hint > 0
            and abs(lane_w - lane_width_hint) > self._lane_width_tol_px
        ):
            left = -1
            right = -1

        if left >= 0 and right >= 0:
            return left, right

        # 回退：扫描该行所有游程，找最接近 hint 和 width hint 的配对
        runs = self._collect_track_runs_on_row(gray, y, x0, x1, black_th, track_is_dark)

        if len(runs) < 2:
            return None

        best = None
        best_score = 1e9
        for i in range(len(runs)):
            li = 0.5 * (runs[i][0] + runs[i][1])
            for j in range(i + 1, len(runs)):
                rj = 0.5 * (runs[j][0] + runs[j][1])
                lane_w = rj - li
                if self._min_track_width <= lane_w <= self._max_track_width:
                    center = 0.5 * (li + rj)
                    if abs(center - hint_x) <= (self._max_center_jump_px * 1.8):
                        width_err = (
                            abs(lane_w - lane_width_hint) if lane_width_hint > 0 else 0.0
                        )
                        if lane_width_hint <= 0 or width_err <= (self._lane_width_tol_px * 2.0):
                            score = abs(center - hint_x) + 0.7 * width_err
                            if score < best_score:
                                best_score = score
                                best = (int(li), int(rj))

        return best

    # ═══════════════════════════════════════════════════════════
    # 机制 5：游程对选择（评分排序）
    # ═══════════════════════════════════════════════════════════

    def _choose_pair_center_from_runs(self, runs, hint_center, lane_width_hint, x0, x1):
        if len(runs) < 2:
            return None

        best = None
        best_score = 1e9
        for i in range(len(runs)):
            li = 0.5 * (runs[i][0] + runs[i][1])
            for j in range(i + 1, len(runs)):
                rj = 0.5 * (runs[j][0] + runs[j][1])
                lane_w = rj - li
                if self._min_track_width <= lane_w <= self._max_track_width:
                    if lane_width_hint > 0:
                        max_width_err = max(24.0, self._lane_width_tol_px * 1.6)
                        if abs(lane_w - lane_width_hint) > max_width_err:
                            continue
                    center = 0.5 * (li + rj)
                    if x0 <= center <= x1:
                        width_err = (
                            abs(lane_w - lane_width_hint) if lane_width_hint > 0 else 0.0
                        )
                        center_err = abs(center - hint_center)
                        score = 1.0 * center_err + 0.8 * width_err
                        if score < best_score:
                            best_score = score
                            best = {
                                "center_px": center,
                                "lane_width_px": lane_w,
                                "conf": 1.0,
                                "line_mode": 2,
                            }

        return best

    # ═══════════════════════════════════════════════════════════
    # 机制 6：单线推断
    # ═══════════════════════════════════════════════════════════

    def _choose_single_run_near_hint(self, runs, hint_center):
        if not runs:
            return None
        best = None
        best_err = 1e9
        for run in runs:
            c = 0.5 * (run[0] + run[1])
            err = abs(c - hint_center)
            if err < best_err:
                best_err = err
                best = run
        return best

    def _infer_center_from_single_run(self, run, hint_center, lane_width_hint, x0, x1):
        c = 0.5 * (run[0] + run[1])
        w = max(float(lane_width_hint), float(self._min_track_width))
        img_cx = self._img_cx_base

        cand_left = c + 0.5 * w
        cand_right = c - 0.5 * w

        if abs(cand_left - hint_center) < abs(cand_right - hint_center):
            center = cand_left
        elif abs(cand_left - hint_center) > abs(cand_right - hint_center):
            center = cand_right
        else:
            center = cand_left if c < img_cx else cand_right

        center = clamp(center, x0, x1)
        return {
            "center_px": center,
            "lane_width_px": w,
            "conf": self._single_line_conf,
            "line_mode": 1,
        }

    # ═══════════════════════════════════════════════════════════
    # 机制 7：三带扫描（级联）
    # ═══════════════════════════════════════════════════════════

    def _scan_band_midline(self, gray, bgr_small, black_th, track_is_dark,
                           hint_x, lane_width_hint,
                           y_start_ratio, y_end_ratio, max_rows, row_step):
        row_step = max(1, row_step)
        img_w = self._base_w
        img_h = self._base_h
        img_cx = self._img_cx_base
        x0 = 0
        x1 = img_w - 1
        y_start = int(clamp(y_start_ratio * img_h, 0, img_h - 1))
        y_end = int(clamp(y_end_ratio * img_h, 0, img_h - 1))
        if y_end < y_start:
            y_end = y_start

        centers_px = []
        centers_cm = []
        lane_widths = []
        ys = []
        zs_cm = []
        conf_sum = 0.0
        pair_rows = 0
        single_rows = 0
        red_block_rows = 0
        black_block_rows = 0

        last_center = hint_x
        last_width = lane_width_hint

        rows_done = 0
        y = y_start
        while y <= y_end and rows_done < max_rows:
            red_block, black_block = self._detect_row_blocker(
                gray, bgr_small, y, x0, x1, black_th, track_is_dark
            )
            if red_block:
                red_block_rows += 1
                rows_done += 1
                y += row_step
                continue
            if black_block:
                black_block_rows += 1
                rows_done += 1
                y += row_step
                continue

            runs = self._collect_track_runs_on_row(
                gray, y, x0, x1, black_th, track_is_dark
            )
            chosen = self._choose_pair_center_from_runs(
                runs, last_center, last_width, x0, x1
            )
            if chosen is None and len(runs) >= 1:
                best_run = self._choose_single_run_near_hint(runs, last_center)
                if best_run is not None:
                    chosen = self._infer_center_from_single_run(
                        best_run, last_center, last_width, x0, x1
                    )

            if chosen is not None:
                center_px = chosen["center_px"]
                lane_w = chosen["lane_width_px"]
                if abs(center_px - last_center) > (self._max_center_jump_px * 2.2):
                    rows_done += 1
                    y += row_step
                    continue
                x_cm, z_cm = self._px_to_ground_cm(center_px, y)
                centers_px.append(center_px)
                centers_cm.append(x_cm)
                lane_widths.append(lane_w)
                ys.append(y)
                zs_cm.append(z_cm)
                conf_sum += chosen["conf"]
                if int(chosen.get("line_mode", 1)) >= 2:
                    pair_rows += 1
                else:
                    single_rows += 1
                last_center = center_px
                last_width = lane_w

            rows_done += 1
            y += row_step

        if len(centers_px) < 3:
            return None

        center_px = median(centers_px)
        center_cm = median(centers_cm)
        lane_width_px = median(lane_widths)
        dist_cm = median(zs_cm)
        width_std = stdev(lane_widths)
        a, _ = line_fit(ys, centers_px)
        angle = math.degrees(math.atan(a))

        hit_ratio = len(centers_px) / float(max(1, max_rows))
        conf_raw = (conf_sum / float(max(1, len(centers_px)))) * hit_ratio
        conf = conf_raw * (1.0 - clamp(width_std / max(self._width_std_max, 1e-6), 0.0, 1.0))
        blocker_ratio = (red_block_rows + black_block_rows) / float(max(1, max_rows))
        if blocker_ratio > 0.25:
            conf *= (1.0 - 0.55 * clamp((blocker_ratio - 0.25) / 0.75, 0.0, 1.0))

        valid_rows = max(1, pair_rows + single_rows)
        pair_ratio = pair_rows / float(valid_rows)
        single_ratio = single_rows / float(valid_rows)

        return {
            "center_cm": center_cm,
            "center_px": center_px,
            "dist_cm": dist_cm,
            "lane_width_px": lane_width_px,
            "weight": 1.0,
            "angle": angle,
            "conf": conf,
            "pair_ratio": pair_ratio,
            "single_ratio": single_ratio,
            "red_block_ratio": red_block_rows / float(max(1, max_rows)),
            "black_block_ratio": black_block_rows / float(max(1, max_rows)),
        }

    # ═══════════════════════════════════════════════════════════
    # 机制 14：底部四分位中线 + 辅助带
    # ═══════════════════════════════════════════════════════════

    def _bottom_quarter_midline(self, gray, bgr_small, black_th, track_is_dark,
                                hint_x, lane_width_hint):
        base = self._scan_band_midline(
            gray, bgr_small, black_th, track_is_dark,
            hint_x, lane_width_hint,
            self._bottom_start_ratio, 1.0,
            self._bottom_rows, self._bottom_step,
        )
        if base is None:
            return None

        if self._assist_enable:
            assist = self._scan_band_midline(
                gray, bgr_small, black_th, track_is_dark,
                base["center_px"], base["lane_width_px"],
                self._assist_start_ratio, self._assist_end_ratio,
                self._assist_rows, self._assist_step,
            )
            if assist is not None:
                base["assist_center_px"] = assist["center_px"]
                base["assist_center_cm"] = assist["center_cm"]
                base["assist_dist_cm"] = assist["dist_cm"]
                base["assist_angle_deg"] = assist["angle"]
                base["assist_conf"] = assist["conf"]

        return base

    # ═══════════════════════════════════════════════════════════
    # 机制 7：三带检测（级联调用）
    # ═══════════════════════════════════════════════════════════

    def _detect_three_band_lanes(self, gray, bgr_small, black_th, track_is_dark,
                                 hint_x, lane_width_hint):
        band_specs = [
            ("down", self._band_down_start_ratio, 1.0,
             self._band_rows_down, self._band_step_down, self._band_weight_down),
            ("mid", self._band_mid_start_ratio, self._band_down_start_ratio,
             self._band_rows_mid, self._band_step_mid, self._band_weight_mid),
            ("up", self._band_up_start_ratio, self._band_mid_start_ratio,
             self._band_rows_up, self._band_step_up, self._band_weight_up),
        ]

        results = []
        last_center = hint_x
        last_width = lane_width_hint
        for name, ys, ye, rows, step, weight in band_specs:
            res = self._scan_band_midline(
                gray, bgr_small, black_th, track_is_dark,
                last_center, last_width,
                ys, ye, rows, step,
            )
            if res is None:
                continue
            res["weight"] = weight
            res["band_name"] = name
            results.append(res)
            last_center = res["center_px"]
            last_width = res["lane_width_px"]

        return results

    # ═══════════════════════════════════════════════════════════
    # 机制 8：底部对称锁定
    # ═══════════════════════════════════════════════════════════

    def _detect_bottom_center_lock(self, gray, bgr_small, black_th, track_is_dark):
        if not self._bottom_lock_enable:
            return {
                "valid": True,
                "quality": 1.0,
                "pair_ratio": 1.0,
                "center_px": float(self._img_cx_base),
                "center_err_px": 0.0,
                "symmetry_abs_px": 0.0,
            }

        img_w = self._base_w
        img_h = self._base_h
        img_cx = self._img_cx_base
        x0 = 0
        x1 = img_w - 1
        y_start = int(clamp(self._bottom_lock_start_ratio * img_h, 0, img_h - 1))
        row_step = max(1, self._bottom_lock_step)

        pair_rows = 0
        rows_done = 0
        centers = []

        y = y_start
        while y < img_h and rows_done < max(1, self._bottom_lock_rows):
            red_block, black_block = self._detect_row_blocker(
                gray, bgr_small, y, x0, x1, black_th, track_is_dark
            )
            if red_block or black_block:
                rows_done += 1
                y += row_step
                continue

            runs = self._collect_track_runs_on_row(
                gray, y, x0, x1, black_th, track_is_dark
            )
            chosen = self._choose_pair_center_from_runs(
                runs, img_cx, 0.0, x0, x1
            )
            if chosen is not None:
                pair_rows += 1
                centers.append(float(chosen["center_px"]))

            rows_done += 1
            y += row_step

        if rows_done <= 0 or not centers:
            return {
                "valid": False,
                "quality": 0.0,
                "pair_ratio": 0.0,
                "center_px": float(img_cx),
                "center_err_px": 0.0,
                "symmetry_abs_px": float(img_w),
            }

        pair_ratio = pair_rows / float(rows_done)
        center_px = float(median(centers))
        center_err_px = center_px - float(img_cx)
        symmetry_abs_px = abs(center_err_px)

        pair_q = clamp(
            (pair_ratio - self._bottom_lock_min_pair_ratio)
            / max(1.0 - self._bottom_lock_min_pair_ratio, 1e-6),
            0.0,
            1.0,
        )
        sym_q = 1.0 - clamp(
            symmetry_abs_px / max(self._bottom_lock_sym_tol_px * 2.0, 1e-6), 0.0, 1.0
        )
        quality = clamp(0.65 * pair_q + 0.35 * sym_q, 0.0, 1.0)
        valid = (
            pair_ratio >= self._bottom_lock_min_pair_ratio
            and symmetry_abs_px <= self._bottom_lock_sym_tol_px
        )

        return {
            "valid": valid,
            "quality": quality,
            "pair_ratio": pair_ratio,
            "center_px": center_px,
            "center_err_px": center_err_px,
            "symmetry_abs_px": symmetry_abs_px,
        }

    # ═══════════════════════════════════════════════════════════
    # 辅助： band 位 / 选择 / 质量权重
    # ═══════════════════════════════════════════════════════════

    @staticmethod
    def _band_bit(name):
        if name == "down":
            return 0x1
        if name == "mid":
            return 0x2
        if name == "up":
            return 0x4
        return 0

    @staticmethod
    def _single_band_mask(mask):
        return mask in (0x1, 0x2, 0x4)

    @staticmethod
    def _pick_result_by_band(results, order):
        for name in order:
            for r in results:
                if str(r.get("band_name", "")) == name:
                    return r
        return None

    def _result_quality_weight(self, r):
        pair_ratio = float(r.get("pair_ratio", 0.0))
        single_ratio = float(r.get("single_ratio", 1.0 - pair_ratio))
        q = 0.60 + 0.40 * pair_ratio
        if pair_ratio < self._min_pair_ratio:
            q *= 0.80
        q *= (1.0 - 0.12 * clamp(single_ratio, 0.0, 1.0))
        return clamp(q, 0.20, 1.00)

    # ═══════════════════════════════════════════════════════════
    # 机制 10：像素域融合辅助函数
    # ═══════════════════════════════════════════════════════════

    # ═══════════════════════════════════════════════════════════
    # 主处理入口
    # ═══════════════════════════════════════════════════════════

    def process(self, bgr):
        """处理一帧 BGR 图像。

        Returns:
            dev_px:     横向偏差 (px)，正值 = 赛道中心在图像中心右侧
            heading_deg: 朝向角 (deg)
            conf:       置信度 0–1
            vis:        BGR 可视化图像 (cam_w x cam_h)
            debug:      诊断信息 dict
        """
        state = self._state
        state["startup_frames"] += 1

        # ── 步骤 1：下采样到 160x120 基准分辨率 ──
        gray_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray_full, (self._base_w, self._base_h),
                          interpolation=cv2.INTER_LINEAR)
        bgr_small = cv2.resize(bgr, (self._base_w, self._base_h),
                               interpolation=cv2.INTER_LINEAR)

        img_w = self._base_w
        img_h = self._base_h
        img_cx = self._img_cx_base

        # ── 步骤 2：Otsu 自适应阈值 ──
        black_th = clamp(
            self._otsu_threshold(gray) + self._th_offset,
            self._th_min,
            self._th_max,
        )

        # ── 步骤 3：赛道颜色检测 ──
        track_dark_candidate = self._detect_track_is_dark(gray, black_th)
        if track_dark_candidate:
            state["track_dark_score"] = int(
                clamp(state["track_dark_score"] + 1, -6, 6)
            )
        else:
            state["track_dark_score"] = int(
                clamp(state["track_dark_score"] - 1, -6, 6)
            )
        track_is_dark = state["track_dark_score"] >= 0

        # ── 启动暂态参数 ──
        startup_active = (
            self._startup_settle_frames > 0
            and state["startup_frames"] < self._startup_settle_frames
        )
        if startup_active:
            conf_min_dyn = self._conf_min * clamp(self._startup_conf_min_scale, 0.20, 1.00)
            min_weight_dyn = self._min_weight * clamp(self._startup_min_weight_scale, 0.20, 1.00)
        else:
            conf_min_dyn = self._conf_min
            min_weight_dyn = self._min_weight

        # ── 扫描提示 ──
        if startup_active:
            scan_hint_center = float(img_cx)
            scan_hint_width = 0.0
        else:
            scan_hint_center = state["last_lane_center_x"]
            scan_hint_width = state["last_lane_width_px"]

        # ── 运行检测器 ──
        roi_results = []
        if startup_active and self._startup_force_simple_bottom:
            res = self._bottom_quarter_midline(
                gray, bgr_small, black_th, track_is_dark,
                scan_hint_center, scan_hint_width,
            )
            if res is not None and res["conf"] >= conf_min_dyn:
                roi_results.append(res)
            elif self._three_band_mode:
                roi_results = self._detect_three_band_lanes(
                    gray, bgr_small, black_th, track_is_dark,
                    scan_hint_center, scan_hint_width,
                )
                roi_results = [r for r in roi_results if r["conf"] >= conf_min_dyn]
        elif self._three_band_mode:
            roi_results = self._detect_three_band_lanes(
                gray, bgr_small, black_th, track_is_dark,
                scan_hint_center, scan_hint_width,
            )
            roi_results = [r for r in roi_results if r["conf"] >= conf_min_dyn]
        elif self._simple_bottom_mode:
            res = self._bottom_quarter_midline(
                gray, bgr_small, black_th, track_is_dark,
                scan_hint_center, scan_hint_width,
            )
            if res is not None and res["conf"] >= conf_min_dyn:
                roi_results.append(res)

        # ── 初始化输出 ──
        base_err_px = 0.0
        base_err_cm = 0.0
        angle_err = 0.0
        far_dist_cm = 0.0
        avg_conf = 0.0
        band_mask = 0
        red_block_score = 0.0
        black_block_score = 0.0
        bottom_pair_ratio = 0.0
        bottom_sym_err_px = 0.0
        center_lock_quality = 1.0
        bottom_lock_valid = True

        # ── 步骤 8：底部对称锁定 ──
        bottom_lock = self._detect_bottom_center_lock(
            gray, bgr_small, black_th, track_is_dark,
        )
        bottom_pair_ratio = float(bottom_lock.get("pair_ratio", 0.0))
        bottom_sym_err_px = float(bottom_lock.get("center_err_px", 0.0))
        center_lock_quality = float(bottom_lock.get("quality", 0.0))
        bottom_lock_valid = bool(bottom_lock.get("valid", False))

        if (
            self._lock_reacquire_reset
            and bottom_lock_valid
            and (not state["last_bottom_lock_valid"])
        ):
            state["smoothed_err"] *= 0.35
        state["last_bottom_lock_valid"] = bottom_lock_valid

        # ── 按总权重过滤 ──
        if roi_results:
            score_total = 0.0
            for r in roi_results:
                score_total += r["weight"] * r["conf"] * self._result_quality_weight(r)
            if score_total <= min_weight_dyn:
                roi_results = []

        # ── 机制 10：像素域误差融合 ──
        if roi_results:
            state["lost_frames"] = 0

            near = self._pick_result_by_band(roi_results, ("down", "mid", "up"))
            if near is None:
                near = min(roi_results, key=lambda r: r["dist_cm"])

            far = self._pick_result_by_band(roi_results, ("up", "mid", "down"))
            if far is None:
                far = max(roi_results, key=lambda r: r["dist_cm"])

            for r in roi_results:
                bn = str(r.get("band_name", ""))
                band_mask |= self._band_bit(bn)
                red_block_score = max(
                    red_block_score, float(r.get("red_block_ratio", 0.0))
                )
                black_block_score = max(
                    black_block_score, float(r.get("black_block_ratio", 0.0))
                )
            state["last_band_mask"] = band_mask

            near_err_cm = near["center_cm"]
            far_err_cm = far["center_cm"]
            near_err_px = near["center_px"] - img_cx
            far_err_px = far["center_px"] - img_cx

            # 如果近带缺失（仅 mid/up 可见），与历史混合以避免跳变
            if str(near.get("band_name", "")) != "down":
                near_err_cm = 0.68 * near_err_cm + 0.32 * state["last_base_err"]
                near_err_px = 0.68 * near_err_px + 0.32 * (
                    state["last_lane_center_x"] - img_cx
                )

            # ── 机制 11：抗抖动鲁棒层 ──
            shake_active = self._robust_enable and (state["shake_active_frames"] > 0)
            alpha_eff = self._robust_alpha_high if shake_active else self._smooth_alpha
            lock_blend_scale = (
                self._robust_bottom_lock_blend_scale if shake_active else 1.0
            )

            # 底部对称锁定融合
            if bottom_pair_ratio > 0.0:
                lock_gain = (
                    (self._bottom_lock_blend * lock_blend_scale)
                    * (0.55 + 0.45 * center_lock_quality)
                )
                lock_gain = clamp(lock_gain, 0.0, 0.95)
                near_err_px = (
                    1.0 - lock_gain
                ) * near_err_px + lock_gain * bottom_sym_err_px
                near_err_cm = near_err_px * self._row_cm_per_px[img_h - 1]

            far_dist_cm = far["dist_cm"]
            state["last_lane_center_x"] = clamp(
                float(img_cx + near_err_px), 0.0, float(img_w - 1)
            )
            state["last_lane_width_px"] = clamp(
                float(near["lane_width_px"]),
                float(self._min_track_width),
                float(self._max_track_width),
            )

            base_err_cm = near_err_cm
            base_err_px = near_err_px

            use_assist = False
            if self._simple_bottom_mode and ("assist_center_px" in near):
                assist_conf = float(near.get("assist_conf", 0.0))
                assist_delta = abs(
                    float(near["assist_center_px"]) - float(near["center_px"])
                )
                if assist_conf >= max(self._conf_min, 0.28) and assist_delta <= (
                    self._max_center_jump_px * 1.2
                ):
                    use_assist = True

            if use_assist:
                far_err_px = float(near["assist_center_px"]) - img_cx
                far_err_cm = float(near.get("assist_center_cm", near_err_cm))
                far_dist_cm = float(near.get("assist_dist_cm", far_dist_cm))
                angle_err = 0.5 * (
                    near["angle"]
                    + float(near.get("assist_angle_deg", near["angle"]))
                )
            else:
                angle_err = 0.5 * (near["angle"] + far["angle"])

            curve_px = far_err_px - near_err_px
            avg_conf = sum(
                (r["conf"] * self._result_quality_weight(r)) for r in roi_results
            ) / float(len(roi_results))
            if not bottom_lock_valid:
                avg_conf *= (
                    1.0 - self._bottom_lock_conf_penalty * (1.0 - center_lock_quality)
                )
                avg_conf = clamp(avg_conf, 0.0, 1.0)

            near_norm = near_err_px / max(0.5 * img_w, 1.0)
            far_norm = far_err_px / max(0.5 * img_w, 1.0)
            curve_norm = curve_px / max(0.5 * img_w, 1.0)

            # ── 像素域误差融合 ──
            fused_err = -near_norm
            fused_err += self._pix_lookahead_gain * (-far_norm)
            fused_err += self._pix_curve_gain * (-curve_norm)
            fused_err += self._pix_angle_gain * (-angle_err / 45.0)
            if curve_px < -self._left_curve_outward_px:
                fused_err += self._left_curve_outward_gain * curve_norm

            state["smoothed_err"] = (
                alpha_eff * state["smoothed_err"] + (1.0 - alpha_eff) * fused_err
            )

            # 抗抖动 diff RMS 追踪
            hist = state["near_err_history"]
            hist.append(float(near_err_px))
            if len(hist) > self._robust_diff_window + 1:
                del hist[0]
            if len(hist) >= 3:
                diffs = [hist[i] - hist[i - 1] for i in range(1, len(hist))]
                rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
                state["diff_rms_px"] = rms
                if rms >= self._robust_diff_rms_trigger_px:
                    state["shake_active_frames"] = self._robust_decay_frames
                elif state["shake_active_frames"] > 0:
                    state["shake_active_frames"] -= 1

            state["last_base_err"] = base_err_cm
            state["last_angle_err"] = angle_err
            state["last_far_dist"] = far_dist_cm

        else:
            # 丢失跟踪
            state["lost_frames"] += 1
            base_err_cm = state["last_base_err"]
            base_err_px = state["last_lane_center_x"] - img_cx
            angle_err = state["last_angle_err"]
            far_dist_cm = state["last_far_dist"]
            avg_conf = 0.0
            band_mask = state["last_band_mask"]

        # ── 输出：缩放回原始分辨率 ──
        dev_px = base_err_px * self._scale_x
        heading_deg = angle_err

        # 正向定义：轨道中心在图像中心右侧 → 正值
        # (original base_err_px = near["center_px"] - img_cx 已经是正值偏右)
        # 我们保持原样返回

        conf = clamp(avg_conf, 0.0, 1.0)

        # ── 构建可视化 (在原始分辨率上) ──
        vis = self._build_visualization(
            bgr, gray, bgr_small, roi_results, black_th, track_is_dark,
            dev_px, heading_deg, conf, base_err_px, band_mask,
            near_err_px if roi_results else 0.0,
        )

        # ── 调试信息 ──
        debug = {
            "black_th": black_th,
            "track_is_dark": track_is_dark,
            "base_err_px_base": base_err_px,
            "base_err_cm": base_err_cm,
            "angle_err_deg": angle_err,
            "far_dist_cm": far_dist_cm,
            "avg_conf": avg_conf,
            "band_mask": band_mask,
            "red_block_score": red_block_score,
            "black_block_score": black_block_score,
            "bottom_pair_ratio": bottom_pair_ratio,
            "bottom_sym_err_px": bottom_sym_err_px,
            "bottom_lock_valid": bottom_lock_valid,
            "center_lock_quality": center_lock_quality,
            "lost_frames": state["lost_frames"],
            "startup_frames": state["startup_frames"],
            "diff_rms_px": state["diff_rms_px"],
            "shake_active_frames": state["shake_active_frames"],
            "n_roi_results": len(roi_results),
        }

        return dev_px, heading_deg, conf, vis, debug

    # ═══════════════════════════════════════════════════════════
    # 可视化
    # ═══════════════════════════════════════════════════════════

    def _build_visualization(self, bgr_full, gray_small, bgr_small,
                             roi_results, black_th, track_is_dark,
                             dev_px, heading_deg, conf, base_err_px_base,
                             band_mask, near_err_px):
        """在原始分辨率图像上叠加检测结果。"""
        vis = bgr_full.copy()
        cam_w = self._cam_w
        cam_h = self._cam_h

        # ── 绘制三个 band 区域 ──
        band_regions = [
            (self._band_down_start_ratio, 1.0, (255, 200, 100)),
            (self._band_mid_start_ratio, self._band_down_start_ratio, (100, 200, 255)),
            (self._band_up_start_ratio, self._band_mid_start_ratio, (100, 255, 100)),
        ]
        for y0_ratio, y1_ratio, color in band_regions:
            y0 = int(clamp(y0_ratio * cam_h, 0, cam_h - 1))
            y1 = int(clamp(y1_ratio * cam_h, 0, cam_h - 1))
            overlay = vis.copy()
            cv2.rectangle(overlay, (0, y0), (cam_w - 1, y1), color, -1)
            cv2.addWeighted(overlay, 0.08, vis, 0.92, 0, vis)
            cv2.line(vis, (0, y0), (cam_w - 1, y0), color, 1)
            cv2.line(vis, (0, y1), (cam_w - 1, y1), color, 1)

        # ── 底部锁定区域 ──
        lock_y0 = int(clamp(self._bottom_lock_start_ratio * cam_h, 0, cam_h - 1))
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, lock_y0), (cam_w - 1, cam_h - 1), (0, 100, 100), -1)
        cv2.addWeighted(overlay, 0.06, vis, 0.94, 0, vis)

        # ── 绘制检测到的轨道中心点 ──
        for r in roi_results:
            if "centers_px" not in dir():
                pass
            cx = int(r.get("center_px", 0) * self._scale_x)
            cy = int(r.get("dist_cm", 0))  # 不适合作 y，改用 band 中点
            # 用 band 中间行作为近似 y
            if "band_name" in r:
                bn = r["band_name"]
                if bn == "down":
                    approx_y = int((self._band_down_start_ratio + 1.0) / 2.0 * cam_h)
                elif bn == "mid":
                    approx_y = int((self._band_mid_start_ratio + self._band_down_start_ratio) / 2.0 * cam_h)
                elif bn == "up":
                    approx_y = int((self._band_up_start_ratio + self._band_mid_start_ratio) / 2.0 * cam_h)
                else:
                    approx_y = cam_h // 2
            else:
                approx_y = cam_h // 2
            cv2.circle(vis, (cx, approx_y), 5, (0, 255, 255), -1)
            cv2.circle(vis, (cx, approx_y), 7, (0, 180, 180), 1)

        # ── 图像中心十字线 ──
        cx_full = self._img_cx_full
        cv2.line(vis, (cx_full, 0), (cx_full, cam_h - 1), (128, 128, 128), 1)
        cv2.line(vis, (0, cam_h // 2), (cam_w - 1, cam_h // 2), (128, 128, 128), 1)

        # ── 横向偏差指示 ──
        dev_x = int(cx_full + dev_px)  # 正值偏右
        cv2.line(vis, (cx_full, cam_h - 20), (cx_full, cam_h - 5), (255, 255, 255), 2)
        cv2.circle(vis, (dev_x, cam_h - 12), 5, (0, 255, 0), -1)
        cv2.line(vis, (cx_full, cam_h - 12), (dev_x, cam_h - 12), (0, 255, 0), 2)

        # ── 朝向指示 ──
        arrow_len = 35
        h_rad = math.radians(heading_deg)
        dx = int(arrow_len * math.sin(h_rad) * self._scale_x / self._scale_y)
        dy = -int(arrow_len * math.cos(h_rad))
        arrow_start = (cx_full, cam_h - 40)
        arrow_end = (cx_full + dx, cam_h - 40 + dy)
        cv2.arrowedLine(vis, arrow_start, arrow_end, (0, 255, 255), 2, tipLength=0.4)

        # ── 文字信息 ──
        font = cv2.FONT_HERSHEY_SIMPLEX
        lines = [
            f"dev={dev_px:+.1f}px  hdg={heading_deg:+.1f}deg  conf={conf:.2f}",
            f"th={black_th}  lost={self._state['lost_frames']}  bmask={band_mask}",
        ]
        for i, text in enumerate(lines):
            y_pos = 16 + i * 18
            cv2.putText(vis, text, (6, y_pos), font, 0.45, (255, 255, 0), 1)

        # ── Band 标签 ──
        label_y = cam_h - 6
        cv2.putText(vis, "down", (6, label_y), font, 0.35, (180, 180, 255), 1)
        cv2.putText(vis, "mid", (50, label_y), font, 0.35, (180, 255, 180), 1)
        cv2.putText(vis, "up", (94, label_y), font, 0.35, (180, 255, 180), 1)

        return vis


# ═══════════════════════════════════════════════════════════════════════
# 自测
# ═══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import numpy as np

    print("LineDetector V0 (OpenCV) — 自测")
    ld = LineDetector(cam_w=320, cam_h=240)
    bgr = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
    dev, hdg, conf, vis, dbg = ld.process(bgr)
    print(f"OK: dev={dev:.2f}px  hdg={hdg:.2f}deg  conf={conf:.3f}")
    print(f"  lost={dbg['lost_frames']}  n_roi={dbg['n_roi_results']}  black_th={dbg['black_th']}")
    print(f"  vis.shape={vis.shape}")
    print("测试通过！")
