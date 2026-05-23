"""巡线模块 — 几何原语拟合（两条平行线 / 两个同心半圆）。

利用赛道已知几何约束（线宽 W、圆弧半径 R）做模型驱动拟合，
比通用抛物线更稳定。直线段拟合两条平行线，圆弧段拟合同心圆。
中线直接从几何关系计算，不需逐行采样近似。
"""
import cv2
import math
import numpy as np

SHAKE_RMS_TRIGGER_PX = 6.0
SHAKE_DECAY_FRAMES = 8
SHAKE_ALPHA_HIGH = 0.88


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LineDetector:
    def __init__(
        self,
        cam_height_cm=40.0,
        cam_pitch_deg=45.0,
        cam_vfov_deg=44.0,
        cam_w=640,
        cam_h=480,
        bird_h=200,
        bird_w=160,
        lookahead_cm=(10, 80),
        track_width_cm=35.5,
        inner_radius_cm=59.75,
        outer_radius_cm=95.25,
        th_offset=-8,
        K=None,
        dist=None,
        calib_w=None,
        calib_h=None,
    ):
        self.cam_height = cam_height_cm
        self.cam_pitch = np.radians(cam_pitch_deg)
        self.cam_vfov_deg = cam_vfov_deg
        self.img_w = cam_w
        self.img_h = cam_h
        self.bird_h = bird_h
        self.bird_w = bird_w
        self.th_offset = th_offset  # Otsu 偏移（正值→更多黑）

        # 畸变校正（可选，真机用，仿真不传）
        self._K = None
        self._dist = None
        if K is not None and dist is not None:
            cal_w = calib_w or cam_w
            cal_h = calib_h or cam_h
            sx = cam_w / cal_w
            sy = cam_h / cal_h
            K_scaled = np.array(K, dtype=np.float32).reshape(3, 3).copy()
            K_scaled[0] *= sx
            K_scaled[1] *= sy
            self._K = K_scaled
            self._dist = np.array(dist, dtype=np.float32)

        self.M = self._build_birdseye_matrix(lookahead_cm)
        self._lookahead_near = lookahead_cm[0]
        self._center_x = bird_w // 2

        # 赛道几何 (cm → px)
        self.track_width_cm = track_width_cm
        self.inner_radius_cm = inner_radius_cm
        self.outer_radius_cm = outer_radius_cm
        self.cm_per_px = self._compute_cm_per_px()
        self.track_width_px = track_width_cm / self.cm_per_px
        self.inner_radius_px = (inner_radius_cm / self.cm_per_px
                                if inner_radius_cm else None)
        self.outer_radius_px = (outer_radius_cm / self.cm_per_px
                                if outer_radius_cm else None)

        # 时序状态
        self._prev_model = None       # 'straight' | 'arc'
        self._prev_dev = None
        self._prev_heading = 0.0
        self._prev_params = None
        self._jump_cnt = 0

        # 底部对称锁定
        self._bottom_lock_valid = False
        self._bottom_lock_center = bird_w // 2

        # 结构化置信度缓存
        self._edge_stats = None

        # 晃动检测（在检测器内部）
        self._near_err_history = []
        self._shake_active = 0
        self._shake_rms = 0.0

    # ── 鸟瞰变换 ──
    def _build_birdseye_matrix(self, lookahead):
        """IPM (Inverse Perspective Mapping) 标准做法：
        1. 定义地面上的矩形区域（物理坐标）
        2. 用 pinhole 模型投影到图像 → 得到梯形 src
        3. dst 是规则的矩形 → getPerspectiveTransform
        """
        near, far = lookahead
        near = max(near, 20.0)

        # ── Pinhole 相机参数 (square pixels → fx=fy) ──
        vfov_rad = np.radians(self.cam_vfov_deg)
        hfov_rad = 2.0 * np.arctan(np.tan(vfov_rad / 2.0) * self.img_w / self.img_h)
        fx = self.img_w / (2.0 * np.tan(hfov_rad / 2.0))
        fy_calc = self.img_h / (2.0 * np.tan(vfov_rad / 2.0))
        cx = self.img_w / 2.0
        cy = self.img_h / 2.0

        # ── 地面矩形四角: 在地平面上，hFOV 对应的水平宽度 = 2*z*tan(hfov/2) ──
        ground_w_far = 2.0 * far * np.tan(hfov_rad / 2.0)
        W = ground_w_far * 0.7  # 远处地面更宽，确保赛道两条线都在视野内

        world_pts = np.float32([
            [W / 2, near], [-W / 2, near],   # near right, near left
            [-W / 2, far], [W / 2, far],     # far left, far right
        ])

        # ── 投影 world→image（pinhole 模型）──
        cp, sp = np.cos(self.cam_pitch), np.sin(self.cam_pitch)
        src_pts = []
        for wx, wz in world_pts:
            # 世界 → 相机坐标 (旋转 pitch，相机 Y 朝下)
            Xc = wx
            Yc = self.cam_height * cp - wz * sp
            Zc = self.cam_height * sp + wz * cp
            if Zc < 0.01:
                Zc = 0.01
            u = fx * Xc / Zc + cx
            v = fy_calc * Yc / Zc + cy
            src_pts.append([u, v])
        src = np.float32(src_pts)

        # ── dst 矩形 (标准: 近=下, 远=上) ──
        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],  # near right, left → bottom
            [0, 0], [self.bird_w - 1, 0],                                # far left, right → top
        ])
        return cv2.getPerspectiveTransform(src, dst)

    def _compute_cm_per_px(self):
        """鸟瞰图像素对应厘米数（和 IPM warp 同公式）。"""
        hfov_rad = 2 * np.arctan(
            np.tan(np.radians(self.cam_vfov_deg / 2)) * self.img_w / self.img_h)
        ground_w_near = 2.0 * max(self._lookahead_near, 20.0) * np.tan(hfov_rad / 2.0)
        W = ground_w_near * 0.85  # 同 _build_birdseye_matrix
        return W / self.bird_w

    # ── 预处理 ──
    def _preprocess(self, bgr):
        # max(R,G,B) 与标准灰度的逐像素最大值——兼顾颜色和亮度
        gray_max = np.max(bgr, axis=2)
        gray_std = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = np.maximum(gray_max, gray_std)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        # Otsu 自适应阈值
        th_val, _ = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_val = clamp(th_val + self.th_offset, 30, 200)
        _, binary_raw = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)

        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary_raw, cv2.MORPH_CLOSE, k3, iterations=1)

        # 提取黑色边缘像素坐标 (x, y)
        ys, xs = np.where(binary == 0)
        if len(xs) < 10:
            return bird, binary_raw, binary, None  # pts = None
        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        return bird, binary_raw, binary, pts

    # ── 直线模型拟合 ──
    def _fit_straight_model(self, binary, pts):
        """Edge-pair based straight model fitting with RANSAC fallback.

        Uses _scan_edge_pairs data to find left/right track edges per row,
        fits lines to each group (PCA), and averages for the center line.
        Falls back to 2-point RANSAC if insufficient pair data (< 5 pair rows).

        返回 {'model':'straight', 'a','b','c', 'inlier_ratio', 'left_c', 'right_c'} 或 None。
        中心线: a*x + b*y + c = 0  (a^2+b^2=1)
        左/右边缘: a*x + b*y + c +/- w/2 = 0
        """
        if pts is None or len(pts) < 20:
            return None

        w = self.track_width_px
        N = len(pts)
        edge_data = self._scan_edge_pairs(binary)

        if edge_data is not None:
            pc = edge_data.get("pair_centers", [])
            if len(pc) >= 5:
                # ── 收集左右中心组 ──
                left_pts = np.array([[lx, y] for y, lx, rx in pc], dtype=np.float32)
                right_pts = np.array([[rx, y] for y, lx, rx in pc], dtype=np.float32)

                # PCA 线拟合
                def _pca_line(grp):
                    mean = grp.mean(axis=0)
                    centered = grp - mean
                    cov = np.dot(centered.T, centered)
                    eigvals, eigvecs = np.linalg.eigh(cov)
                    d = eigvecs[:, 1]  # 最大特征值方向 = 线方向
                    a0, b0 = -d[1], d[0]
                    n = np.sqrt(a0 * a0 + b0 * b0)
                    if n < 1e-10:
                        return None
                    return (a0 / n, b0 / n, -(a0 / n * mean[0] + b0 / n * mean[1]))

                ll = _pca_line(left_pts)
                rl = _pca_line(right_pts)
                if ll is not None and rl is not None:
                    a_l, b_l, c_l = ll
                    a_r, b_r, c_r = rl

                    # 对齐法向量方向
                    if a_l * a_r + b_l * b_r < 0:
                        a_r, b_r = -a_r, -b_r

                    # 中线 = 左右平均
                    a = (a_l + a_r) * 0.5
                    b = (b_l + b_r) * 0.5
                    nr = np.sqrt(a * a + b * b)
                    if nr < 1e-10:
                        a, b = a_l, b_l
                    else:
                        a /= nr
                        b /= nr
                    if a < 0:
                        a, b = -a, -b
                    c = (c_l + c_r) * 0.5

                    # 内点比例
                    d1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c + w / 2)
                    d2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c - w / 2)
                    ratio = float(np.sum((d1 < 3.5) | (d2 < 3.5))) / N
                    if ratio >= 0.05:
                        return {
                            "model": "straight",
                            "a": float(a), "b": float(b), "c": float(c),
                            "inlier_ratio": ratio,
                            "left_c": float(c + w / 2),
                            "right_c": float(c - w / 2),
                        }

        # ── RANSAC 回退 ──
        n_iter = 180
        best_score = 0
        best = None

        for _ in range(n_iter):
            idx = np.random.choice(N, 2, replace=False)
            p1, p2 = pts[idx]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if abs(dx) < 0.5 and abs(dy) < 0.5:
                continue
            # 拒绝：同一条竖线上的点 dx≤15, 跨线对 dx≈118
            if abs(dx) > 30:
                continue
            a = -dy
            b = dx
            norm_val = np.sqrt(a * a + b * b)
            a /= norm_val
            b /= norm_val
            # 确保法向量指向右方 (a > 0)，否则 heading=atan2(b,a) 给出错误符号
            if a < 0:
                a, b = -a, -b
            c = -(a * p1[0] + b * p1[1])

            best_local = 0
            best_local_c = c
            # H1: 采样线 = 中心 → 边缘在 c +/- w/2
            d1_1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c + w / 2)
            d1_2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c - w / 2)
            s1 = np.sum((d1_1 < 3.5) | (d1_2 < 3.5))
            if s1 > best_local:
                best_local = s1
                best_local_c = c
            # H2: 采样线 = 左边缘 → 中心 c_center = c - w/2
            cc2 = c - w / 2
            d2_1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + cc2 + w / 2)
            d2_2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + cc2 - w / 2)
            s2 = np.sum((d2_1 < 3.5) | (d2_2 < 3.5))
            if s2 > best_local:
                best_local = s2
                best_local_c = cc2
            # H3: 采样线 = 右边缘 → 中心 c_center = c + w/2
            cc3 = c + w / 2
            d3_1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + cc3 + w / 2)
            d3_2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + cc3 - w / 2)
            s3 = np.sum((d3_1 < 3.5) | (d3_2 < 3.5))
            if s3 > best_local:
                best_local = s3
                best_local_c = cc3

            if best_local > best_score:
                best_score = best_local
                best = (a, b, best_local_c)

        if best is None or best_score / N < 0.08:
            return None

        a, b, c = best
        inlier_ratio = best_score / N
        return {
            "model": "straight",
            "a": a, "b": b, "c": c,
            "inlier_ratio": inlier_ratio,
            "left_c": c + w / 2,
            "right_c": c - w / 2,
        }

    # ── 圆弧模型拟合（需已知半径）──
    def _fit_arc_model(self, pts, n_iter=200):
        """代数圆拟合 + 同心验证。需 R_inner/outer 已知。
        返回 {'model':'arc', 'cx','cy','r_center', 'inlier_ratio'} 或 None。
        """
        if self.inner_radius_px is None or self.outer_radius_px is None:
            return None
        if len(pts) < 15:
            return None
        N = len(pts)
        w = self.track_width_px
        r_inner = self.inner_radius_px
        r_outer = self.outer_radius_px
        r_center = (r_inner + r_outer) / 2
        best_score = 0
        best_center = None
        best_r_center = None

        for _ in range(n_iter):
            idx = np.random.choice(N, min(5, N), replace=False)
            sample = pts[idx]
            xs_s = sample[:, 0]
            ys_s = sample[:, 1]
            # 代数圆拟合: x²+y² = Ax + By + C
            A_mat = np.column_stack((xs_s, ys_s, np.ones_like(xs_s)))
            b_vec = xs_s * xs_s + ys_s * ys_s
            try:
                A_, B_, C_ = np.linalg.lstsq(A_mat, b_vec, rcond=None)[0]
            except np.linalg.LinAlgError:
                continue
            cx = A_ / 2
            cy = B_ / 2
            r_fit = np.sqrt(max(0, C_ + cx * cx + cy * cy))
            if r_fit < 5:
                continue
            # 判断拟合到的圆是内圈还是外圈
            for (r_edge, r_other) in [(r_inner, r_outer), (r_outer, r_inner)]:
                if abs(r_fit - r_edge) / r_edge > 0.35:
                    continue
                rc = r_fit + (1 if r_edge == r_inner else -1) * w / 2
                # 计算所有点到两条同心圆的距离
                dist_all = np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2)
                d1 = np.abs(dist_all - r_edge)
                d2 = np.abs(dist_all - r_other)
                inl = np.sum((d1 < 4.5) | (d2 < 4.5))
                if inl > best_score:
                    best_score = inl
                    best_center = (cx, cy)
                    best_r_center = rc

        if best_center is None or best_score / N < 0.06:
            return None
        return {
            "model": "arc",
            "cx": best_center[0],
            "cy": best_center[1],
            "r_center": best_r_center,
            "r_inner": r_inner,
            "r_outer": r_outer,
            "inlier_ratio": best_score / N,
        }

    # ── 从模型计算偏差和朝向 ──
    def _compute_dev_heading(self, model):
        """从拟合模型计算 deviation_px 和 heading_deg。
        deviation_px: 横向偏差（正值=偏右 / 负值=偏左，匹配老代码符号约定）
        heading_deg: 中线方向与 robot forward (鸟瞰图上=↑) 的夹角
        """
        robot_x = self._center_x
        robot_y = self.bird_h - 1  # 图底 = 机器人位置，前方=↑

        if model["model"] == "straight":
            a, b, c = model["a"], model["b"], model["c"]
            # 有符号距离：线在机器人右侧 → 正（匹配老代码 near_c - center_x 约定）
            deviation_px = -(a * robot_x + b * robot_y + c)
            # 中线方向 = (b, -a) 即 法向量逆时针转90°
            heading_deg = np.degrees(np.arctan2(b, a))
        else:
            cx, cy, r_center = model["cx"], model["cy"], model["r_center"]
            dist = np.sqrt((robot_x - cx) ** 2 + (robot_y - cy) ** 2)
            deviation_px = dist - r_center
            vx = robot_x - cx
            vy = robot_y - cy
            if abs(vx) + abs(vy) < 1e-6:
                heading_deg = 0.0
            else:
                tx = -vy  # CCW tangent
                ty = vx
                if ty > 0:  # 指向下方 → 翻转为向上
                    tx, ty = -tx, -ty
                heading_deg = np.degrees(np.arctan2(tx, -ty))

        return deviation_px, heading_deg

    # ── 底部对称锁定 ──
    def _bottom_lock_check(self, binary):
        """扫描鸟瞰图底部~20%行，检测黑线左右边缘对，评估对称性。

        返回 dict: valid, center_px, pair_ratio, symmetry_error
        """
        h, w = binary.shape
        y_start = int(h * 0.80)
        total_rows = h - y_start
        if total_rows <= 0:
            return {"valid": False, "center_px": float(self._center_x),
                    "pair_ratio": 0.0, "symmetry_error": 0.0}

        valid_rows = 0
        centers = []

        for y in range(y_start, h):
            row = binary[y, :].astype(np.int16)
            d = np.diff(row)
            # 白→黑：d ≈ -255（进入赛道线）
            # 黑→白：d ≈ +255（离开赛道线）
            falls = np.where(d <= -200)[0]   # 左边缘前一列
            rises = np.where(d >= 200)[0]    # 右边缘列（最后一列黑像素）

            if len(falls) == 0 or len(rises) == 0:
                continue

            # 为每一行找出最佳 fall–rise 对（中心最接近 self._center_x）
            best_center = None
            best_dist = float("inf")
            for f in falls:
                for r in rises:
                    if r < f:
                        continue
                    first_black = f + 1   # 该段第一个黑色像素
                    last_black = r        # 该段最后一个黑色像素
                    # 双边缘均可见：左边缘不在 x=0，右边缘不在 x=w-1
                    # diff 方式天然跳过触碰边界的段
                    if last_black <= first_black:
                        continue
                    c = (first_black + last_black) / 2.0
                    d = abs(c - self._center_x)
                    if d < best_dist:
                        best_dist = d
                        best_center = c

            if best_center is not None:
                valid_rows += 1
                centers.append(best_center)

        pair_ratio = valid_rows / float(total_rows) if total_rows > 0 else 0.0

        if len(centers) == 0 or pair_ratio < 0.55:
            return {"valid": False, "center_px": float(self._center_x),
                    "pair_ratio": pair_ratio, "symmetry_error": 0.0}

        center_median = float(np.median(centers))
        symmetry_error = center_median - self._center_x
        symmetry_tol = 12.0

        if abs(symmetry_error) > symmetry_tol:
            return {"valid": False, "center_px": center_median,
                    "pair_ratio": pair_ratio, "symmetry_error": symmetry_error}

        return {"valid": True, "center_px": center_median,
                "pair_ratio": pair_ratio, "symmetry_error": symmetry_error}

    # ── 结构化置信度：边沿对扫描 ──
    def _scan_edge_pairs(self, binary):
        """扫描鸟瞰图每行找出车道边沿对，用于结构化置信度。
        逐行查找白→黑→白过渡，识别连续的黑色游程，
        然后看是否能配对成 track_width_px 间距的左右边沿对。

        Returns:
            dict: hit_ratio, edge_quality, width_std, width_factor 等
            None: 数据不足（回退到单纯 inlier_ratio 置信度）
        """
        scan_step = max(1, self.bird_h // 50)   # 约扫描 50 行
        track_w = self.track_width_px
        width_tol = max(track_w * 0.30, 15.0)   # 配对容许误差
        min_run_width = 2                        # 游程最小像素宽度（过滤噪点）
        h, w = self.bird_h, self.bird_w

        total_scanned = 0
        pair_rows = 0
        single_rows = 0
        pair_widths = []
        pair_centers = []  # list of (y, left_cx, right_cx) for each row with a valid pair

        for y in range(0, h, scan_step):
            total_scanned += 1
            row = binary[y, :].astype(np.int16)

            # 黑白过渡检测
            d = np.diff(row)
            # white→black: diff < 0, 黑色起点在 i+1
            starts = np.where(d < 0)[0] + 1
            # black→white: diff > 0, 黑色终点在 i
            ends = np.where(d > 0)[0]

            # 处理图像边界
            if row[0] == 0:
                starts = np.concatenate([[0], starts])
            if row[-1] == 0:
                ends = np.concatenate([ends, [w - 1]])

            if len(starts) == 0 or len(ends) == 0:
                continue

            # 匹配起止点构建游程列表
            runs = []
            si = ei = 0
            while si < len(starts) and ei < len(ends):
                if ends[ei] >= starts[si]:
                    run_w = int(ends[ei] - starts[si] + 1)
                    if run_w >= min_run_width:
                        runs.append((int(starts[si]), int(ends[ei])))
                    si += 1
                    ei += 1
                elif ends[ei] < starts[si]:
                    ei += 1
                else:
                    si += 1

            if len(runs) == 0:
                continue

            if len(runs) >= 2:
                # 寻找中心距最接近 track_w 的游程对
                best_dist = None
                best_err = width_tol + 1.0
                best_pair_runs = None
                for i_idx in range(len(runs)):
                    c1 = (runs[i_idx][0] + runs[i_idx][1]) * 0.5
                    for j_idx in range(i_idx + 1, len(runs)):
                        c2 = (runs[j_idx][0] + runs[j_idx][1]) * 0.5
                        dd = abs(c2 - c1)
                        centers_avg = (c1 + c2) * 0.5
                        center_err = abs(centers_avg - self._center_x)
                        err = abs(dd - track_w) + 0.7 * center_err
                        if err < best_err:
                            best_err = err
                            best_dist = dd
                            best_pair_runs = (runs[i_idx], runs[j_idx])
                if best_dist is not None:
                    pair_rows += 1
                    pair_widths.append(best_dist)
                    # 记录左右中心 (left = 较小 x, right = 较大 x)
                    r1, r2 = best_pair_runs
                    cx1 = (r1[0] + r1[1]) * 0.5
                    cx2 = (r2[0] + r2[1]) * 0.5
                    lx = min(cx1, cx2)
                    rx = max(cx1, cx2)
                    pair_centers.append((y, lx, rx))
                else:
                    single_rows += 1
            else:
                single_rows += 1

        valid_rows = pair_rows + single_rows
        if valid_rows < 3 or total_scanned < 2:
            return None

        # ── 三因子置信度组件 ──
        hit_ratio = pair_rows / max(1.0, float(total_scanned))
        edge_quality = (pair_rows * 1.0 + single_rows * 0.5) / max(1.0, float(valid_rows))

        width_std = 0.0
        width_factor = 1.0
        if len(pair_widths) >= 2:
            width_std = float(np.std(pair_widths, ddof=1))
            max_std = max(track_w * 0.25, 10.0)
            width_factor = max(0.05, 1.0 - min(1.0, width_std / max(1.0, max_std)))

        return {
            "hit_ratio": max(0.01, min(1.0, hit_ratio)),
            "edge_quality": max(0.1, min(1.0, edge_quality)),
            "width_std": width_std,
            "width_factor": max(0.05, min(1.0, width_factor)),
            "pair_rows": pair_rows,
            "single_rows": single_rows,
            "valid_rows": valid_rows,
            "pair_centers": pair_centers,
        }

    # ── 主处理入口 ──
    def process(self, bgr):
        if self._K is not None:
            bgr = cv2.undistort(bgr, self._K, self._dist)
        bird, binary_raw, binary, pts = self._preprocess(bgr)

        # ── 底部对称锁定 ──
        lock = self._bottom_lock_check(binary)
        prev_bl_valid = getattr(self, '_bottom_lock_valid', False)
        self._bottom_lock_valid = lock["valid"]
        self._bottom_lock_center = lock["center_px"]
        # 重捕获重置：底部锁从不 valid 变 valid 时清平滑状态
        if self._bottom_lock_valid and not prev_bl_valid:
            self._prev_dev = None
            self._prev_heading = 0.0

        # 默认值
        dev_px = None
        heading_deg = 0.0
        conf = 0.0
        model = None
        self._edge_stats = None

        if pts is not None and len(pts) >= 10:
            # 并行拟合两个模型
            straight = self._fit_straight_model(binary, pts)
            arc = self._fit_arc_model(pts)

            # 选择更好的模型（hysteresis：当前模型有30%加成）
            candidates = []
            if straight is not None:
                s = straight["inlier_ratio"]
                if self._prev_model == "straight":
                    s *= 1.3
                candidates.append((s, straight))
            if arc is not None:
                s = arc["inlier_ratio"]
                if self._prev_model == "arc":
                    s *= 1.3
                candidates.append((s, arc))

            if candidates:
                candidates.sort(key=lambda x: x[0], reverse=True)
                model = candidates[0][1]
                dev_px, heading_deg = self._compute_dev_heading(model)
                # 结构化置信度：几何 inlier × 边沿对质量三因子
                self._edge_stats = self._scan_edge_pairs(binary)
                if self._edge_stats is not None:
                    base = model["inlier_ratio"]
                    conf = base * self._edge_stats["hit_ratio"] * \
                           self._edge_stats["edge_quality"] * self._edge_stats["width_factor"]
                    conf = clamp(conf, 0.10, 1.0)
                else:
                    conf = clamp(model["inlier_ratio"] * 1.5, 0.1, 1.0)
                    self._edge_stats = None

                # ── 底部对称锁定融合 ──
                BOTTOM_LOCK_BLEND = 0.2
                if self._bottom_lock_valid:
                    lock_dev = self._bottom_lock_center - self._center_x
                    dev_px = (1.0 - BOTTOM_LOCK_BLEND) * dev_px + BOTTOM_LOCK_BLEND * lock_dev

        # ── 晃动检测（检测器内部，跟踪 raw dev 帧间 RMS）──
        if dev_px is not None:
            hist = self._near_err_history
            hist.append(float(dev_px))
            if len(hist) > 6:
                del hist[0]
            if len(hist) >= 3:
                diffs = [hist[i] - hist[i - 1] for i in range(1, len(hist))]
                rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
                self._shake_rms = rms
                if rms >= SHAKE_RMS_TRIGGER_PX:
                    self._shake_active = SHAKE_DECAY_FRAMES
                elif self._shake_active > 0:
                    self._shake_active -= 1

        # ── 时序平滑 + 跳变拒绝 ──
        if dev_px is not None and model is not None:
            if self._prev_dev is not None:
                jump = abs(dev_px - self._prev_dev)
                jump_h = abs(heading_deg - self._prev_heading)
                if jump > 20 or jump_h > 25:
                    self._jump_cnt += 1
                    if self._jump_cnt < 15:
                        dev_px = self._prev_dev
                        heading_deg = self._prev_heading
                        conf = max(0.08, conf * 0.35)
                    else:
                        self._jump_cnt = 0
                else:
                    self._jump_cnt = 0
                    if jump > 8:
                        alpha = 0.40
                    else:
                        alpha = 0.65
                    if self._shake_active > 0:
                        alpha = max(alpha, SHAKE_ALPHA_HIGH)
                    dev_px = alpha * dev_px + (1 - alpha) * self._prev_dev
                    heading_deg = alpha * heading_deg + (1 - alpha) * self._prev_heading
            self._prev_dev = dev_px
            self._prev_heading = heading_deg
            self._prev_model = model["model"]
            self._prev_params = model
        elif self._prev_dev is not None:
            dev_px = self._prev_dev * 0.90
            heading_deg = self._prev_heading * 0.85
            conf = 0.10
            self._prev_dev = dev_px
            self._prev_heading = heading_deg

        # ── 可视化 ──
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        # 画出黑色像素点（稀疏采样显示）
        if pts is not None and len(pts) > 0:
            step = max(1, len(pts) // 600)
            for i in range(0, len(pts), step):
                x, y = int(pts[i, 0]), int(pts[i, 1])
                cv2.circle(vis, (x, y), 1, (0, 220, 220), -1)

        if model is not None:
            model_type = model["model"]
            if model_type == "straight":
                a, b, c = model["a"], model["b"], model["c"]
                w2 = self.track_width_px / 2
                self._draw_line(vis, a, b, c, (0, 220, 0), 2)          # 中线
                self._draw_line(vis, a, b, c + w2, (0, 180, 0), 1)     # 左边缘
                self._draw_line(vis, a, b, c - w2, (0, 180, 0), 1)     # 右边缘
            else:
                cx, cy = model["cx"], model["cy"]
                for (r, color, thick) in [
                    (model["r_center"], (0, 220, 0), 2),
                    (model["r_inner"], (0, 180, 0), 1),
                    (model["r_outer"], (0, 180, 0), 1),
                ]:
                    self._draw_circle(vis, cx, cy, r, color, thick)
                # 圆心
                if 0 <= int(cx) < self.bird_w and 0 <= int(cy) < self.bird_h:
                    cv2.circle(vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)

            # 偏差和朝向指示（机器人位于图底，前方=↑）
            robot_x = self._center_x
            robot_y = self.bird_h - 1
            cv2.circle(vis, (robot_x, robot_y), 5, (255, 255, 255), -1)
            if dev_px is not None:
                lbl_x = clamp(int(robot_x - dev_px * 0.8), 5, self.bird_w - 5)
                cv2.line(vis, (robot_x, robot_y), (lbl_x, robot_y),
                         (255, 0, 255), 2)
                cv2.circle(vis, (lbl_x, robot_y), 3, (255, 0, 255), -1)

            arrow_len = 22
            h_rad = np.radians(heading_deg)
            dx = int(arrow_len * np.sin(h_rad))
            dy = -int(arrow_len * np.cos(h_rad))
            cv2.arrowedLine(vis, (robot_x, robot_y),
                            (robot_x + dx, robot_y + dy),
                            (255, 255, 0), 2, tipLength=0.5)

            # 文字信息
            cv2.putText(vis,
                        f"{model_type} dev={dev_px:+.1f} h={heading_deg:+.0f}deg c={conf:.2f}",
                        (4, self.bird_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
            cv2.putText(vis,
                        f"inlier={model['inlier_ratio']:.2f} W={self.track_width_px:.0f}px",
                        (4, 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
        else:
            cv2.putText(vis, "NO MODEL", (4, self.bird_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        debug = {
            "bird": bird,
            "binary_raw": binary_raw,
            "binary": binary,
            "model_type": model["model"] if model else None,
            "inlier_ratio": model["inlier_ratio"] if model else 0.0,
            "heading_deg": heading_deg,
            "shake_active": self._shake_active,
            "shake_rms": self._shake_rms,
            "_bottom_lock_valid": self._bottom_lock_valid,
            "edge_hit_ratio": self._edge_stats["hit_ratio"] if self._edge_stats else 0.0,
            "edge_quality": self._edge_stats["edge_quality"] if self._edge_stats else 0.0,
            "edge_width_std": self._edge_stats["width_std"] if self._edge_stats else 0.0,
            "edge_width_factor": self._edge_stats["width_factor"] if self._edge_stats else 0.0,
            "edge_pair_rows": self._edge_stats["pair_rows"] if self._edge_stats else 0,
            "edge_single_rows": self._edge_stats["single_rows"] if self._edge_stats else 0,
        }
        return dev_px, heading_deg, conf, vis, debug

    # ── 绘制辅助 ──
    def _draw_line(self, img, a, b, c, color, thickness):
        """在鸟瞰图上画线 a*x + b*y + c = 0。"""
        h, w = img.shape[:2]
        pts_line = []
        # 与左右边 x=0, x=w-1 的交点
        for x in [0, w - 1]:
            if abs(b) > 1e-6:
                y = -(a * x + c) / b
                if 0 <= y < h:
                    pts_line.append((int(x), int(y)))
        # 与上下边 y=0, y=h-1 的交点
        for y in [0, h - 1]:
            if abs(a) > 1e-6:
                x = -(b * y + c) / a
                if 0 <= x < w:
                    pts_line.append((int(x), int(y)))
        if len(pts_line) >= 2:
            # 取最远的两个点
            pts_line.sort()
            cv2.line(img, pts_line[0], pts_line[-1], color, thickness)

    def _draw_circle(self, img, cx, cy, r, color, thickness):
        """画圆（仅画可视部分用于说明；半径可能很大）"""
        icx, icy = int(round(cx)), int(round(cy))
        ir = int(round(r))
        if ir <= 0:
            return
        # 限制半径避免画太慢
        if ir < max(img.shape) * 3:
            cv2.circle(img, (icx, icy), ir, color, thickness)
