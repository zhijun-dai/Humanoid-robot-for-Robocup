"""巡线模块 — 几何原语拟合（两条平行线 / 两个同心半圆）。

利用赛道已知几何约束（线宽 W、圆弧半径 R）做模型驱动拟合，
比通用抛物线更稳定。直线段拟合两条平行线，圆弧段拟合同心圆。
中线直接从几何关系计算，不需逐行采样近似。
"""
import cv2
import numpy as np


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
        th_offset=12,
        th_min=30,
        th_max=160,
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
        self.th_offset = th_offset
        self.th_min = th_min
        self.th_max = th_max

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

    # ── 鸟瞰变换 ──
    def _build_birdseye_matrix(self, lookahead):
        near, far = lookahead
        near = max(near, 20.0)
        vfov_rad = np.radians(self.cam_vfov_deg)

        def ground_y(z_cm):
            ray = np.arctan2(self.cam_height, z_cm)
            v = ray - self.cam_pitch
            return (0.5 - v / vfov_rad) * self.img_h

        y_far = ground_y(far)
        y_near = ground_y(near)
        src = np.float32([
            [self.img_w - 1, y_near], [0, y_near],
            [self.img_w - 1, y_far],  [0, y_far],
        ])
        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],
            [self.bird_w - 1, 0],               [0, 0],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    def _compute_cm_per_px(self):
        """鸟瞰图水平方向每像素对应厘米数（在 lookahead 中点估算）。"""
        hfov = 2 * np.arctan(
            np.tan(np.radians(self.cam_vfov_deg / 2)) * self.img_w / self.img_h)
        lookahead_mid = 45.0  # ~ (10+80)/2
        ground_width = 2 * lookahead_mid * np.tan(hfov / 2)
        return ground_width / self.bird_w

    # ── 预处理 ──
    def _preprocess(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        th_val, _ = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_val = clamp(th_val + self.th_offset, self.th_min, self.th_max)
        _, binary = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)

        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k3)

        # 提取黑色边缘像素坐标 (x, y)
        ys, xs = np.where(binary == 0)
        if len(xs) < 10:
            return bird, binary, None
        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        return bird, binary, pts

    # ── 直线模型拟合 ──
    def _fit_straight_model(self, pts, n_iter=180):
        """RANSAC 拟合两条平行线（间距 = track_width_px）。
        返回 {'model':'straight', 'a','b','c', 'inlier_ratio', 'left_c', 'right_c'} 或 None。
        中心线: a*x + b*y + c = 0  (a²+b²=1)
        左/右边缘: a*x + b*y + c ± w/2 = 0
        """
        if len(pts) < 20:
            return None
        w = self.track_width_px
        best_score = 0
        best = None
        N = len(pts)

        for _ in range(n_iter):
            idx = np.random.choice(N, 2, replace=False)
            p1, p2 = pts[idx]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if abs(dx) < 0.5 and abs(dy) < 0.5:
                continue
            # 法向量
            a = -dy
            b = dx
            norm = np.sqrt(a * a + b * b)
            a /= norm
            b /= norm
            c = -(a * p1[0] + b * p1[1])

            # 三条假设：采样线是中心 / 左边缘 / 右边缘
            # 边缘线在 c ± w/2，另一条在 c ∓ w/2
            best_local = 0
            best_local_c = c
            # H1: 采样线 = 中心 → 边缘在 c ± w/2
            h1_d1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c + w / 2)
            h1_d2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c - w / 2)
            s1 = np.sum((h1_d1 < 3.5) | (h1_d2 < 3.5))
            if s1 > best_local:
                best_local = s1
                best_local_c = c
            # H2: 采样线 = 左边缘 → 中心 c = c_line + w/2, 右边缘 c + w
            c_center2 = c - w / 2  # 如果采样线是左边缘，中心在右
            h2_d1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c_center2 + w / 2)
            h2_d2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c_center2 - w / 2)
            s2 = np.sum((h2_d1 < 3.5) | (h2_d2 < 3.5))
            if s2 > best_local:
                best_local = s2
                best_local_c = c_center2
            # H3: 采样线 = 右边缘 → 中心 c = c_line - w/2
            c_center3 = c + w / 2
            h3_d1 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c_center3 + w / 2)
            h3_d2 = np.abs(a * pts[:, 0] + b * pts[:, 1] + c_center3 - w / 2)
            s3 = np.sum((h3_d1 < 3.5) | (h3_d2 < 3.5))
            if s3 > best_local:
                best_local = s3
                best_local_c = c_center3

            if best_local > best_score:
                best_score = best_local
                # 确保 a² + b² = 1
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
        robot_y = self.bird_h - 1  # 图底 = 机器人近处

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
            # 切线方向：圆心→机器人 逆时针转90°（半圆向左转）
            vx = robot_x - cx
            vy = robot_y - cy
            if abs(vx) + abs(vy) < 1e-6:
                heading_deg = 0.0
            else:
                tx = -vy
                ty = vx
                # 确保大致指向上方（-y 方向）
                if ty > 0:
                    tx, ty = -tx, -ty
                heading_deg = np.degrees(np.arctan2(tx, -ty))

        return deviation_px, heading_deg

    # ── 主处理入口 ──
    def process(self, bgr):
        if self._K is not None:
            bgr = cv2.undistort(bgr, self._K, self._dist)
        bird, binary, pts = self._preprocess(bgr)

        # 默认值
        dev_px = None
        heading_deg = 0.0
        conf = 0.0
        model = None

        if pts is not None and len(pts) >= 10:
            # 并行拟合两个模型
            straight = self._fit_straight_model(pts)
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
                conf = clamp(model["inlier_ratio"] * 1.5, 0.1, 1.0)

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

            # 偏差和朝向指示
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
            "binary": binary,
            "model_type": model["model"] if model else None,
            "inlier_ratio": model["inlier_ratio"] if model else 0.0,
            "heading_deg": heading_deg,
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
