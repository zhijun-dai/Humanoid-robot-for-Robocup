"""巡线模块 — 鸟瞰变换 + 二次曲线拟合。
逐行采样 → 加权 RANSAC 抛物线拟合 x = a*y² + b*y + c。
在鸟瞰图中，圆弧轨迹 = 抛物线。直道时 a≈0，弯道时 a 捕捉曲率。
比直线拟合更能正确追踪急弯——不用舍弃画面上方。
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
        th_offset=4,
        th_min=30,
        th_max=160,
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

        self.M = self._build_birdseye_matrix(lookahead_cm)
        self._center = bird_w // 2
        self._last_dev = None
        self._last_heading = 0.0

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
            [self.img_w - 1, y_far], [0, y_far],
        ])
        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],
            [self.bird_w - 1, 0], [0, 0],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    def process(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        th_val, _ = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_val = clamp(th_val + self.th_offset, self.th_min, self.th_max)
        _, binary = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)

        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k3)

        # ── 逐行采样 ──
        points = []
        step = 4
        for row in range(0, self.bird_h, step):
            black_cols = np.where(binary[row, :] == 0)[0]
            if len(black_cols) >= 3:
                cx = float(np.mean(black_cols))
                points.append((cx, row))

        deviation_px = None
        heading_deg = 0.0
        conf = 0.0
        curve = 0.0
        near_c = far_c = mid_c = None
        curv = 0.0  # 二次曲率系数 a

        if len(points) >= 8:
            pts = np.array(points, dtype=np.float32)
            xs = pts[:, 0]
            ys = pts[:, 1]
            w = 0.4 + 0.6 * (ys / self.bird_h)  # 近处权重大

            # ── 归一化 y 防数值溢出（y² 太大）──
            y_norm = ys / self.bird_h       # [0, 1]
            y_mean = y_norm.mean()
            y_centered = y_norm - y_mean    # 中心化

            # 抛物线: x = a*y_centered² + b*y_centered + c
            # polyfit degree=2
            coeffs = np.polyfit(y_centered, xs, 2, w=w)
            a, b, c = coeffs  # a=曲率, b=朝向, c=中心偏移

            # 评估拟合质量（RMSE）
            pred = a * y_centered**2 + b * y_centered + c
            residuals = np.abs(xs - pred)
            inliers = np.sum(residuals < 4.0)

            if inliers >= 6:
                # 若 RMSE 太大，降级到直线拟合
                rmse = np.sqrt(np.mean(residuals[residuals < 4.0]**2))
                if rmse > 3.0 or abs(a) > 0.5 * self.bird_w:
                    # 降级：直线 x = b*y_centered + c
                    a, b, c = 0.0, b, c

                # 在 y = 1（图底 / 近处）处求值
                y_bottom = 1.0 - y_mean
                near_c = a * y_bottom**2 + b * y_bottom + c
                # 在 y = 0（图顶 / 远处）
                y_top = 0.0 - y_mean
                far_c = a * y_top**2 + b * y_top + c
                y_mid = 0.5 - y_mean
                mid_c = a * y_mid**2 + b * y_mid + c

                deviation_px = near_c - self._center

                # 局部切线方向（在 y_bottom 处的导数）
                # slope = dx/d(y_norm),  实际 dx/dy = slope / bird_h
                slope = 2 * a * y_bottom + b
                heading_deg = np.degrees(np.arctan(slope / self.bird_h))

                curve = near_c - far_c
                curv = a
                conf = clamp(inliers / 30.0, 0.15, 1.0)

        # ── 跨帧平滑 + 跳变拒绝 ──
        if deviation_px is not None:
            if self._last_dev is not None:
                jump = abs(deviation_px - self._last_dev)
                if jump > 20:
                    if getattr(self, "_jump_cnt", 0) < 15:
                        # 短期跳变 → 拒绝，用历史
                        self._jump_cnt = getattr(self, "_jump_cnt", 0) + 1
                        deviation_px = self._last_dev
                        heading_deg = self._last_heading
                        conf = max(0.08, conf * 0.5)
                    else:
                        # 持续 15+ 帧 → 环境真变了，接受
                        self._jump_cnt = 0
                else:
                    self._jump_cnt = 0
                    if jump > 8:
                        alpha = 0.35
                    else:
                        alpha = 0.7
                    deviation_px = alpha * deviation_px + (1 - alpha) * self._last_dev
            self._last_dev = deviation_px
            self._last_heading = heading_deg
        elif self._last_dev is not None:
            deviation_px = self._last_dev * 0.92
            heading_deg = self._last_heading * 0.88
            conf = 0.12
            self._last_dev = deviation_px
            self._last_heading = heading_deg

        # ── 可视化 ──
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (self._center, 0), (self._center, self.bird_h), (0, 0, 255), 1)
        for cx, row in points:
            cv2.circle(vis, (int(cx), row), 1, (0, 255, 255), -1)
        if deviation_px is not None and near_c is not None:
            # 画出拟合曲线
            for y_pt in range(0, self.bird_h, 5):
                yn = y_pt / self.bird_h - y_mean
                x_pt = int(a * yn**2 + b * yn + c)
                if 0 <= x_pt < self.bird_w:
                    cv2.circle(vis, (x_pt, y_pt), 2, (0, 255, 0), -1)
            cv2.putText(vis, f"d={deviation_px:+.1f} h={heading_deg:+.0f}deg a={curv:+.3f} in={inliers}/{len(points)}",
                        (4, self.bird_h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)

        debug = {
            "binary": binary, "bird": bird,
            "centers": [near_c, mid_c, far_c],
            "curve": curve,
            "heading_deg": heading_deg,
        }
        return deviation_px, heading_deg, conf, vis, debug
