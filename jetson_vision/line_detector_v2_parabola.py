"""巡线模块 V2 — 鸟瞰 + 二次抛物线拟合（方法二）

流程：
  BGR → max(R,G,B) → warp → Otsu → 形态学闭运算
  → 逐行采样黑像素中心 → 加权抛物线拟合 x = a·y² + b·y + c
  → 计算偏差（切线近端截距）和朝向（切线角）

与 V3 (几何原语) 的区别：抛物线无先验约束，弯道直道通用，
但拟合自由度更高（3参数 vs 2参数+间距约束）。
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
        th_offset=6,
    ):
        self.cam_height = cam_height_cm
        self.cam_pitch = np.radians(cam_pitch_deg)
        self.cam_vfov_deg = cam_vfov_deg
        self.img_w = cam_w
        self.img_h = cam_h
        self.bird_h = bird_h
        self.bird_w = bird_w
        self._center_x = bird_w // 2
        self.th_offset = th_offset

        # 畸变校正（可选）
        self._K = None
        self._dist = None

        self.M = self._build_birdseye_matrix(lookahead_cm)

        # 时序状态
        self._last_dev = None
        self._last_heading = 0.0
        self._jump_cnt = 0

    # ── 鸟瞰变换 ──
    def _build_birdseye_matrix(self, lookahead):
        near, far = lookahead
        near = max(near, 20.0)
        vfov_rad = np.radians(self.cam_vfov_deg)

        def ground_y(z_cm):
            ray = np.arctan2(self.cam_height, z_cm)
            v = ray - self.cam_pitch
            return (0.5 + v / vfov_rad) * self.img_h

        y_near = ground_y(near)
        y_far = ground_y(far)
        src = np.float32([
            [self.img_w - 1, y_near], [0, y_near],
            [self.img_w - 1, y_far],  [0, y_far],
        ])
        dst = np.float32([
            [self.bird_w - 1, 0],               [0, 0],
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    # ── 主入口 ──
    def process(self, bgr):
        if self._K is not None:
            bgr = cv2.undistort(bgr, self._K, self._dist)

        # max(R,G,B) → warp → 二值化
        gray = np.max(bgr, axis=2)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))
        th_val, _ = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_val = clamp(th_val + self.th_offset, 30, 200)
        binary = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)[1]

        # ── 逐行采样 ──
        points = []
        step = 4
        for row in range(0, self.bird_h, step):
            black = np.where(binary[row, :] == 0)[0]
            if len(black) >= 3:
                points.append((float(np.mean(black)), float(row)))

        deviation_px = None
        heading_deg = 0.0
        conf = 0.0
        inlier_count = 0
        a = b = c = 0.0
        y_mean = 0.5

        if len(points) >= 8:
            pts = np.array(points, dtype=np.float32)
            xs, ys = pts[:, 0], pts[:, 1]
            w = 0.4 + 0.6 * (ys / self.bird_h)

            y_norm = ys / self.bird_h
            y_mean = y_norm.mean()
            y_centered = y_norm - y_mean

            coeffs = np.polyfit(y_centered, xs, 2, w=w)
            a, b, c = coeffs

            pred = a * y_centered**2 + b * y_centered + c
            residuals = np.abs(xs - pred)
            inlier_mask = residuals < 4.0
            inlier_count = int(inlier_mask.sum())
            conf = clamp(inlier_count / 30.0, 0.15, 1.0)

            if inlier_count >= 6:
                rmse = np.sqrt(np.mean(residuals[inlier_mask]**2))
                if rmse > 3.0 or abs(a) > 0.5 * self.bird_w:
                    # 降级到直线
                    a, b, c = 0.0, b, c

                y_bottom = 1.0 - y_mean
                near_x = a * y_bottom**2 + b * y_bottom + c
                deviation_px = near_x - self._center_x

                slope = 2 * a * y_bottom + b
                heading_deg = np.degrees(np.arctan(slope / self.bird_h))

        # ── 时序平滑 ──
        if deviation_px is not None:
            if self._last_dev is not None:
                jump = abs(deviation_px - self._last_dev)
                if jump > 20:
                    if self._jump_cnt < 15:
                        self._jump_cnt += 1
                        deviation_px = self._last_dev
                        heading_deg = self._last_heading
                        conf = max(0.08, conf * 0.5)
                    else:
                        self._jump_cnt = 0
                else:
                    self._jump_cnt = 0
                    alpha = 0.35 if jump > 8 else 0.7
                    deviation_px = alpha * deviation_px + (1 - alpha) * self._last_dev
            self._last_dev = deviation_px
            self._last_heading = heading_deg
        elif self._last_dev is not None:
            deviation_px = self._last_dev * 0.92
            heading_deg = self._last_heading * 0.88
            conf = 0.12

        # ── 可视化 ──
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (self._center_x, 0), (self._center_x, self.bird_h), (0, 0, 255), 1)
        for px, py in points:
            cv2.circle(vis, (int(px), int(py)), 1, (0, 255, 255), -1)
        if deviation_px is not None:
            for y_pt in range(0, self.bird_h, 5):
                yn = y_pt / self.bird_h - y_mean
                x_pt = int(a * yn**2 + b * yn + c)
                if 0 <= x_pt < self.bird_w:
                    cv2.circle(vis, (x_pt, y_pt), 2, (0, 255, 0), -1)
            cv2.putText(vis,
                f"d={deviation_px:+.1f} h={heading_deg:+.0f}deg a={a:+.3f} in={inlier_count}/{len(points)}",
                (4, self.bird_h - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)

        debug = {"bird": bird, "binary": binary, "heading_deg": heading_deg}
        return deviation_px, heading_deg, conf, vis, debug
