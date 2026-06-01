"""巡线模块 V0 — 无 warp，3 层 ROI 扫描线

直接在摄像头画面（无透视变换）上定义 3 层水平 ROI。
每层扫描多行灰度找黑白边缘 → 算居中点 → 3 点拟合直线。
偏差 = 最近 ROI 处中心偏移 (px)，朝向 = 直线倾角 (deg)。

比 warp 方案简单，但对弯道预判有限（远处像素不够密）。
"""
import cv2
import numpy as np


class LineDetector:
    def __init__(
        self,
        cam_w=640,
        cam_h=480,
        rois=None,
        scan_lines_per_roi=5,
        th_offset=6,
    ):
        self.img_w = cam_w
        self.img_h = cam_h
        self._center_x = cam_w // 2
        self.th_offset = th_offset

        # 默认 ROI: [x, y, w, h] (同 main1.py 按比例缩放)
        if rois is None:
            h = cam_h
            self.rois = [
                [0, int(h * 0.70), cam_w, int(h * 0.25), 0.20],   # 近
                [0, int(h * 0.45), cam_w, int(h * 0.23), 0.55],   # 中
                [0, int(h * 0.20), cam_w, int(h * 0.20), 0.25],   # 远
            ]
        else:
            self.rois = rois
        self.scan_lines = scan_lines_per_roi
        self._prev_centers = None
        self._prev_dev = 0.0
        self._prev_heading = 0.0

    def _scan_roi(self, gray, x, y, w, h):
        """扫描一个 ROI，返回该层轨道中心 x 或 None。"""
        centers = []
        row_step = max(1, h // self.scan_lines)
        for row in range(y, y + h, row_step):
            line = gray[row, x:x + w]
            th = cv2.threshold(line, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
            th = max(30, min(200, int(th) + self.th_offset))
            _, binary = cv2.threshold(line, th, 255, cv2.THRESH_BINARY)

            # 找黑白转换（左黑→白→右黑）
            diff = np.diff(binary.astype(np.int32) // 255)
            rising = np.where(diff > 0)[0]   # 黑→白
            falling = np.where(diff < 0)[0]   # 白→黑
            if len(rising) >= 2 and len(falling) >= 2:
                left = falling[0] + x
                right = rising[-1] + x
                if right - left > 20:
                    centers.append((left + right) / 2)
        if not centers:
            return None
        return float(np.median(centers))

    def process(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # 每层 ROI 扫描出一个中心点
        centers_yx = []
        for x, y, w, h, _ in self.rois:
            cx = self._scan_roi(gray, x, y, w, h)
            if cx is not None:
                centers_yx.append((cx, y + h // 2))

        deviation_px = None
        heading_deg = 0.0
        conf = 0.0

        if len(centers_yx) >= 2:
            pts = np.array(centers_yx, dtype=np.float32)
            xs, ys = pts[:, 0], pts[:, 1]
            # 直线拟合: y = mx + b → x = (y - b)/m
            coeffs = np.polyfit(ys, xs, 1)
            m, c = coeffs  # x = m * y + c

            # 在最底层（nearest ROI）的偏差
            nearest_y = self.rois[0][1] + self.rois[0][3] // 2
            center_x = m * nearest_y + c
            deviation_px = center_x - self._center_x

            # 倾角：dx/dy = m, heading = atan(m) 近似横向偏差率
            heading_deg = np.degrees(np.arctan(m))

            conf = clamp(len(centers_yx) / 3.0, 0.2, 1.0)

        # ── 时序平滑 ──
        if deviation_px is not None:
            if self._prev_centers is not None:
                alpha = 0.6
                deviation_px = alpha * deviation_px + (1 - alpha) * self._prev_dev
                heading_deg = alpha * heading_deg + (1 - alpha) * self._prev_heading
            self._prev_dev = deviation_px
            self._prev_heading = heading_deg
        else:
            deviation_px = self._prev_dev * 0.92
            heading_deg = self._prev_heading * 0.88
            conf = 0.1

        # ── 可视化 ──
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for x, y, w, h, _ in self.rois:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (100, 100, 200), 1)
        for cx, cy in centers_yx:
            cv2.circle(vis, (int(cx), int(cy)), 4, (0, 255, 255), -1)
        if deviation_px is not None:
            y0 = self.rois[0][1] + self.rois[0][3] // 2
            cv2.circle(vis, (int(self._center_x + deviation_px), y0), 5, (255, 0, 255), -1)
            cv2.putText(vis, f"V1 dev={deviation_px:+.1f} h={heading_deg:+.0f}",
                        (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        debug = {}
        return deviation_px, heading_deg, conf, vis, debug

    def set_calibration(self, K, dist, calib_w=None, calib_h=None):
        """V1 不需要畸变校正（可以直接忽略），留空。
        如需可在此处存入，在 process 开头 undistort。
        """
        pass


def clamp(v, lo, hi):
    return max(lo, min(hi, v))
