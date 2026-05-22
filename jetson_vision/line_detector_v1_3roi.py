"""巡线模块 V1 — warp 鸟瞰 + 3 层 ROI 扫描

V0: 无 warp，直接在摄像头画面上扫
V1: 先 IPM warp → 鸟瞰图，再 3 层 ROI 扫边缘 → 直线拟合
V2: warp + 抛物线拟合
V3: warp + 几何原语（平行线/同心圆）
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
        scan_lines=5,
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
        self.scan_lines = scan_lines

        self._K = None
        self._dist = None

        self.M = self._build_birdseye_matrix(lookahead_cm)

        self._prev_dev = 0.0
        self._prev_heading = 0.0

    # ── IPM 鸟瞰变换（同 V2/V3）──
    def _build_birdseye_matrix(self, lookahead):
        near, far = lookahead
        near = max(near, 20.0)

        vfov_rad = np.radians(self.cam_vfov_deg)
        hfov_rad = 2.0 * np.arctan(np.tan(vfov_rad / 2.0) * self.img_w / self.img_h)
        fx = self.img_w / (2.0 * np.tan(hfov_rad / 2.0))
        fy = self.img_h / (2.0 * np.tan(vfov_rad / 2.0))
        cx, cy = self.img_w / 2.0, self.img_h / 2.0

        ground_w_near = 2.0 * near * np.tan(hfov_rad / 2.0)
        W = ground_w_near * 0.85

        world_pts = np.float32([
            [W / 2, near], [-W / 2, near],
            [-W / 2, far], [W / 2, far],
        ])
        cp, sp = np.cos(self.cam_pitch), np.sin(self.cam_pitch)
        src_pts = []
        for wx, wz in world_pts:
            Xc = wx
            Yc = self.cam_height * cp - wz * sp
            Zc = self.cam_height * sp + wz * cp
            if Zc < 0.01: Zc = 0.01
            src_pts.append([fx * Xc / Zc + cx, fy * Yc / Zc + cy])
        src = np.float32(src_pts)

        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],
            [0, 0], [self.bird_w - 1, 0],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    # ── 单个 ROI 扫描 ──
    def _scan_roi(self, gray, y0, y1):
        """扫描灰度图的一段 ROI 行，返回轨道中心 x 中位数或 None。"""
        centers = []
        row_step = max(1, (y1 - y0) // self.scan_lines)
        for y in range(y0, y1, row_step):
            line = gray[y, :]
            th = cv2.threshold(line, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
            th = clamp(int(th) + self.th_offset, 30, 200)
            binary = cv2.threshold(line, th, 255, cv2.THRESH_BINARY)[1]

            diff = np.diff(binary.astype(np.int32))
            rising = np.where(diff > 0)[0]    # 黑→白
            falling = np.where(diff < 0)[0]   # 白→黑

            if len(rising) >= 2 and len(falling) >= 2:
                left = falling[0]
                right = rising[-1]
                if right - left > 10:
                    centers.append((left + right) / 2)
        if not centers:
            return None
        return float(np.median(centers))

    # ── 主入口 ──
    def process(self, bgr):
        if self._K is not None:
            bgr = cv2.undistort(bgr, self._K, self._dist)

        gray = np.max(bgr, axis=2)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        # 3 层 ROI（近/中/远, 在鸟瞰图上从上到下）
        near_y0 = self.bird_h - 30
        near_y1 = self.bird_h
        mid_y0 = self.bird_h // 2 - 15
        mid_y1 = self.bird_h // 2 + 15
        far_y0 = 0
        far_y1 = 30

        centers = []
        for name, y0, y1 in [("near", near_y0, near_y1),
                               ("mid", mid_y0, mid_y1),
                               ("far", far_y0, far_y1)]:
            cx = self._scan_roi(bird, y0, y1)
            if cx is not None:
                centers.append((cx, (y0 + y1) / 2))

        deviation_px = None
        heading_deg = 0.0
        conf = 0.0

        if len(centers) >= 2:
            pts = np.array(centers, dtype=np.float32)
            xs, ys = pts[:, 0], pts[:, 1]
            coeffs = np.polyfit(ys, xs, 1)
            m, c = coeffs

            # 近处偏差（y = bird_h, 图底）
            center_near = m * self.bird_h + c
            deviation_px = center_near - self._center_x

            # 朝向：dx/dy in birdseye pixels
            heading_deg = np.degrees(np.arctan(m))

            conf = clamp(len(centers) / 3.0, 0.2, 1.0)

        # ── 时序平滑 ──
        if deviation_px is not None:
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
        vis = cv2.cvtColor(bird, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (0, far_y0), (self.bird_w, far_y1), (100, 100, 200), 1)
        cv2.rectangle(vis, (0, mid_y0), (self.bird_w, mid_y1), (100, 100, 200), 1)
        cv2.rectangle(vis, (0, near_y0), (self.bird_w, near_y1), (100, 100, 200), 1)
        for cx, cy in centers:
            cv2.circle(vis, (int(cx), int(cy)), 4, (0, 255, 255), -1)
        if deviation_px is not None:
            cv2.circle(vis, (self._center_x, self.bird_h - 1), 5, (255, 255, 255), -1)
            cv2.circle(vis, (int(self._center_x + deviation_px), self.bird_h - 1), 5, (255, 0, 255), -1)
            cv2.putText(vis, f"V1 dev={deviation_px:+.1f} h={heading_deg:+.0f}",
                        (4, self.bird_h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)

        debug = {"bird": bird}
        return deviation_px, heading_deg, conf, vis, debug

    def set_calibration(self, K, dist, calib_w=None, calib_h=None):
        pass
