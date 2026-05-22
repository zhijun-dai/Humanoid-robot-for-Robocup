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
        track_width_cm=35.5,
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

        # —— 从鸟瞰几何推导赛道两条黑线中心间距 (px) ——
        mid_cm = (lookahead_cm[0] + lookahead_cm[1]) / 2.0
        vfov_rad = np.radians(cam_vfov_deg)
        hfov_rad = 2.0 * np.arctan(np.tan(vfov_rad / 2.0) * cam_w / cam_h)
        ground_w_mid = 2.0 * mid_cm * np.tan(hfov_rad / 2.0)
        self._track_width_px = track_width_cm * (bird_w / ground_w_mid)

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

    # ── 预处理（整张鸟瞰图一次 Otsu，同 V2）──
    def _preprocess(self, bgr):
        """灰度转换 + IPM 鸟瞰 + 整图 Otsu 二值化 + 形态学闭运算。"""
        if self._K is not None:
            bgr = cv2.undistort(bgr, self._K, self._dist)
        gray = np.max(bgr, axis=2)
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        th_val = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
        th_val = clamp(int(th_val) + self.th_offset, 30, 200)
        _, binary = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)

        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k3, iterations=1)
        return bird, binary

    # ── 单个 ROI 扫描 ──
    def _scan_roi(self, binary, y0, y1):
        """扫描二值图的一段 ROI 行，识别两条黑线后推算赛道中心 x 中位数。

        每条黑线(白→黑→白)构成一个段。段间距约等于 track_width_px 的两个段
        配对即为左右赛道边线；赛道中心 = 两段中心的均值。
        若仅可见一条边线，则从该段中心外推 track_width_px/2 得到赛道中心。
        返回 None 表示该 ROI 内未找到任何黑段。
        """
        track_centers = []
        half_track = self._track_width_px / 2.0
        min_seg_w = 4                              # 单条黑线最小宽度
        max_seg_w = 40                             # 单条黑线最大宽度
        track_tol = max(20.0, self._track_width_px * 0.4)  # 双段配对容差
        row_step = max(1, (y1 - y0) // self.scan_lines)

        for y in range(y0, y1, row_step):
            line = binary[y, :].astype(np.int32)
            diff = np.diff(line)
            rising = np.where(diff > 0)[0]   # 黑→白 (right edge)
            falling = np.where(diff < 0)[0]  # 白→黑 (left edge)

            # ── 构建当前行所有黑段（左沿 + 紧邻右沿 → 段）──
            segments = []
            for fl in falling:
                candidates = rising[rising > fl]
                if len(candidates) == 0:
                    continue
                ri = candidates[0]                # 紧邻右沿
                width = ri - fl
                if min_seg_w <= width <= max_seg_w:
                    segments.append({'cx': float((fl + ri) * 0.5)})

            # ── 双段：找间距最接近赛道中心间距的段对 ──
            if len(segments) >= 2:
                best_pair = None
                best_err = float('inf')
                for i in range(len(segments)):
                    for j in range(i + 1, len(segments)):
                        gap = abs(segments[j]['cx'] - segments[i]['cx'])
                        err = abs(gap - self._track_width_px)
                        if err < best_err:
                            best_err = err
                            best_pair = (segments[i], segments[j])
                if best_pair is not None:
                    gap = abs(best_pair[0]['cx'] - best_pair[1]['cx'])
                    if abs(gap - self._track_width_px) <= track_tol:
                        # 赛道中心 = 左右黑线中心的均值
                        track_cx = (best_pair[0]['cx'] + best_pair[1]['cx']) * 0.5
                        track_centers.append(track_cx)
                        continue
                # 未配成对 → 继续执行单段逻辑

            # ── 单段：用距图像中心最近的段外推赛道中心 ──
            if len(segments) >= 1:
                best_seg = min(segments, key=lambda s: abs(s['cx'] - self._center_x))
                if best_seg['cx'] < self._center_x:
                    # 可见段在左边 → 赛道中心在右边 half_track 处
                    cx = best_seg['cx'] + half_track
                else:
                    # 可见段在右边 → 赛道中心在左边 half_track 处
                    cx = best_seg['cx'] - half_track
                if 0 <= cx < self.bird_w:
                    track_centers.append(cx)

        if not track_centers:
            return None
        return float(np.median(track_centers))

    # ── 主入口 ──
    def process(self, bgr):
        bird, binary = self._preprocess(bgr)

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
            cx = self._scan_roi(binary, y0, y1)
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
