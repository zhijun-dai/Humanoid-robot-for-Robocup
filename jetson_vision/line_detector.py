"""巡线模块 — Jetson Nano / OpenCV 版。
核心思路：鸟瞰变换把地面"拍平"→ 赛道变两条平行线 → 偏差 = 中心偏移。
比 OpenMV 逐行扫描 + cm投影减少 ~500 行代码，同时更稳定。
"""
import cv2
import numpy as np


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LineDetector:
    def __init__(
        self,
        cam_height_cm=40.0,      # 相机离地高度
        cam_pitch_deg=45.0,      # 俯角
        cam_vfov_deg=44.0,       # 垂直视场角
        cam_hfov_deg=57.0,       # 水平视场角
        cam_w=640,                # 图像宽
        cam_h=480,                # 图像高
        track_width_cm=35.0,     # 赛道宽度
        bird_h=200,               # 鸟瞰图高度 (px)
        bird_w=160,               # 鸟瞰图宽度 (px)
        lookahead_cm=(10, 80),   # 鸟瞰图覆盖的前视距离 (cm)
        th_offset=8,              # Otsu 阈值偏移
        th_min=25,
        th_max=120,
    ):
        self.cam_height = cam_height_cm
        self.cam_pitch = np.radians(cam_pitch_deg)
        self.img_w = cam_w
        self.img_h = cam_h
        self.track_w = track_width_cm
        self.bird_h = bird_h
        self.bird_w = bird_w
        self.th_offset = th_offset
        self.th_min = th_min
        self.th_max = th_max

        # 鸟瞰变换矩阵（一次性计算）
        self.M = self._build_birdseye_matrix(lookahead_cm)
        self._bird_center = bird_w // 2

    def _build_birdseye_matrix(self, lookahead):
        """构建从相机视图到地面俯视图的单应矩阵。"""
        near, far = lookahead
        tan_v = np.tan(self.cam_pitch)
        tan_hf = np.tan(np.radians(57.0) / 2)

        # 地面前方 near~far cm 对应的图像行
        def ground_y(z_cm):
            ray = np.arctan2(self.cam_height, z_cm)
            v = ray - self.cam_pitch
            return (0.5 - v / np.radians(44.0)) * self.img_h

        y_far = ground_y(far)
        y_near = ground_y(near)

        # 源点：图像中的四边形（地面矩形在图像中的梯形投影）
        src = np.float32([
            [self.img_w - 1, y_near],           # 近处右
            [0, y_near],                         # 近处左
            [self.img_w - 1, y_far],            # 远处右
            [0, y_far],                          # 远处左
        ])

        # 目标点：鸟瞰图中的矩形
        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1],
            [0, self.bird_h - 1],
            [self.bird_w - 1, 0],
            [0, 0],
        ])

        return cv2.getPerspectiveTransform(src, dst)

    def process(self, bgr):
        """返回 (偏差像素, 置信度, 可视化图, 调试信息)。
        偏差: 正=偏右, 负=偏左, 0=居中。
        """
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # 1. 鸟瞰变换
        bird = cv2.warpPerspective(gray, self.M, (self.bird_w, self.bird_h))

        # 2. Otsu 阈值
        th_val, _ = cv2.threshold(bird, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_val = clamp(th_val + self.th_offset, self.th_min, self.th_max)
        _, binary = cv2.threshold(bird, th_val, 255, cv2.THRESH_BINARY)

        # 3. 闭运算填补线内缝隙
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 4. 左右半图分别统计黑像素质心 → 左线和右线
        half = self.bird_w // 2
        left_half = binary[:, :half]
        right_half = binary[:, half:]

        left_cols = np.argwhere(left_half == 0)  # 黑色像素
        right_cols = np.argwhere(right_half == 0)

        left_x = float(np.median(left_cols[:, 1])) if len(left_cols) > 20 else None
        right_x = float(half + np.median(right_cols[:, 1])) if len(right_cols) > 20 else None

        # 5. 计算偏差
        conf = 0.0
        deviation_px = None
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) * 0.5
            deviation_px = lane_center - self._bird_center
            lane_w = right_x - left_x
            # 置信度：检测到的线宽 vs 期望线宽
            expected_w = self.bird_w * (self.track_w / 80.0)  # approximate
            conf = clamp(1.0 - abs(lane_w - expected_w) / max(expected_w, 10), 0.0, 1.0)
        elif left_x is not None:
            deviation_px = left_x - self._bird_center + 20  # 只有左线，估右
            conf = 0.4
        elif right_x is not None:
            deviation_px = right_x - self._bird_center - 20  # 只有右线，估左
            conf = 0.4
        else:
            conf = 0.0

        # 6. 可视化
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        if left_x is not None:
            cv2.line(vis, (int(left_x), 0), (int(left_x), self.bird_h), (0, 255, 0), 2)
        if right_x is not None:
            cv2.line(vis, (int(right_x), 0), (int(right_x), self.bird_h), (0, 255, 0), 2)
        cv2.line(vis, (self._bird_center, 0), (self._bird_center, self.bird_h), (0, 0, 255), 1)

        debug = {"binary": binary, "bird": bird, "left_x": left_x, "right_x": right_x}
        return deviation_px, conf, vis, debug
