"""巡线模块 V2 — 鸟瞰 + 二次抛物线拟合（方法二）

流程：
  BGR → max(R,G,B) → warp → Otsu → 形态学闭运算
  → 逐行边缘对检测（左/右边过渡 + 宽度验证 + 单边外推）→ 加权抛物线拟合
  → 计算偏差（切线近端截距）和朝向（切线角）

时序稳定性机制（移植自 V0 3-ROI 检测器）：
  - 逐行 diff 边缘对验证：找白→黑（左沿）和黑→白（右沿）过渡，拒绝过窄/过宽对
  - 单边可见时从轨道半间距外推赛道中心（不再用 np.mean 偏到可见边）
  - 丢失检测时输出 last_dev*0.92 / last_heading*0.88（V0 非复合衰减模式）
  - 参数平滑仅在拟合高置信时激活，错误参数从 last-good 回退

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
        track_width_cm=35.5,         # 赛道两条黑线中心间距 (cm)
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
        self.track_width_cm = track_width_cm

        # 畸变校正（可选）
        self._K = None
        self._dist = None

        self.M = self._build_birdseye_matrix(lookahead_cm)

        # ── 从鸟瞰几何推导赛道黑线间距 (px) ──
        mid_cm = (lookahead_cm[0] + lookahead_cm[1]) / 2.0
        vfov_rad = np.radians(cam_vfov_deg)
        hfov_rad = 2.0 * np.arctan(np.tan(vfov_rad / 2.0) * cam_w / cam_h)
        ground_w_mid = 2.0 * mid_cm * np.tan(hfov_rad / 2.0)
        self._track_width_px = track_width_cm * (bird_w / ground_w_mid)

        # ── 时序状态 ──
        self._last_dev = None          # 上帧最终偏差
        self._last_heading = 0.0       # 上帧最终朝向
        self._last_a = 0.0             # 上帧抛物线系数
        self._last_b = 0.0
        self._last_c = 0.0
        self._last_y_mean = 0.5
        self._jump_cnt = 0             # 连续大跳帧计数

        # 高置信时保存的参数，用于差拟合时回退
        self._good_a = None
        self._good_b = None
        self._good_c = None
        self._good_y_mean = 0.5

        # V0 式丢失检测锚点（非复合衰减用）
        self._anchor_dev = None
        self._anchor_heading = 0.0

    # ── 鸟瞰变换 ──
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
            if Zc < 0.01:
                Zc = 0.01
            src_pts.append([fx * Xc / Zc + cx, fy * Yc / Zc + cy])
        src = np.float32(src_pts)

        dst = np.float32([
            [self.bird_w - 1, self.bird_h - 1], [0, self.bird_h - 1],
            [0, 0], [self.bird_w - 1, 0],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    # ── 逐行双黑线检测 → 推算赛道中心 ──
    def _scan_rows(self, binary, step=4):
        """每行找所有黑线段（4–40px宽），按段数推算赛道中心。

        赛道有两条平行黑线（左右边缘线），中心间距约 track_width_cm。
        binary: 0=黑(线), 255=白(背景)

        Returns
        -------
        points : list of (cx, row_y, weight)
            weight 1.0 = 双段赛道中心, 0.5 = 单段外推赛道中心
        stats  : dict {'full': N, 'half': N}
        """
        points = []
        n_full, n_half = 0, 0
        half_track = self._track_width_px / 2.0   # 赛道半间距 (~59 px)
        min_seg_w = 4                              # 单条黑线最小宽度
        max_seg_w = 40                             # 单条黑线最大宽度
        track_tol = max(20.0, self._track_width_px * 0.4)  # 双段配对容差

        for row in range(0, self.bird_h, step):
            line = binary[row, :]
            # diff <0: 255→0 = 白→黑 = 黑段左沿
            # diff >0: 0→255 = 黑→白 = 黑段右沿
            diff = np.diff(line.astype(np.int32) // 255)

            left_edges  = np.where(diff < 0)[0]
            right_edges = np.where(diff > 0)[0]

            # ── 构建黑段列表（左沿+右沿配对，按宽度过滤） ──
            segments = []
            for le in left_edges:
                rc = right_edges[right_edges > le]
                if len(rc) == 0:
                    continue
                re = rc[0]                         # 紧邻的右沿
                width = re - le
                if min_seg_w <= width <= max_seg_w:
                    segments.append({
                        'left': float(le),
                        'right': float(re),
                        'cx': float((le + re) * 0.5),
                    })

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
                        track_cx = (best_pair[0]['cx'] + best_pair[1]['cx']) * 0.5
                        points.append((track_cx, float(row), 1.0))
                        n_full += 1
                        continue
                # 未配成对 → 继续执行单段逻辑

            # ── 单段：用距离中心最近的段做外推 ──
            if len(segments) >= 1:
                best_seg = min(segments, key=lambda s: abs(s['cx'] - self._center_x))
                if best_seg['cx'] < self._center_x:
                    # 可见段在左边 → 赛道中心在右边
                    cx = best_seg['cx'] + half_track
                else:
                    # 可见段在右边 → 赛道中心在左边
                    cx = best_seg['cx'] - half_track
                if 0 <= cx < self.bird_w:
                    points.append((cx, float(row), 0.5))
                    n_half += 1

        return points, {'full': n_full, 'half': n_half}

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

        # ── 逐行边缘对采样（代替原 np.mean(black)） ──
        points, scan_stats = self._scan_rows(binary, step=4)

        deviation_px = None
        heading_deg = 0.0
        conf = 0.0
        inlier_count = 0
        a = b = c = 0.0
        y_mean = 0.5

        if len(points) >= 6:       # 放宽阈值：允许部分单边行
            pts_arr = np.array([(p[0], p[1]) for p in points], dtype=np.float32)
            weights = np.array([p[2] for p in points], dtype=np.float32)
            xs, ys = pts_arr[:, 0], pts_arr[:, 1]

            # 组合权重: 行位置权重(远行越重) × 边缘质量权重(双边=1, 单边=0.5)
            row_w = 0.4 + 0.6 * (ys / self.bird_h)
            combined_w = row_w * weights

            y_norm = ys / self.bird_h
            y_mean = y_norm.mean()
            y_centered = y_norm - y_mean

            coeffs = np.polyfit(y_centered, xs, 2, w=combined_w)
            a, b, c = coeffs

            pred = a * y_centered**2 + b * y_centered + c
            residuals = np.abs(xs - pred)
            inlier_mask = residuals < 4.0
            inlier_count = int(inlier_mask.sum())

            # 置信度: 内点率 × 双边行占比
            raw_conf = clamp(inlier_count / 30.0, 0.15, 1.0)
            total_edges = scan_stats['full'] + scan_stats['half']
            full_ratio = scan_stats['full'] / max(total_edges, 1)
            conf = clamp(raw_conf * (0.6 + 0.4 * full_ratio), 0.10, 1.0)

            valid_fit = False
            if inlier_count >= 6:
                rmse = float(np.sqrt(np.mean(residuals[inlier_mask]**2)))

                # ---- 拒绝明显错误的抛物线 ----
                if rmse > 3.0 or abs(a) > 0.5 * self.bird_w:
                    # 回退到上一次"好"参数
                    if self._good_a is not None and conf > 0.25:
                        a, b, c = self._good_a, self._good_b, self._good_c
                        y_mean = self._good_y_mean
                    else:
                        a, b, c = 0.0, b, c       # 降级为直线
                else:
                    valid_fit = True

                # ---- 参数级时序平滑（仅当拟合可靠且跳变小） ----
                if valid_fit and conf > 0.35 and self._last_a is not None:
                    param_jump = abs(a - self._last_a) + abs(b - self._last_b)
                    if param_jump < 0.3:
                        # 平滑因子: 高置信时更多信任历史
                        alpha_param = 0.55 if conf > 0.5 else 0.7
                        a = alpha_param * self._last_a + (1 - alpha_param) * a
                        b = alpha_param * self._last_b + (1 - alpha_param) * b
                        c = alpha_param * self._last_c + (1 - alpha_param) * c
                        y_mean = alpha_param * self._last_y_mean + (1 - alpha_param) * y_mean

                # 始终记录本帧参数（无论是否平滑）
                self._last_a, self._last_b, self._last_c = a, b, c
                self._last_y_mean = y_mean

                # 仅在拟合可靠时更新"好"参数锚点
                if conf > 0.40 and inlier_count >= 10:
                    self._good_a, self._good_b, self._good_c = a, b, c
                    self._good_y_mean = y_mean

                # 计算近端偏差和朝向
                y_bottom = 1.0 - y_mean
                near_x = a * y_bottom**2 + b * y_bottom + c
                deviation_px = near_x - self._center_x

                slope = 2 * a * y_bottom + b
                heading_deg = np.degrees(np.arctan(slope / self.bird_h))

        # ── 输出级时序平滑 ──
        if deviation_px is not None:
            # 检测到线：V2 原有跳变守卫 + V0 式 EMA
            if self._last_dev is not None:
                jump = abs(deviation_px - self._last_dev)
                if jump > 20:
                    # 疑似误检跳变 — 抑制，但限15帧超时
                    if self._jump_cnt < 15:
                        self._jump_cnt += 1
                        deviation_px = self._last_dev
                        heading_deg = self._last_heading
                        conf = max(0.08, conf * 0.5)
                    else:
                        self._jump_cnt = 0
                else:
                    self._jump_cnt = 0
                    # 自适应 alpha: <8px 轻跳用较大 alpha, >=8px 中跳用较小 alpha
                    alpha = 0.35 if jump > 8 else 0.60
                    deviation_px = alpha * deviation_px + (1 - alpha) * self._last_dev
                    heading_deg   = alpha * heading_deg   + (1 - alpha) * self._last_heading

            self._last_dev = deviation_px
            self._last_heading = heading_deg
            # 更新丢失锚点（最近一次有效检测）
            self._anchor_dev = deviation_px
            self._anchor_heading = heading_deg
        else:
            # 完全丢失：V0 式非复合指数衰减
            if self._anchor_dev is not None:
                deviation_px = self._anchor_dev * 0.92
                heading_deg   = self._anchor_heading * 0.88
                conf = 0.10
                # V0 行为：不更新 _anchor_*，下一帧仍从原锚点衰减
                # （避免复合衰减→0；维持"靠近最后已知位置"策略）

        # ── 可视化 ──
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (self._center_x, 0), (self._center_x, self.bird_h), (0, 0, 255), 1)
        # 双边点绿色，单边点黄色
        for px, py, wgt in points:
            color = (0, 255, 0) if wgt > 0.8 else (0, 200, 255)
            cv2.circle(vis, (int(px), int(py)), 1, color, -1)
        if deviation_px is not None:
            for y_pt in range(0, self.bird_h, 5):
                yn = y_pt / self.bird_h - y_mean
                x_pt = int(a * yn**2 + b * yn + c)
                if 0 <= x_pt < self.bird_w:
                    cv2.circle(vis, (x_pt, y_pt), 2, (0, 255, 0), -1)
            n_full = scan_stats.get('full', 0)
            n_half = scan_stats.get('half', 0)
            cv2.putText(vis,
                f"d={deviation_px:+.1f} h={heading_deg:+.0f} a={a:+.3f} in={inlier_count}/{len(points)}"
                f" F{n_full}H{n_half}",
                (4, self.bird_h - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)

        debug = {"bird": bird, "binary": binary, "heading_deg": heading_deg}
        return deviation_px, heading_deg, conf, vis, debug
