"""ShapeDetector — 2026新规则几何图卡识别（6种几何图形）。

圆形/五角星/正方形/菱形/十字形/三角形，10cm×10cm白底黑线卡，带矩形外边框。
v2 找框重构：线宽选择性预处理（blackhat核9 + 各向异性闭 + 笔画宽过滤）
+ Hough双族候选 + 量化验证（闭合度/线宽/环内含量）+ 帧间IoU跟踪。

场景：图卡平贴白色有污渍地面，赛道粗黑线（2cm）干扰。
粗线被 blackhat 核9 抑制 + 笔画宽[1.5,7]px 拒绝；细框线（0.5cm→3-5px）保留。
逐操作审阅巡线管线（line_detector_v1_warp.py 914-957行）：
  blackhat→保留（核31→9）；adaptive→保留；black_th二次阈值→废弃（细线瓶颈）；
  close5→保留（桥接断口，改用1×5/5×1各向异性）；open5/open3→废弃（磨细线）；
  CC面积/高度过滤→废弃（删细线CC），改笔画宽判据。

接口与 QRDetector 保持一致：update(bgr) → (action_number, dbg) 或 (None, None)。
动作映射（与2025二维码1-6对应）：
  圆形=1举左手 五角星=2举右手 正方形=3抬左腿 菱形=4抬右腿 十字形=5举双手 三角形=6摇头
"""
import os
import cv2
import time
import numpy as np

try:
    import torch
    from shape_cnn import ShapeCNN, CLASS_NAMES
except Exception:
    torch = None
    ShapeCNN = None
    CLASS_NAMES = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# 固定工作分辨率（参数标定基准，与YOLO方案A一致）
WORK_W = 960
WORK_H = 540


class ShapeDetector:
    def __init__(
        self,
        stable_frames=3,        # 连续确认帧数
        cooldown_ms=3200,       # 发送冷却
        roi_ratio=1.0,          # 检测ROI：画面下roi_ratio区域（默认全图）
        debug=True,
        classify_mode="auto",   # auto=CNN主判规则兜底 / cnn=只CNN / rules=纯CV
        compare_both=False,     # True: 两条路径都算，结果放 dbg
    ):
        self.stable_frames = stable_frames
        self.cooldown_ms = cooldown_ms
        self.roi_ratio = roi_ratio
        self.debug = debug
        self.classify_mode = os.environ.get("SHAPE_CLASSIFY_MODE",
                                            classify_mode)
        self.compare_both = compare_both

        # ── 找框参数（集中管理）──
        self.cfg = {
            # 线宽选择性预处理
            "bh_kernel": 9,          # 线宽选择性：只增强<核的细线（找框目标）
            "adaptive_block": 31,    # 自适应阈值窗口（与巡线一致）
            "adaptive_c": -16,       # 阈值偏移
            "stroke_min": 1.0,       # 笔画宽下限px（distanceTransform中位半径×2）
            "stroke_max": 7.0,       # 笔画宽上限px（拒巡线）
            # Hough候选
            "hough_thresh": 20,      # HoughLinesP投票阈值
            "min_line": 12,          # 线段最小长度（远距56px框的边）
            "max_gap": 10,           # 线段拼接最大间距（断线桥接）
            "topk": 4,               # 每族取前K条线组合
            "lsd_min_len": 10,       # LSD线段最短长度（LSD补HoughLinesP短边盲区）
            "corner_gap": 15,        # 角点通道：交点到线段近端端点容差（远卡断口碎片差12px，10误杀）
            "angle_gray": 5.0,       # 分族灰色带：与45°边界差<此值的线双族收录
            # 几何闸门
            "min_w": 30,             # 框最小宽（960×540，图卡56px@1.3m）
            "min_h": 12,             # 框最小高（56×14@1.3m）
            "aspect_min": 1.0,       # 宽高比下限（真实wh最低1.9；1.0兜住正视图/极端姿态，由其他验证把关）
            "aspect_max": 5.0,
            "area_min": 550,         # 面积下限（56×14=784）
            "area_max": 20000,
            "ang_min": 40,           # quad内角范围（度）；远桶GT实测38.6-143.5°
            "ang_max": 150,          # 原135/45误杀远桶透视压扁+旋转卡
            "edge_h_tol": 25.0,      # 边方向容差：至少2条边接近水平（±此角度）
            "edge_h_min": 2,         # 需满足的"接近水平"边数（图卡上下边；透视侧边放宽）
            # 验证阈值
            "closure_total": 0.45,   # 4边采样命中率均值
            "closure_edge": 0.30,    # 单边最低命中率（竖边放宽）
            "sample_band": 3,        # 采样带半宽px
            "n_samples": 10,         # 每边采样点数
            "inner_ratio": (0.02, 0.8),  # warp后中心区图形线占比（五角星5边实测0.72）
            "warp_size": 200,
            "warp_inset": 0.14,      # warp向内收缩比例（外框环不进warp）
            "refine_band": 8,        # quad逐边精调搜索带宽px
            "track_iou": 0.5,        # 帧间续锁IoU
            # CNN 分类（路线B分类器）
            "cnn_enable": os.environ.get("SHAPE_CNN_ENABLE", "1") != "0",
            "cnn_conf_min": 0.85,    # 低于此置信不输出（回退规则法）
        }

        # 动作映射: shape_name -> action_number (1-6)
        self.action_map = {
            "circle": 1,     # 举左手
            "pentagon": 2,   # 举右手
            "square": 3,     # 抬左腿
            "diamond": 4,    # 抬右腿
            "cross": 5,      # 举双手
            "triangle": 6,   # 摇头
        }

        # 状态
        self.candidate = None
        self.candidate_count = 0
        self.last_send_ms = None
        self.first_candidate_ms = None
        self.last_quad = None       # 帧间跟踪锁

        # ── Hu 矩模板（辅助判据，不改判定树；缺失则跳过）──
        self.hu_templates = {}
        self.last_hu = None
        try:
            hu_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "shape_hu_templates.npz")
            with np.load(hu_path) as f:
                self.hu_templates = {k: f[k] for k in f.files}
        except Exception:
            self.hu_templates = {}

        # ── CNN 分类器（权重缺失/torch缺失 → 规则法兜底）──
        self.cnn = None
        self.last_cnn_prob = None
        if self.cfg["cnn_enable"] and torch is not None:
            try:
                w_path = os.environ.get(
                    "SHAPE_CNN_WEIGHT",
                    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "shape_cnn_best_v2.pt"))
                model = ShapeCNN()
                model.load_state_dict(
                    torch.load(w_path, map_location="cpu"))
                model.eval()
                self.cnn = model
                print(f"[shape] CNN 已加载: {w_path}")
            except Exception as e:
                print(f"[shape] WARNING: CNN 加载失败({e})，回退规则分类")
                self.cnn = None

    # ═══════════════════════════════════════════════════════════
    # 主入口
    # ═══════════════════════════════════════════════════════════

    def update(self, bgr_or_gray):
        """返回 (action_number, debug_dict) 或 (None, None)。

        输入归一化：任意分辨率 → resize到960×540（参数标定基准，
        YOLO方案A同款）——参数与分辨率解耦；输出quad坐标映射回原图。
        """
        if len(bgr_or_gray.shape) == 3:
            gray = cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = bgr_or_gray
        h0, w0 = gray.shape[:2]
        self._scale_x = w0 / WORK_W
        self._scale_y = h0 / WORK_H

        if self.roi_ratio < 1.0:
            h = gray.shape[0]
            y0 = int(h * (1.0 - self.roi_ratio))
            gray = gray[y0:, :]
            self._roi_y0 = y0
        else:
            self._roi_y0 = 0

        # resize到固定工作分辨率（960×540）
        if (w0, h0) != (WORK_W, WORK_H):
            gray = cv2.resize(gray, (WORK_W, WORK_H))

        # S1 线宽选择性二值化（线=白255）
        binary = self._binary_selective(gray)
        dt = cv2.distanceTransform(binary, cv2.DIST_L2, 5)

        # S2 候选生成
        hsegs, lsegs = self._detect_segments(binary)
        quads = self._hough_quads(binary, hsegs)
        quads += self._lsd_quads(binary, lsegs)
        quads += self._corner_quads(binary, hsegs + lsegs)
        quads += self._cc_quads(binary)

        # S3 验证 + 评分
        # 性能：候选可能数百个（视频帧纹理），先轻量几何预筛（纯数值，
        # 不采样不warp），通过的才做完整验证（采样+DT+warp200）——
        # 实测512候选完整验证2.5s → 预筛后剩几十个
        best, best_score, scores = None, 0.0, []
        for q in quads:
            if not self._geom_ok(q):
                continue
            q = self._refine_quad(binary, q)
            if not self._geom_ok(q):
                continue
            v = self._verify_quad(binary, dt, q)
            if v is not None:
                score, closure = v
                scores.append((round(score, 3), closure))
                # 帧间跟踪：与上一帧候选IoU高的加分
                if self.last_quad is not None:
                    iou = self._poly_iou(q, self.last_quad)
                    if iou > self.cfg["track_iou"]:
                        score += 0.3
                if score > best_score:
                    best, best_score = q, score
        if best is not None:
            self.last_quad = best
        else:
            self.last_quad = None

        shape = None
        dbg = {"card_found": best is not None, "roi_y0": self._roi_y0,
               "roi_ratio": self.roi_ratio, "scores": scores[:8]}

        if best is not None:
            warp = self._warp_card(binary, best)
            dbg["warp"] = warp
            shape = self._classify(warp, dbg)
            # quad映射回原图分辨率（找框在960×540上做）
            q_orig = best.astype(np.float32) * np.array(
                [self._scale_x, self._scale_y], np.float32)
            dbg["quad"] = q_orig
            dbg["closure"] = best_score
        else:
            self.last_cnn_prob = None  # 无框路径未跑 CNN
            shape = self._classify_shape_full(binary)
            dbg["fallback"] = True

        dbg["cnn_prob"] = self.last_cnn_prob if self.cnn is not None else None
        if shape is None:
            self.candidate = None
            self.candidate_count = 0
            dbg["shape"] = None
            return None, dbg

        dbg["shape"] = shape
        return self._confirm(shape, dbg)

    # ═══════════════════════════════════════════════════════════
    # S1 线宽选择性二值化
    # ═══════════════════════════════════════════════════════════

    def _binary_selective(self, gray):
        """复用YOLO预处理方案（shape_preprocess）+ 找框保留环节。

        YOLO方案：blackhat(31) → adaptive(31,-12) → close(3×3)；
        不反转（YOLO要白底黑线，找框/CNN要黑底白线=线白）。
        找框保留：各向异性闭（桥接竖边断点）、笔画宽过滤（核31会
        增强2cm巡线，必须按线宽拒掉）、细长度过滤（拒圆斑污渍）。"""
        kbh = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                        (self.cfg["bh_kernel"],
                                         self.cfg["bh_kernel"]))
        gd = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kbh)

        binary = cv2.adaptiveThreshold(
            gd, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
            self.cfg["adaptive_block"], self.cfg["adaptive_c"])

        # close(3×3)（YOLO方案形态学）
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k3)

        # 各向异性闭：1×5竖桥接远距竖边断点，5×1横补角
        kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
        kh = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kv)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kh)

        binary = self._stroke_width_filter(binary)
        return self._elongation_filter(binary)

    def _elongation_filter(self, binary, area_max=60.0):
        """小CC细长度过滤：面积<area_max且长宽比<2的CC删除（圆斑污渍）。

        只过滤小面积：图形环（圆形/方形/三角）bbox长宽比≈1但面积≥80px²
        （最小卡51px的图形环），外框环段面积更大；污渍圆斑直径1-7px
        面积≤38px²——面积上限+长宽比双闸区分线/斑，不误删图形。
        向量化（stats数组numpy操作，无Python逐CC循环——视频帧CC数百）。"""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if n <= 1:
            return np.zeros_like(binary)
        w = stats[1:, cv2.CC_STAT_WIDTH]
        h = stats[1:, cv2.CC_STAT_HEIGHT]
        a = stats[1:, cv2.CC_STAT_AREA]
        ratio = np.maximum(w, h) / np.maximum(np.minimum(w, h), 1)
        bad = (a < area_max) & (ratio < 2.0)
        keep = np.ones(n, bool)
        keep[0] = False  # 背景
        keep[1:][bad] = False
        return (np.isin(labels, np.flatnonzero(keep)).astype(np.uint8) * 255)

    def _stroke_width_filter(self, binary):
        """笔画宽过滤：CC内DT中位半径×2∈[1.5,7]px（拒巡线、留细框线）。

        向量化：labels排序后reduceat分段取中位，避免Python逐CC循环
        （视频帧CC数百→千，原循环是预处理耗时大头）。"""
        dt = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if n <= 1:
            return np.zeros_like(binary)
        lo, hi = self.cfg["stroke_min"], self.cfg["stroke_max"]
        lab_v = labels.ravel()
        dt_v = dt.ravel()
        valid = lab_v > 0
        lab_s = lab_v[valid]
        dt_s = dt_v[valid]
        order = np.argsort(lab_s, kind="stable")
        lab_o = lab_s[order]
        dt_o = dt_s[order]
        counts = np.bincount(lab_s, minlength=n)
        starts = np.searchsorted(lab_o, np.arange(n))
        med = np.zeros(n)
        for lid in range(1, n):
            seg = dt_o[starts[lid]:starts[lid] + counts[lid]]
            seg = seg[seg > 0]
            if len(seg):
                med[lid] = np.median(seg)
        keep = (med >= lo / 2) & (med <= hi / 2)
        keep[0] = False
        return (np.isin(labels, np.flatnonzero(keep)).astype(np.uint8) * 255)

    # ═══════════════════════════════════════════════════════════
    # S2 候选生成
    # ═══════════════════════════════════════════════════════════

    def _detect_segments(self, binary):
        """HoughLinesP + LSD 线段一次检出，供各候选通道共用。"""
        c = self.cfg
        hough_segs = []
        lines = cv2.HoughLinesP(binary, 1, np.pi / 180,
                                c["hough_thresh"],
                                minLineLength=c["min_line"],
                                maxLineGap=c["max_gap"])
        if lines is not None:
            hough_segs = [tuple(int(v) for v in ln) for ln in lines[:, 0]]
        lsd_segs = []
        lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
        det = lsd.detect(binary)[0]
        if det is not None:
            lsd_segs = [tuple(int(v) for v in s[0]) for s in det]
        return hough_segs, lsd_segs

    def _hough_quads(self, binary, segs=None):
        """Hough主通道：线段按角度分横/竖族，极值线组合求交点成quad。"""
        c = self.cfg
        if segs is None:
            segs, _ = self._detect_segments(binary)
        horiz, vert = self._split_families(segs, c["min_line"])
        return self._quads_from_families(horiz, vert)

    @staticmethod
    def _split_families(segs, min_len, gray_band=18.0):
        """线段按角度分横/竖族；45°边界灰色带内的线双族收录。

        segs: [(x1,y1,x2,y2), ...] → (horiz, vert)，各为 [(坐标,线段),...]。
        坐标：横族用y中值（上下边），竖族用x中值（左右边）。
        """
        horiz, vert = [], []
        for x1, y1, x2, y2 in segs:
            dx, dy = x2 - x1, y2 - y1
            length = np.hypot(dx, dy)
            if length < min_len:
                continue
            theta = abs(np.degrees(np.arctan2(dy, dx)))
            if theta > 90.0:
                theta = 180.0 - theta
            # theta∈[0,90]：0=水平 90=垂直；45°附近灰色带双族收录
            if abs(theta - 45.0) > gray_band:
                if theta < 45.0:
                    horiz.append(((y1 + y2) / 2.0, (x1, y1, x2, y2)))
                else:
                    vert.append(((x1 + x2) / 2.0, (x1, y1, x2, y2)))
            else:
                horiz.append(((y1 + y2) / 2.0, (x1, y1, x2, y2)))
                vert.append(((x1 + x2) / 2.0, (x1, y1, x2, y2)))
        return horiz, vert

    def _quads_from_families(self, horiz, vert):
        """横/竖族极值线组合成quad（top/bottom/left/right各取topk）。"""
        c = self.cfg
        if len(horiz) < 2 or len(vert) < 2:
            return []
        horiz.sort()
        vert.sort()
        top_cands = horiz[:c["topk"]]
        bot_cands = horiz[-c["topk"]:]
        lft_cands = vert[:c["topk"]]
        rgt_cands = vert[-c["topk"]:]
        quads = []
        for yt, l_top in top_cands:
            for yb, l_bot in bot_cands:
                if yb - yt < c["min_h"]:
                    continue
                for xl, l_lft in lft_cands:
                    for xr, l_rgt in rgt_cands:
                        if xr - xl < c["min_w"]:
                            continue
                        q = self._quad_from_lines(l_top, l_bot, l_lft, l_rgt)
                        if q is not None:
                            quads.append(q)
        return quads

    def _lsd_quads(self, binary, segs=None):
        """LSD辅助通道：确定性线段检测，补HoughLinesP短边漏检（远距断线）。

        HoughLinesP是概率算法（内部RNG），20-40px短边时好时坏；
        LSD确定性检出所有细线段，双族组合成quad。
        """
        c = self.cfg
        if segs is None:
            _, segs = self._detect_segments(binary)
        horiz, vert = self._split_families(segs, c["lsd_min_len"])
        return self._quads_from_families(horiz, vert)

    def _corner_quads(self, binary, segs=None, gap=None, min_len=6):
        """角点4环通道：线段端点近交成角点，4条段闭环成quad。

        远桶（卡<80px）外框断裂成碎片且卡不在画面极值处，Hough/LSD
        的"每族取极值线"组合被噪声线干扰（59/115失败样本）。本通道
        直接找"两线段交点在各自近端端点gap内"的角点，再取横上/横下
        两段共同垂直伙伴构成4环——只生成有端点支撑的quad，与卡位置
        无关。角点本身由线段端点簇确定，碎片越多角点证据越强。
        """
        if gap is None:
            gap = self.cfg["corner_gap"]
        if segs is None:
            hs, ls = self._detect_segments(binary)
            segs = hs + ls
        # 去重（网格量化O(n)）+ 长度过滤 + 数量上限
        # 上限原因：视频帧纹理线段可达6000+条，O(n²)求交爆炸
        # （实测f0: Hough 2355+LSD 3677 → 3600万次求交卡死分钟级）；
        # 图卡框线15-50px比纹理线长，按长度取top200足够。
        seen_keys = set()
        uniq = []
        for s in segs:
            if np.hypot(s[2]-s[0], s[3]-s[1]) < min_len:
                continue
            key = (s[0] // 8, s[1] // 8, s[2] // 8, s[3] // 8)
            if key in seen_keys:
                continue
            seen_keys.add(key)
            uniq.append(s)
        uniq.sort(key=lambda s: -(np.hypot(s[2]-s[0], s[3]-s[1])))
        segs = uniq[:200]
        N = len(segs)
        if N < 4:
            return []
        angs = np.zeros(N)
        ex = np.zeros((N, 2), np.float32)
        ey = np.zeros((N, 2), np.float32)
        for i, (x1, y1, x2, y2) in enumerate(segs):
            th = abs(np.degrees(np.arctan2(y2-y1, x2-x1))) % 180
            angs[i] = th if th <= 90 else 180 - th
            ex[i] = (x1, x2)
            ey[i] = (y1, y2)
        # 角点：近垂直对且交点在近端端点gap内
        partners = [set() for _ in range(N)]
        for i in range(N):
            xi1, yi1, xi2, yi2 = segs[i]
            ai = angs[i]
            for j in range(i+1, N):
                da = abs(ai - angs[j])
                da = min(da, 180.0 - da)
                if da < 35 or da > 145:
                    continue
                xj1, yj1, xj2, yj2 = segs[j]
                denom = (xi1-xi2)*(yj1-yj2) - (yi1-yi2)*(xj1-xj2)
                if abs(denom) < 1e-9:
                    continue
                t = ((xi1-xj1)*(yj1-yj2) - (yi1-yj1)*(xj1-xj2)) / denom
                px, py = xi1 + t*(xi2-xi1), yi1 + t*(yi2-yi1)
                # 到两段最近端点的距离（标量，避免numpy逐对开销）
                d1 = min(np.hypot(px-xi1, py-yi1), np.hypot(px-xi2, py-yi2))
                d2 = min(np.hypot(px-xj1, py-yj1), np.hypot(px-xj2, py-yj2))
                if d1 <= gap and d2 <= gap:
                    partners[i].add(j)
                    partners[j].add(i)
        h_idx = [i for i in range(N) if angs[i] <= 62]
        v_idx = [i for i in range(N) if angs[i] >= 28]
        quads, seen = [], []
        for a in range(len(h_idx)):
            for b in range(a+1, len(h_idx)):
                i, j = h_idx[a], h_idx[b]
                # 上下边对：两段y投影须分离（ey存端点序，需先取min/max）
                yi0, yi1 = min(ey[i][0], ey[i][1]), max(ey[i][0], ey[i][1])
                yj0, yj1 = min(ey[j][0], ey[j][1]), max(ey[j][0], ey[j][1])
                if min(yi1, yj1) - max(yi0, yj0) > 0:
                    continue
                common = partners[i] & partners[j]
                if len(common) < 2:
                    continue
                cl = sorted(common)
                for m in range(len(cl)):
                    for n in range(m+1, len(cl)):
                        p, q = cl[m], cl[n]
                        if p not in v_idx or q not in v_idx:
                            continue
                        if ex[p].mean() > ex[q].mean():
                            p, q = q, p
                        qd = self._quad_from_lines(segs[i], segs[j], segs[p], segs[q])
                        if qd is None:
                            continue
                        qf = qd.astype(np.float32)
                        x, y, w, h = cv2.boundingRect(qf.astype(np.int32))
                        if w < 24 or h < 10:
                            continue
                        area = cv2.contourArea(qf)
                        if area < 300 or area > 40000:
                            continue
                        if not self._quad_convex(qf):
                            continue
                        dup = any(self._quad_close(qf, s2) for s2 in seen)
                        if dup:
                            continue
                        seen.append(qf)
                        quads.append(qf)
        return quads

    @staticmethod
    def _quad_convex(q):
        """凸度：面积/凸包面积≥0.85。远距1-2px测点噪声会让正确quad
        一个角呈微凹（叉积符号翻转），严格符号判据误杀，改用面积比。"""
        pts = q.astype(np.float32)
        hull_area = cv2.contourArea(cv2.convexHull(pts))
        if hull_area <= 1e-6:
            return False
        return cv2.contourArea(pts) / hull_area >= 0.85

    @staticmethod
    def _quad_close(a, b, tol=8.0):
        """两quad是否重复：a的4角到b的最近角距离均值≤tol。

        不用bbox-IoU（斜卡bbox重叠大但多边形不重叠，会误判重复）。"""
        pa = a.astype(np.float32)
        pb = b.astype(np.float32)
        d = 0.0
        for p in pa:
            d += np.min(np.linalg.norm(pb - p, axis=1))
        return d / 4.0 <= tol

    @staticmethod
    def _line_intersect(l1, l2):
        (x1, y1, x2, y2), (x3, y3, x4, y4) = l1, l2
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-9:
            return None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        return np.array([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])

    def _quad_from_lines(self, top, bot, lft, rgt):
        tl = self._line_intersect(lft, top)
        tr = self._line_intersect(rgt, top)
        br = self._line_intersect(rgt, bot)
        bl = self._line_intersect(lft, bot)
        if any(p is None for p in (tl, tr, br, bl)):
            return None
        return np.stack([tl, tr, br, bl])

    def _cc_quads(self, binary):
        """CC辅助通道：连通域四边形拟合（近卡冗余，远距断线时失效）。"""
        c = self.cfg
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        quads = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            if peri <= 0:
                continue
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if not (3 <= len(approx) <= 6):
                continue
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w < c["min_w"] or h < c["min_h"]:
                continue
            area = cv2.contourArea(cnt)
            if not (c["area_min"] <= area <= c["area_max"]):
                continue
            box = cv2.boxPoints(rect)
            quads.append(box)
        return quads

    # ═══════════════════════════════════════════════════════════
    # S3 量化验证
    # ═══════════════════════════════════════════════════════════

    def _refine_quad(self, binary, quad, band=8):
        """逐边垂直平移±band，取沿线支撑最多的位置对准框线。

        Hough/LSD线段是框线的中心线且可能偏离数px（远距卡3-16px），
        直接把边滑到框线中心，warp后图形不与外框粘连。
        """
        h, w = binary.shape
        q = quad.astype(np.float32)
        edges = []
        for e in range(4):
            p1, p2 = q[e], q[(e + 1) % 4]
            d = p2 - p1
            L = np.hypot(*d)
            if L < 1e-6:
                return quad
            n = np.array([-d[1], d[0]]) / L
            npts = max(10, int(L * 0.6))
            base_t = np.linspace(0.0, 1.0, npts)
            best_off, best_sup = 0, -1
            for off in range(-band, band + 1):
                q1 = p1 + n * off
                q2 = p2 + n * off
                xs = q1[0] + base_t * (q2[0] - q1[0])
                ys = q1[1] + base_t * (q2[1] - q1[1])
                xi = np.clip(xs.astype(int), 0, w - 1)
                yi = np.clip(ys.astype(int), 0, h - 1)
                sup = int(np.count_nonzero(binary[yi, xi]))
                sup += int(np.count_nonzero(binary[np.clip(yi + 1, 0, h - 1), xi]))
                sup += int(np.count_nonzero(binary[np.clip(yi - 1, 0, h - 1), xi]))
                sup += int(np.count_nonzero(binary[yi, np.clip(xi + 1, 0, w - 1)]))
                sup += int(np.count_nonzero(binary[yi, np.clip(xi - 1, 0, w - 1)]))
                if sup > best_sup:
                    best_sup, best_off = sup, off
            edges.append((p1 + n * best_off, p2 + n * best_off, d / L))
        corners = []
        for i in range(4):
            a, _, da = edges[(i - 1) % 4]
            b, _, db = edges[i]
            denom = da[0] * db[1] - da[1] * db[0]
            if abs(denom) < 1e-9:
                return quad
            t = ((b[0] - a[0]) * db[1] - (b[1] - a[1]) * db[0]) / denom
            corners.append(a + da * t)
        refined = np.stack(corners)
        # 外扩（仅对过小quad）：quad边来自线段中心交点，比GT外框
        # 小1-6px/边，远卡相对损失大被area闸门拒；逐边沿外法向滑到
        # 外框外缘。支撑判据=采样点带内命中占比≥0.6（外框线全长覆盖；
        # 图形内笔画只局部斜穿，占比低，不会停错）。已正大的quad跳过。
        if cv2.contourArea(refined) < self.cfg["area_min"] * 1.5:
            ctr = refined.mean(axis=0)
            edges_out = []
            for e in range(4):
                p1, p2 = refined[e], refined[(e + 1) % 4]
                d = p2 - p1
                L = np.hypot(*d)
                if L < 1e-6:
                    break
                n = np.array([-d[1], d[0]]) / L
                mid = (p1 + p2) / 2.0
                if np.dot(n, mid - ctr) < 0:
                    n = -n
                npts = max(10, int(L * 0.6))
                base_t = np.linspace(0.0, 1.0, npts)
                # 外框线是quad外法向最外侧的全长线：扫描全程取最后一个
                # 全长覆盖位置（内侧图形笔画只局部覆盖，不会入选）
                best_off = 0
                for off in range(1, 16):
                    q1 = p1 + n * off
                    q2 = p2 + n * off
                    xs = q1[0] + base_t * (q2[0] - q1[0])
                    ys = q1[1] + base_t * (q2[1] - q1[1])
                    xi = np.clip(xs.astype(int), 0, w - 1)
                    yi = np.clip(ys.astype(int), 0, h - 1)
                    # 只取线本身命中（±1带会让线停在框外2-3px，线宽闸门dt=0拒）
                    if np.count_nonzero(binary[yi, xi]) >= npts * 0.5:
                        best_off = off
                edges_out.append((p1 + n * best_off, p2 + n * best_off, d / L))
            if len(edges_out) == 4:
                out_corners = []
                for i in range(4):
                    a, _, da = edges_out[(i - 1) % 4]
                    b, _, db = edges_out[i]
                    denom = da[0] * db[1] - da[1] * db[0]
                    if abs(denom) < 1e-9:
                        break
                    t = ((b[0] - a[0]) * db[1] - (b[1] - a[1]) * db[0]) / denom
                    out_corners.append(a + da * t)
                if len(out_corners) == 4:
                    expanded = np.stack(out_corners)
                    if cv2.contourArea(expanded) > cv2.contourArea(refined):
                        refined = expanded
        # 精调回退：远距短线卡（高16-30px）边滑到图形内平行线时，
        # 角点求交会爆开（41×27→291×427）。回退保持原quad，验证闸门把关。
        if self._poly_iou(refined, quad) < 0.4:
            return quad
        return refined

    def _geom_ok(self, quad):
        """轻量几何预筛（纯数值，不采样）：面积/宽高/宽高比/内角/边方向。"""
        c = self.cfg
        q = quad.astype(np.float32)
        area = cv2.contourArea(q)
        if not (c["area_min"] <= area <= c["area_max"]):
            return False
        x, y, w, h = cv2.boundingRect(q.astype(np.int32))
        if w < c["min_w"] or h < c["min_h"]:
            return False
        aspect = max(w, h) / max(min(w, h), 1.0)
        if not (c["aspect_min"] <= aspect <= c["aspect_max"]):
            return False
        for i in range(4):
            p1 = q[(i - 1) % 4]
            p2 = q[i]
            p3 = q[(i + 1) % 4]
            v1 = p1 - p2
            v2 = p3 - p2
            cos = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9)
            ang = np.degrees(np.arccos(np.clip(cos, -1, 1)))
            if not (c["ang_min"] <= ang <= c["ang_max"]):
                return False
        # 边方向：至少 N 条边接近水平（图卡平放地面时上下边近水平；
        # 阴影/干扰形成的歪斜四边形通常无水平边）
        n_h = 0
        for i in range(4):
            p1, p2 = q[i], q[(i + 1) % 4]
            a = abs(np.degrees(np.arctan2(p2[1] - p1[1],
                                         p2[0] - p1[0]))) % 180.0
            if min(a, abs(a - 180.0)) <= c["edge_h_tol"]:
                n_h += 1
        if n_h < c["edge_h_min"]:
            return False
        return True

    def _verify_quad(self, binary, dt, quad):
        """返回 (score, closure) 或 None。闭合度/线宽/环内含量（几何闸门在_geom_ok）。"""
        c = self.cfg
        q = quad.astype(np.float32)

        # 闭合度：边采样 ±band 带内命中
        ns = c["n_samples"]
        band = c["sample_band"]
        hits_per_edge = []
        widths = []
        for e in range(4):
            p1 = q[e]
            p2 = q[(e + 1) % 4]
            hits = 0
            for k in range(1, ns):
                t = k / ns
                px = int(round(p1[0] + t * (p2[0] - p1[0])))
                py = int(round(p1[1] + t * (p2[1] - p1[1])))
                y0 = max(0, py - band)
                y1 = min(binary.shape[0], py + band + 1)
                x0 = max(0, px - band)
                x1 = min(binary.shape[1], px + band + 1)
                if np.any(binary[y0:y1, x0:x1] > 0):
                    hits += 1
                    if (y0 < py < y1 and x0 < px < x1):
                        widths.append(dt[py, px])
            hits_per_edge.append(hits / (ns - 1))
        closure = float(np.mean(hits_per_edge))
        if closure < c["closure_total"] or min(hits_per_edge) < c["closure_edge"]:
            return None

        # 线宽一致性
        if widths:
            med_w = 2.0 * float(np.median(widths))
            if not (c["stroke_min"] <= med_w <= c["stroke_max"]):
                return None

        # 环内含量（warp后中心区图形线占比）
        warp = self._warp_card(binary, q)
        ws = c["warp_size"]
        margin = int(ws * 0.15)
        inner = warp[margin:ws - margin, margin:ws - margin]
        ratio = np.count_nonzero(inner) / inner.size
        r_lo, r_hi = c["inner_ratio"]
        if not (r_lo <= ratio <= r_hi):
            return None

        # score：闭合度为主 + 线宽/几何符合加分
        score = closure
        if widths and 1.5 <= 2.0 * np.median(widths) <= 7.0:
            score += 0.2
        x, y, w, h = cv2.boundingRect(q.astype(np.int32))
        aspect = max(w, h) / max(min(w, h), 1.0)
        if 1.5 <= aspect <= 4.0:
            score += 0.1
        return score, closure

    # ═══════════════════════════════════════════════════════════
    # 单应性矫正
    # ═══════════════════════════════════════════════════════════

    def _warp_card(self, binary, quad):
        """4角点 → warp_size×warp_size 正视图。

        角排序修复：绕质心按atan2排序后旋转到左上起点（旧版sum/diff假设
        凸四边形完整，缺角/木纹合并时排序错乱）。
        quad来自Hough线段中心线→先沿中心外扩约半个线宽，避免外框
        线被裁到warp边缘（否则RETR_LIST拿不到完整外框环）。
        """
        ws = self.cfg["warp_size"]
        q = quad.astype(np.float32)
        c = q.mean(axis=0)
        # 先外扩4px：quad可能落在框线内缘（远距1-2px细线，精调后
        # 内外缘难分），外扩到框外缘
        v = q - c
        norms = np.linalg.norm(v, axis=1, keepdims=True) + 1e-9
        q = c + v / norms * (norms + 4.0)
        # 再向内收缩：外框环（卡边5%+框线5%）不进warp，图形不会与环
        # 粘连（远距卡透视压缩后图形-环间隙只有2-4px，warp后必并）。
        q = c + (q - c) * (1.0 - self.cfg["warp_inset"])
        cx, cy = q.mean(axis=0)
        ang = np.arctan2(q[:, 1] - cy, q[:, 0] - cx)
        order = np.argsort(ang)
        qs = q[order]
        # 旋转起点：使第一个角点是"左上"（x+y最小，与dst映射一致）
        start = int(np.argmin(qs.sum(axis=1)))
        qs = np.roll(qs, -start, axis=0)
        dst = np.float32([[0, 0], [ws - 1, 0], [ws - 1, ws - 1], [0, ws - 1]])
        M = cv2.getPerspectiveTransform(qs, dst)
        # INTER_NEAREST：二值图线性插值会把1-2px细笔画磨断（软边被
        # 阈值吃掉），最近邻保持笔画完整
        return cv2.warpPerspective(binary, M, (ws, ws),
                                   flags=cv2.INTER_NEAREST)

    @staticmethod
    def _poly_iou(q1, q2):
        """两个四边形的IoU（用轮廓相交近似）。"""
        p1 = q1.astype(np.int32).reshape(-1, 1, 2)
        p2 = q2.astype(np.int32).reshape(-1, 1, 2)
        r1 = cv2.boundingRect(p1)
        r2 = cv2.boundingRect(p2)
        x = max(r1[0], r2[0])
        y = max(r1[1], r2[1])
        w = min(r1[0] + r1[2], r2[0] + r2[2]) - x
        h = min(r1[1] + r1[3], r2[1] + r2[3]) - y
        inter = max(0, w) * max(0, h)
        a1 = cv2.contourArea(p1)
        a2 = cv2.contourArea(p2)
        return inter / max(a1 + a2 - inter, 1e-6)

    # ═══════════════════════════════════════════════════════════
    # 形状分类双路径：CNN（_classify_cnn）/ 纯CV规则（_classify_shape）
    # ═══════════════════════════════════════════════════════════

    def _classify(self, warp, dbg):
        """按 classify_mode 选路径；compare_both 时两路都算存 dbg。

        auto = CNN 主判，判不出（背景/低置信/无权重）回退规则法
        cnn  = 只用 CNN
        rules= 只用纯 CV 规则法（资格审核/对比用）
        """
        mode = self.classify_mode
        shape_cnn = shape_rules = None
        self.last_cnn_prob = None
        self.last_hu = None
        if mode in ("auto", "cnn") and self.cnn is not None:
            shape_cnn = self._classify_cnn(warp)
        if (mode == "rules" or self.compare_both
                or (mode == "auto" and shape_cnn is None)):
            shape_rules = self._classify_shape(warp)
        if mode == "rules":
            shape = shape_rules
        elif mode == "cnn":
            shape = shape_cnn
        else:
            shape = shape_cnn if shape_cnn is not None else shape_rules
        dbg["shape_cnn"] = shape_cnn
        dbg["shape_rules"] = shape_rules
        dbg["hu_best"] = self.last_hu[0] if self.last_hu else None
        dbg["hu_dist"] = self.last_hu[1] if self.last_hu else None
        return shape

    def _classify_cnn(self, warp):
        """ShapeCNN 分类 warp200（黑底白线）→ shape name 或 None。

        输入与训练一致：_warp_card 输出（黑底白线 255）→ resize 96
        INTER_LINEAR → /255 → (1,1,96,96)。label 6=背景 或
        置信 < cnn_conf_min → None（调用方回退规则法）。
        """
        try:
            x = cv2.resize(warp, (96, 96), interpolation=cv2.INTER_LINEAR)
            x = torch.from_numpy(x).float().unsqueeze(0).unsqueeze(0) / 255.0
            with torch.no_grad():
                prob = torch.softmax(self.cnn(x), dim=1)[0]
            p_max, idx = float(prob.max()), int(prob.argmax())
            self.last_cnn_prob = p_max
            if idx >= len(CLASS_NAMES) or p_max < self.cfg["cnn_conf_min"]:
                return None
            return CLASS_NAMES[idx]
        except Exception:
            self.last_cnn_prob = None
            return None

    def _classify_shape(self, warp):
        # 远距卡warp后笔画1-2px且有2-10px断口：close(5,5)+dilate(1px)桥接
        # 笔画碎片（五角星臂/十字臂/三角形边）；图形与外框环间隙≥8px，
        # 不会把它们连起来
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        warp = cv2.morphologyEx(warp, cv2.MORPH_CLOSE, kernel)
        k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        warp = cv2.dilate(warp, k3, iterations=1)
        contours, _ = cv2.findContours(warp, cv2.RETR_LIST,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        # 参考=卡大小（warp边长）：inset-warp后外框环不进warp，最大
        # 轮廓就是图形本体，以它为参考会排除图形自身的内边界。
        # bbox≤0.85×warp挡环残留（quad偏时环残片bbox≈0.8-0.9×warp，
        # 但其质心偏边，居中闸门也会排除）。
        size_ref = warp.shape[0]
        area_ref = float(size_ref * size_ref)
        cw, ch = warp.shape[1], warp.shape[0]
        cands = []
        for c in contours:
            a = cv2.contourArea(c)
            if a >= area_ref * 0.92:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            if (abs(cx - cw / 2) > cw * 0.15
                    or abs(cy - ch / 2) > ch * 0.15):
                continue
            ix, iy, iw, ih = cv2.boundingRect(c)
            if max(iw, ih) > size_ref * 0.95:
                continue
            cands.append((a, c, (ix, iy, iw, ih)))
        if not cands:
            return None
        # 以最大候选为主体，并集与其bbox重叠（含10px余量）的碎片/内边界：
        # 五角星/三角形的细边碎片（星臂与主体分离数px）并入主体；
        # 远离主体的轮廓（环残留等）不并，避免锯齿导致凸度虚高。
        cands.sort(key=lambda t: -t[0])
        _, main, (mx, my, mw, mh) = cands[0]
        mask = np.zeros_like(warp)
        cv2.drawContours(mask, [main], -1, 255, -1)
        for a, c, (ix, iy, iw, ih) in cands[1:]:
            if (ix < mx + mw + 10 and ix + iw > mx - 10
                    and iy < my + mh + 10 and iy + ih > my - 10):
                cv2.drawContours(mask, [c], -1, 255, -1)
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        if not blobs:
            return None
        blob = max(blobs, key=cv2.contourArea)
        if cv2.contourArea(blob) < 30 * 30:
            return None
        return self._classify_contour(blob)

    def _classify_shape_full(self, binary):
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a > best_area:
                best_area = a
                best = c
        if best is None or best_area < 30 * 30:
            return None
        return self._classify_contour(best)

    def _hu_match(self, contour):
        """Hu 矩模板匹配（辅助信息）：返回 (最接近类名, 距离) 或 None。

        注意：Hu 矩旋转不变 → 方形/菱形理论同距，仅作参考，不参与判定。
        """
        if not self.hu_templates:
            return None
        best = None
        for name, tpl in self.hu_templates.items():
            d = cv2.matchShapes(contour, tpl, cv2.CONTOURS_MATCH_I1, 0.0)
            if best is None or d < best[1]:
                best = (name, float(d))
        return best

    def _classify_contour(self, contour):
        self.last_hu = self._hu_match(contour)
        peri = cv2.arcLength(contour, True)
        if peri <= 0:
            return None
        area = cv2.contourArea(contour)
        if area < 30 * 30:
            return None
        # 顶点数用粗epsilon（0.035），凹口数用细epsilon（0.02）——
        # 粗epsilon会把五角星的凹口合并掉（0.035×长周长≈17px）
        approx = cv2.approxPolyDP(contour, 0.035 * peri, True)
        n_vertices = len(approx)
        # 凹顶点数：顶点转向与多边形环绕方向相反即凹。
        # 五角星5个凹口，十字4个，圆形/方形/菱形/三角形0个。
        n_conc = 0
        if n_vertices >= 4:
            fine = cv2.approxPolyDP(contour, 0.02 * peri, True)
            pts = fine[:, 0].astype(np.float32)
            nf = len(pts)
            sarea = 0.0
            for i in range(nf):
                p, q = pts[i], pts[(i + 1) % nf]
                sarea += p[0] * q[1] - q[0] * p[1]
            # 转向=入边(b-a)到出边(c-b)的叉积；凹顶点转向与环绕方向相反
            for i in range(nf):
                a = pts[(i - 1) % nf]
                b = pts[i]
                c = pts[(i + 1) % nf]
                cross = ((b[0] - a[0]) * (c[1] - b[1])
                         - (b[1] - a[1]) * (c[0] - b[0]))
                if cross * sarea < 0:
                    n_conc += 1
        # 轮廓点到质心距离的变异系数（圆形判据，对锯齿不敏感）
        M = cv2.moments(contour)
        cv_dist = 1.0
        if M["m00"] > 0:
            cx_m = M["m10"] / M["m00"]
            cy_m = M["m01"] / M["m00"]
            dists = np.sqrt((contour[:, 0, 0] - cx_m) ** 2
                            + (contour[:, 0, 1] - cy_m) ** 2)
            cv_dist = float(np.std(dists) / max(np.mean(dists), 1e-6))
        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        fill_rect = area / max(rw * rh, 1e-6)
        ang = abs(rect[2]) % 90.0
        ang = min(ang, 90.0 - ang)
        # 凹形：十字 vs 五角星。十字臂细（fillR<0.42）；五角星凹口
        # 深且占满外接矩形（fillR 0.42-0.52）。凹口3个+fillR<0.60
        # 兜住厚臂/非对称十字（自测卡十字fillR≈0.56）
        if n_conc >= 3:
            if fill_rect < 0.42:
                return "cross"
            if n_conc >= 5:
                return "pentagon"
            if fill_rect < 0.60:
                return "cross"
            return "pentagon"
        # 凸形
        if n_vertices == 3:
            return "triangle"
        if ang >= 25.0:
            return "diamond"
        if fill_rect >= 0.84:
            return "square"
        # 圆形（fillR 0.66-0.85、半径变异小）vs 三角形
        # （fillR 0.53-0.75、半径变异大）——cvd分界0.13
        if cv_dist < 0.13:
            return "circle"
        return "triangle"

    # ═══════════════════════════════════════════════════════════
    # 确认 + 冷却（与QRDetector一致）
    # ═══════════════════════════════════════════════════════════

    def _confirm(self, shape, dbg):
        now = int(time.time() * 1000)
        if shape == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = shape
            self.candidate_count = 1
            self.first_candidate_ms = now
            if self.debug:
                print(f"  [shape] NEW {shape}")

        if self.candidate_count >= self.stable_frames:
            ready = (self.last_send_ms is None
                     or (now - self.last_send_ms) >= self.cooldown_ms)
            if ready:
                action = self.action_map[shape]
                self.last_send_ms = now
                self.candidate_count = 0
                latency = now - (self.first_candidate_ms or now)
                dbg["action"] = action
                dbg["latency_ms"] = latency
                if self.debug:
                    print(f"  [shape] >>> SEND action={action} ({shape}) "
                          f"latency={latency}ms")
                return action, dbg
        return None, dbg


# ═══════════════════════════════════════════════════════════
# 自测：合成图卡验证分类
# ═══════════════════════════════════════════════════════════

def _make_card(shape_name, size=200):
    img = np.ones((size, size, 3), dtype=np.uint8) * 255
    cx, cy = size // 2, size // 2
    cv2.rectangle(img, (10, 10), (size - 10, size - 10), (0, 0, 0), 3)
    c = (0, 0, 0)
    if shape_name == "circle":
        cv2.circle(img, (cx, cy), size // 3, c, 3)
    elif shape_name == "triangle":
        pts = np.array([[cx, cy - size // 3], [cx - size // 3, cy + size // 4],
                        [cx + size // 3, cy + size // 4]], np.int32)
        cv2.polylines(img, [pts], True, c, 3)
    elif shape_name == "square":
        cv2.rectangle(img, (cx - size // 3, cy - size // 3),
                      (cx + size // 3, cy + size // 3), c, 3)
    elif shape_name == "diamond":
        pts = np.array([[cx, cy - size // 3], [cx - size // 3, cy],
                        [cx, cy + size // 3], [cx + size // 3, cy]], np.int32)
        cv2.polylines(img, [pts], True, c, 3)
    elif shape_name == "pentagon":
        r_outer = size // 3
        r_inner = r_outer * 0.45
        pts = []
        for i in range(10):
            ang = -np.pi / 2 + np.pi * i / 5
            r = r_outer if i % 2 == 0 else r_inner
            pts.append([int(cx + r * np.cos(ang)), int(cy + r * np.sin(ang))])
        cv2.polylines(img, [np.array(pts, np.int32)], True, c, 3)
    elif shape_name == "cross":
        w = size // 25
        arm = size // 3
        pts = [
            [cx - w, cy - arm], [cx + w, cy - arm],
            [cx + w, cy - w],   [cx + arm, cy - w],
            [cx + arm, cy + w], [cx + w, cy + w],
            [cx + w, cy + arm], [cx - w, cy + arm],
            [cx - w, cy + arm], [cx - arm, cy + w],
            [cx - arm, cy - w], [cx - w, cy - w],
        ]
        cv2.polylines(img, [np.array(pts, np.int32)], True, c, 3)
    return img


if __name__ == "__main__":
    names = ["circle", "pentagon", "square", "diamond", "cross", "triangle"]
    sd = ShapeDetector(stable_frames=1, cooldown_ms=0, debug=False)
    ok = 0
    for name in names:
        card = _make_card(name)
        # 真实场景：图卡贴地45°俯视 → 纵向压缩0.55，放白底场景
        card = cv2.resize(card, (200, int(200 * 0.55)))
        canvas = np.ones((540, 960, 3), dtype=np.uint8) * 240
        canvas[200:200 + card.shape[0], 380:380 + card.shape[1]] = card
        action, dbg = sd.update(canvas)
        expect = sd.action_map[name]
        got = dbg.get("shape")
        status = "OK" if got == name else f"FAIL(got {got})"
        if got == name:
            ok += 1
        print(f"  {name:>10} -> {str(got):>10}  action={action}  [{status}]")
    print(f"Result: {ok}/6")
