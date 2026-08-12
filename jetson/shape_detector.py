"""ShapeDetector — 2026新规则几何图卡识别（6种几何图形）。

圆形/五角星/正方形/菱形/十字形/三角形，10cm×10cm白底黑线卡，带矩形外边框。
传统CV方案：外框检测 → 单应性矫正 → 轮廓分类（顶点计数 + 凸性 + 圆形度）。

接口与 QRDetector 保持一致：update(bgr) → (action_number, dbg) 或 (None, None)。
动作映射（与2025二维码1-6对应）：
  圆形=1举左手 五角星=2举右手 正方形=3抬左腿 菱形=4抬右腿 十字形=5举双手 三角形=6摇头
"""
import cv2
import time
import numpy as np


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ShapeDetector:
    def __init__(
        self,
        stable_frames=3,        # 连续确认帧数
        cooldown_ms=3200,       # 发送冷却
        min_edge_px=30,         # 图卡最小边长（像素）
        max_edge_px=500,        # 图卡最大边长
        debug=True,
    ):
        self.stable_frames = stable_frames
        self.cooldown_ms = cooldown_ms
        self.min_edge = min_edge_px
        self.max_edge = max_edge_px
        self.debug = debug

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

        # 单应性矫正输出尺寸（图卡正视图）
        self.warp_size = 200

    # ═══════════════════════════════════════════════════════════
    # 主入口
    # ═══════════════════════════════════════════════════════════

    def update(self, bgr_or_gray):
        """返回 (action_number, debug_dict) 或 (None, None)。"""
        if len(bgr_or_gray.shape) == 3:
            gray = cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = bgr_or_gray

        # 预处理：CLAHE + 二值化（白底黑线，THRESH_BINARY_INV: 线=白）
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        _, binary = cv2.threshold(enhanced, 0, 255,
                                  cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 找图卡外框（10cm矩形）
        card_quad = self._find_card_quad(binary)
        shape = None
        dbg = {"card_found": card_quad is not None}

        if card_quad is not None:
            # 单应性矫正为正视图
            warp = self._warp_card(binary, card_quad)
            shape = self._classify_shape(warp)
            dbg["warp"] = warp
            dbg["quad"] = card_quad
        else:
            # 外框没找到（太远/被遮挡），fallback：全图直接分类
            shape = self._classify_shape_full(binary)
            dbg["fallback"] = True

        if shape is None:
            self.candidate = None
            self.candidate_count = 0
            dbg["shape"] = None
            return None, dbg

        dbg["shape"] = shape
        return self._confirm(shape, dbg)

    # ═══════════════════════════════════════════════════════════
    # 外框检测
    # ═══════════════════════════════════════════════════════════

    def _find_card_quad(self, binary):
        """找图卡外框：所有4顶点候选中最优者。

        关键：真图卡外框是"空心环"——填充其内部后，环内还有内部形状的
        非白像素（验证通过）。黑线/斑块是实心条，填充后内部没有形状。
        """
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_edge * self.min_edge:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) != 4:
                continue
            # 宽高比 ≈ 1（正方形外框）
            x, y, w, h = cv2.boundingRect(approx)
            if w < self.min_edge or h < self.min_edge:
                continue
            aspect = max(w, h) / max(min(w, h), 1)
            if aspect > 1.4:
                continue
            quad = approx.reshape(-1, 2)
            # 环验证：填充四边形内部，内部应有形状（非外框线本身）
            if self._is_ring_with_content(binary, quad, area):
                candidates.append((area, quad))
        if not candidates:
            return None
        # 取面积最大的通过验证的候选
        candidates.sort(key=lambda t: -t[0])
        return candidates[0][1]

    def _is_ring_with_content(self, binary, quad, outer_area):
        """验证四边形是空心环且内部有内容（图卡外框特征）。"""
        h, w = binary.shape
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, [quad.astype(np.int32)], -1, 255, cv2.FILLED)
        inner = cv2.bitwise_and(binary, mask)
        # 内部非白像素（形状线）占比：环形区域约25-75%
        inner_area = np.count_nonzero(inner)
        if inner_area <= 0:
            return False
        ratio = inner_area / max(outer_area, 1.0)
        # 实心条：ratio≈1.0；空心环+内部形状：ratio≈0.3-0.75
        return 0.15 < ratio < 0.9

    def _warp_card(self, binary, quad):
        """4角点 → 200×200 正视图（逆时针排序，保证内形状不镜像）。"""
        rect = cv2.minAreaRect(quad)
        order = np.zeros((4, 2), dtype=np.float32)
        pts = quad.astype(np.float32)
        # 按角度排序：minAreaRect 返回 (cx,cy,w,h,angle)，直接用它拿顺序不可靠
        # 手动排序：左上/右上/右下/左下
        s = pts.sum(axis=1)
        d = np.diff(pts, axis=1).ravel()
        order[0] = pts[np.argmin(s)]   # top-left
        order[2] = pts[np.argmax(s)]   # bottom-right
        order[1] = pts[np.argmin(d)]   # top-right
        order[3] = pts[np.argmax(d)]   # bottom-left
        dst = np.float32([[0, 0], [self.warp_size - 1, 0],
                          [self.warp_size - 1, self.warp_size - 1],
                          [0, self.warp_size - 1]])
        M = cv2.getPerspectiveTransform(order, dst)
        return cv2.warpPerspective(binary, M, (self.warp_size, self.warp_size))

    # ═══════════════════════════════════════════════════════════
    # 形状分类
    # ═══════════════════════════════════════════════════════════

    def _classify_shape(self, warp):
        """在矫正后的正视图上分类形状。"""
        # 闭运算连接细臂断裂（十字的臂在低分辨率下易断）
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        warp = cv2.morphologyEx(warp, cv2.MORPH_CLOSE, kernel)
        # RETR_LIST: 需要内部嵌套形状（外框环内），EXTERNAL 拿不到
        contours, _ = cv2.findContours(warp, cv2.RETR_LIST,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        # 外框是最大的，去掉它找内部形状
        outer_area = max(cv2.contourArea(c) for c in contours)
        inner = None
        for c in contours:
            a = cv2.contourArea(c)
            if a < outer_area * 0.8:  # 明显小于外框
                if inner is None or a > cv2.contourArea(inner):
                    inner = c
        if inner is None:
            return None
        return self._classify_contour(inner)

    def _classify_shape_full(self, binary):
        """fallback：全图直接分类（外框没找到时）。"""
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a > best_area:
                best_area = a
                best = c
        if best is None or best_area < self.min_edge * self.min_edge:
            return None
        return self._classify_contour(best)

    def _classify_contour(self, contour):
        """核心分类：顶点计数 + 凸性 + 圆形度。

        关键规则（调研结论）：
        - 正方形 vs 菱形：都4顶点，Hu矩相同 → 用外接矩形宽高比区分
          （菱形45°时外接矩形宽高比≈√2，正方形≈1）
        - 五角星：approxPolyDP 后10顶点（凹形）→ 凸包面积比 > 1.15
        - 十字形：12顶点左右（凹形）→ 凸包面积比 > 1.15
        - 圆形：圆形度 4πA/P² ≈ 1
        """
        peri = cv2.arcLength(contour, True)
        if peri <= 0:
            return None
        area = cv2.contourArea(contour)
        if area < self.min_edge * self.min_edge:
            return None

        # 圆形度（圆 = 1，正方形 ≈ 0.785，星形 ≈ 0.5）
        circularity = 4.0 * np.pi * area / (peri * peri)

        # 顶点数
        approx = cv2.approxPolyDP(contour, 0.035 * peri, True)
        n_vertices = len(approx)

        # 圆形：轮廓点到质心距离的变异系数（对锯齿不敏感）
        # 圆 CV≈0.05-0.1，正方形/菱形≈0.22，五角星/十字更大
        # 必须在顶点数>=6时判定（正方形/菱形4顶点不会误判）
        if n_vertices >= 6:
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx_m = M["m10"] / M["m00"]
                cy_m = M["m01"] / M["m00"]
                dists = np.sqrt((contour[:, 0, 0] - cx_m) ** 2
                                + (contour[:, 0, 1] - cy_m) ** 2)
                cv_dist = float(np.std(dists) / max(np.mean(dists), 1e-6))
                if cv_dist < 0.15:
                    return "circle"

        # 凸性：凸包面积 / 轮廓面积（凹形 > 1.12）
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        concavity = hull_area / max(area, 1.0)
        hull_approx = cv2.approxPolyDP(hull, 0.035 * peri, True)
        n_hull = len(hull_approx)

        if n_vertices == 3:
            return "triangle"
        if n_vertices == 4:
            # 正方形 vs 菱形：minAreaRect 主轴角度（矫正后正视图）
            # square 边∥坐标轴 → angle≈0/90；diamond 旋转45° → angle≈45
            rect = cv2.minAreaRect(contour)
            ang = abs(rect[2]) % 90.0
            ang = min(ang, 90.0 - ang)  # 归一化到 [0,45]
            if ang < 20.0:
                return "square"
            return "diamond"
        if n_vertices == 5:
            return "pentagon"  # 凸五边形
        if n_vertices >= 8 and concavity > 1.12:
            # 凹形多顶点：五角星 vs 十字
            # 五角星：凸包=5 + 凸性适中（合成1.61/真实1.76）
            # 十字：凸包≠5（合成8）或深凹（真实4.5）
            if n_hull == 5 and concavity < 2.5:
                return "pentagon"
            return "cross"
        return None

    # ═══════════════════════════════════════════════════════════
    # 确认 + 冷却（与QRDetector一致）
    # ═══════════════════════════════════════════════════════════

    def _confirm(self, shape, dbg):
        """确认逻辑 + 冷却，返回 (action, dbg) 或 (None, dbg)。"""
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
    """合成一张图卡（白底黑线，带外框）。返回BGR图。"""
    img = np.ones((size, size, 3), dtype=np.uint8) * 255
    cx, cy = size // 2, size // 2
    # 外框
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
        # 五角星（星形）：外5顶点 + 内5顶点
        r_outer = size // 3
        r_inner = r_outer * 0.45
        pts = []
        for i in range(10):
            ang = -np.pi / 2 + np.pi * i / 5
            r = r_outer if i % 2 == 0 else r_inner
            pts.append([int(cx + r * np.cos(ang)), int(cy + r * np.sin(ang))])
        cv2.polylines(img, [np.array(pts, np.int32)], True, c, 3)
    elif shape_name == "cross":
        # 十字：12顶点多边形轮廓（单条闭合线，避免拆分）
        w = size // 6
        arm = size // 3
        pts = [
            [cx - w, cy - arm], [cx + w, cy - arm],
            [cx + w, cy - w],   [cx + arm, cy - w],
            [cx + arm, cy + w], [cx + w, cy + w],
            [cx + w, cy + arm], [cx - w, cy + arm],
            [cx - w, cy + w],   [cx - arm, cy + w],
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
        action, dbg = sd.update(card)
        expect = sd.action_map[name]
        got = dbg.get("shape")
        status = "OK" if got == name else f"FAIL(got {got})"
        if got == name:
            ok += 1
        print(f"  {name:>10} -> {str(got):>10}  action={action}  [{status}]")
    print(f"Result: {ok}/6")
